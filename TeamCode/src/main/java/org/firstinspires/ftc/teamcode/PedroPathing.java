package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Our own path follower. No Pedro Pathing library needed.
 *
 * How it works:
 * - The goBILDA Pinpoint (built into the FTC SDK) tells us where the robot is.
 * - For each waypoint, we drive straight at it with a P controller and
 * turn to the target heading at the same time.
 * - When we are close enough, we move on to the next waypoint.
 * - After the last waypoint, the Shooter fires.
 *
 * Field coordinates are the same as the Pedro visualizer:
 * inches, (0,0) in a corner, heading 0 = facing +X, counterclockwise is positive.
 */
@Autonomous(name = "PedroPathing", group = "Auto")
public class PedroPathing extends OpMode {

    // ===== Robot configuration names (CHANGE THESE to match the Driver Station) =====
    private static final String FRONT_LEFT = "lf";
    private static final String FRONT_RIGHT = "rf";
    private static final String BACK_LEFT = "lb";
    private static final String BACK_RIGHT = "rb";
    private static final String PINPOINT = "pinpoint";

    // ===== Pinpoint setup (MEASURE these on your robot) =====
// X offset: how far LEFT of center the forward pod is (left = positive), in mm.
// Y offset: how far FORWARD of center the strafe pod is (forward = positive), in mm.
    private static final double X_POD_OFFSET_MM = 0.0;
    private static final double Y_POD_OFFSET_MM = 0.0;

    // ===== Tuning =====
    private static final double DRIVE_KP = 0.06; // power per inch of error
    private static final double TURN_KP = 1.2; // power per radian of error
    private static final double MAX_POWER = 0.7; // top speed (0 to 1). Start low!
    private static final double MIN_POWER = 0.08; // enough to overcome friction
    private static final double STRAFE_MULT = 1.1; // mecanum strafes a bit weaker
    private static final double POS_TOL_IN = 1.0; // "arrived" distance
    private static final double HEAD_TOL_DEG = 2.0; // "arrived" heading
    private static final double SEGMENT_TIMEOUT_S = 5.0; // skip ahead if stuck

    private static final int SHOTS = 3;
    private static final double SHOOT_TIMEOUT_S = 6.0;

    // ===== Path from the visualizer: {x, y, heading in degrees} =====
    private static final double[] START = {131.6123, 42.2748, 90};
    private static final double[][] WAYPOINTS = {
            {130.6859, 122.7815, 90.6593}, // path 1: up the field
            {84.6235, 122.671, 0}, // path 2: across the top
            {131.9003, 41.8908, 120.3384}, // path 3: back to start area, then shoot
    };

    // ===== Hardware =====
    private DcMotor fl, fr, bl, br;
    private GoBildaPinpointDriver pinpoint;
    private Shooter shooter;

    // ===== State =====
    private enum Step { DRIVING, SHOOTING, DONE }
    private Step step = Step.DRIVING;
    private int waypoint = 0;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Current robot pose (updated every loop)
    private double robotX, robotY, robotH; // inches, inches, radians

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, FRONT_LEFT);
        fr = hardwareMap.get(DcMotor.class, FRONT_RIGHT);
        bl = hardwareMap.get(DcMotor.class, BACK_LEFT);
        br = hardwareMap.get(DcMotor.class, BACK_RIGHT);

// goBILDA starter bot: left side reversed. Flip if the robot drives backward or spins.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
// Flip one of these if pushing the robot forward makes the position go the wrong way.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // robot must be still during init

        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Place robot at (131.6, 42.3), facing 90 deg. Keep it still.");
        telemetry.update();
    }

    @Override
    public void start() {
// Tell the Pinpoint where we are on the field.
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START[0], START[1],
                AngleUnit.DEGREES, START[2]));
        matchTimer.reset();
        stepTimer.reset();
    }

    @Override
    public void loop() {
        updatePose();
        shooter.update(waypoint >= WAYPOINTS.length - 1); // aim turret on the last leg

        switch (step) {
            case DRIVING: {
                double[] target = WAYPOINTS[waypoint];
                boolean arrived = driveTo(target[0], target[1], target[2]);
                boolean stuck = stepTimer.seconds() > SEGMENT_TIMEOUT_S;

                if (arrived || stuck) {
                    waypoint++;
                    stepTimer.reset();
                    if (waypoint == WAYPOINTS.length - 1) {
                        shooter.spinUp(); // spin flywheels on the drive back
                    }
                    if (waypoint >= WAYPOINTS.length) {
                        waypoint = WAYPOINTS.length - 1;
                        stopDrive();
                        shooter.startShooting(SHOTS);
                        step = Step.SHOOTING;
                    }
                }
                break;
            }

            case SHOOTING: {
// Keep holding the final spot while shooting.
                double[] last = WAYPOINTS[WAYPOINTS.length - 1];
                driveTo(last[0], last[1], last[2]);

                if (shooter.isDone() || stepTimer.seconds() > SHOOT_TIMEOUT_S) {
                    shooter.stop();
                    stopDrive();
                    step = Step.DONE;
                }
                break;
            }

            case DONE:
            default:
                stopDrive();
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Waypoint", waypoint + 1);
        telemetry.addData("X", "%.1f", robotX);
        telemetry.addData("Y", "%.1f", robotY);
        telemetry.addData("Heading", "%.1f", Math.toDegrees(robotH));
        shooter.addTelemetry(telemetry);
        telemetry.addData("Time", "%.1f", matchTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
        if (shooter != null) shooter.stop();
    }

// ================= Path follower =================

    private void updatePose() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        robotX = p.getX(DistanceUnit.INCH);
        robotY = p.getY(DistanceUnit.INCH);
        robotH = p.getHeading(AngleUnit.RADIANS);
    }

    /**
     * Drive toward a field position and heading. Call every loop.
     * @return true once the robot is within tolerance.
     */
    private boolean driveTo(double targetX, double targetY, double targetHeadingDeg) {
        double errX = targetX - robotX;
        double errY = targetY - robotY;
        double dist = Math.hypot(errX, errY);
        double errH = angleWrap(Math.toRadians(targetHeadingDeg) - robotH);

        if (dist < POS_TOL_IN && Math.abs(errH) < Math.toRadians(HEAD_TOL_DEG)) {
            stopDrive();
            return true;
        }

// Speed toward the target: fast when far, slows down as it gets close.
        double speed = 0;
        if (dist >= POS_TOL_IN) {
            speed = Range.clip(DRIVE_KP * dist, MIN_POWER, MAX_POWER);
        }
        double fieldX = (dist > 1e-6) ? errX / dist * speed : 0;
        double fieldY = (dist > 1e-6) ? errY / dist * speed : 0;

// Rotate field direction into robot direction.
        double cos = Math.cos(robotH);
        double sin = Math.sin(robotH);
        double forward = fieldX * cos + fieldY * sin;
        double left = -fieldX * sin + fieldY * cos;

        double turn = 0;
        if (Math.abs(errH) >= Math.toRadians(HEAD_TOL_DEG)) {
            turn = Range.clip(TURN_KP * errH, -MAX_POWER, MAX_POWER);
            if (Math.abs(turn) < MIN_POWER) turn = Math.signum(turn) * MIN_POWER;
        }

        setDrive(forward, left * STRAFE_MULT, turn);
        return false;
    }

    /** forward = +forward, left = strafe left, turn = counterclockwise. */
    private void setDrive(double forward, double left, double turn) {
        double flP = forward - left - turn;
        double blP = forward + left - turn;
        double frP = forward + left + turn;
        double brP = forward - left + turn;

// Scale down so no wheel goes over 1.0 (keeps the direction correct).
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(flP), Math.abs(blP)),
                Math.max(Math.abs(frP), Math.abs(brP))));
        fl.setPower(flP / max);
        bl.setPower(blP / max);
        fr.setPower(frP / max);
        br.setPower(brP / max);
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /** Keep an angle between -PI and PI so the robot turns the short way. */
    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

// ================= Shooter (flywheels, feeder, turret, Limelight) =================

    /**
     * Differential shooter (two flywheels at different speeds), feeder, and
     * Limelight-aimed turret. All names below are PLACEHOLDERS. Change them to
     * exactly match the names in your Driver Station robot configuration.
     */
    public static class Shooter {

        // ===== Robot configuration names (CHANGE THESE) =====
        public static String TOP_FLYWHEEL = "topFlywheel";
        public static String BOTTOM_FLYWHEEL = "bottomFlywheel";
        public static String FEEDER = "feeder"; // CRServo that pushes artifacts in
        public static String TURRET = "turret"; // DcMotorEx with encoder
        public static String LIMELIGHT = "limelight";

        // ===== Flywheel tuning =====
// goBILDA 6000 RPM motor = 28 ticks/rev. ticks/sec = RPM * 28 / 60.
// Different top/bottom speeds give the artifact spin. Tune on the field.
        public static double TOP_TPS = 1900;
        public static double BOTTOM_TPS = 1500;
        public static double SPEED_TOL = 60; // ticks/sec window to count as "at speed"

        // ===== Feeder timing =====
        public static double FEED_POWER = 1.0;
        public static double FEED_TIME_S = 0.35; // how long to push one artifact
        public static double RECOVER_S = 0.40; // let flywheels recover between shots

        // ===== Turret aiming =====
        public static int LL_PIPELINE = 0;
        public static double TURRET_KP = 0.02; // power per degree of tx
        public static double TURRET_MAX_POWER = 0.5;
        public static double AIM_TOL_DEG = 1.5;
        public static int TURRET_MIN_TICKS = -600; // soft limits so wires don't wrap
        public static int TURRET_MAX_TICKS = 600;

        private final DcMotorEx top, bottom, turret;
        private final CRServo feeder;
        private final Limelight3A limelight;

        private enum Mode { IDLE, SPINNING, WAIT_READY, FEEDING, RECOVER, DONE }
        private Mode mode = Mode.IDLE;
        private int shotsLeft = 0;
        private final ElapsedTime timer = new ElapsedTime();

        private boolean targetVisible = false;
        private double tx = 0;

        public Shooter(HardwareMap hw) {
            top = hw.get(DcMotorEx.class, TOP_FLYWHEEL);
            bottom = hw.get(DcMotorEx.class, BOTTOM_FLYWHEEL);
            turret = hw.get(DcMotorEx.class, TURRET);
            feeder = hw.get(CRServo.class, FEEDER);
            limelight = hw.get(Limelight3A.class, LIMELIGHT);

// Flip one of these if a wheel spins the wrong way.
            top.setDirection(DcMotorSimple.Direction.FORWARD);
            bottom.setDirection(DcMotorSimple.Direction.REVERSE);
            for (DcMotorEx m : new DcMotorEx[]{top, bottom}) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

// Turret must start centered at init so encoder 0 = straight ahead.
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            feeder.setPower(0);

            limelight.pipelineSwitch(LL_PIPELINE);
            limelight.start();
        }

        /** Start flywheels early so they are at speed when we arrive. */
        public void spinUp() {
            top.setVelocity(TOP_TPS);
            bottom.setVelocity(BOTTOM_TPS);
            if (mode == Mode.IDLE) mode = Mode.SPINNING;
        }

        /** Fire n artifacts. Waits for speed and aim before each one. */
        public void startShooting(int n) {
            spinUp();
            shotsLeft = n;
            mode = Mode.WAIT_READY;
        }

        public boolean isDone() { return mode == Mode.DONE; }

        public boolean atSpeed() {
            return Math.abs(top.getVelocity() - TOP_TPS) < SPEED_TOL
                    && Math.abs(bottom.getVelocity() - BOTTOM_TPS) < SPEED_TOL;
        }

        public boolean aimed() {
            return targetVisible && Math.abs(tx) < AIM_TOL_DEG;
        }

        /** Call every loop. aim = true lets the Limelight drive the turret. */
        public void update(boolean aim) {
            updateTurret(aim);

            switch (mode) {
                case WAIT_READY:
// If the Limelight can't see the goal, shoot on speed alone
// (robot heading is already pointing at the goal).
                    boolean aimOk = aimed() || !targetVisible;
                    if (atSpeed() && aimOk) {
                        feeder.setPower(FEED_POWER);
                        timer.reset();
                        mode = Mode.FEEDING;
                    }
                    break;

                case FEEDING:
                    if (timer.seconds() > FEED_TIME_S) {
                        feeder.setPower(0);
                        shotsLeft--;
                        timer.reset();
                        mode = (shotsLeft > 0) ? Mode.RECOVER : Mode.DONE;
                    }
                    break;

                case RECOVER:
                    if (timer.seconds() > RECOVER_S) mode = Mode.WAIT_READY;
                    break;

                default:
                    break;
            }
        }

        private void updateTurret(boolean aim) {
            LLResult r = limelight.getLatestResult();
            targetVisible = (r != null && r.isValid());
            tx = targetVisible ? r.getTx() : 0;

            double power = 0;
            int pos = turret.getCurrentPosition();
            if (aim && targetVisible) {
// Flip the sign if the turret turns away from the target.
                power = Range.clip(-TURRET_KP * tx, -TURRET_MAX_POWER, TURRET_MAX_POWER);
            } else {
// No target: drift back toward center.
                power = Range.clip(-0.002 * pos, -0.3, 0.3);
            }
// Soft limits.
            if (pos >= TURRET_MAX_TICKS && power > 0) power = 0;
            if (pos <= TURRET_MIN_TICKS && power < 0) power = 0;
            turret.setPower(power);
        }

        public void stop() {
            top.setVelocity(0);
            bottom.setVelocity(0);
            feeder.setPower(0);
            turret.setPower(0);
            limelight.stop();
            mode = Mode.IDLE;
        }

        public void addTelemetry(Telemetry t) {
            t.addData("Shooter", mode);
            t.addData("Shots left", shotsLeft);
            t.addData("Top vel", "%.0f / %.0f", top.getVelocity(), TOP_TPS);
            t.addData("Bottom vel", "%.0f / %.0f", bottom.getVelocity(), BOTTOM_TPS);
            t.addData("Target", targetVisible ? String.format("tx %.1f", tx) : "not seen");
            t.addData("Turret ticks", turret.getCurrentPosition());
        }
    }
}