package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Shoot Strafe", group = "StarterBot")
public class AutoShoot2 extends LinearOpMode {

    // Launcher (same as teleop)
    final int LAUNCHER_TARGET_VELOCITY = 1160;
    final int LAUNCHER_MIN_VELOCITY = 1150;

    // Shot counting
    final int TOTAL_SHOTS = 4;
    final int SHOT_DIP_VELOCITY = 1050;
    final double SHOOT_TIMEOUT_SECONDS = 8.0;
    final long FINISH_DELAY_MS = 500;
    final double SPINUP_TIMEOUT_SECONDS = 3.0;

    // ---- PATH TUNING (change these on the field) ----
    final double DRIVE_POWER = 0.5; // flip sign if robot goes the wrong way
    final double STRAFE_POWER = 0.5; // flip sign if strafe goes the wrong way
    final long FORWARD_MS = 2000; // step 1: drive forward
    final long STRAFE_LEFT_MS = 700; // step 2: strafe left
    final long STRAFE_RIGHT_MS = 700; // step 4: strafe right (same as left to undo it)
    final long BACK_MS = 2000; // step 5: drive back (same as forward to undo it)
    final long SETTLE_MS = 200; // pause between moves so the robot stops drifting

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;
    private DcMotorEx launcher;
    private CRServo windmillServo, leftIntakeServo, rightIntakeServo;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class, "i");
        launcher = hardwareMap.get(DcMotorEx.class, "l");
        windmillServo = hardwareMap.get(CRServo.class, "t");
        leftIntakeServo = hardwareMap.get(CRServo.class, "li");
        rightIntakeServo = hardwareMap.get(CRServo.class, "ri");

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

// Start spinning the launcher early so it is ready when we arrive
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

// 1. Forward
        move("Forward", DRIVE_POWER, 0, FORWARD_MS);

// 2. Strafe left
        move("Strafe left", 0, -STRAFE_POWER, STRAFE_LEFT_MS);

// 3. Shoot
        shoot();

// 4. Strafe right
        move("Strafe right", 0, STRAFE_POWER, STRAFE_RIGHT_MS);

// 5. Come back
        move("Back", -DRIVE_POWER, 0, BACK_MS);

        stopDrive();
        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    /**
     * Mecanum move for a fixed time.
     * forward: + = forward, - = backward
     * strafe: + = right, - = left
     */
    void move(String label, double forward, double strafe, long ms) {
        if (!opModeIsActive() || ms <= 0) return;

        leftFrontDrive.setPower(forward + strafe);
        rightFrontDrive.setPower(forward - strafe);
        leftBackDrive.setPower(forward - strafe);
        rightBackDrive.setPower(forward + strafe);

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) {
            telemetry.addData("Step", label);
            telemetry.addData("Launcher velocity", launcher.getVelocity());
            telemetry.update();
            idle();
        }

        stopDrive();
        sleep(SETTLE_MS);
    }

    void shoot() {
// Wait for flywheel to reach speed (or timeout)
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()
                && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY
                && timer.seconds() < SPINUP_TIMEOUT_SECONDS) {
            telemetry.addData("Step", "Spinning up");
            telemetry.addData("Launcher velocity", launcher.getVelocity());
            telemetry.update();
            idle();
        }

// Feed and count shots by watching for speed dips
        int shotsFired = 0;
        boolean armed = true;
        timer.reset();

        while (opModeIsActive() && shotsFired < TOTAL_SHOTS
                && timer.seconds() < SHOOT_TIMEOUT_SECONDS) {

            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            double velocity = launcher.getVelocity();

            if (velocity > LAUNCHER_MIN_VELOCITY) {
                windmillServo.setPower(1);
                setIntakePower(0.5);
            } else {
                windmillServo.setPower(0);
                setIntakePower(0);
            }

            if (armed && velocity < SHOT_DIP_VELOCITY) {
                shotsFired++;
                armed = false;
            } else if (!armed && velocity > LAUNCHER_MIN_VELOCITY) {
                armed = true;
            }

            telemetry.addData("Step", "Shooting");
            telemetry.addData("Shots fired", "%d / %d", shotsFired, TOTAL_SHOTS);
            telemetry.addData("Launcher velocity", velocity);
            telemetry.update();
        }

// Let the last ball clear, then shut the shooter down
        sleep(FINISH_DELAY_MS);
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        setIntakePower(0);
    }

    void setIntakePower(double p) {
        intake.setPower(p);
        leftIntakeServo.setPower(p);
        rightIntakeServo.setPower(p);
    }

    void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
