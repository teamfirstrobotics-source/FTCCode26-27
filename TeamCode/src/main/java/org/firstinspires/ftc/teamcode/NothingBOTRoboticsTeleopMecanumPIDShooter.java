/*  /*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Teleop for the goBILDA StarterBot with mecanum wheels.
 *
 * The launcher uses our OWN controller (feedforward + PID) instead of the SDK's built-in
 * velocity PIDF. Because of that, the launcher motor runs in RUN_WITHOUT_ENCODER: we still read
 * the encoder velocity, but we are the ones deciding the motor power every loop.
 *
 * All launcher velocities and gains are in ENCODER TICKS PER SECOND so that kV/kP match the
 * units they were tuned in. RPM is only used for display.
 */
@TeleOp(name = "Nothing BOT Robotics Teleop With PID", group = "StarterBot")
//@Disabled
public class NothingBOTRoboticsTeleopMecanumPIDShooter extends OpMode {

    // ------------------------------------------------------------------------------------
    // Flywheel controller (feedforward + PID)
    // ------------------------------------------------------------------------------------
    static class Flywheel {
        private final DcMotorEx motor;

        // ---- Tuning values (units: ticks/sec) ----
        public double kV = 0.000367;  // power per (tick/sec). ~1 / max ticks-per-sec
        public double kS = 0.0428;    // power needed just to overcome friction
        public double kP = 0.00013;   // power per (tick/sec) of error
        public double kI = 0.0;       // start at 0, add last if it settles short of target
        public double kD = 0.0;       // start at 0, add small amount only if it overshoots

        // Limit on how much power the integral term is allowed to contribute (anti-windup)
        public double maxIntegralPower = 0.2;

        // ---- Motor info (for RPM display only) ----
        public double ticksPerRev = 28;
        public double gearRatio = 1;

        private double targetVelocity = 0;   // ticks/sec
        private double integral = 0;
        private double lastError = 0;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean wasRunning = false;

        Flywheel(DcMotorEx motor) {
            this.motor = motor;
            // We apply power ourselves, so don't use the SDK's built-in velocity loop.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(FLOAT); // let the wheel coast down, easier on the motor
        }

        /** Set the goal in ticks/sec. 0 turns the flywheel off. */
        public void setTargetVelocity(double ticksPerSec) {
            targetVelocity = ticksPerSec;
        }

        public double getTargetVelocity() {
            return targetVelocity;
        }

        public double getVelocity() {
            return motor.getVelocity(); // ticks/sec
        }

        public double getRPM() {
            return (getVelocity() / ticksPerRev) * 60.0 / gearRatio;
        }

        /** CALL THIS EVERY LOOP. */
        public void update() {
            if (targetVelocity <= 0) {
                motor.setPower(0);
                integral = 0;
                lastError = 0;
                wasRunning = false;
                return;
            }

            double dt = timer.seconds();
            timer.reset();
            if (!wasRunning) {
                // first loop after spin-up started: dt is meaningless, so skip I and D
                integral = 0;
                lastError = targetVelocity - getVelocity();
                dt = 0;
                wasRunning = true;
            }

            double error = targetVelocity - getVelocity();

            double derivative = 0;
            if (dt > 0) {
                integral += error * dt;
                derivative = (error - lastError) / dt;
            }
            lastError = error;

            // Anti-windup: keep the I contribution inside +/- maxIntegralPower
            if (kI != 0) {
                double maxIntegral = maxIntegralPower / kI;
                integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
            }

            double feedforward = (kV * targetVelocity) + kS;
            double feedback = (kP * error) + (kI * integral) + (kD * derivative);

            double power = feedforward + feedback;
            power = Math.max(0, Math.min(1, power)); // flywheel only spins one way
            motor.setPower(power);
        }
    }

    // ------------------------------------------------------------------------------------
    // OpMode members
    // ------------------------------------------------------------------------------------
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    private Flywheel flywheel;

    /*
     * Launcher velocities in encoder ticks per second (28 ticks per rev on FTC motors,
     * so RPM = ticks/sec / 28 * 60). These two values are the sock StarterBot numbers;
     * adjust them for your shooter.
     * The min velocity is the speed the flywheel must reach before the windmill feeds.
     */
    private static final double LAUNCHER_TARGET_VELOCITY = 1270;
    private static final double LAUNCHER_MIN_VELOCITY = 1170;
    //Change these values when you need to change the power of the shooter it'll afect how high and low the shooter will shoot the balls.

    // Power values stored so we can show them in telemetry
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    // Intake power (set from triggers each loop)
    private double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // The strings here must match the names assigned in the robot configuration.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class, "i");
        launcher = hardwareMap.get(DcMotorEx.class, "l");
        windmillServo = hardwareMap.get(CRServo.class, "t");
        leftIntakeServo = hardwareMap.get(CRServo.class, "li");
        rightIntakeServo = hardwareMap.get(CRServo.class, "ri");

        // Drivetrain directions. Pushing the stick forward MUST make the robot go forward.
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE makes the robot stop much faster, which is more controllable.
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // If the launcher spins the wrong way, uncomment this:
        // launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheel sets its own run mode (RUN_WITHOUT_ENCODER) and zero-power behavior.
        flywheel = new Flywheel(launcher);

        // Initialize the servo controller with a starting value
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        // Reverse the right intake servo so both pull elements in.
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Drive. Note: drive is on gamepad2, launcher/intake are on gamepad1.
        // If forward/back feels reversed, negate the first argument.
        mecanumDrive(gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);

        // Right trigger = intake in, left trigger = reverse. Both = off.
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // launch() may modify intakePower, so it runs BEFORE we write to the intake hardware.
        launch();

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // Telemetry
        telemetry.addData("Front", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Back", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Launcher target (ticks/s)", "%.0f", flywheel.getTargetVelocity());
        telemetry.addData("Launcher actual (ticks/s)", "%.0f", flywheel.getVelocity());
        telemetry.addData("Launcher RPM", "%.0f", flywheel.getRPM());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        flywheel.setTargetVelocity(0);
        flywheel.update();
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void launch() {
        // Hold right bumper to spin up the launcher; release to spin down.
        if (gamepad1.right_bumper) {
            flywheel.setTargetVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            flywheel.setTargetVelocity(0);
        }

        // Run the PID/feedforward controller. Must be called every loop.
        flywheel.update();

        // Windmill feeds while A is held AND the launcher is fast enough.
        if (gamepad1.a && flywheel.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
        } else {
            windmillServo.setPower(0);
        }
    }
}