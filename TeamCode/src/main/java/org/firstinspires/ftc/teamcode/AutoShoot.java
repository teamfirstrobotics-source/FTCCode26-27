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

@Autonomous(name = "Auto Shoot ", group = "StarterBot")
public class AutoShoot extends LinearOpMode {

    // Same values as your teleop
    final int LAUNCHER_TARGET_VELOCITY = 1250;
    final int LAUNCHER_MIN_VELOCITY    = 1200;

    // Shot counting
    final int    TOTAL_SHOTS            = 3;
    final int SHOT_DIP_VELOCITY        = 1150;
    final double SHOOT_TIMEOUT_SECONDS  = 8.0;  // safety: stop even if we never count 3
    final long   FINISH_DELAY_MS        = 500;  // let the last ball clear before shutting down

    // Other tuning
    final double SPINUP_TIMEOUT_SECONDS = 3.0;
    final long   DRIVE_TIME_MS          = 1000;    // set > 0 to drive before shooting
    final double DRIVE_POWER            = -0.5; // negative = forward

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;
    private DcMotorEx launcher;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    @Override
    public void runOpMode() {
        leftFrontDrive   = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive    = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive   = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class, "i");
        launcher = hardwareMap.get(DcMotorEx.class, "l");
        windmillServo = hardwareMap.get(CRServo.class, "t");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "li");
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

        // 1. Optional: drive to shooting spot
        if (DRIVE_TIME_MS > 0) {
            setDrivePower(DRIVE_POWER);
            sleep(DRIVE_TIME_MS);
            setDrivePower(0);
        }

        // 2. Spin up launcher and wait until it's at speed (or timeout)
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()
                && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY
                && timer.seconds() < SPINUP_TIMEOUT_SECONDS) {
            telemetry.addData("Spinning up", launcher.getVelocity());
            telemetry.update();
            idle();
        }

        // 3. Feed balls and count shots by watching for speed dips
        int shotsFired = 0;
        boolean armed = true;   // true = ready to detect the next dip
        timer.reset();

        while (opModeIsActive() && shotsFired < TOTAL_SHOTS
                && timer.seconds() < SHOOT_TIMEOUT_SECONDS) {

            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            double velocity = launcher.getVelocity();

            // Same feeding logic as your teleop
            if (velocity > LAUNCHER_MIN_VELOCITY) {
                windmillServo.setPower(1);
                setIntakePower(0.5);
            } else {
                windmillServo.setPower(0);
                setIntakePower(0);
            }

            // Count a shot on the dip, re-arm once the flywheel recovers
            if (armed && velocity < SHOT_DIP_VELOCITY) {
                shotsFired++;
                armed = false;
            } else if (!armed && velocity > LAUNCHER_MIN_VELOCITY) {
                armed = true;
            }

            telemetry.addData("Shots fired", "%d / %d", shotsFired, TOTAL_SHOTS);
            telemetry.addData("Launcher velocity", velocity);
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
        }

        // 4. Let the last ball clear the flywheel, then shut down
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

    void setDrivePower(double p) {
        leftFrontDrive.setPower(p);
        rightFrontDrive.setPower(p);
        leftBackDrive.setPower(p);
        rightBackDrive.setPower(p);
    }
}