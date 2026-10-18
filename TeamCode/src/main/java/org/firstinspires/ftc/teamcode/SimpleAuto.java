package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Simple Auto Forward Back", group = "StarterBot")
public class SimpleAuto extends LinearOpMode {

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // Drive forward for 1 second
        setPower(-0.5);
        sleep(1000);

        // Stop and wait 30 seconds
        setPower(0);
        sleep(28000);

        // Drive backward for 1 second
        setPower(0.5);
        sleep(1000);

        // Stop
        setPower(0);
    }

    void setPower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
}