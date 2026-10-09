package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Expansion Hub Port 3 is configured as "intakeMotor"
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        telemetry.addLine("Intake Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                intakeMotor.setPower(1.0);
            }
            else if (gamepad1.b) {
                intakeMotor.setPower(-1.0);
            }
            else {
                intakeMotor.setPower(0);
            }
        }
    }
}