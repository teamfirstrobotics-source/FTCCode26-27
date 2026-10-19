package org.firstinspires.ftc.FlywheelAdvancedAdhvik;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class kVTuner extends OpMode {

    CRServo windmillServo;
    FlywheelClass flywheel = new FlywheelClass();

    public static double kV=0;
    public double kS=.0428;
    public double goalRPM =2700;

    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    int incrementIdx = 4; // Start at 0.001

    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        windmillServo.setPower(-1);
        if (gamepad1.dpadRightWasReleased() && incrementIdx <4) {
            incrementIdx++;
        } else if (gamepad1.dpadLeftWasPressed() && incrementIdx >0) {
            incrementIdx--;
        }

        if (gamepad1.a) {
            goalRPM=1200;
        }
        else if (gamepad1.b){
            goalRPM=500;
        }
        double currentStep = increments[incrementIdx];

        //change kS
        if (gamepad1.dpadUpWasPressed()) {kV += currentStep; }
        if (gamepad1.dpadDownWasPressed()) { kV -=currentStep; }

        double power = (kV*goalRPM) + kS;
        flywheel.setMotorPower(power);

        telemetry.addData("Step", "%.6f", currentStep);
        telemetry.addData("kV", "%.6f", kV);
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("Ticks per Sec", flywheel.getTicksPerSec());
    }
}

