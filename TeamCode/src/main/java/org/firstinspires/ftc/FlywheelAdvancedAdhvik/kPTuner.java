package org.firstinspires.ftc.FlywheelAdvancedAdhvik;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class kPTuner extends OpMode {

    FlywheelClass flywheel = new FlywheelClass();

    public static double kV=0.0000367;
    public double kS=.0428;

    public double kP=0.0;
    public double goalRPM =2700;

    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    int incrementIdx = 2; // Start at 0.001

    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasReleased() && incrementIdx <4) {
            incrementIdx++;
        } else if (gamepad1.dpadLeftWasPressed() && incrementIdx >0) {
            incrementIdx--;
        }

        if (gamepad1.a) {
            goalRPM=2700;
        }
        else if (gamepad1.b){
            goalRPM=2700;
        }
        double currentStep = increments[incrementIdx];

        //change kS
        if (gamepad1.dpadUpWasPressed()) {kP += currentStep; }
        if (gamepad1.dpadDownWasPressed()) { kP -=currentStep; }

        double feedforward = (kV*goalRPM) + kS;
        double error=goalRPM- flywheel.getRPM();
        double feedBack=error*kP;

        flywheel.setMotorPower(feedforward+feedBack);

        telemetry.addData("Step", "%.6f", currentStep);
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData( "Error", error);
        telemetry.addData("RPM", flywheel.getRPM());
    }
}
//Just doing this to commit and push