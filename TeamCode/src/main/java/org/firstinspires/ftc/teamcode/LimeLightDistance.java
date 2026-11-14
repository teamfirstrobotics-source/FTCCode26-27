package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LimeLightDistance extends OpMode {

    private Limelight3A limelight3A;

    private double CAMERA_HEIGHT_CM=35.5;

    private double CAMERA_ANGLE=76;

    private double GOAL_Height =126.12161339;

    private double distance=0;

    @Override
    public void init() {
        limelight3A=hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(9);
    }

    public void start(){
        limelight3A.start();
    }


    @Override
    public void loop(){
        LLResult llResult=limelight3A.getLatestResult();

        if (llResult != null&& llResult.isValid()){
            distance=getDistance(llResult.getTy());
        telemetry.addData("Distance",distance);
        }
        else {
            telemetry.addData("No Valid Target", "Found");
        }
    }

    public double getDistance(double ty){
        double angleToTarget=CAMERA_ANGLE + ty;
        double heightDifference = GOAL_Height-CAMERA_HEIGHT_CM;

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }
}
