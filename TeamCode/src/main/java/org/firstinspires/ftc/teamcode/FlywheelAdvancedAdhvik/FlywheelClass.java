package org.firstinspires.ftc.teamcode.FlywheelAdvancedAdhvik;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelClass {
    private DcMotorEx m1;

    private double enconderCPM=28;

    private double gearRatio =1600;

    private double kV, kS, kP;

    public void init(HardwareMap hwMap) {
        m1=hwMap.get(DcMotorEx.class, "l") ;
        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorPower(double power){
        m1.setPower(power);
    }

    public double getTicksPerSec(){
        return m1.getVelocity();
    }

    public double getRPM(){
        return ((getTicksPerSec()/enconderCPM) * 60) / gearRatio;
    }

}

