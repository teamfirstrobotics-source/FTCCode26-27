package org.firstinspires.ftc.FlywheelAdvancedAdhvik;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class FlywheelClass {
    private DcMotorEx m1;

    private double enconderCPM=28;

    private double gearRatio =1;

    private double kV=.000367, kS=.0428, kP=.00013;

    public void init(HardwareMap hwMap) {
        m1=hwMap.get(DcMotorEx.class, "l") ;
        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorPower(double power){
        m1.setPower(power);
    }

    public void stopMotors(){
        m1.setPower(0);
    }

    public void setMotorRPM(double targetRpm){;
        //call this every loop
        double error= targetRpm-getRPM();
        double ff= (kV*targetRpm)+kS;
        double fb= error*kP;
        double power= ff+fb;

        setMotorPower(power);
    }
    public double getTicksPerSec(){
        return m1.getVelocity();
    }

    public double getRPM(){
        return ((getTicksPerSec()/enconderCPM) * 60) / gearRatio;
    }

}
//Goal RPM is 1200