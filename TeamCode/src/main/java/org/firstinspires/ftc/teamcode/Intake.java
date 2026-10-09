

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake {


    private final DcMotor intakeMotor;


    // Constructor
    public Intake(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;


        // Set the motor to stop when no power is being applied
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Make sure the intake starts stopped
        intakeMotor.setPower(0);
    }


    // Run the intake forward
    public void intake() {
        intakeMotor.setPower(1.0);
    }


    // Run the intake backward
    public void reverse() {
        intakeMotor.setPower(-1.0);
    }


    // Stop the intake
    public void stop() {
        intakeMotor.setPower(0);
    }
}