package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Four Preload Auto W/ Intake 10/25/2026")
public class FourPreloadAutoShootIntake_10_25_2026 extends LinearOpMode {




    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, intake;
    DcMotorEx launcher;
    CRServo windmillServo, leftIntakeServo, rightIntakeServo;




    static final double TICKS_PER_REV = 537.7;   // goBILDA 312 RPM
    static final double WHEEL_DIAM_IN = 4.09;    // 104mm mecanum
    static final double TICKS_PER_IN = TICKS_PER_REV / (WHEEL_DIAM_IN * Math.PI);




    // TODO: tune these in the sim
    static final double LAUNCH_POWER = 0.9;
    static final long SPIN_UP_MS = 1500;
    static final long FEED_MS = 400;
    static final long RECOVER_MS = 600;


    private void move(double lf, double rf, double lb, double rb, long cruiseMs) {
        // Speed up
        for (double s = 0.1; s <= 1.0; s += 0.1) {
            setDrive(lf * s, rf * s, lb * s, rb * s);
            sleep(50);
        }
        // Cruise
        setDrive(lf, rf, lb, rb);
        sleep(cruiseMs);
        // Slow down
        for (double s = 0.9; s >= 0; s -= 0.1) {
            setDrive(lf * s, rf * s, lb * s, rb * s);
            sleep(50);
        }
        setDrive(0, 0, 0, 0);
    }


    private void setDrive(double lf, double rf, double lb, double rb) {
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }


    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class, "i");
        launcher = hardwareMap.get(DcMotorEx.class, "l");
        windmillServo = hardwareMap.get(CRServo.class, "t");
        leftIntakeServo = hardwareMap.get(CRServo.class, "li");
        rightIntakeServo = hardwareMap.get(CRServo.class, "ri");




        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //RI.setDirection(DcMotorSimple.Direction.REVERSE);




        for (DcMotor m : new DcMotor[]{leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Writes these messages on the driver hub to tell us information
        telemetry.addLine("Ready");
        telemetry.addLine("Launcher Velocity = " + launcher.getVelocity());
        telemetry.update();
        // This is where the code for moving actually starts
        waitForStart();

        move(-0.2, -0.2, -0.2, -0.2, 40);
        launcher.setVelocity(1150); // sets the speed of the launcher
        intake.setPower(1.0);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && launcher.getVelocity() < 1140 && timer.seconds() < 5) { // makes sure that the code only runs when the Launcher velocity is above 1100 AND the timer is less than 5
            idle();
        }
        for (int shot = 0; shot < 4; shot++) { // Function to repeat 4 times
            windmillServo.setPower(-1);
            sleep(700); // Servo feeds balls to shooter
            windmillServo.setPower(1);
            sleep(200); // Servo resets
            windmillServo.setPower(0);
            sleep(500); // Waits
        }
        launcher.setVelocity(0);
        intake.setPower(0);
        move (0.4, -0.4, 0.4, -0.4, 240); // turns to the right to get prepared to intake from the garden
        move (-0.5, 0.5, 0.5, -0.5, 500);
        intake.setPower(1.0);
        leftIntakeServo.setPower(1.0);
        rightIntakeServo.setPower(-1.0);
        move (0.3, 0.3, 0.3, 0.3, 2900); // starts moving forwards while intaking towards the garden
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0); // stops intake motors from moving
        move(-0.5, -0.5, -0.5, -0.5, 30); // moves backwards
        move(0.7, -0.7, -0.7, 0.7, 1800); // strafes right to get back to base
        move(-0.5,0.5,-0.5,0.5,825); // turns to get in line
        move(-0.4, -0.4, -0.4, -0.4, 100); // goes backwards
    }




    void drive(double inches, double power) {
        move(inches, inches, inches, inches, power);
    }




    // positive = right, negative = left
    void strafe(double inches, double power) {
        move(inches, -inches, -inches, inches, power);
    }




    void move(double lf, double rf, double lb, double rb, double power) {
        DcMotor[] ms = {leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive};
        double[] ins = {lf, rf, lb, rb};




        for (int i = 0; i < 4; i++) {
            ms[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ms[i].setTargetPosition((int) (ins[i] * TICKS_PER_IN));
            ms[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ms[i].setPower(power);
        }




        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("lf", leftFrontDrive.getCurrentPosition());
            telemetry.addData("rf", rightFrontDrive.getCurrentPosition());
            telemetry.addData("lb", leftBackDrive.getCurrentPosition());
            telemetry.addData("rb", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }




        for (DcMotor m : ms) m.setPower(0);
        sleep(150);
    }




    void shootAll(int balls) {
        sleep(2000);
    }
}



