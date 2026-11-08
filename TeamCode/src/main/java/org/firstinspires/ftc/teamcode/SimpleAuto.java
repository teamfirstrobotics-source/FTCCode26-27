package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Simple Auto")
public class SimpleAuto extends LinearOpMode {

    // The battery voltage you tuned everything at (flywheel kV/kS AND the drive timings).
    // Read it off the "Battery V" telemetry with a fresh battery and put it here.
    static final double NOMINAL_VOLTAGE = 12.5;

    // Timed drive moves also change with battery voltage, so scale them too.
    // Set to false if you want the drive to behave exactly like before.
    static final boolean DRIVE_VOLTAGE_COMPENSATION = true;

    // ------------------------------------------------------------------------------------
    // Flywheel controller (feedforward + PID + battery voltage compensation)
    // All velocities are in ENCODER TICKS PER SECOND.
    // ------------------------------------------------------------------------------------
    static class Flywheel {
        private final DcMotorEx motor;
        private final VoltageSensor voltageSensor;

        // ---- Tuning values (units: ticks/sec) ----\\\\\\\\\\\\\\\\\\\\\\\\\\\
        public double kV = 0.00039;  // power per (tick/sec)
        public double kS = 0.043;    // power needed just to overcome friction
        public double kP = 0.00013;   // power per (tick/sec) of error
        public double kI = 0.0;
        public double kD = 0.0;

        public double maxIntegralPower = 0.2;

        public double nominalVoltage = NOMINAL_VOLTAGE;

        public double ticksPerRev = 28;
        public double gearRatio = 1;

        private double targetVelocity = 0;
        private double integral = 0;
        private double lastError = 0;
        private double voltage;
        private final ElapsedTime timer = new ElapsedTime();
        private final ElapsedTime voltageTimer = new ElapsedTime();
        private boolean wasRunning = false;

        Flywheel(DcMotorEx motor, VoltageSensor voltageSensor) {
            this.motor = motor;
            this.voltageSensor = voltageSensor;
            this.voltage = voltageSensor.getVoltage();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void setTargetVelocity(double ticksPerSec) {
            targetVelocity = ticksPerSec;
        }

        public double getTargetVelocity() {
            return targetVelocity;
        }

        public double getVelocity() {
            return motor.getVelocity();
        }

        public double getRPM() {
            return (getVelocity() / ticksPerRev) * 60.0 / gearRatio;
        }

        public double getVoltage() {
            return voltage;
        }

        /**
         * CALL THIS EVERY LOOP.
         */
        public void update() {
            if (targetVelocity <= 0) {
                motor.setPower(0);
                integral = 0;
                lastError = 0;
                wasRunning = false;
                return;
            }

            // Re-read battery voltage 4x per second (reading every loop is slow)
            if (voltageTimer.milliseconds() > 250) {
                voltage = voltageSensor.getVoltage();
                voltageTimer.reset();
            }
            double voltageScale = nominalVoltage / Math.max(voltage, 1.0);

            double dt = timer.seconds();
            timer.reset();
            if (!wasRunning) {
                integral = 0;
                lastError = targetVelocity - getVelocity();
                dt = 0;
                wasRunning = true;
            }

            double error = targetVelocity - getVelocity();

            double derivative = 0;
            if (dt > 0) {
                integral += error * dt;
                derivative = (error - lastError) / dt;
            }
            lastError = error;

            if (kI != 0) {
                double maxIntegral = maxIntegralPower / kI;
                integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
            }

            // Feedforward is scaled by battery voltage so a weak battery gets more power
            double feedforward = ((kV * targetVelocity) + kS) * voltageScale;
            double feedback = (kP * error) + (kI * integral) + (kD * derivative);

            double power = feedforward + feedback;
            power = Math.max(0, Math.min(1, power));
            motor.setPower(power);
        }
    }

    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, intake;
    DcMotorEx launcher;
    CRServo windmillServo, leftIntakeServo, rightIntakeServo;
    Flywheel flywheel;
    VoltageSensor battery;

    // Cached battery reading for the drive scaling
    private double driveVoltage = NOMINAL_VOLTAGE;
    private final ElapsedTime driveVoltageTimer = new ElapsedTime();

    static final double TICKS_PER_REV = 537.7;   // goBILDA 312 RPM
    static final double WHEEL_DIAM_IN = 4.09;    // 104mm mecanum
    static final double TICKS_PER_IN = TICKS_PER_REV / (WHEEL_DIAM_IN * Math.PI);

    // Launcher velocities in ticks/sec
    static final double LAUNCHER_TARGET_VELOCITY = 1150;
    static final double LAUNCHER_MIN_VELOCITY = 1140;  // must reach this before feeding

    /**
     * Waits for ms milliseconds while KEEPING THE PID RUNNING.
     * Use this instead of sleep() any time the flywheel is spinning,
     * because sleep() stops flywheel.update() from being called.
     */
    private void waitWithFlywheel(long ms) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) {
            flywheel.update();
        }
    }

    /** nominal / current battery voltage, re-read 4x per second. 1.0 if compensation is off. */
    private double driveScale() {
        if (!DRIVE_VOLTAGE_COMPENSATION) return 1.0;
        if (driveVoltageTimer.milliseconds() > 250) {
            driveVoltage = battery.getVoltage();
            driveVoltageTimer.reset();
        }
        return NOMINAL_VOLTAGE / Math.max(driveVoltage, 1.0);
    }

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
        double k = driveScale();
        leftFrontDrive.setPower(clip(lf * k));
        rightFrontDrive.setPower(clip(rf * k));
        leftBackDrive.setPower(clip(lb * k));
        rightBackDrive.setPower(clip(rb * k));
    }

    private double clip(double p) {
        return Math.max(-1.0, Math.min(1.0, p));
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

        battery = hardwareMap.voltageSensor.iterator().next();
        driveVoltage = battery.getVoltage();

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Flywheel sets its own run mode (RUN_WITHOUT_ENCODER) and zero-power behavior.
        flywheel = new Flywheel(launcher, battery);

        telemetry.addLine("Ready");
        telemetry.addData("Battery V", "%.2f (nominal %.2f)", battery.getVoltage(), NOMINAL_VOLTAGE);
        telemetry.addData("Launcher velocity", flywheel.getVelocity());
        telemetry.update();

        waitForStart();

        move(0.5, 0.5, 0.5, 0.5, 100);
        move(0.5, -0.5, -0.5, 0.5, 50);
    }
}