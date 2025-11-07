package org.firstinspires.ftc.teamcode.Control;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Shooter {

    // --- PIDF coefficients ---
    public static double kP = 0.005;
    public static double kI = 0.00000001;
    public static double kD = 0.000;
    public static double kF = 0.00029;

    // --- Target RPM range ---
    public static double minRPM = 0;
    public static double maxRPM = 6000;

    // --- Motor configuration ---
    private static final double TICKS_PER_REV = 28.0; // Adjust for your motor encoder CPR

    // --- Controller and state ---
    private final PIDController controller;
    private final DcMotorEx flywheel;
    private boolean usePID = true;

    private double targetRPM = 0;
    private double currentRPM = 0;
    private double lastOutput = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        controller = new PIDController(kP, kI, kD);

        flywheel.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Enables or disables PID control (for manual power testing) */
    public void setPID(boolean enabled) {
        this.usePID = enabled;
    }

    /** Sets the target velocity in RPM */
    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, minRPM, maxRPM);
    }

    /** Gets the current measured velocity in RPM */
    public double getCurrentRPM() {
        // getVelocity() returns ticks/second
        double ticksPerSecond = flywheel.getVelocity();
        currentRPM = ticksPerSecond * 60.0 / TICKS_PER_REV;
        return currentRPM;
    }

    /** Returns the PID output (correction term) */
    public double getPID() {
        controller.setPID(kP, kI, kD);
        double measured = getCurrentRPM();
        double pid = controller.calculate(measured, targetRPM);
        return pid;
    }

    /** Returns the feedforward output based on target RPM */
    public double getFF() {
        return kF * targetRPM;
    }

    /** Combines PID + Feedforward into final motor power */
    public double getPower() {
        double pid = getPID();
        double ff = getFF();
        double power = pid + ff;

        // Clip to [-1, 1] range for safety
        power = Range.clip(power, -1.0, 1.0);
        lastOutput = power;
        return power;
    }

    /** Main update loop — call this every cycle */
    public void update() {
        if (usePID) {
            flywheel.setPower(getPower());
        } else {
            flywheel.setPower(lastOutput);
        }
    }

    /** Set manual power (if PID disabled) */
    public void setManualPower(double power) {
        lastOutput = Range.clip(power, -1.0, 1.0);
    }

    /** Get current target velocity */
    public double getTargetRPM() {
        return targetRPM;
    }

    /** Stop the flywheel */
    public void stop() {
        flywheel.setPower(0);
    }
}
