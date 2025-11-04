package org.firstinspires.ftc.teamcode.Control;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret{
    public static double kP = .12, kI = 0, kD = 0.003;
    public static double kF=0.086;
    public static double lowerBound = -120, upperBound = 100;
    private PIDController controller;
    public static double hangTarget = 0;
    public double hangCurrent = 0;
    private final double ticks_in_degree = 	537.7/360;
    private final double gearing = 24.0/100;



    private DcMotor hang;
    private boolean isPID= true;
    private double hangControl;

    public Turret(HardwareMap hardwareMap) {
        this.hang = hardwareMap.get(DcMotor.class, "tu");
        controller = new PIDController(kP, kI, kD);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setPID(boolean isPID) {
        this.isPID = isPID;
    }
    public void setControl(double x){
        hangControl=x;
    }

    public double getPID(){
        hangCurrent=getCurrentPosition();
        hangTarget=Range.clip(hangTarget, lowerBound, upperBound);
        controller.setPID(kP,kI,kD);

        return controller.calculate(hangCurrent, hangTarget);
    }

    public double getFF(){
        return kF ;
    }


    public void resetEncoder() {
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private double ticksToAngle(double x){
        return x/ticks_in_degree*gearing;
    }
    private double angleToTicks(double x){
        return ticks_in_degree/x;
    }



    public double getPower(double drive) {
        return Math.signum(drive)*Math.abs(drive)+getFF();
    }
    public double getCurrentPosition() {
        return ticksToAngle(hang.getCurrentPosition());
    }

    public double getTargetPosition() {
        return hangTarget;
    }
    public void setTargetPosition(double position) {
        hangTarget= Range.clip(position, lowerBound, upperBound);
    }


    public void update() {
        if(isPID) {
            hang.setPower(getPower(getPID()));
        }
        else {
            hang.setPower(getPower(hangControl));}
        hangCurrent=getCurrentPosition();

    }
}