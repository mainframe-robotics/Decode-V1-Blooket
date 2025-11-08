package org.firstinspires.ftc.teamcode.Control;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Driver {
    DcMotor frontRight, frontLeft, backLeft, backRight;
    PIDController forward, strafe, rotational;
    public static double fKp=.1,fKi,fKd=.01;
    public static double sKp=.1,sKi,sKd=.008;
    public static double rKp=.026,rKi,rKd;
    Localizer localizer ;

    public static double XTarget;
    public static double YTarget;
    public static double YAWTarget;

    private double XCurrent,YCurrent,YAWCurrent;

    double frPow,flPow,blPow,brPow;


    public Driver(HardwareMap hardwareMap){
        localizer = new Localizer(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class,"fl");
        backLeft = hardwareMap.get(DcMotor.class,"bl");
        frontRight = hardwareMap.get(DcMotor.class,"fr");
        backRight = hardwareMap.get(DcMotor.class,"br");
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        forward = new PIDController(fKp,fKi, fKd);
        strafe = new PIDController(sKp,sKi, sKd);
        rotational = new PIDController(rKp,rKi,rKd);
    }

    public void calculate(){
        forward.setPID(fKp,fKi, fKd);
        strafe.setPID(sKp,sKi, sKd);
        rotational.setPID(rKp,rKi,rKd);

        move(
                forward.calculate(XCurrent,XTarget),
                -strafe.calculate(YCurrent,YTarget),
                -rotational.calculate((YAWCurrent>180)?(YAWCurrent-360):(YAWCurrent+360) ,(YAWTarget>180)?(YAWTarget-360):(YAWTarget+360) )
        );
    }

    public void setTarget(Pose2D targetPose){
        XTarget = targetPose.getX(DistanceUnit.INCH);
        YTarget = targetPose.getY(DistanceUnit.INCH);
        YAWTarget = targetPose.getHeading(AngleUnit.DEGREES);
    }
    public void setCurrent(Pose2D currentPose){
        Localizer.update(currentPose);
        update();
    }

    public boolean atPose(){
        return forward.atSetPoint()&&strafe.atSetPoint()&&rotational.atSetPoint();
    }

    public double[] getPowers(){
        return new double[]{forward.calculate(XCurrent, XTarget),
                strafe.calculate(YCurrent, YTarget),
                rotational.calculate(YAWCurrent, YAWTarget)};
    }






    public void move(double y, double x, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        flPow = (y + x + rx) / denominator;
        frPow = (y - x - rx) / denominator;
        blPow = (y - x + rx) / denominator;
        brPow = (y + x - rx) / denominator;

    }
    public void update(){
        XCurrent = Localizer.getCurrentPose().getX(DistanceUnit.INCH);
        YCurrent = Localizer.getCurrentPose().getY(DistanceUnit.INCH);
        YAWCurrent = Localizer.getCurrentPose().getHeading(AngleUnit.DEGREES);

        frontLeft.setPower(flPow);
        backLeft.setPower(blPow);
        frontRight.setPower(frPow);
        backRight.setPower(brPow);
    }

}