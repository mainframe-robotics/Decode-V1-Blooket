package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Driver {
    DcMotor frontRight, frontLeft, backLeft, backRight;
    public Driver(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotor.class,"fl");
        backLeft = hardwareMap.get(DcMotor.class,"bl");
        frontRight = hardwareMap.get(DcMotor.class,"fr");
        backRight = hardwareMap.get(DcMotor.class,"br");
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void move(double y, double x, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }
}