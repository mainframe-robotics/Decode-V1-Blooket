package org.firstinspires.ftc.teamcode.Control;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Subsystems {
    private DcMotor intake, turret;
    private CRServo servo1,servo2;
    private Servo kicker;
    private VoltageSensor volts;
//    private Gamepad gamepad1;
    private ShooterController controller ;
    private DcMotorEx shooter;

    double kickerDown = .175;//kicker positions
    double kickerUp = .458                                             ;

    int ballCount = -1;
    public int shootingState = -1;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timerShooter = new ElapsedTime();
    private double lastVelocity = 0;
    private double lastTime =0;
    public static double threshold = 4000;
    public static double increment =100;
    private int veloDrops = 0;
    double slope;


    boolean lastBState = false;
    public Subsystems(HardwareMap hardwareMap,Gamepad gamepad1) {
        intake = hardwareMap.get(DcMotor.class,"int");
//        turret = hardwareMap.dcMotor.get("tur");
        controller=new ShooterController(hardwareMap);
        shooter=hardwareMap.get(DcMotorEx.class,"shooter");

        servo1 = hardwareMap.crservo.get("rs");
        servo2 = hardwareMap.crservo.get("ls");
        kicker = hardwareMap.servo.get("s1");
        volts= hardwareMap.get(VoltageSensor.class,"Control Hub");
//        this.gamepad1 = gamepad1;
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
    }

    public void rollers(double intakeSpeed, double servoSpeed){
        intake.setPower(-intakeSpeed);
        servo1.setPower(-servoSpeed);
        servo2.setPower(servoSpeed);
    }
    public void kicker (boolean up){
        if (up){
            kicker.setPosition(kickerUp);
        }else{
            kicker.setPosition(kickerDown);
        }
    }
    public void shoot(){
        if (shootingState == -1){
            shootingState = 0;
            ballCount = 0;
        }
    }
    public double getRawVelocity(){
        return controller.getShooterVelocityRaw();
    }

    public double getVelocity(){
        return controller.getShooterVelocity();
    }
    public String getErrors(){
        return controller.getErrors();
    }
    public boolean isReadyToShoot(){
        return controller.readyToShoot();
    }

    public double ballCount(){
        return ballCount;
    }

    public void ballShot(){
        ballCount++;
    }






    public void shooterUpdate(){
//        if (gamepad1.bWasPressed()){
//            ballCount++;
//        }
//        lastBState = gamepad1.b;


        switch(shootingState){
            case -1:
                controller.update(false);
                kicker(false);

                break;
            case 0:

                controller.update(true);
                shootingState = 1;
                break;
            case 1: //check for velocity reached
                controller.update(true);
                if (controller.readyToShoot()) {
                    rollers(1,1);
                    shootingState = 2;
                }
                break;
            case 2: //ball detection
                controller.update(true);
                if(controller.getShooterVelocityRaw()/controller.getShooterVelocity()<(2300.0/2700)){
                    ballCount++;
                }
                if (ballCount >= 1){
                    rollers(0,-.2);
                    timer.reset();
                    shootingState = 3;
                }
                break;
            case 3:
                controller.update(true);
                if (controller.readyToShoot()){
                    rollers(1,1);
                    shootingState= 4;
                }

                break;

            case 4:
                controller.update(true);
                if(controller.getShooterVelocityRaw()/controller.getShooterVelocity()<(2300.0/2700)){
                    ballCount++;
                }
                if (ballCount>=2){
                    rollers(0,-.0);
                    timer.reset();
                    shootingState = 5;
                }
                break;
            case 5:
                controller.update(true);
                if (controller.readyToShoot()){
                    rollers(1,1);
                    shootingState= 6;
                }

                break;
            case 6:
                controller.update(true);
                if (controller.readyToShoot()){
                    kicker(true);

                }
                if(controller.getShooterVelocityRaw()/controller.getShooterVelocity()<(2300.0/2700)){
                    ballCount=0;
                    shootingState=7;
                    rollers(1,0);
                    shootingState = -1;
                }
                break;
            case 7:
                controller.update(false);
                if (ballCount==0){
                    rollers(1,0);
                    shootingState=-1;
                }
                break;
        }
    }
}