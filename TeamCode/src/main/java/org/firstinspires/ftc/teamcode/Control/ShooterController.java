package org.firstinspires.ftc.teamcode.Control;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Config
public class ShooterController {

    public static double goalX=-64.5 , goalY=-66;
    private Shooter shooter;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 2.831, 9, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -65, 180, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    private Turret turret;
    private Localizer localizer;
    double myX = 0;
    double myY = 0;
    double myYaw = 0;
    double myXc = 0;
    double myYc = 0;
    double myYawc = 0;

    double turretAngle =0;
    double shooterVelocity =0;

    double veloMult = 2.73;



    public ShooterController(HardwareMap hardwareMap,boolean isBlue){
        initAprilTag(hardwareMap);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        FtcDashboard.getInstance().startCameraStream(camera, 0);
        goalY = isBlue? goalY:-goalY;
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        localizer = new Localizer(hardwareMap);
    }

    public double getShooterVelocity(){
        return shooterVelocity;
    }
    public double getShooterVelocityRaw(){
        return shooter.getCurrentRPM();
    }

    public double getTurretAngle(){
        return turretAngle;
    }

    public boolean readyToShoot(){
        return Math.abs(turretAngle-turret.getCurrentPosition())<2&&Math.abs(shooterVelocity-shooter.getCurrentRPM())<70;
    }

    public String getErrors(){
        return " turret:"+Math.abs(turretAngle-turret.getCurrentPosition())+" ,shooter: " +Math.abs(shooterVelocity-shooter.getCurrentRPM());
    }


    public void update(boolean shoot){
        if(!aprilTag.getDetections().isEmpty()&&aprilTag.getDetections().get(0)!=null){
            AprilTagDetection aprilTagx  = aprilTag.getDetections().get(0);

            double myX = aprilTagx.robotPose.getPosition().x;
            double myY = aprilTagx.robotPose.getPosition().y;
            double myYaw = aprilTagx.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

             myXc = aprilTagx.robotPose.getPosition().x;
             myYc = aprilTagx.robotPose.getPosition().y;
             myYawc = aprilTagx.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);





            localizer.update(FieldConverter.aprilTagToPinpoint(
                    new Pose2D(DistanceUnit.INCH, -myY , -myX, AngleUnit.DEGREES, myYaw)
            ));
        }
        else{
            localizer.update();
        }

        Pose2D pose = FieldConverter.pinpointToApriltags(localizer.getCurrentPose());

        myX = pose.getX(DistanceUnit.INCH);
        myY = pose.getY(DistanceUnit.INCH);
        myYaw = pose.getHeading(AngleUnit.DEGREES);




//            AprilTagDetection detection = null;
//
//
//            if( !aprilTag.getDetections().isEmpty()) {
//                detection = aprilTag.getDetections().get(0);
//                myX = detection.robotPose.getPosition().x;
//                myY = detection.robotPose.getPosition().y;
//                myZ = detection.robotPose.getPosition().z;
//
//                myPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
//                myRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
//                myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
//            }





        double ballX = myX + Math.sin(Math.toRadians(myYaw))*2.5;
        double ballY = myY + Math.cos(Math.toRadians(myYaw))*2.5;
        double ballZ = 12.8;


        double xDiff = Math.abs(goalX-ballX);
        double yDiff = Math.abs(goalY-ballY);

        double wrappedYaw = (myYaw + 360) % 360;


        double targetAngle = Math.toDegrees(Math.atan2( ballX-goalX,goalY-ballY));
        targetAngle = (targetAngle + 360) % 360;

        turretAngle = wrappedYaw-targetAngle;

        turretAngle = ((turretAngle + 540) % 360) - 180;



        turret.setTargetPosition(shoot?(turretAngle):0);
        turret.update();



        double launchAngle = 65;

        double goalDistance = d2d(goalX,goalY,myX,myY);
        double goalHeight = 43;

        double g = 9.81;
        double theta = Math.toRadians(65);
        double x = goalDistance / 39.37;
        double deltaY = (goalHeight - ballZ) / 39.37;

        double launchVelocity = Math.sqrt(
                (g * x * x) /
                        (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - deltaY))
        );

        double radPerSec = launchVelocity/.048;



        shooterVelocity = ((60.0*radPerSec)/(2.0*Math.PI))*veloMult;



        shooter.setTargetRPM(shoot?(shooterVelocity):0);
        shooter.update();

    }
    public Pose2D getCameraPoseRaw(){
        return new Pose2D(DistanceUnit.INCH,myXc,myYc,AngleUnit.DEGREES,myYawc);
    }

    public Pose2D getCameraPoseWrapped(){
        return new Pose2D(DistanceUnit.INCH,myX,myY,AngleUnit.DEGREES,myYaw);
    }
    public Pose2D getLocalizerPose(){
        return localizer.getCurrentPose();
    }





    private double d2d(double goalX,double goalY,double robotX,double robotY){
        return Math.sqrt(
                Math.pow(goalX-robotX,2) + Math.pow(goalY-robotY,2)
        );
    }

    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(907.111,907.111,661.59,343.096)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 720));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }



}