package org.firstinspires.ftc.teamcode.Control;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

@TeleOp
public class ShooterController extends LinearOpMode {
    private DcMotorEx shooter;
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
    double myZ = 0;

    double myPitch = 0;
    double myRoll = 0;
    double myYaw = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initAprilTag();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        FtcDashboard.getInstance().startCameraStream(camera, 0);

        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        turret = new Turret(hardwareMap);
        localizer = new Localizer(hardwareMap);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }


            if(!aprilTag.getDetections().isEmpty()&&aprilTag.getDetections().get(0)!=null){
                AprilTagDetection aprilTagx  = aprilTag.getDetections().get(0);

                double myX = aprilTagx.robotPose.getPosition().x;
                double myY = aprilTagx.robotPose.getPosition().y;
                double myYaw = aprilTagx.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);





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

            double goalX = -64;
            double goalY = -66;

            double xDiff = Math.abs(goalX-ballX);
            double yDiff = Math.abs(goalY-ballY);

            double wrappedYaw = (myYaw + 360) % 360;


            double targetAngle = Math.toDegrees(Math.atan2( ballX-goalX,goalY-ballY));
            targetAngle = (targetAngle + 360) % 360;

            double turretAngle = targetAngle-wrappedYaw;

            turretAngle = ((turretAngle + 540) % 360) - 180;



            turret.setTargetPosition(-turretAngle);
            turret.update();



            double launchAngle = 65;

            double goalDistance = d2d(-64,-66,myX,myY);
            double goalHeight = 40;

            double g = 9.81;
            double theta = Math.toRadians(65);
            double x = goalDistance / 39.37;
            double deltaY = (goalHeight - ballZ) / 39.37;

            double launchVelocity = Math.sqrt(
                    (g * x * x) /
                            (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - deltaY))
            );

            double radPerSec = launchVelocity/.048;



            double motorRPM = (60.0*radPerSec)/(2.0*Math.PI);

            shooter.setVelocity(motorRPM*(28.0/60.0)*2.75);



            telemetryAprilTag();

//             Push telemetry to the Driver Station.
            telemetry.addLine("camera XYH  (inch)"+
                    localizer.getCurrentPose().toString());

            telemetry.addLine(String.format("localizer XYH %6.1f %6.1f %6.1f  (inch)",
                    myX,
                    myY,
                    myYaw));

            double radi = 8;
            packet.fieldOverlay().strokeCircle(myX,myY,radi).setStroke("red");
            packet.fieldOverlay().strokeLine(myX*Math.sin(Math.toRadians(myYaw)),myY*Math.cos(Math.toRadians(myYaw)),(myX-2)*Math.sin(Math.toRadians(myYaw)),(myY-2)*Math.cos(Math.toRadians(myYaw)));


            telemetry.addData("distance from goal: ",goalDistance);
            telemetry.addData("launch velocity: ",launchVelocity);
            telemetry.addData("rad per sec: ",radPerSec);
            telemetry.addData("motor rpm: ",motorRPM);
            telemetry.addData("ball z: ",ballZ);
            telemetry.addData("target angle: ",targetAngle);
            telemetry.addData("turret angle: ",turretAngle);
            telemetry.addData("wrapped yaw: ",wrappedYaw);
//            telemetry.addData("ball z: ",ballZ);
//            telemetry.addData("ball z: ",ballZ);
            telemetry.update();
//            sleep(20);

        }

        visionPortal.close();

    }
    private double d2d(double goalX,double goalY,double robotX,double robotY){
        return Math.sqrt(
                Math.pow(goalX-robotX,2) + Math.pow(goalY-robotY,2)
        );
    }

    private void initAprilTag() {

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

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()


}