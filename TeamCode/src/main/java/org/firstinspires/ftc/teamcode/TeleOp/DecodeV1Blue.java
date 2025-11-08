package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.Driver;
import org.firstinspires.ftc.teamcode.Control.Subsystems;

@TeleOp
public class DecodeV1Blue extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Subsystems subsystem = new Subsystems(hardwareMap,true);
        Driver drive  = new Driver(hardwareMap);
        subsystem.kicker(false);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //dt
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            drive.move(y, x, rx);
            drive.update();
            //intake

//            if (gamepad1.xWasPressed()){
//                subsystem.kicker(true);
//            }else{
//                subsystem.kicker(false);
//            }
            //Shooter
            if (gamepad1.yWasPressed()){
                subsystem.shootingState=-1;
            }
            if (gamepad1.x){
                subsystem.kicker(true);
                subsystem.rollers(-1,-1);
            }
            else if (subsystem.shootingState == -1){

                subsystem.rollers(gamepad1.right_trigger-gamepad1.left_trigger,0.06);

            }
            if (gamepad1.aWasPressed()){
                subsystem.shoot();
            }
            if (subsystem.shootingState!=-1&& gamepad1.bWasPressed()){
                subsystem.ballShot();
            }
            subsystem.shooterUpdate();
            telemetry.addData("shootingState: ",subsystem.shootingState);
            telemetry.addData("errors: ",subsystem.getErrors());
            telemetry.addData("ready to shoot: ",subsystem.isReadyToShoot());
            telemetry.addData("ball count",subsystem.ballCount());
            telemetry.addData("target velo",subsystem.getVelocity());
            telemetry.addData("current velo",subsystem.getRawVelocity());

            telemetry.addLine(String.format("camera raw XYH %6.1f %6.1f %6.1f  (inch)",
                    subsystem.getCamerPoseRaw().getX(DistanceUnit.INCH),
                    subsystem.getCamerPoseRaw().getY(DistanceUnit.INCH),
                    subsystem.getCamerPoseRaw().getHeading(AngleUnit.DEGREES)));
            telemetry.addLine(String.format("camera wrapped XYH %6.1f %6.1f %6.1f  (inch)",
                    subsystem.getCameraPoseWrapped().getX(DistanceUnit.INCH),
                    subsystem.getCameraPoseWrapped().getY(DistanceUnit.INCH),
                    subsystem.getCameraPoseWrapped().getHeading(AngleUnit.DEGREES)));
            telemetry.addLine(String.format("localizer XYH %6.1f %6.1f %6.1f  (inch)",
                    subsystem.getLocalizerPose().getX(DistanceUnit.INCH),
                    subsystem.getLocalizerPose().getY(DistanceUnit.INCH),
                    subsystem.getLocalizerPose().getHeading(AngleUnit.DEGREES)));

            telemetry.update();
//            telemetryM.addData("velocity", subsystem.getVelocity()*(60.0/28));
//            telemetryM.addData("drops", subsystem.getVeloDrops());
//            telemetryM.addData("slope", subsystem.getSlope());
//            graphManager.addData("velo", subsystem.getVelocity()*(60.0/28));
//            telemetryM.update();
//            graphManager.update();
        }
    }
}
