package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Control.*;

@TeleOp
public class DecodeV1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Subsystems subsystem = new Subsystems(hardwareMap,gamepad1);
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
            //intake
            if (subsystem.shootingState == -1){

                    subsystem.rollers(gamepad1.right_trigger-gamepad1.left_trigger,0.065);

            }
//            if (gamepad1.xWasPressed()){
//                subsystem.kicker(true);
//            }else{
//                subsystem.kicker(false);
//            }
            //Shooter
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
