package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Control.Driver;
import org.firstinspires.ftc.teamcode.Control.Localizer;
import org.firstinspires.ftc.teamcode.Control.Subsystems;

import java.util.Arrays;

@Config
@Autonomous
public class ThriceBallRed extends LinearOpMode {

    private Pose2D start = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0);
    private Pose2D shootingPose = new Pose2D(DistanceUnit.INCH,-50,0, AngleUnit.DEGREES,0);
    private Pose2D end = new Pose2D(DistanceUnit.INCH,-50,10, AngleUnit.DEGREES,0);

    private Driver driver;
    private Subsystems subsystems;


    private int state=-1;
    private ElapsedTime timer;


    @Override
    public void runOpMode(){
        driver = new Driver(hardwareMap);
        subsystems = new Subsystems(hardwareMap,false);

        timer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()){
//            driver.setCurrent(Localizer.getCurrentPose());
            switch (state){
                case -1:
                    timer.reset();
                    state=0;
                case 0:
                    driver.move(-.4,0,0);
                    if (timer.milliseconds()>1000){
                        driver.move(0,0,0);
                        timer.reset();
                        state=1;
                    }
                    break;
                case 1:
                    subsystems.shoot();
                    state=2;
                case 2:
                    if(subsystems.shootingState == -1||timer.milliseconds()>9000){
                        timer.reset();
                        state=3;
                    }
                    break;
                case 3:
                    driver.move(0,.4,0);
                    if (timer.milliseconds()>2000){
                       driver.move(0,0,0);
                        state=4;
                    }
                    break;
            }

            subsystems.shooterUpdate();
//            driver.calculate();
            driver.update();
        }
        telemetry.addData("Current: ",Localizer.getCurrentPose());
        telemetry.addData("poqwers: ", Arrays.toString((driver.getPowers())) ) ;
        telemetry.update();
    }

}
