package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Control.Driver;
import org.firstinspires.ftc.teamcode.Control.Localizer;

import java.util.Arrays;

@Config
@Autonomous
public class Tunin extends LinearOpMode {
    private Driver driver;
    private Localizer localizer;
    public static Pose2D target  = new Pose2D(DistanceUnit.INCH, 0 ,0 , AngleUnit.DEGREES,0);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driver = new Driver(hardwareMap);
        driver.setCurrent(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0));
        localizer = new Localizer(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            driver.setCurrent(localizer.getCurrentPose());
            driver.setTarget(target);
            driver.calculate();
            driver.update();
            localizer.update();
            telemetry.addData("Current: ",localizer.getCurrentPose());
            telemetry.addData("poqwers: ", Arrays.toString((driver.getPowers())) ) ;
            telemetry.update();
        }
    }
}
