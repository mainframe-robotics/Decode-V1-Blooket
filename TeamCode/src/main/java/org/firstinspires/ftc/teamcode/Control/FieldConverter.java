package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FieldConverter {
    public static Pose2D aprilTagToPinpoint(Pose2D aprilPose){
        double aprilX = aprilPose.getX(DistanceUnit.INCH);
        double aprilY = aprilPose.getY(DistanceUnit.INCH);
        double aprilH = aprilPose.getHeading(AngleUnit.DEGREES);

        aprilH = (aprilH+360)%360;

        double pinX = -aprilX;
        double pinY = -aprilY;
        double pinH = aprilH-90;

        if (pinH > 180) pinH -= 360;
        if (pinH < -180) pinH += 360;

        return new Pose2D(DistanceUnit.INCH,pinX,pinY,AngleUnit.DEGREES,pinH);

    }
    public static Pose2D pinpointToApriltags(Pose2D pinPose){
        double pinX = pinPose.getX(DistanceUnit.INCH);
        double pinY = pinPose.getY(DistanceUnit.INCH);
        double pinH = pinPose.getHeading(AngleUnit.DEGREES);

        pinH = (pinH+360)%360;

        double aprilX = pinY;
        double aprilY = pinX;
        double aprilH = pinH+90;

        if (aprilH > 180) aprilH -= 360;
        if (aprilH < -180) aprilH += 360;

        return new Pose2D(DistanceUnit.INCH,aprilX,aprilY,AngleUnit.DEGREES,aprilH);

    }

}
