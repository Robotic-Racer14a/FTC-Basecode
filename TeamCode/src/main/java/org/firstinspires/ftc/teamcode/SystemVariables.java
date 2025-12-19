package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SystemVariables {


    public static double drivetrainDistanceToGoal = 0;
    public static double drivetrainAngleToGoal = 0;
    public static String allianceColor = "BLUE";
    public static boolean utilizingPose = false;


    public static final class DeliveryConstants {
        public static final double deliverySpeed = 1;
    }


    public static final class DrivetrainConstants {
        public static final int megaTag1Trust = 50;
        public static final int megaTag2Trust = 50;
    }


    public static final class IntakeConstants {
        public static final double intakeSpeed = 1;
        public static final double slowIntakeSpeed = 0.3;
    }


    public static final class PosesOnField {
        public static final double humanStationX = 50, blueHumanStationY = -50, redHumanStationY = 50, blueHumanStationAngle = 90, redHumanStationAngle = -90;
        public static final double closeShotX = -12, blueCloseShotY = -12, redCloseShotY = 12;
        public static final double farShotX = 60, blueFarShotY = -6, redFarShotY = 6;
        public static final double redGoalX = -68, blueGoalX = -68, redGoalY = 68, blueGoalY = -68;

    }
}
