package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SystemVariables;

import static org.firstinspires.ftc.teamcode.SystemVariables.PosesOnField;
import static org.firstinspires.ftc.teamcode.SystemVariables.DrivetrainConstants;

public class DriveSubsystem extends SubsystemBase {

    private Telemetry telemetry;

    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftRear;
    private final Motor rightRear;

    private final RevIMU imu;

    private MecanumDrive drive;

    private GoBildaPinpointDriver pinpoint;

    private Limelight3A limelight;

    public boolean useMT1 = true, limelightCreated = true;

    /**
     * Creates the drivetrain code
     * @param hardwareMap The active configuration
     * @param telemetry Allows drivetrain to put values on telemetry
     */
    public DriveSubsystem(
            HardwareMap hardwareMap,
            Telemetry telemetry
    ) {

        this.telemetry = telemetry;
        this.leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        this.rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        this.leftRear = new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312);
        this.rightRear = new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312);
        this.imu = new RevIMU(hardwareMap);

        drive= new MecanumDrive(false,
                leftFront, rightFront, leftRear, rightRear
        );

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-39.6875, -26.19375, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        leftFront.setInverted(true);
        leftRear.setInverted(true);
        rightFront.setInverted(false);
        rightRear.setInverted(false);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // This tells Limelight to start looking!
            limelightCreated = true;
        } catch (NullPointerException nullPointerException) {
            limelightCreated = false;
        }
        imu.init();
    }

    @Override
    public void periodic() {
        updatePinpoint();
        //pinpoint.update();

        if (useMT1 && !SystemVariables.utilizingPose) updateRobotPoseMT1();
        //else if (!SystemVariables.utilizingPose) updateRobotPoseMT2();

        printCurrentPose();
    }

    public Pose2D getCurrentPose() {
        return pinpoint.getPosition();
    }

    public void addTelemetry(String string) {
        telemetry.addLine(string);
        //telemetry.update();
    }

    ///////////////////////////// Driving Methods //////////////////////////////////////

    public void fieldCentricDrive(double strafePow, double forwardPow, double rotationPow) {
        final double orientationOffset;
        if (SystemVariables.allianceColor == "BLUE") {
            orientationOffset = 90;
        } else {
            orientationOffset = -90;
        }

        drive.driveFieldCentric(
                strafePow,
                forwardPow,
                rotationPow,
                getCurrentPose().getHeading(AngleUnit.DEGREES) + orientationOffset
        );
    }

    public void driveToPosePower(double strafePow, double forwardPow, double rotationPow) {
        drive.driveFieldCentric(
                strafePow,
                forwardPow,
                rotationPow,
                getCurrentPose().getHeading(AngleUnit.DEGREES)
        );
    }

    public void robotCentricDrive(double leftX, double leftY, double rightX) {
        drive.driveRobotCentric(
                leftX,
                leftY,
                rightX
        );
    }

    //////////////////////////// Robot Heading Methods ///////////////////////////////////

    public void setAllianceColor(String color) {
        SystemVariables.allianceColor = color;
    }

    public void setUseMT1(boolean useMT1) {
        this.useMT1 = useMT1;
    }
    public void ToggleEnableLimelight() {
        if(useMT1){
            useMT1=false;
        }
        else{
            useMT1=true;
        }


    }
    public double getAngleToGoal() {
        SystemVariables.utilizingPose = true;
        double deltaX = -getCurrentPose().getX(DistanceUnit.INCH), deltaY = -getCurrentPose().getY(DistanceUnit.INCH);
        if (SystemVariables.allianceColor == "BLUE") {
            deltaY += PosesOnField.blueGoalY;
            deltaX += PosesOnField.blueGoalX;
        } else {
            deltaY += PosesOnField.blueGoalY;
            deltaX += PosesOnField.blueGoalX;
        }
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromRadians(Math.atan2(deltaY, deltaX)));
    }

    public void updateRobotPoseMT1() {
        LLResult result;
        if (limelightCreated) {
            result = limelight.getLatestResult();
        } else {
            result = null;
            telemetry.addLine("NOT BUILT");
        }

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double a = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ", " + a + ")");

                x = correctedVisionOutput(getCurrentPose().getX(DistanceUnit.METER), x, DrivetrainConstants.megaTag1Trust, DistanceUnit.METER.fromInches(12));
                y = correctedVisionOutput(getCurrentPose().getY(DistanceUnit.METER), y, DrivetrainConstants.megaTag1Trust, DistanceUnit.METER.fromInches(12));
                a = correctedVisionOutput(getCurrentPose().getHeading(AngleUnit.DEGREES), a, DrivetrainConstants.megaTag1Trust, AngleUnit.DEGREES.fromDegrees(20));

                pinpoint.setPosition(new Pose2D(DistanceUnit.METER, x, y, AngleUnit.DEGREES, a));
            }
        }
    }

    public void updateRobotPoseMT2() {
        LLResult result;
        if (limelightCreated) {
            limelight.updateRobotOrientation(getCurrentPose().getHeading(AngleUnit.DEGREES));
            result = limelight.getLatestResult();
        } else {
            result = null;
        }

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");

                x = correctedVisionOutput(getCurrentPose().getX(DistanceUnit.METER), x, DrivetrainConstants.megaTag2Trust, DistanceUnit.METER.fromInches(12));
                y = correctedVisionOutput(getCurrentPose().getY(DistanceUnit.METER), y, DrivetrainConstants.megaTag2Trust, DistanceUnit.METER.fromInches(12));

                pinpoint.setPosition(new Pose2D(DistanceUnit.METER, x, y, AngleUnit.DEGREES, getCurrentPose().getHeading(AngleUnit.DEGREES)));
            }
        }
    }

    public void zeroHeading() {
        pinpoint.setHeading(0, AngleUnit.DEGREES);
    }

    public void updatePinpoint() {
        pinpoint.update();
    }

    public void setPinpointPose(double x, double y, double a) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, a));
    }

    public void printCurrentPose() {
        telemetry.addLine("Current Pose:" + getCurrentPose().getX(DistanceUnit.INCH) + ", " +  getCurrentPose().getY(DistanceUnit.INCH) + ", " +  getCurrentPose().getHeading(AngleUnit.DEGREES));
    }




    /**
     * This method is used to prevent oscillation of pose during vision updates
     * @param prevAverage The total average of dataset, for vision use current position
     * @param input The additional input into dataset, for vision this is the vision result
     * @param trustFactor Amount previous position is trusted more, by a factor
     * @param radiusOfOscillation Range for expected oscillation, if input is outside of range, bypasses correction
     * @return The average of the dataset
     */
    public double correctedVisionOutput(double prevAverage, double input, int trustFactor, double radiusOfOscillation) {
        double average = 0;
        average = (prevAverage * trustFactor) + input;
        average /= trustFactor + 1;
        if (Math.abs(average - input) > radiusOfOscillation) return input;
        return average;
    }


}
