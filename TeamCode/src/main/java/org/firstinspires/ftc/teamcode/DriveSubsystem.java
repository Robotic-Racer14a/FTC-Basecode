package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveSubsystem {
    Telemetry telemetry;
    GoBildaPinpointDriver pinpoint;
    DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    boolean isRobotAtTarget = false;

    public DriveSubsystem(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        rearLeft = hardwareMap.get(DcMotorEx.class, "BL");
        rearRight = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-5.5, -5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    }

    public void driveToPose (double x, double y, double a) {
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, a), currentPose = pinpoint.getPosition();

        //Figure out the distance away from end pose
        double distanceAwayX = targetPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double distanceAwayY = targetPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        //Make sure it drives in a straight line
        double translationOutput = pidCalculate(0.15, 0, 0.05, distanceAway, 0);
        translationOutput = Math.copySign(Math.min(Math.abs(translationOutput), 1), distanceAway);

        //Set New Rotation so it can cross -180
        double targetAngle = targetPose.getHeading(AngleUnit.DEGREES), currentAngle = currentPose.getHeading(AngleUnit.DEGREES);
        if (Math.abs(targetAngle - currentAngle) > 180) {
            double delta = (180 - Math.abs(targetAngle));

            if (currentAngle >= 0) {
                targetAngle = 180 + delta;
            } else {
                targetAngle = -180 - delta;
            }
        }

        //Power needed in each direction
        double rotPow = -pidCalculate(.05, 0, 0, currentAngle, targetAngle);
        double xPow = translationOutput * Math.cos(angleOfDistance);
        double yPow = -translationOutput * Math.sin(angleOfDistance);

        telemetry.addLine("Distance Away : " + distanceAway);
        telemetry.addLine("Angle: " + angleOfDistance * 180 / Math.PI);

        //Apply power
        fieldCentricDrive(xPow, yPow, rotPow);

        if (distanceAway < 1 && Math.abs(currentAngle - targetAngle) < 5) {
            fieldCentricDrive(0, 0, 0);
            isRobotAtTarget = true;
        } else {
            isRobotAtTarget = false;
        }
    }

    public boolean isRobotAtTarget() {
        return isRobotAtTarget;
    }

    private double pidCalculate(double p, double i, double d, double currentPoint, double setpoint) {
        double error = setpoint - currentPoint;
        double power = error * p;

        return power;
    }

    // This routine drives the robot field relative
    public void fieldCentricDrive(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint.getHeading(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        robotCentricDrive(newForward, newRight, rotate);
    }

    public void robotCentricDrive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        rearLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        rearRight.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
