package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.SystemVariables;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


import java.util.function.Supplier;
@Config
public class DriveToPose extends CommandBase {

    private final DriveSubsystem drive;
    private final Supplier<Pose2D> currentPose;
    private Pose2D targetPose;
    private double x, y;
    private Supplier<Double> a;
    public static double Kp = 0.15, Ki = 0, Kd = 0.05;

    private final PIDController translationPID = new PIDController(
            Kp, Ki, Kd
    );
    private final PIDController rotationPID = new PIDController(
            0.05,0,0
    );

    /**
     * This command will move the robot in a straight line to the given pose
     * @param drive The DriveSubsystem, given to run the robot and make sure nothing else does
     * @param targetPose The end pose of robot
     */
    public DriveToPose(DriveSubsystem drive, Pose2D targetPose) {
        this(drive, targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), () -> targetPose.getHeading(AngleUnit.DEGREES));
    }

    public DriveToPose(DriveSubsystem drive, double x, double y, double a) {
        this(drive, x, y, () -> a);
    }

    public DriveToPose(DriveSubsystem drive, double x, double y, Supplier<Double> a) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.a = a;

        currentPose = drive::getCurrentPose;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        translationPID.setTolerance(1);
        rotationPID.setTolerance(3);
        SystemVariables.utilizingPose = true;
    }

    @Override
    public void execute() {
        //Update Target Pose (used for dynamic angle)
        translationPID.setPID(
                Kp, Ki, Kd
        );
        targetPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, a.get());

        //Figure out the distance away from end pose
        double distanceAwayX = targetPose.getX(DistanceUnit.INCH) - currentPose.get().getX(DistanceUnit.INCH);
        double distanceAwayY = targetPose.getY(DistanceUnit.INCH) - currentPose.get().getY(DistanceUnit.INCH);
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        //Make sure it drives in a straight line
        double translationOutput = translationPID.calculate(distanceAway, 0);
        translationOutput = Math.copySign(Math.min(Math.abs(translationOutput), 1), distanceAway);

        //Set New Rotation so it can cross -180
        double targetAngle = targetPose.getHeading(AngleUnit.DEGREES), currentAngle = currentPose.get().getHeading(AngleUnit.DEGREES);
        if (Math.abs(targetAngle - currentAngle) > 180) {
            double delta = (180 - Math.abs(targetAngle));

            if (currentAngle >= 0) {
                targetAngle = 180 + delta;
            } else {
                targetAngle = -180 - delta;
            }
        }

        //Power needed in each direction
        double rotPow = -rotationPID.calculate(currentAngle, targetAngle);
        double xPow = translationOutput * Math.cos(angleOfDistance);
        double yPow = -translationOutput * Math.sin(angleOfDistance);

        drive.addTelemetry("distanceAway : " + distanceAway);
        drive.addTelemetry("angle: " + (angleOfDistance * (180/3.14)));
        drive.addTelemetry("YPow: " + yPow + "\nXPow: " + xPow + "\nRotPow" + rotPow);

        //Apply power
        drive.driveToPosePower(yPow, xPow, rotPow);
    }

    @Override
    public void end(boolean interrupted) {
        drive.fieldCentricDrive(0, 0, 0);
        SystemVariables.utilizingPose = false;
    }

    @Override
    public boolean isFinished() {
        return translationPID.atSetPoint() && rotationPID.atSetPoint();
    }
}
