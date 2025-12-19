package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SystemVariables;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.Supplier;


public class FieldCentricDrive extends CommandBase {

    private DriveSubsystem drive;
    private Supplier<Double> forwardController, strafeController, rotationController;
    private PIDController rotationPID = new PIDController(
            0.05,0,0
    );

    public FieldCentricDrive(DriveSubsystem drive, GamepadEx driverJoystick) {
        this.drive = drive;
        this.forwardController = () -> driverJoystick.getLeftY();
        this.strafeController = () -> driverJoystick.getLeftX();
        this.rotationController = () -> driverJoystick.getRightX();
        addRequirements(drive);
    }

    public FieldCentricDrive(DriveSubsystem drive, GamepadEx driverJoystick, Supplier<Double> targetAngle) {

        SystemVariables.utilizingPose = true;

        this.drive = drive;
        this.forwardController = () -> driverJoystick.getLeftY();
        this.strafeController = () -> driverJoystick.getLeftX();
        this.rotationController = () -> -rotationPID.calculate(drive.getCurrentPose().getHeading(AngleUnit.DEGREES), targetAngle.get());
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        SystemVariables.utilizingPose = false;
    }

    @Override
    public void execute() {
        double leftX = forwardController.get();
        double leftY = strafeController.get();
        double rightX = rotationController.get();
        drive.fieldCentricDrive(leftY, leftX, rightX);
    }

    @Override
    public void end(boolean interrupted) {
        drive.fieldCentricDrive(0, 0, 0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double getAnglePower(double targetAngle) {
        //Set New Rotation so it can cross -180
        double currentAngle = drive.getCurrentPose().getHeading(AngleUnit.DEGREES);
        if (Math.abs(targetAngle - currentAngle) > 180) {
            double delta = (180 - Math.abs(targetAngle));

            if (currentAngle >= 0) {
                targetAngle = 180 + delta;
            } else {
                targetAngle = -180 - delta;
            }
        }

        return -rotationPID.calculate(drive.getCurrentPose().getHeading(AngleUnit.DEGREES), targetAngle);
    }
}
