package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.Supplier;


public class RobotCentricDrive extends CommandBase {

    private DriveSubsystem drive;
    private Supplier<Double> forwardController, strafeController, rotationController;
    public RobotCentricDrive(DriveSubsystem drive, GamepadEx driverJoystick) {
        this.drive = drive;
        this.forwardController = () -> driverJoystick.getLeftY();
        this.strafeController = () -> driverJoystick.getLeftX();
        this.rotationController = () -> driverJoystick.getRightX();
        addRequirements(drive);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double leftY = forwardController.get();
        double leftX = strafeController.get();
        double rightX = rotationController.get();
        drive.robotCentricDrive(leftX, leftY, rightX);
    }

    @Override
    public void end(boolean interrupted) {
        drive.fieldCentricDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
