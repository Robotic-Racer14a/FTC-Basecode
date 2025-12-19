package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveToPose;
import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.commands.drivetrain.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.HashMap;

public class RobotClass {

    public final DriveSubsystem drive;
    public final GamepadEx driverJoystick;
    public final GamepadEx operatorJoystick;
    public final HashMap<String, Object> blackboard;
    public FtcDashboard dashboard;
    public Telemetry telemetry;

    public RobotClass(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            HashMap<String, Object> blackboard,
            GamepadEx driverJoystick,
            GamepadEx operatorJoystick
    ) {

        drive = new DriveSubsystem(
                hardwareMap, telemetry);

        this.telemetry = telemetry;
        this.blackboard = blackboard;

        this.driverJoystick = driverJoystick;
        this.operatorJoystick = operatorJoystick;

        //////////////////////////////// Default Commands ////////////////////////////////////////////////
        drive.setDefaultCommand(new FieldCentricDrive(drive, driverJoystick));

        //////////////////////////////// Driver Controls /////////////////////////////////////////////////
        driverJoystick.getGamepadButton(GamepadKeys.Button.BACK).whileActiveOnce(new InstantCommand(() -> drive.ToggleEnableLimelight()));
        driverJoystick.getGamepadButton(GamepadKeys.Button.START).whenActive(new InstantCommand(drive::zeroHeading));
        driverJoystick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> setAllianceColor("BLUE")));
        driverJoystick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> setAllianceColor("RED")));

        driverJoystick.getGamepadButton(GamepadKeys.Button.A).whileHeld(new DriveToPose(
                drive,
                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)
        ));
        driverJoystick.getGamepadButton(GamepadKeys.Button.X).whileHeld(new FieldCentricDrive(
                drive,
                driverJoystick,
                () -> drive.getAngleToGoal()
        ));

        driverJoystick.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(new RobotCentricDrive(drive, driverJoystick));

        //////////////////////////////// Operator Controls ///////////////////////////////////////////////
        operatorJoystick.getGamepadButton(GamepadKeys.Button.BACK).whileActiveOnce(new InstantCommand(() -> drive.ToggleEnableLimelight()));
        operatorJoystick.getGamepadButton(GamepadKeys.Button.START).whenActive(new InstantCommand(drive::zeroHeading));
        operatorJoystick.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> setAllianceColor("BLUE")));
        operatorJoystick.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whileActiveOnce(new InstantCommand(() -> setAllianceColor("RED")));




        initBlackboardCode();

        dashboard = FtcDashboard.getInstance();
    }

    public void universalInitLoop() {
        drive.updateRobotPoseMT1();
        drive.printCurrentPose();
        drive.updatePinpoint();
    }


    public void initBlackboardCode() {
        try {
            if (blackboard.containsKey("AllianceColor"))
                drive.setAllianceColor((String) blackboard.get("AllianceColor"));
//            if (blackboard.containsKey("Auto End Arm Pos"))
//                arm.setArmPos((double) blackboard.get("Auto End Arm Pos"));
        } catch (NullPointerException nullPointerException) {
            telemetry.addLine("Unable to load position from auto");
        }
    }

    public void setAllianceColor(String color) {
        drive.setAllianceColor(color);
        blackboard.put("AllianceColor", color);
    }

    public void setPinpointPose(double x, double y, double a) {
        drive.setPinpointPose(x, y, a);
    }
}
