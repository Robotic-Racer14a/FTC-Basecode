package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveToPose;

@Autonomous
public class BasicAuto extends OpMode {
    //private BasicSubsystem subsystem;
    private RobotClass robot;

    @Override
    public void init() {

        robot = new RobotClass(hardwareMap, telemetry, blackboard, new GamepadEx(gamepad1), new GamepadEx(gamepad2));

        robot.setAllianceColor("BLUE");

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(new WaitCommand(0),
                    new DriveToPose(robot.drive, new Pose2D(DistanceUnit.INCH, -9,  0, AngleUnit.DEGREES, -90)),
                    new WaitCommand(2000).withTimeout(1000),
                    new DriveToPose(robot.drive, new Pose2D(DistanceUnit.INCH, -9, -32, AngleUnit.DEGREES, -90))
                )
        );

    }

    @Override
    public void init_loop() {
        robot.universalInitLoop();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        TelemetryPacket packet = new TelemetryPacket();
        //packet.fieldOverlay().setStrokeWidth(1);
        //packet.fieldOverlay().strokeCircle(0,0,9);
        packet.fieldOverlay().drawImage("/images/ftc.jpg", 24, 24, 48, 48,Math.toRadians(45), 24, 24, false);
        robot.dashboard.sendTelemetryPacket(packet);
    }
}
