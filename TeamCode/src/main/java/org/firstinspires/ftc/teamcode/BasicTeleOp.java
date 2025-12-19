package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp
public class BasicTeleOp extends OpMode {
    //private BasicSubsystem subsystem;
    private RobotClass robot;

    @Override
    public void init() {
        robot = new RobotClass(hardwareMap, telemetry, blackboard, new GamepadEx(gamepad1), new GamepadEx(gamepad2));
    }

    @Override
    public void init_loop() {
        robot.universalInitLoop();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetry.addLine(SystemVariables.allianceColor);
        telemetry.addData("Limelight Enabled:", String.valueOf(robot.drive.useMT1));
        TelemetryPacket packet = new TelemetryPacket();
        //packet.fieldOverlay().setStrokeWidth(1);
        //packet.fieldOverlay().strokeCircle(0,0,9);
        packet.fieldOverlay().drawImage("/images/ftc.jpg", robot.drive.getCurrentPose().getX(DistanceUnit.INCH), robot.drive.getCurrentPose().getY(DistanceUnit.INCH), 18, 18,robot.drive.getCurrentPose().getHeading(AngleUnit.RADIANS), 24, 24, false);
        robot.dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        return;
    }
}
