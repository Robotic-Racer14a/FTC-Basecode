package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
public class BlueClose extends TurtleOpMode {

    int step = 0;

    @Override
    public void init() {
       drive.seedPose( -60, -38, 46);
    }

    @Override
    public void loop() {

        telemetry.addLine("Step: " + step);

        switch (step) {


            case 0:
                drive.driveToPose(0, 0, 0);
                if (drive.isRobotAtTarget()) {
                    step = 10;
                }
                break;


        }



    }

}
