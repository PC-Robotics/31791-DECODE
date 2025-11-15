package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;

@Autonomous(name = "Red Auto Far Side")
public class RedAutoFar extends LinearOpMode {

    AprilTagVision FarRed = new AprilTagVision(this, false);


    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FarRed.init();


        waitForStart();

        if (opModeIsActive()) {

            FarRed.goToPosition(0, 76, 134, 0.6, 0.5);

            //Far.alignToTag(55, 0, 1.5, 1, 1.5, 0.5);

            FarRed.autoLaunch(1540, 1550);

            FarRed.goToPosition(0, 3, 0, 0.6, 0.5);

            FarRed.goToPosition(-28,3, 0, 0.5, 0.5);


        }
    }

}