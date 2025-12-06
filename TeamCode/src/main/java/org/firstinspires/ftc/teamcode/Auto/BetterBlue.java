package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.WisdomBot;

@Autonomous(name = "Blue Auton Side")
public class BetterBlue extends LinearOpMode {

    AprilTagVision Blue = new AprilTagVision(this, false);


    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Blue.init();


        waitForStart();

        if (opModeIsActive()) {

            Blue.goToPosition(0, 52, 0, 0.6, 0.5);

            //Far.alignToTag(55, 0, 1.5, 1, 1.5, 0.5);

            Blue.autoLaunch(1400, 1390);

            Blue.goToPosition(16, 30, 0, 0.6, 0.5);


        }
    }

}