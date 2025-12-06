package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.WisdomBot;

@Autonomous(name = "Red Auton Side")
public class BetterRed extends LinearOpMode {

    AprilTagVision Red = new AprilTagVision(this, false);


    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Red.init();


        waitForStart();

        if (opModeIsActive()) {

            Red.goToPosition(0, 52, 0, 0.6, 0.5);

            //Far.alignToTag(55, 0, 1.5, 1, 1.5, 0.5);

            Red.autoLaunch(1400, 1390);

            Red.goToPosition(16, 52, 0, 0.6, 0.5);


        }
    }

}