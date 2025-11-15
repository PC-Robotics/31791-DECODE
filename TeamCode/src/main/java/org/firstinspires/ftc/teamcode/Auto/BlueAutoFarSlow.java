package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.WisdomBot;

@Autonomous(name = "Blue Auto Far Side Slow")
public class BlueAutoFarSlow extends LinearOpMode {

    AprilTagVision FarBlue = new AprilTagVision(this, false);


    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FarBlue.init();


        waitForStart();

        if (opModeIsActive()) {

            sleep(4000);

            FarBlue.goToPosition(0, 76, -136, 0.6, 0.2);

            //Far.alignToTag(55, 0, 1.5, 1, 1.5, 0.5);

            FarBlue.autoLaunch(1540, 1550);

            FarBlue.goToPosition(0, 3, 0, 0.6, 0.2);

            FarBlue.goToPosition(26,3, 0, 0.5, 0.2);


        }
    }

}