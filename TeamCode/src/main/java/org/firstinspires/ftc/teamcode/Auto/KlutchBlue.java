package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;

@Autonomous(name = "KlutchBlue")
public class KlutchBlue extends LinearOpMode {

    AprilTagVision FarRed = new AprilTagVision(this, false);


    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FarRed.init();


        waitForStart();

        if (opModeIsActive()) {
            //                     y=strafe   x=fwrd/bwrd


            FarRed.goToPosition(16.9, -13.5, 27.3, 0.5, 0.5);

            //Far.alignToTag(55, 0, 1.5, 1, 1.5, 0.5);

            FarRed.autoLaunch(1850, 1900);




        }
    }

}