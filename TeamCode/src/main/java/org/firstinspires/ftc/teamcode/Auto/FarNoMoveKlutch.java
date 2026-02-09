package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision.RGBColor;

@Autonomous(name = "Delayed Far Red")
public class FarNoMoveKlutch extends LinearOpMode {

    AprilTagVision Blue = new AprilTagVision(this, false);

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Blue.init();
        while (!opModeIsActive()) {
            double Dist = Blue.getTagDistance();
            double Bearing = Blue.getTagBearing();
            double Velocity = Blue.getVelocity2();

            telemetry.addData("========== APRIL TAG ==========", "");
            telemetry.addData("April Tag Distance  ::   ", Dist);
            telemetry.addData("April Tag Bearing  ::   ", Bearing);

            telemetry.addData("", "");
            telemetry.update();
            Blue.update();



            if(Blue.getTagID() == 24) {
                if (Bearing > -3 && Bearing < 1 && Dist > 94 && Dist < 108) {
                    Blue.setRGBColor(RGBColor.RED); // RED

                } else {
                    Blue.setRGBColor(RGBColor.VIOLET);

                }
            }
            if(Blue.getTagID() == 20){
                if (Bearing > -7 && Bearing < -3 && Dist > 94 && Dist < 108) {
                    Blue.setRGBColor(RGBColor.BLUE); // RED

                } else {
                    Blue.setRGBColor(RGBColor.VIOLET);

                }
            }


            if (opModeIsActive()) {
                Thread.sleep(4000);
                Blue.goToPosition(0, -7, 0, 0.4, 0.2);
                Blue.goToPosition(0,-3,0,0.4, 0.2);
                Thread.sleep(1000);
                Blue.autoLaunch(1790,1780);
                Blue.goToPosition(30, -15., 0, 0.4, 1);
                /*
                if(Blue.getTagID() == 24){
                    Blue.autoLaunch(1810, 1800);
                    Blue.goToPosition(30, -15, 0, 0.5, 0.2);
                }
                if(Blue.getTagID() == 20){
                    Blue.autoLaunch(1810, 1800);
                    Blue.goToPosition(-30, -15, 0, 0.5, 0.2);
                }

                 */




            }

        }


    }
}
