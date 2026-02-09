package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com. qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires. ftc.robotcore.external. navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation. DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation. Pose2D;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision.RGBColor;

@TeleOp(name = "WisdomTeleopBlue", group = "Test")
public class WisdomTeleopBlue extends LinearOpMode {
    AprilTagVision robot = new AprilTagVision(this, false);

    boolean positionLockActive = false;
    boolean tagAlign = false;
    double lockx = 0.0;
    double locky = 0.0;
    double lockheading = 0.0;



    boolean isInput = false;

    double lastRGBValue = 0.0;
    String lastRGBColor = "OFF";

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();
        robot.restoreSavedPoseIfAvailable();

        telemetry. addLine("Ready - Press Start");
        telemetry.addLine("Square = Test RGB");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            robot.update();
            robot.updatePosition();

            double Dist = robot.getTagDistance();
            double Bearing = robot.getTagBearing();
            double Velocity = robot.getVelocity2();


            if(robot.getTagID() == 20){
                if (Bearing > -10 && Bearing < -3 && Dist > 94 && Dist < 108) {
                    robot.setRGBColor(RGBColor.BLUE); // RED
                    lastRGBColor = "Blue (Tag Left)";
                    lastRGBValue = 0.611;
                } else {
                    robot.setRGBColor(RGBColor.VIOLET);
                    lastRGBColor = "PURPLE (Centered)";
                    lastRGBValue = 0.65;
                }
            }




            if (gamepad1.triangleWasPressed()) {
                Pose2D pose = robot.getRobotPosition();
                lockx = pose.getX(DistanceUnit.INCH);
                locky = pose.getY(DistanceUnit.INCH);
                robot.setLockY(locky);
                robot. setLockX(lockx);
                robot.setLockHeading(pose.getHeading(AngleUnit.DEGREES));
                positionLockActive = true;
                tagAlign = false;
            }

            if(gamepad1.circleWasPressed()){
                tagAlign = true;
                positionLockActive = false;
                robot.resetAllControllers();
            }

            if(gamepad1.crossWasPressed()){
                positionLockActive = false;
                tagAlign = false;
            }

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            if(! positionLockActive && !tagAlign) {
                if(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw) != 0){
                    isInput = true;
                    robot.drive(axial, lateral, yaw, 0.89);
                }
                else{
                    if(isInput){
                        Pose2D pose = robot.getRobotPosition();
                        lockx = pose.getX(DistanceUnit.INCH);
                        locky = pose.getY(DistanceUnit. INCH);
                        robot.setLockY(locky);
                        robot.setLockX(lockx);
                        robot.setLockHeading(pose.getHeading(AngleUnit.DEGREES));
                    }
                    isInput = false;
                }
            }

            if(positionLockActive || !isInput){
                robot. goToPositionNonBlocking(locky, lockx, robot.getHeading(AngleUnit.DEGREES), 0.7);
            }
            else if (tagAlign) {
                robot.alignToTagNonBlocking(97, -3, 0.7);
            }

            robot.launch(gamepad1.rightBumperWasPressed());
            robot.launchHigh(gamepad1.leftBumperWasPressed());

            if(gamepad1.right_trigger > 0.5){
                robot.autoLaunch(1476, 1475);
            }
            if(gamepad1.left_trigger > 0.5){
                robot.autoLaunch(2050, 2000);
            }

            if(gamepad1.startWasPressed()){
                robot.toggleFC();
            }

            // Telemetry
            telemetry. addData("========== RGB TEST ==========", "");
            telemetry.addData("Last RGB Command", lastRGBColor);
            telemetry.addData("Last RGB Value", lastRGBValue);
            telemetry.addData("Square Button", gamepad1.square ?  "PRESSED" : "not pressed");
            telemetry. addData("", "");

            telemetry.addData("========== POSITION ==========", "");
            telemetry.addData("Lock x ::  ", lockx);
            telemetry.addData("Lock y ::  ", locky);
            telemetry.addData("", "");

            telemetry.addData("========== APRIL TAG ==========", "");
            telemetry.addData("April Tag Distance  ::   ", Dist);
            telemetry.addData("April Tag Bearing  ::   ", Bearing);
            telemetry.addData("Tag Align Mode ::   ", tagAlign);
            telemetry.addData("", "");

            telemetry.addData("========== LAUNCHER ==========", "");
            telemetry.addData("Launcher Velocity ::   ", Velocity);
            telemetry.addData("Low Launch State  ::  ", robot.getLaunchState(0));
            telemetry.addData("High Launch State  ::  ", robot.getLaunchState(1));
            telemetry.addData("Position Lock ::   ", positionLockActive);

            telemetry.update();
        }

        robot.setRGBColor(RGBColor.OFF);
        robot.saveCurrentPose();
    }
}