package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.DriveBasePID;
import org.firstinspires.ftc.teamcode.robots.WisdomBot;

@TeleOp(name = "WisdomTeleop", group = "Test")
public class WisdomTeleop extends LinearOpMode {
    AprilTagVision robot = new AprilTagVision(this, false);

    boolean positionLockActive = false;
    boolean tagAlign = false;
    double lockx, locky, lockheading;

    boolean isInput = false;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        robot.restoreSavedPoseIfAvailable();





        telemetry.addLine("Ready - Press Start");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){

            robot.update();
            robot.updatePosition();
            double Dist = robot.getTagDistance();
            double Angle = robot.getTagAngle();
            double Velocity = robot.getVelocity2();

            if (gamepad1.triangleWasPressed()) {
                Pose2D pose = robot.getRobotPosition();
                lockx = pose.getX(DistanceUnit.INCH);
                locky = pose.getY(DistanceUnit.INCH);
                robot.setLockY(locky);
                robot.setLockX(lockx);
                robot.setLockHeading(pose.getHeading(AngleUnit.DEGREES));
                positionLockActive = true;
            }
            if(gamepad1.circleWasPressed()){
                tagAlign = true;
            }

            if(gamepad1.crossWasPressed()){
                positionLockActive = false;
                tagAlign = false;
            }

            if(positionLockActive || !isInput){
                robot.goToPositionNonBlocking(locky, lockx, robot.getHeading(AngleUnit.DEGREES), 0.7);
            }
            else if (tagAlign) {
                robot.alignToTagNonBlocking(60, 0, 0.7);
            }

                double axial = -gamepad1.left_stick_y;   // Forward on left stick yields negative val
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                if(!positionLockActive) {
                    if(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw) != 0){
                        isInput = true;
                        robot.drive(axial, lateral, yaw, 0.92);
                    }
                    else{
                        if(isInput){
                            Pose2D pose = robot.getRobotPosition();
                            lockx = pose.getX(DistanceUnit.INCH);
                            locky = pose.getY(DistanceUnit.INCH);
                            robot.setLockY(locky);
                            robot.setLockX(lockx);
                            robot.setLockHeading(pose.getHeading(AngleUnit.DEGREES));
                        }
                        isInput = false;
                    }


                }

                robot.launch(gamepad1.rightBumperWasPressed());
                robot.launchHigh(gamepad1.leftBumperWasPressed());
                if(gamepad1.right_trigger > 0.5){
                    robot.autoLaunch(1600, 1600);
                }
                if(gamepad1.left_trigger > 0.5){
                    robot.autoLaunch(2050, 2000);
                }
                if(gamepad1.dpad_up){
                    robot.stopLauncher();
                }
                if(gamepad1.startWasPressed()){
                    robot.toggleFC();
                }
                if(gamepad1.cross){
                    positionLockActive = false;
                }
                if(gamepad1.triangle){
                    Pose2D pose = robot.getRobotPosition();
                    lockx = pose.getX(DistanceUnit.INCH);
                    locky = pose.getY(DistanceUnit.INCH);
                    lockheading = pose.getHeading(AngleUnit.DEGREES);
                    positionLockActive = true;
                }

                if(gamepad1.cross){
                    positionLockActive = false;
                }


            telemetry.addData("Lock x :: ", lockx );
            telemetry.addData("Lock y :: ", locky );
            telemetry.addData("April Tag Distance  ::  ", Dist );
            telemetry.addData("April Tag Angle  ::  ", Angle);
            telemetry.addData("Launcher Velocity ::  " , Velocity);


            robot.launch(gamepad1.rightBumperWasPressed());
            telemetry.update();
        }

    }

    public void gamepad1Controls(){

    }



}


//adb connect 192.168.43.1:5555

