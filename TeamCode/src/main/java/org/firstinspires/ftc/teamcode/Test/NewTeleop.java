package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision.RGBColor;

@TeleOp(name="NewTeleop", group="Test")
public class NewTeleop extends LinearOpMode {

    AprilTagVision robot = new AprilTagVision(this,false);

    private enum DriveState {
        TAG_ALIGN,
        POSITION_LOCK,
        DRIVE
    }

    private DriveState driveState;

    double lockx=0;
    double locky=0;

    double kp=0.0240;
    double kd=0.0008;
    double goalX=-4;
    double angleTolerance=0.4;

    double error=0;
    double lastError=0;
    double currTimer=0;
    double lastTime=0;

    double lastRGBValue=0;
    String lastRGBColor="OFF";

    @Override
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry,
                FtcDashboard.getInstance().getTelemetry());

        robot.init();
        robot.restoreSavedPoseIfAvailable();

        driveState = DriveState.POSITION_LOCK;

        telemetry.addLine("Ready - Press Start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            robot.update();
            robot.updatePosition();

            double axial=-gamepad1.left_stick_y;
            double lateral=gamepad1.left_stick_x;
            double yaw=gamepad1.right_stick_x;

            double Dist=robot.getTagDistance();
            double Bearing=robot.getTagBearing();
            double Velocity=robot.getVelocity2();

            // ================= RGB =================
            if(robot.getTagID()==20){
                if(Bearing>-10 && Bearing<-3 && Dist>94 && Dist<108){
                    robot.setRGBColor(RGBColor.BLUE);
                    lastRGBColor="Blue (Tag Left)";
                    lastRGBValue=0.611;
                }else{
                    robot.setRGBColor(RGBColor.VIOLET);
                    lastRGBColor="PURPLE (Centered)";
                    lastRGBValue=0.65;
                }
            }


            boolean sticksActive =
                    Math.abs(axial)>0.01 ||
                            Math.abs(lateral)>0.01 ||
                            Math.abs(yaw)>0.01;

            if(sticksActive){
                driveState=DriveState.DRIVE;
            }


            if(gamepad1.left_trigger>0.5){
                driveState=DriveState.TAG_ALIGN;
            }



            else if(!sticksActive && driveState==DriveState.DRIVE){

                Pose2D pose=robot.getRobotPosition();

                lockx=pose.getX(DistanceUnit.INCH);
                locky=pose.getY(DistanceUnit.INCH);

                robot.setLockX(lockx);
                robot.setLockY(locky);
                robot.setLockHeading(
                        pose.getHeading(AngleUnit.DEGREES));

                driveState=DriveState.POSITION_LOCK;
            }


            switch(driveState){

                case POSITION_LOCK:

                    robot.goToPositionNonBlocking(
                            locky,
                            lockx,
                            robot.getHeading(AngleUnit.DEGREES),
                            0.7);
                    break;

                case TAG_ALIGN:

                    if(robot.getTagID()==20 ||
                            robot.getTagID()==24){

                        error=goalX-Bearing;

                        if(Math.abs(error)<angleTolerance){
                            yaw=0;
                        }else{
                            double pterm=error*kp;

                            currTimer=getRuntime();
                            double dT=currTimer-lastTime;
                            double dTerm=((error-lastError)/dT)*kd;

                            yaw=Range.clip(pterm+dTerm,-1,1);

                            lastError=error;
                            lastTime=currTimer;
                        }
                    }

                    robot.drive(axial,lateral,yaw,1);
                    break;

                case DRIVE:

                    robot.drive(axial,lateral,yaw,0.89);
                    break;
            }

            // ================= LAUNCHERS =================
            robot.launch(gamepad1.rightBumperWasPressed());
            robot.launchHigh(gamepad1.leftBumperWasPressed());

            if(gamepad1.right_trigger>0.5){
                robot.autoLaunch(1476,1475);
            }

            if(gamepad1.startWasPressed()){
                robot.toggleFC();
            }

            // ================= TELEMETRY =================
            telemetry.addData("========== RGB TEST ==========","");
            telemetry.addData("Last RGB Command",lastRGBColor);
            telemetry.addData("Last RGB Value",lastRGBValue);
            telemetry.addData("Square Button",
                    gamepad1.square?"PRESSED":"not pressed");
            telemetry.addData("","");

            telemetry.addData("========== POSITION ==========","");
            telemetry.addData("Drive State :: ",driveState);
            telemetry.addData("Lock x :: ",lockx);
            telemetry.addData("Lock y :: ",locky);
            telemetry.addData("","");

            telemetry.addData("========== APRIL TAG ==========","");
            telemetry.addData("April Tag Distance :: ",Dist);
            telemetry.addData("April Tag Bearing :: ",Bearing);
            telemetry.addData("","");

            telemetry.addData("========== LAUNCHER ==========","");
            telemetry.addData("Launcher Velocity :: ",Velocity);
            telemetry.addData("Low Launch State :: ",
                    robot.getLaunchState(0));
            telemetry.addData("High Launch State :: ",
                    robot.getLaunchState(1));

            telemetry.update();
        }

        robot.setRGBColor(RGBColor.OFF);
        robot.saveCurrentPose();
    }
}
