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
    double lockx, locky, lockheading;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        robot.restoreSavedPoseIfAvailable();

        boolean xPreviouslyPressed = false;
        boolean positionLockEnabled = false;
        double savedX = 0;
        double savedY = 0;
        double savedHeading = 0;


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
                robot.alignToTagNonBlocking(60, 0, 0.3);
            }

            if(gamepad1.crossWasPressed()){
                positionLockActive = false;
            }

            if(positionLockActive){
                robot.goToPositionNonBlocking(locky, lockx, robot.getHeading(AngleUnit.DEGREES), 0.7);
            } else {
                gamepad1Controls();
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
        if(!positionLockActive) {
            double axial = -gamepad1.left_stick_y;   // Forward on left stick yields negative val
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            robot.drive(axial, lateral, yaw, 0.92);
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
    }



}


//adb connect 192.168.43.1:5555

