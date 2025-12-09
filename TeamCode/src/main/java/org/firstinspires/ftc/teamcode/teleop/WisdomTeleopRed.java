package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.support.ConstantsPID.LAUNCHER_HIGH_VELOCITY;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.LAUNCHER_LOWER_VELOCITY;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.LAUNCHER_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.LAUNCHER_TARGET_VELOCITY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision;

@TeleOp(name = "WisdomTeleopRed", group = "Test")
public class WisdomTeleopRed extends LinearOpMode {
    AprilTagVision robot = new AprilTagVision(this, false);

    boolean positionLockActive = false;
    boolean tagAlign = false;
    double lockx, locky, lockheading;

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
                robot.resetYawController();
            }

            if(gamepad1.crossWasPressed()){
                positionLockActive = false;
                tagAlign = false;
            }

            if(positionLockActive){
                robot.goToPositionNonBlocking(locky, lockx, robot.getHeading(AngleUnit.DEGREES), 1);
            }
            else if (tagAlign) {
                robot.alignToTagAngleOnly(-29, 1, 24);
            }
            else {
                gamepad1Controls();
            }

            telemetry.addData("Lock x :: ", lockx );
            telemetry.addData("Lock y :: ", locky );
            telemetry.addData("April Tag Distance  ::  ", Dist );
            telemetry.addData("April Tag Angle  ::  ", Angle);
            telemetry.addData("Launcher Velocity ::  " , Velocity);
            telemetry.addData("Launcher Value Normal :: ", LAUNCHER_TARGET_VELOCITY);
            telemetry.addData("Launcher Value Far :: ", LAUNCHER_HIGH_VELOCITY);



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
            robot.autoLaunch(1475, 1450);
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
        if(gamepad1.dpadUpWasPressed()){
            LAUNCHER_TARGET_VELOCITY += 10;
            LAUNCHER_MIN_VELOCITY += 10;
        }
        if(gamepad1.dpadDownWasPressed()){
            LAUNCHER_TARGET_VELOCITY -= 10;
            LAUNCHER_MIN_VELOCITY -= 10;
        }
        if(gamepad1.dpadLeftWasPressed()){
            LAUNCHER_HIGH_VELOCITY += 10;
            LAUNCHER_LOWER_VELOCITY += 10;
        }
        if(gamepad1.dpadRightWasPressed()){
            LAUNCHER_HIGH_VELOCITY -= 10;
            LAUNCHER_LOWER_VELOCITY -=10;
        }
    }





}


//adb connect 192.168.43.1:5555

