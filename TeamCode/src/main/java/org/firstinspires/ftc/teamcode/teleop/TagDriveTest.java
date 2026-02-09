package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;


@TeleOp(name = "Tag Drive Test", group = "Test")
public class TagDriveTest extends LinearOpMode {
    AprilTagVision robot = new AprilTagVision(this, false);

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        telemetry.addLine("Ready - Press Start");
        telemetry.update();
        waitForStart();

        robot.initDriveHold();


        while(opModeIsActive()){
            robot.update();
            double Dist = robot.getTagDistance();
            //double Angle = robot.getTagAngle();
            double Velocity = robot.getVelocity2();
            //boolean isInput = false;

            telemetry.addData("April Tag Distance  ::  ", Dist );
            //telemetry.addData("April Tag Angle  ::  ", Angle);
            telemetry.addData("Launcher Velocity ::  " , Velocity);

            gamepad1Controls();

            telemetry.update();
        }

    }

    public void gamepad1Controls(){
        robot.driveWithHold(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );


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
    }


}

