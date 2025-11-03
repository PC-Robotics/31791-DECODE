package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.DriveBasePID;
import org.firstinspires.ftc.teamcode.robots.WisdomBot;

@TeleOp(name = "WisdomTeleop", group = "StarterBot")
public class WisdomTeleop extends LinearOpMode {
    WisdomBot robot = new WisdomBot(this, false);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        robot.resetPositionAndOdometry();

        waitForStart();

        while(opModeIsActive()){
            robot.updatePositionAndTelemetry();

            gamepad1Controls();


            telemetry.update();
        }
    }

    public void gamepad1Controls(){
        double axial = -gamepad1.left_stick_y;   // Forward on left stick yields negative val
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        robot.drive(axial,lateral,yaw, 0.92);

        robot.launch(gamepad1.rightBumperWasPressed());
        robot.launchHigh(gamepad1.leftBumperWasPressed());
        if(gamepad1.right_trigger > 0.5){
            robot.autoLaunch(1450, 1400);
        }
        if(gamepad1.left_trigger > 0.5){
            robot.autoLaunch(2050, 2000);
        }
        if(gamepad1.dpad_up){
            robot.stopLauncher();
        }
        if(gamepad1.start){
            robot.toggleFC();
        }
    }

}
