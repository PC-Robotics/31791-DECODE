package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.DriveBasePID;

@Autonomous(name = "Autonomous Testing")
public class AutonTesting extends LinearOpMode {

    DriveBasePID robot = new DriveBasePID(this, false);

    public void runOpMode() throws InterruptedException
    {
        robot.init();

        waitForStart();

        if(opModeIsActive())
        {

            //robot.goToPosition(24, 0, 0, .5, 20);
            robot.forward(24,.4, 10);
            sleep(1000);
            telemetry.addLine("Made past forward");
            //robot.turnTo(90, .2, 100);
            telemetry.addLine("We made it past turn");

            //robot.forward(24, .2, 100);

            //robot.turnTo(180, 1, 100);

            //robot.strafe(-24,1,100);


        }
        telemetry.addLine("Auton Finished");
        telemetry.update();
    }
}
