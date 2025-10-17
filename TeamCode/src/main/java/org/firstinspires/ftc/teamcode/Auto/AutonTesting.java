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

        while(opModeIsActive())
        {
            robot.forward(24,.4, 100);

            //robot.turnTo(90, .2, 100);

            //robot.forward(24, .2, 100);

            //robot.turnTo(180, 1, 100);

            //robot.strafe(-24,1,100);


        }
    }
}
