package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.DriveBasePID;

@Autonomous(name = "Autonomous Testing")
public class AutonTesting extends LinearOpMode {

    DriveBasePID robot = new DriveBasePID(this, false);

    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        waitForStart();

        if(opModeIsActive())
        {

            robot.goToPosition(0, 18, 0, .5, 2);

            //robot.forward(24,.65, 5);
            telemetry.addLine("Made past forward");
            //robot.turnTo(90, .5, 5);
            telemetry.addLine("We made it past turn");
            //robot.strafe(24, .5, 5);

            //robot.forward(24, .2, 100);

            //robot.turnTo(180, 1, 100);

            //robot.strafe(-24,1,100);


        }
        telemetry.addLine("Auton Finished");
        telemetry.update();
    }
}
