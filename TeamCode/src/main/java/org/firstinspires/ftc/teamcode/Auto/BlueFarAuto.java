package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.WisdomBot;
@Autonomous(name = "Far Shot Blue")
public class BlueFarAuto extends LinearOpMode {

    WisdomBot Blue = new WisdomBot(this, false);

    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Blue.init();

        waitForStart();

        if(opModeIsActive())
        {
            Blue.goToPosition(5, -4, 16.2, 0.5, 2);
            //Blue.goToPosition(0, 24, 0, 0.3,2);
            Blue.autoLaunch(1875, 1850);

            Blue.goToPosition(-40, 0, 0,0.5,2);



        }

    }
}
