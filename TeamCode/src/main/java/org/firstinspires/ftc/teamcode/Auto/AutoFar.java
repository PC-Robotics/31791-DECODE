package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.WisdomBot;
@Disabled
@Autonomous(name = "Auto Far Side")
public class AutoFar extends LinearOpMode {

    WisdomBot Far = new WisdomBot(this, false);

    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Far.init();

        waitForStart();

        if(opModeIsActive())
        {

            //Blue.goToPosition(0, 24, 0, 0.3,2);
            Far.forward(-8, 0.5, 1);
            Far.autoLaunch(2050, 2000);
            Far.forward(-28, 0.6, 2);


        }

    }
}