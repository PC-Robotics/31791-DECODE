package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robots.WisdomBot;
@Disabled
@Autonomous(name = "Auto Blue Side")
public class Klutch extends LinearOpMode {

    WisdomBot Blue = new WisdomBot(this, false);

    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Blue.init();

        waitForStart();

        if(opModeIsActive())
        {
            Blue.goToPosition(-5, 30, 0, 0.5, 2);
            //Blue.goToPosition(0, 24, 0, 0.3,2);
            Blue.autoLaunch(1200, 1200);



        }

    }
}
