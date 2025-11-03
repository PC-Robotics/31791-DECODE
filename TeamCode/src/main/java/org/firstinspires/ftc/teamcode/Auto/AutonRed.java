package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robots.WisdomBot;

@Autonomous(name = "Auto Red Side")
public class AutonRed extends LinearOpMode {

    WisdomBot Red = new WisdomBot(this, false);

    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Red.init();

        waitForStart();

        if(opModeIsActive())
        {
            Red.forward(48, 0.6, 2);
            //Blue.goToPosition(0, 24, 0, 0.3,2);
            Red.autoLaunch(1500, 1400);
            Red.goToPosition(24, 48 , 0, 0.3, 2);
            Red.turnTo(-135, 0.5, 2);


        }

    }
}
