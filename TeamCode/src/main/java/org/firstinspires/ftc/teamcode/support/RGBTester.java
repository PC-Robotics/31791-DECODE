package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RGB Testing")
public class RGBTester extends LinearOpMode
{
    Servo RGBLight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RGBLight = hardwareMap.get(Servo.class,"rgb_indicator");

        telemetry.addLine("RGB Found and Initialized");
        telemetry.update();

        double RGBValue = 0.0;

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.square)
            {
                RGBValue = 0.6666;
            }
            if(gamepad1.circle)
            {
                RGBValue = 0.45;
            }

            RGBLight.setPosition(RGBValue);
        }
    }
}
