package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ShooterTest")
public class ShootTester extends LinearOpMode {

    CRServo leftFeeder;
    CRServo rightFeeder;

    @Override
    public void runOpMode() throws InterruptedException {

        init2();

        waitForStart();

        while(opModeIsActive()){
            leftFeeder.setPower(0.8);
            rightFeeder.setPower(0.8);
        }




    }

    public void init2()
    {
        leftFeeder = hardwareMap.get(CRServo.class,"left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,"right_feeder");

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
