package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "FlyWheelPIDF", group = "Test")

public class FlyWheelPIDF extends OpMode {
    public DcMotorEx flywheel;
    double highVelocity = 1800;
    double lowVelocity = 1540;
    double currTargetVelocity;
    double f = 0;
    double p = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init(){
        flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");

    }

    @Override
    public void loop(){
        if(gamepad1.triangleWasPressed()){
            if(currTargetVelocity == highVelocity){
                currTargetVelocity = lowVelocity;
            }
            else{
                currTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.crossWasPressed()){
            stepIndex = (stepIndex+1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            f -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            f += stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            p -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            p += stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(currTargetVelocity);

        double currVelocity = flywheel.getVelocity();
        double error = currTargetVelocity - currVelocity;

        telemetry.addData("Target Velocity  ::  ", currTargetVelocity);
        telemetry.addData("Current Velocity  ::  ", "%.2f", currVelocity);
        telemetry.addData("Error  ::  ", "%.2f" , error);
        telemetry.addLine("=============================================");
        telemetry.addData("Tuning P  ::  ", "%.4f (D-pad U/D)", p);
        telemetry.addData("Tuning F  ::  ", "%.4f (D-pad L/R)", f);
        telemetry.addData("Step Size  ::  ", "%.4f (Cross Button)", stepSizes[stepIndex]);


    }
}
