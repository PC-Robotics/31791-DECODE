package org.firstinspires.ftc.teamcode.Test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;

@TeleOp
public class Align extends LinearOpMode {
    AprilTagVision robot = new AprilTagVision(this, false);




    //================== PD Controller =====================
    double kp = 0.0340;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.4;

    double kd = 0.0008;
    double currTimer = 0;
    double lastTime = 0;

    // ================ Driving setup =================
    double forward, strafe, rotate;

    // ================= controller based tuning ==============
    double[] stepSizes = {1 ,0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();
        robot.restoreSavedPoseIfAvailable();

        telemetry. addLine("Ready - Press Start");
        telemetry.update();
        waitForStart();

        while(opModeIsActive())
        {
            // ================ Get Mecanum Drive Inputs
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            // ============= get tag info =================
            robot.update();
            telemetry.addData("Current tag", robot.getTagID());

            //=============== Auto Align ================
            if(gamepad1.left_trigger > 0.5){
                if(robot.getTagID() == 20 || robot.getTagID() == 24 ){
                    error = goalX - robot.getTagBearing();  // tx

                    if(Math.abs(error) < angleTolerance){
                        rotate = 0;
                    }else{
                        double pterm = error * kp;

                        currTimer = getRuntime();
                        double dT = currTimer - lastTime;
                        double dTerm = ((error - lastError) / dT) * kd;
                        rotate = Range.clip(pterm + dTerm, -1, 1);
                        lastError = error;
                        lastTime = currTimer;
                    }
                }else{
                    lastTime = getRuntime();
                    lastError = 0;
                }



            }else{
                lastError =0;
                lastTime = getRuntime();
            }

            // Drive Robot
            robot.drive(forward, strafe, rotate, 1);


            if(gamepad1.crossWasPressed()){
                stepIndex = (stepIndex + 1)% stepSizes.length;
            }
            if(gamepad1.dpadLeftWasPressed()){
                kp -= stepSizes[stepIndex];
            }
            if(gamepad1.dpadRightWasPressed()) {
                kp += stepSizes[stepIndex];
            }
            if(gamepad1.dpadUpWasPressed()){
                kd +=stepSizes[stepIndex];
            }
            if(gamepad1.dpadDownWasPressed()){
                kd -=stepSizes[stepIndex];
            }
            telemetry.addData("error  ::  ", error);
            telemetry.addData("Tuning  P  ::  ", "%.4f (D-pad L/R)", kp);
            telemetry.addData("Tuning  D  ::  ", "%.4f (D-pad U/D)", kd);
            telemetry.addData("StepSize  ::  ", "%.4f (Cross)", stepSizes[stepIndex]);



            telemetry.update();
        }






    }

    public void go()
    {
        resetRuntime();
        currTimer = getRuntime();
    }




}
