package org.firstinspires.ftc.teamcode.Odometry;

import static org.firstinspires.ftc.teamcode.support.OdometryConstants.DRIVE_MAX_AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.support.PIDRobot;

/**
 * The purpose of this Autonomous OP Mode is to test that the PID constants for drive, strafe, and turn
 * are correctly setup. This code will move around a 1x1 square keeping a consistent orientation
 * Then, the robot will move around the same 1x1 square but turn 90 degrees to the left each leg.
 * Finally, the robot will make large and small rotations to test turning.
 */

@Autonomous(name="PID Value Calculation",group="Testing")
public class PIDConstantTuningAutonomous extends LinearOpMode {
    private PIDRobot robot = new PIDRobot(this, false);

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(true);
        robot.imu.resetYaw();
        telemetry.addData(">","Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();

        robot.readSensors();

        telemetry.update();

        if(opModeIsActive())
        {

            robot.moveToPointRelative(24, 24, DRIVE_MAX_AUTO, 1);








            // Test driving and strafing
            //robot.drive(2*FLOOR_TILE_INCHES,DRIVE_MAX_AUTO,0.25);

            /*
            robot.strafe(FLOOR_TILE_INCHES,STRAFE_MAX_AUTO,0.25);
            robot.drive(-FLOOR_TILE_INCHES,DRIVE_MAX_AUTO,0.25);
            robot.strafe(-FLOOR_TILE_INCHES,STRAFE_MAX_AUTO,0.25);

            sleep(500);

            // Test driving and turning
            robot.drive(  FLOOR_TILE_INCHES, DRIVE_MAX_AUTO, 0.25);
            robot.turnTo(Math.PI/2, YAW_MAX_AUTO, 0.5);
            robot.drive(  FLOOR_TILE_INCHES, DRIVE_MAX_AUTO, 0.25);
            robot.turnTo(Math.PI, YAW_MAX_AUTO, 0.5);
            robot.drive(  FLOOR_TILE_INCHES, DRIVE_MAX_AUTO, 0.25);
            robot.turnTo(3*Math.PI/2, YAW_MAX_AUTO, 0.5);
            robot.drive(  FLOOR_TILE_INCHES, DRIVE_MAX_AUTO, 0.25);
            robot.turnTo(0, YAW_MAX_AUTO, 0.5);

            sleep(500);

            // Test turning left and right, large amounts and small
            robot.turnTo(0,YAW_MAX_AUTO, 0.25);
            robot.turnTo(Math.PI,YAW_MAX_AUTO, 0.25);
            robot.turnTo(0,YAW_MAX_AUTO, 0.25);
            robot.turnTo(Math.PI / 2,YAW_MAX_AUTO, 0.25);
            robot.turnTo(0,YAW_MAX_AUTO, 0.25);
            robot.turnTo(3* Math.PI /2,YAW_MAX_AUTO,0.25);
            robot.turnTo(0,YAW_MAX_AUTO,0.25);
            */
        }
    }
}
