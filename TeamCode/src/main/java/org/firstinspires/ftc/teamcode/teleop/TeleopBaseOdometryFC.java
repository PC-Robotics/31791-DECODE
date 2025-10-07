package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.DriveBaseOdometry;

@TeleOp(name="Odometry Drive FC",group="Test Modes")
public class TeleopBaseOdometryFC extends LinearOpMode
{
    private DriveBaseOdometry robot = new DriveBaseOdometry(this, true);

    public void runOpMode()
    {
        robot.init();

        robot.resetPositionAndOdometry();

        waitForStart();

        while(opModeIsActive())
        {
            gamepad1Controls();
            gamepad2Controls();

            robot.updatePositionAndTelemetry();

            telemetry.update();
        }
    }

    /**
     * Stores all logic corresponding to Gamepad 1 [Driver]
     */
    public void gamepad1Controls()
    {
        double axial = -gamepad1.left_stick_y;   // Forward on left stick yields negative val
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        robot.drive(axial,lateral,yaw);
    }

    /**
     * Stores all logic corresponding to Gamepad 2 [Operator]
     */
    public void gamepad2Controls()
    {
        // Resets the robot telemetry to position (0,0), heading 0 degrees
        if(gamepad2.cross)
        {
            robot.resetPositionAndOdometry();
        }
    }


}
