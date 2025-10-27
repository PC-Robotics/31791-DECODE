package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.support.ConstantsPID.*;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_TOLERANCE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.support.PIDController;

public class DriveBasePID extends DriveBaseOdometry
{
    private ElapsedTime holdTimer = new ElapsedTime();

    private PIDController driveController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD,DRIVE_MAX_AUTO,DRIVE_TOLERANCE,DRIVE_DEADBAND,false);
    private PIDController strafeController = new PIDController(STRAFE_KP,STRAFE_KI,STRAFE_KD,STRAFE_MAX_AUTO,STRAFE_TOLERANCE,STRAFE_DEADBAND, false);
    private PIDController yawController = new PIDController(YAW_KP,YAW_KI,YAW_KD,YAW_MAX_AUTO,YAW_TOLERANCE,YAW_DEADBAND,true);

    private double xPosition;
    private double yPosition;
    private double heading;

    private double targetX;
    private double targetY;
    private double targetHeading;

    public DriveBasePID(LinearOpMode mode, boolean isFC)
    {
        super(mode,isFC);
    }

    public void init()
    {
        super.init();

        addLocationToTelemetry();
        myOpMode.telemetry.update();
    }

    public void forward(double distanceInches, double power, double holdTime)
    {
        targetX = getXPosition(DistanceUnit.INCH)+distanceInches;
        targetY = getYPosition(DistanceUnit.INCH);
        targetHeading = getHeading(AngleUnit.DEGREES);

        driveController.reset(targetX,power);
        strafeController.reset(targetY);
        yawController.reset(targetHeading);
        holdTimer.reset();

        while (myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            xPosition = getXPosition(DistanceUnit.INCH);
            yPosition = getYPosition(DistanceUnit.INCH);
            heading = getHeading(AngleUnit.DEGREES);

            double axial = driveController.getOutput(xPosition);
            double lateral = -strafeController.getOutput(yPosition);
            double yaw = -yawController.getOutput(heading);

            addLocationToTelemetry();

            drive(axial,lateral,yaw,power);

            myOpMode.telemetry.update();

            if(driveController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime)  break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void strafe(double distanceInches, double power, double holdTime)
    {
        targetX = getXPosition(DistanceUnit.INCH);
        targetY = getYPosition(DistanceUnit.INCH)+distanceInches;
        targetHeading = getHeading(AngleUnit.DEGREES);




        driveController.reset(targetX);
        strafeController.reset(targetY,power);
        yawController.reset(targetHeading);
        holdTimer.reset();

        while (myOpMode.opModeIsActive())
        {
            xPosition = getXPosition(DistanceUnit.INCH);
            yPosition = getYPosition(DistanceUnit.INCH);
            heading = getHeading(AngleUnit.DEGREES);

            double axial = driveController.getOutput(xPosition);
            double lateral = -strafeController.getOutput(yPosition);
            double yaw = -yawController.getOutput(heading);

            myOpMode.telemetry.addData("Axial :: " , axial);
            myOpMode.telemetry.addData("Lateral :: ", lateral);
            myOpMode.telemetry.addData("Yaw :: ", yaw);

            addLocationToTelemetry();
            updatePositionAndTelemetry();

            drive(driveController.getOutput(xPosition),-strafeController.getOutput(yPosition), -yawController.getOutput(heading),power);
            myOpMode.telemetry.update();
            if(strafeController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime)  break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void turnTo(double headingDegrees, double power, double holdTime)
    {
        targetX = getXPosition(DistanceUnit.INCH);
        targetY = getYPosition(DistanceUnit.INCH);
        targetHeading = headingDegrees;

        yawController.reset(headingDegrees,power);

        while(myOpMode.opModeIsActive())
        {
            heading = getHeading(AngleUnit.DEGREES);

            addLocationToTelemetry();
            updatePositionAndTelemetry();

            drive(0,0,-yawController.getOutput(heading),power);
            myOpMode.telemetry.update();
            if(yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime) break;
            }
            else holdTimer.reset();
            // Test if needed
            myOpMode.sleep(10);
        }

        // Test if needed
        drive(0,0,0);
    }

    public void goToPosition(double yLocation, double xLocation, double headingDegree, double power, double holdTime)
    {
        driveController.reset(yLocation, power);
        strafeController.reset(xLocation, power);
        yawController.reset(headingDegree, power);

        while(myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            drive(-driveController.getOutput(getYPosition(DistanceUnit.INCH)),-strafeController.getOutput(getXPosition(DistanceUnit.INCH)), yawController.getOutput(getHeading(AngleUnit.DEGREES)));

            myOpMode.telemetry.update();

            if(driveController.isInPosition() && strafeController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime) break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void addLocationToTelemetry()
    {
        myOpMode.telemetry.addData("X Position:: ",xPosition);
        myOpMode.telemetry.addData("Y Position:: ",yPosition);
        myOpMode.telemetry.addData("Heading:: ",heading);

        myOpMode.telemetry.addData("X Target:: ",targetX);
        myOpMode.telemetry.addData("Y Target:: ",targetY);
        myOpMode.telemetry.addData("Heading Target:: ", targetHeading);
    }
}
