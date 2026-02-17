package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision;
import org.firstinspires.ftc.teamcode.robots.AprilTagVision.RGBColor;
import org.firstinspires.ftc.teamcode.support.PoseStorage;

@TeleOp(name="OdoTeleop", group="Test")
public class odoTeleop extends LinearOpMode {

    AprilTagVision robot = new AprilTagVision(this,false);

    private enum DriveState {
        TAG_ALIGN,
        POSITION_LOCK,
        DRIVE
    }

    private DriveState driveState;

    double lockx=0;
    double locky=0;

    double goalFieldX=-80;
    double goalFieldY=-90;

    double lastRGBValue=0;
    String lastRGBColor="OFF";

    @Override
    public void runOpMode(){

        telemetry=new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        robot.init();

        // ===== SET START POSITION =====
        robot.setRobotPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        -47,
                        -53,
                        AngleUnit.DEGREES,
                        48));

        driveState=DriveState.DRIVE;

        telemetry.addLine("Ready - Press Start");
        telemetry.update();

        waitForStart();
        robot.setRobotPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        -47,
                        -53,
                        AngleUnit.DEGREES,
                        48));

        while(opModeIsActive()){

            robot.update();
            robot.updatePosition();

            double axial=-gamepad1.left_stick_y;
            double lateral=gamepad1.left_stick_x;
            double yawInput=gamepad1.right_stick_x;
            double yaw=yawInput;

            double Dist=robot.getTagDistance();
            double Bearing=robot.getTagBearing();
            double Velocity=robot.getVelocity2();

            if(robot.getTagID()==20){
                if(Bearing>-10 && Bearing<-3 &&
                        Dist>94 && Dist<108){
                    robot.setRGBColor(RGBColor.BLUE);
                    lastRGBColor="Blue (Tag Left)";
                    lastRGBValue=0.611;
                }else{
                    robot.setRGBColor(RGBColor.VIOLET);
                    lastRGBColor="PURPLE (Centered)";
                    lastRGBValue=0.65;
                }
            }

            if(gamepad1.triangleWasPressed()){
                Pose2D pose=robot.getRobotPosition();

                lockx=pose.getX(DistanceUnit.INCH);
                locky=pose.getY(DistanceUnit.INCH);

                robot.setLockX(lockx);
                robot.setLockY(locky);
                robot.setLockHeading(
                        pose.getHeading(AngleUnit.DEGREES));

                driveState=DriveState.POSITION_LOCK;
            }

            if(gamepad1.left_trigger>0.5){
                driveState=DriveState.TAG_ALIGN;
            }

            if(gamepad1.crossWasPressed()){
                driveState=DriveState.DRIVE;
            }

            boolean sticksActive=
                    Math.abs(axial)>0.01 ||
                            Math.abs(lateral)>0.01 ||
                            Math.abs(yawInput)>0.01;

            if(sticksActive &&
                    driveState!=DriveState.TAG_ALIGN){
                driveState=DriveState.DRIVE;
            }
            else if(!sticksActive &&
                    driveState==DriveState.DRIVE){

                Pose2D pose=robot.getRobotPosition();

                lockx=pose.getX(DistanceUnit.INCH);
                locky=pose.getY(DistanceUnit.INCH);

                robot.setLockX(lockx);
                robot.setLockY(locky);
                robot.setLockHeading(
                        pose.getHeading(AngleUnit.DEGREES));

                driveState=DriveState.POSITION_LOCK;
            }

            switch(driveState){

                case POSITION_LOCK:

                    robot.goToPositionNonBlocking(
                            locky,
                            lockx,
                            robot.getHeading(AngleUnit.DEGREES),
                            0.7);
                    break;

                case TAG_ALIGN:

                    if(gamepad1.left_trigger<=0.5 ||
                            Math.abs(yawInput)>0.15){
                        driveState=DriveState.DRIVE;
                        break;
                    }

                    Pose2D pose=
                            robot.getRobotPosition();

                    double robotX=
                            pose.getX(DistanceUnit.INCH);

                    double robotY=
                            pose.getY(DistanceUnit.INCH);

                    double currentHeading=
                            robot.getHeading(
                                    AngleUnit.DEGREES);

                    yaw=calculateGoalFacingYaw(
                            currentHeading,
                            robotX,
                            robotY);

                    robot.drive(axial,lateral,yaw,1);
                    break;

                case DRIVE:

                    robot.drive(
                            axial,
                            lateral,
                            yawInput,
                            0.89);
                    break;
            }

            robot.launch(
                    gamepad1.rightBumperWasPressed());

            robot.launchHigh(
                    gamepad1.leftBumperWasPressed());

            if(gamepad1.right_trigger>0.5){
                robot.autoLaunch(1476,1475);
            }

            if(gamepad1.startWasPressed()){
                robot.toggleFC();
            }

            telemetry.addData(
                    "========== POSITION ==========","");
            telemetry.addData(
                    "Drive State",driveState);
            telemetry.addData(
                    "X",robot.getRobotPosition()
                            .getX(DistanceUnit.INCH));
            telemetry.addData(
                    "Y",robot.getRobotPosition()
                            .getY(DistanceUnit.INCH));
            telemetry.update();
        }

        robot.setRGBColor(RGBColor.OFF);
        robot.saveCurrentPose();
    }

    double lastTurnError = 0;
    double lastTurnTime = 0;

    private double calculateGoalFacingYaw(
            double currentHeadingDeg,
            double robotX,
            double robotY){

        double dx = goalFieldX - robotX;
        double dy = goalFieldY - robotY;

        double targetAngle =
                Math.toDegrees(Math.atan2(dy,dx));

        double error =
                targetAngle - currentHeadingDeg;

        while(error > 180) error -= 360;
        while(error < -180) error += 360;

        double kp = 0.001;
        double kd = 0.000001;

        double now = getRuntime();
        double dt = now - lastTurnTime;

        if(dt <= 0) dt = 0.02;


        double derivative =
                (error - lastTurnError) / dt;

        lastTurnError = error;
        lastTurnTime = now;


        double scale =
                Range.clip(Math.abs(error)/40,
                        0.2,
                        0.8);

        return Range.clip(
                (kp*error + kd*derivative)*scale,
                -0.8,
                0.8);
    }

}
