package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.robots.AprilTagVision;

@Autonomous(name = "Launch")
public class Launch extends LinearOpMode {

    AprilTagVision Blue = new AprilTagVision(this, false);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // init robot
        Blue.init();

        // ======================
        // SET STARTING POSITION
        // ======================
        Blue.setRobotPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        -47,
                        -53,
                        AngleUnit.DEGREES,
                        48
                )
        );

        telemetry.addLine("Start pose set!");

        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            Blue.setRobotPosition(//blue launch zone
                    new Pose2D(
                            DistanceUnit.INCH,
                            -47,
                            -53,
                            AngleUnit.DEGREES,
                            48
                    )
            );
            telemetry.addLine("Robot staying still — pose initialized.");
            telemetry.update();

            Blue.fastLaunch(1850, 1860);





        }
    }
}
