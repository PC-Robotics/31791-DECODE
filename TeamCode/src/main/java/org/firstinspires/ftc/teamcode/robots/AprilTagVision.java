package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import com.acmerobotics.dashboard.FtcDashboard;

public class AprilTagVision extends WisdomBot {


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double lastRange = Double.NaN;
    private double lastBearing = Double.NaN;
    private int lastTagID = -1;


    public AprilTagVision(LinearOpMode opMode, boolean isFC) {

        super(opMode, isFC);
    }

    public void init() {
        super.init();
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        WebcamName camera = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(camera, aprilTag);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 60);
    }

    public void update() {

        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            lastTagID = -1;
            return;
        }

        AprilTagDetection tag = detections.get(0); // just take the first one
        lastTagID = tag.id;

        if (tag.ftcPose != null) {
            lastRange = tag.ftcPose.range;     // distance from camera
            lastBearing = tag.ftcPose.bearing; // horizontal angle
        } else if (tag.robotPose != null) {
            double x = tag.robotPose.getPosition().x;
            double y = tag.robotPose.getPosition().y;
            lastRange = Math.sqrt(x * x + y * y);
            lastBearing = Math.toDegrees(Math.atan2(x, y));
        }

        myOpMode.telemetry.addLine(String.format("Tag %d Range=%.2f Bearing=%.2f",
                lastTagID, lastRange, lastBearing));
    }


    public void alignToTag(double targetDistance, double targetAngle, double minDist, double minAngle, double minStrafe, double movePower) {
        update();
        if (getTagID() == -1) {
            myOpMode.telemetry.addLine("No AprilTag detected!");
            myOpMode.telemetry.update();
            return;
        }

            myOpMode.telemetry.addLine("Starting vision alignment with goToPosition...");
            myOpMode.telemetry.update();

            // Tolerances
            double distanceTolerance = 0.6; // inches
            double angleTolerance = 5;    // degrees
            double strafeTolerance = 0.6;   // optional, if you want to center laterally

            while (myOpMode.opModeIsActive() && getTagID() != -1) {
                update(); // refresh tag info

                double distance = getTagDistance();
                double angle = getTagAngle();

                if (Double.isNaN(distance) || Double.isNaN(angle)) continue;

                // Compute error values
                double distanceError = distance + targetDistance;   // forward/backward
                double angleError = angle - targetAngle;           // rotation
                double strafeError = 0;                             // optional lateral offset if available

                boolean distanceAligned = Math.abs(distanceError) <= distanceTolerance;
                boolean angleAligned = Math.abs(angleError) <= angleTolerance;
                boolean strafeAligned = Math.abs(strafeError) <= strafeTolerance;

                // Stop if everything is within tolerances
                if (distanceAligned && angleAligned && strafeAligned) {
                    myOpMode.telemetry.addLine("Fully aligned with AprilTag!");
                    myOpMode.telemetry.update();
                    break;
                }

                // Compute target relative positions
                double currentX = getXPosition(DistanceUnit.INCH);
                double currentY = getYPosition(DistanceUnit.INCH);
                double targetX = currentX + strafeError;          // lateral offset
                double targetY = currentY + distanceError;        // forward/backward offset
                double targetHeading = getHeading(AngleUnit.DEGREES) - angleError;

                // Use goToPosition with very small holdTime for continuous adjustment
                goToPosition(-6, 89, targetHeading, 0.3, 0.05);

                // Telemetry
                myOpMode.telemetry.addData("Distance", "%.2f", distance);
                myOpMode.telemetry.addData("Angle", "%.2f", angle);
                myOpMode.telemetry.addData("Distance Error", "%.2f", distanceError);
                myOpMode.telemetry.addData("Angle Error", "%.2f", angleError);
                myOpMode.telemetry.update();

                myOpMode.sleep(30); // short pause for smoother loop
            }

            // stop motors at the end
            drive(0, 0, 0);
        }


    public double getTagDistance() { return lastRange; }
    public double getTagAngle() { return lastBearing; }

    public int getTagID() { return lastTagID; }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
