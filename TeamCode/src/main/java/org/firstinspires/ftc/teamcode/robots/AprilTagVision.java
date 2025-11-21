package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    private double lastTagFieldX = Double.NaN;
    private double lastTagFieldY = Double.NaN;
    private double lastTagFieldYaw = Double.NaN;
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
            lastRange = Double.NaN;
            lastBearing = Double.NaN;
            lastTagFieldX = Double.NaN;
            lastTagFieldY = Double.NaN;
            lastTagFieldYaw = Double.NaN;
            return;
        }

        // Just use the first detected tag
        AprilTagDetection tag = detections.get(0);
        lastTagID = tag.id;

        // ----------------------------------------------
        // If FTC Pose is available (VisionPortal)
        // ----------------------------------------------
        if (tag.ftcPose != null) {

            // Robot-relative pose
            lastRange   = tag.ftcPose.range;     // inches
            lastBearing = tag.ftcPose.bearing;   // degrees

            // Field-relative pose
            lastTagFieldX = tag.ftcPose.x;       // inches
            lastTagFieldY = tag.ftcPose.y;       // inches
            lastTagFieldYaw = tag.ftcPose.yaw;   // degrees
        }

        // ----------------------------------------------
        // Fallback if robotPose is used (rare)
        // ----------------------------------------------
        else if (tag.robotPose != null) {

            double x = tag.robotPose.getPosition().x;  // meters
            double y = tag.robotPose.getPosition().y;  // meters

            // Convert meters → inches
            double xi = x * 39.37;
            double yi = y * 39.37;

            lastRange = Math.sqrt(xi*xi + yi*yi);
            lastBearing = Math.toDegrees(Math.atan2(xi, yi));

            // FIELD ESTIMATES — robotPose is NOT field coords
            lastTagFieldX = Double.NaN;
            lastTagFieldY = Double.NaN;
            lastTagFieldYaw = Double.NaN;
        }

        // Telemetry for debugging
        myOpMode.telemetry.addLine(
                String.format("Tag %d  Range=%.1f  Bearing=%.1f  Field(%.1f, %.1f, %.1f°)",
                        lastTagID,
                        lastRange,
                        lastBearing,
                        lastTagFieldX,
                        lastTagFieldY,
                        lastTagFieldYaw
                )
        );
    }


    public void alignToTagNonBlocking(double desiredDistance, double desiredAngleFromTag, double movePower) {

        update();

        if(getTagID() == -1) {
            // No tag, stop
            drive(0,0,0);
            return;
        }

        // Get the tag's field pose
        double tagX = getTagFieldX();
        double tagY = getTagFieldY();
        double tagYaw = getTagFieldHeading(); // degrees

        if(Double.isNaN(tagX) || Double.isNaN(tagY) || Double.isNaN(tagYaw)) {
            // tag field pose not valid yet
            return;
        }

        // Desired robot heading
        double desiredHeading = tagYaw + desiredAngleFromTag;

        // Offset in inches from tag
        double offsetX = desiredDistance * Math.sin(Math.toRadians(desiredHeading));
        double offsetY = desiredDistance * Math.cos(Math.toRadians(desiredHeading));

        // Absolute field target
        double targetX = tagX - offsetX;
        double targetY = tagY - offsetY;

        // Move toward target
        goToPositionNonBlocking(targetY, targetX, desiredHeading, movePower);
    }


    public double getTagDistance() { return lastRange; }
    public double getTagAngle() { return lastBearing; }
    public double getTagFieldX() {
        return lastTagFieldX;
    }

    public double getTagFieldY() {
        return lastTagFieldY;
    }

    public double getTagFieldHeading() {   // yaw
        return lastTagFieldYaw;
    }

    public int getTagID() { return lastTagID; }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
