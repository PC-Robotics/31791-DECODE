package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && myOpMode.opModeInInit()) {
            myOpMode.telemetry.addLine("Waiting for camera to start...");
            myOpMode.telemetry.update();
        }
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



    public double getTagDistance() { return lastRange; }
    public double getTagAngle() { return lastBearing; }
    public int getTagID() { return lastTagID; }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
