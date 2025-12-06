package org.firstinspires.ftc.teamcode.robots;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.support.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import com.acmerobotics.dashboard.FtcDashboard;

public class AprilTagVision extends WisdomBot {


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private PIDController driveController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD,DRIVE_MAX_AUTO,DRIVE_TOLERANCE,DRIVE_DEADBAND,false);
    private PIDController strafeController = new PIDController(STRAFE_KP,STRAFE_KI,STRAFE_KD,STRAFE_MAX_AUTO,STRAFE_TOLERANCE,STRAFE_DEADBAND, false);
    private PIDController yawController = new PIDController(YAW_KP,YAW_KI,YAW_KD,YAW_MAX_AUTO,YAW_TOLERANCE,YAW_DEADBAND,true);




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
    public void alignToTagNonBlocking(double desiredDistance, double desiredAngle, double power) {

        update(); // update vision system

        // If no tag, stop
        if (getTagID() == -1) {
            drive(0, 0, 0);
            return;
        }

        double tagDist = getTagDistance();              // forward distance to tag
        double tagAngle = getTagAngle();                // angle offset from center (degrees)

        // --- Compute robot-relative error ---
        double forwardError = tagDist - desiredDistance;

        // Convert angle offset to left/right distance error
        double lateralError = Math.tan(Math.toRadians(tagAngle)) * tagDist;

        // Heading correction: want tag centered → 0°
        double headingError = tagAngle - desiredAngle;

        // --- Convert robot-relative to robot-centric frame (same as goToPosition) ---
        double negativeRadianHeading = -getHeading(AngleUnit.RADIANS);

        double rotatedX = forwardError * Math.cos(negativeRadianHeading)
                - lateralError * Math.sin(negativeRadianHeading);

        double rotatedY = forwardError * Math.sin(negativeRadianHeading)
                + lateralError * Math.cos(negativeRadianHeading);

        // --- PID control ---
        double axialPower = driveController.getOutputFromError(rotatedX);
        double lateralPower = strafeController.getOutputFromError(rotatedY);
        double yawPower = yawController.getOutput(headingError);

        drive(-axialPower * power, -lateralPower * power, -yawPower * power);
    }

    public void alignToTagAngleOnly(double desiredAngle, double power, int desiredTag) {

        update();

        if (getTagID() != desiredTag) {
            drive(0, 0, 0);
            return;
        }

        double tagAngle = getTagAngle();

        double headingError = -tagAngle + desiredAngle;

        double yawPower = yawController.getOutputFromError(headingError);

        drive(0, 0, yawPower);
    }
    public double getTagDistance() { return lastRange; }
    public double getTagAngle() { return lastTagFieldYaw; }
    public double getTagFieldX() {
        return lastTagFieldX;
    }

    public double getTagFieldY() {
        return lastTagFieldY;
    }

    public double getTagFieldHeading() {   // yaw
        return lastTagFieldYaw;
    }

    public void resetYawController(){
        yawController.reset();
    }

    public int getTagID() { return lastTagID; }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
