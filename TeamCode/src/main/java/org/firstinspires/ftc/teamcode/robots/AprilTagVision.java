package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.support. ConstantsPID.DRIVE_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID. DRIVE_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID. DRIVE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode. support.ConstantsPID. DRIVE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_DEADBAND;
import static org.firstinspires. ftc.teamcode.support. ConstantsPID.STRAFE_KD;
import static org.firstinspires.ftc.teamcode.support. ConstantsPID.STRAFE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KP;
import static org. firstinspires.ftc. teamcode.support.ConstantsPID.STRAFE_MAX_AUTO;
import static org. firstinspires.ftc. teamcode.support.ConstantsPID.STRAFE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KD;
import static org.firstinspires. ftc.teamcode.support. ConstantsPID.YAW_KI;
import static org.firstinspires.ftc.teamcode.support. ConstantsPID.YAW_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_TOLERANCE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware. Servo;

import org.firstinspires. ftc.robotcore.external. hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires. ftc.teamcode.support.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.acmerobotics.dashboard.FtcDashboard;

public class AprilTagVision extends WisdomBot {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Servo rgbIndicator;

    private PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    private PIDController strafeController = new PIDController(STRAFE_KP, STRAFE_KI, STRAFE_KD, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    private PIDController yawController = new PIDController(YAW_KP, YAW_KI, YAW_KD, YAW_MAX_AUTO, YAW_TOLERANCE, YAW_DEADBAND, true);

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

        // Initialize RGB Indicator as Servo
        try {
            rgbIndicator = myOpMode.hardwareMap.get(Servo.class, "rgb_indicator");
            myOpMode.telemetry.addData("RGB Indicator", "Found");
        } catch (Exception e) {
            myOpMode. telemetry.addData("RGB Indicator", "NOT FOUND");
            rgbIndicator = null;
        }

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
            lastRange = tag.ftcPose. range;     // inches
            lastBearing = tag. ftcPose.bearing; // degrees (angle from robot to tag)

            // Field-relative pose
            lastTagFieldX = tag. ftcPose.x;     // inches
            lastTagFieldY = tag.ftcPose. y;     // inches
            lastTagFieldYaw = tag.ftcPose.yaw; // degrees (tag's orientation)
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

            lastRange = Math.sqrt(xi * xi + yi * yi);
            lastBearing = Math.toDegrees(Math.atan2(xi, yi));

            // FIELD ESTIMATES — robotPose is NOT field coords
            lastTagFieldX = Double.NaN;
            lastTagFieldY = Double.NaN;
            lastTagFieldYaw = Double.NaN;
        }
    }

    /**
     * Aligns the robot to face an AprilTag at a desired distance and bearing angle.
     * SIMPLIFIED VERSION - Only forward/backward and rotation, NO STRAFING
     *
     * @param desiredDistance Target distance from tag in inches
     * @param desiredAngle Target bearing angle (0 = centered on tag)
     * @param power Maximum power for movement (0.0 to 1.0)
     */
    public void alignToTagNonBlocking(double desiredDistance, double desiredAngle, double power) {
        update(); // update vision system

        // If no tag, stop
        if (getTagID() == -1) {
            drive(0, 0, 0);
            return;
        }

        double tagDist = getTagDistance();      // forward distance to tag
        double tagBearing = getTagBearing();    // angle offset from center (degrees)

        // --- Compute errors ---
        double forwardError = tagDist - desiredDistance;
        double headingError = tagBearing - desiredAngle;

        // Check if we're aligned and stop
        boolean distanceAligned = Math.abs(forwardError) < 3.0;  // within 3 inches
        boolean angleAligned = Math.abs(headingError) < 3.0;      // within 3 degrees

        if (distanceAligned && angleAligned) {
            drive(0, 0, 0);  // STOP when aligned
            return;
        }

        // --- Simple PID control (no field-centric conversion) ---
        double axialPower = driveController.getOutputFromError(forwardError);
        double yawPower = yawController.getOutputFromError(headingError);

        // Drive:  forward/backward and rotation ONLY
        drive(axialPower * power, 0, yawPower * power);  // No lateral (strafe) movement
    }



    /**
     * Aligns robot to only match the bearing angle to a specific tag, without moving forward/back.
     *
     * @param desiredAngle Target bearing angle
     * @param power Maximum power for turning
     * @param desiredTag Only align to this specific tag ID
     */
    public void alignToTagAngleOnly(double desiredAngle, double power, int desiredTag) {
        update();

        if (getTagID() != desiredTag) {
            drive(0, 0, 0);
            return;
        }

        double tagBearing = getTagBearing();
        double headingError = -tagBearing + desiredAngle;

        double yawPower = yawController. getOutputFromError(headingError);

        drive(0, 0, yawPower * power);
    }

    /**
     * Checks if the robot is aligned to the tag within tolerance
     *
     * @param distanceTolerance Tolerance in inches
     * @param angleTolerance Tolerance in degrees
     * @return true if aligned within tolerance
     */
    public boolean isAlignedToTag(double distanceTolerance, double angleTolerance) {
        if (getTagID() == -1) return false;

        double distError = Math.abs(getTagDistance() - 12.0); // assuming 12" target
        double angleError = Math.abs(getTagBearing());

        return (distError < distanceTolerance) && (angleError < angleTolerance);
    }

    // ============================================================
    // RGB INDICATOR METHODS - UPDATED TO MATCH GOBILDA SPECS
    // ============================================================

    /**
     * Enum for goBILDA RGB Indicator colors using official FTC values
     * Based on goBILDA Product Insight #4 (3118-0808-0002)
     */
    public enum RGBColor {
        OFF(0.0),       // 500µs
        RED(0.33),     // 1100µs
        YELLOW(0.388),  // 1300µs
        SAGE(0.444),    // 1400µs
        GREEN(0.500),   // 1500µs
        AZURE(0.555),   // 1600µs
        BLUE(0.611),    // 1700µs
        INDIGO(0.666),  // 1800µs
        VIOLET(0.722),  // 1900µs
        WHITE(1.0);     // 2500µs

        private final double position;

        RGBColor(double position) {
            this. position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    /**
     * Sets the RGB indicator to a specific color using setPosition
     */
    public void setRGBColor(RGBColor color) {
        if(rgbIndicator != null) {
            rgbIndicator.setPosition(color.getPosition());
        }

    }

    /**
     * Updates RGB indicator based on alignment to target angle.
     * Green = good alignment, Yellow = close, Orange = far, Red = no tag
     *
     * @param targetAngle Target bearing angle in degrees
     * @param tolerance Tolerance in degrees for "good" alignment
     */
    /*
    public void updateRGBIndicator(double targetAngle, double tolerance) {
        // If no tag detected, turn indicator red
        if (getTagID() == -1) {
            setRGBColor(RGBColor.RED);
            return;
        }

        double currentBearing = getTagBearing();

        // Check if angle is within tolerance
        double angleDifference = Math.abs(currentBearing - targetAngle);

        if (angleDifference <= tolerance) {
            setRGBColor(RGBColor. GREEN);  // Within target range - good alignment
        } else if (angleDifference <= tolerance * 2) {
            setRGBColor(RGBColor.YELLOW); // Close to target
        } else {
            setRGBColor(RGBColor.ORANGE); // Far from target
        }
    }


     */
    /**
     * Simple RGB indicator showing tag detection status.
     * Blue = tag detected, Off = no tag
     */
    /*
    public void updateRGBTagStatus() {
        if (getTagID() == -1) {
            setRGBColor(RGBColor.OFF);  // No tag detected
        } else {
            setRGBColor(RGBColor. BLUE); // Tag detected
        }
    }


     */
    // ============================================================
    // GETTER METHODS
    // ============================================================

    /**
     * @return Distance to detected tag in inches (NaN if no tag)
     */
    public double getTagDistance() {
        return lastRange;
    }

    /**
     * @return Bearing angle to tag in degrees (NaN if no tag)
     * Positive = tag is to the right, Negative = tag is to the left
     */
    public double getTagBearing() {
        return lastBearing;
    }

    /**
     * @return Tag's field X coordinate in inches (NaN if not available)
     */
    public double getTagFieldX() {
        return lastTagFieldX;
    }

    /**
     * @return Tag's field Y coordinate in inches (NaN if not available)
     */
    public double getTagFieldY() {
        return lastTagFieldY;
    }

    /**
     * @return Tag's orientation/yaw on the field in degrees (NaN if not available)
     * Note: This is the tag's orientation, NOT the bearing to the tag
     */
    public double getTagFieldHeading() {
        return lastTagFieldYaw;
    }

    /**
     * @return Currently detected tag ID (-1 if no tag detected)
     */
    public int getTagID() {
        return lastTagID;
    }

    /**
     * Resets the yaw PID controller.
     * Call this when starting a new alignment operation.
     */
    public void resetYawController() {
        yawController. reset();
    }

    /**
     * Resets all PID controllers.
     * Call this when starting a new alignment operation.
     */
    public void resetAllControllers() {
        driveController.reset();
        strafeController.reset();
        yawController.reset();
    }

    /**
     * Stops the vision system and turns off RGB indicator
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal. close();
        }
        if (rgbIndicator != null) {
            rgbIndicator. setPosition(0.0);  // Turn off
        }
    }
}