package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.support.ConstantsPID;

public class WisdomBot extends DriveBasePID {
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime stopTimer = new ElapsedTime();

    private static final double FEED_TIME_SECONDS = 0.20;
    private static final double STOP_SPEED = 0.0;
    private static final double FULL_SPEED = 1.0;
    private static final double LAUNCH_DURATION_SECONDS = 2.0;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState = LaunchState.IDLE;
    private LaunchState launchStateHigh = LaunchState.IDLE;

    public WisdomBot(LinearOpMode mode, boolean isFC) {
        super(mode, isFC);
    }

    /** Initialize all launcher + feeder hardware **/
    public void init() {

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        super.init();
    }

    /** Regular launch (medium velocity) **/
    public void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                } else if (stopTimer.seconds() > LAUNCH_DURATION_SECONDS) {
                    launcher.setVelocity(0);
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(ConstantsPID.LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > ConstantsPID.LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    stopTimer.reset();
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    /** High launch (high velocity) **/
    public void launchHigh(boolean shotRequested) {
        switch (launchStateHigh) {
            case IDLE:
                if (shotRequested) {
                    launchStateHigh = LaunchState.SPIN_UP;
                } else if (stopTimer.seconds() > LAUNCH_DURATION_SECONDS) {
                    launcher.setVelocity(0);
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(ConstantsPID.LAUNCHER_HIGH_VELOCITY);
                if (launcher.getVelocity() > ConstantsPID.LAUNCHER_LOWER_VELOCITY) {
                    launchStateHigh = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchStateHigh = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchStateHigh = LaunchState.IDLE;
                    stopTimer.reset();
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    /** Optional: stop all launcher systems **/
    public void stopLauncher() {
        launcher.setVelocity(0);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

}
