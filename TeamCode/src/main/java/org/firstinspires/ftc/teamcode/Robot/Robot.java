package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="StarterBotAuto", group="StarterBot")
public abstract class Robot extends Core {
    public Movement movement = new Movement(this);

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LAUNCHDURATION_SECONDS = 2.0; //The amount of time to wait before turning off the flywheel

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1220;
    final double LAUNCHER_MIN_VELOCITY = 1160;
    final double LAUNCHER_HIGH_VELOCITY = 2000;
    final double LAUNCHER_LOWER_VELOCITY = 1900;

    @Override
    public void init() {
        // Setup Robot Generic Variables Here
        resetEncoders();
    };

    @Override
    public void init_loop(){
        super.init_loop();
    };

    @Override
    public void start(){
        super.start();
    };

    @Override
    public void stop(){
        super.stop();
    }

}
