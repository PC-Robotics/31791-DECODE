package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="StarterBotAuto", group="StarterBot")
public abstract class Robot extends Core {
    public Movement movement = new Movement(this);

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
