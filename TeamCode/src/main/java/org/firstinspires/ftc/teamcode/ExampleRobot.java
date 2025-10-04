package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class ExampleRobot extends Robot {

    @Override
    public void init() {
        super.init();
        //Do Specific Mode Init Here

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop(){
        movement.goForward();
    }
}
