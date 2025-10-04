package org.firstinspires.ftc.teamcode.Robot;

public class Movement {
    private final Robot robot;

    // Constructor method to make robot available
    public Movement(Robot robotParam){
        robot = robotParam;
    }

    public void goForward(){
        robot.leftFrontDrive.setPower(100);
    }


}
