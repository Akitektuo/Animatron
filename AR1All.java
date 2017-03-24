package org.firstinspires.ftc.teamro028;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Akitektuo on 26-Jan-17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Red 1 All", group = "Autonomous")
public class AR1All extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        robot.motorMovementBackward(-0.5, 67);
        robot.motorBallFrontIn(1);
        robot.throwBalls(2000);
        robot.motorBallFrontStop();
        robot.motorMovementTurn(-68, false);
        robot.motorMovementBackward(-0.5, 73);
        robot.motorMovementTurn(68, true);
        robot.searchWhiteLine(0.3, true);
        robot.motorMovementBackward(-0.5, 90);
        robot.searchWhiteLine(-0.3, true);
        robot.motorMovementTurn(-50, true);
        robot.motorMovementForward(0.7, 130);
    }
}
