package org.firstinspires.ftc.teamro028;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Akitektuo on 26-Jan-17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Blue 1 All", group = "Autonomous")
public class AB1All extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        waitForStart();
        robot.changeStatus("Start.");
        robot.motorMovementForward(0.3, 30);
        robot.motorMovementTurn(68, false);
        robot.motorMovementForward(0.7, 80);
        robot.changeStatus("Going 90 cm");
        robot.motorMovementTurn(-70, true);
        robot.searchWhiteLine(0.3, false);
        robot.motorMovementForward(0.5, 95);
        robot.searchWhiteLine(0.3, false);
        robot.motorMovementTurn(50, true);
        robot.motorMovementBackward(-0.5, 77);
        robot.motorBallFrontIn(1);
        robot.throwBalls(2000);
        robot.motorBallFrontStop();
        robot.motorMovementBackward(-0.5, 65);
    }
}
