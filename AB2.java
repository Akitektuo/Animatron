package org.firstinspires.ftc.teamro028;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Akitektuo on 24.02.2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Blue 2", group = "Autonomous")
public class AB2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        robot.motorMovementTurn(45, true);
        robot.motorMovementStop(10000);
        robot.motorMovementBackward(-0.5, 75);
        robot.motorBallFrontIn(1);
        robot.throwBalls(4000);
        robot.motorBallFrontStop();
        robot.motorMovementBackward(-0.3, 60);
    }
}
