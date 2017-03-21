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
        waitForStart();
        robot.changeStatus("Start.");
        robot.motorMovementTurn(45, true);
        robot.motorMovementBackward(-0.5, 90);
        robot.motorBallFrontIn(1);
        robot.throwBalls(3000);
        robot.motorBallFrontStop();
        robot.motorMovementBackward(-0.3, 60);
    }
}
