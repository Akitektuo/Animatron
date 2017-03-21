package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Akitektuo on 24.02.2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Red 2", group = "Autonomous")
public class AR2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        waitForStart();
        robot.changeStatus("Start.");
        robot.motorMovementTurn(-45, false);
        robot.motorMovementBackward(-0.5, 85);
        robot.motorBallFrontIn(1);
        robot.throwBalls(2000);
        robot.motorBallFrontStop();
        robot.motorMovementBackward(-0.3, 60);
    }
}
