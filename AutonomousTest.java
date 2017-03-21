package org.firstinspires.ftc.teamro028;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Akitektuo on 26-Jan-17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Test", group = "Autonomous")
public class AutonomousTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        waitForStart();
        robot.changeStatus("Start.");
        robot.motorMovementTurn(45, true);
        robot.motorMovementStop(2000);
        robot.motorMovementTurn(45, true);
        robot.motorStop();
    }
}
