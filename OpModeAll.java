package org.firstinspires.ftc.teamro028;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OpMode All", group = "OpMode")
public class OpModeAll extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        waitForStart();
        robot.changeStatus("Start.");
        while (opModeIsActive()) {
            //B
            if (gamepad1.b || gamepad2.b) {
                robot.motorBallFrontStop();
            }
            //Stick
            if (gamepad1.left_stick_button || gamepad2.left_stick_button) {
                robot.motorMovementLeft(-gamepad2.left_stick_y);
                robot.motorMovementRight(-gamepad2.right_stick_y);
                robot.baseForkMove(-gamepad1.right_stick_y);
            } else {
                robot.motorMovementLeft(-gamepad1.left_stick_y);
                robot.motorMovementRight(-gamepad1.right_stick_y);
                robot.baseForkMove(-gamepad2.right_stick_y);
            }
            //Triggers
            if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
                robot.motorBallFrontOut(0.5);
            } else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
                robot.motorBallFrontIn(0.5);
                //Left
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                robot.releaseBall();
            }
            //Bumpers
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                robot.servoExtendLeft();
            } else {
                robot.servoRetractLeft();
            }
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                robot.servoExtendRight();
            } else {
                robot.servoRetractRight();
            }
            //X
            if (gamepad2.x || gamepad1.x) {
                robot.ballThrow(1);
                robot.ballLift(1);
            } else {
                robot.ballLift(0);
                robot.ballThrow(0);
            }
            //Up
            if (gamepad2.dpad_up || gamepad1.dpad_up) {
                robot.searchWhiteLine(0.3);
            }
            //Down
            if (gamepad2.dpad_down || gamepad1.dpad_down) {
                robot.searchWhiteLine(-0.3);
            }
            //Y - A
            if (gamepad2.y || gamepad1.y) {
                robot.releaseLift();
                robot.liftBallUp();
            } else if (gamepad2.a || gamepad1.a) {
                robot.liftBallDown();
            } else {
                robot.liftBallStop();
            }
            //Right
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                robot.captureBall();
            }
            telemetry.addData("Distance", robot.getDistanceFromWall());
            telemetry.addData("Lift", robot.getStringEncoders());
            telemetry.update();
        }
    }
}
