package org.firstinspires.ftc.teamro028;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamro028.Constants.cancelRequested;

@TeleOp(name="Linear OpMode", group="OpMode")
public class OpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        waitForStart();
        robot.changeStatus("Start.");
        while (opModeIsActive()) {
            robot.motorMovementLeft(-gamepad1.left_stick_y);
            robot.motorMovementRight(-gamepad1.right_stick_y);
            if (gamepad1.left_trigger > 0) {
                robot.motorBallFrontOut(0.5);
            } else if (gamepad1.right_trigger > 0) {
                robot.motorBallFrontIn(0.5);
            } else {
                robot.motorBallFrontStop();
            }
            if (gamepad2.left_bumper) {
                robot.servoExtendLeft();
            } else {
                robot.servoRetractLeft();
            }
            if (gamepad2.right_bumper) {
                robot.servoExtendRight();
            } else {
                robot.servoRetractRight();
            }
            if (gamepad2.x) {
                robot.ballThrow(1);
                robot.ballLift(1);
            } else {
                robot.ballLift(0);
                robot.ballThrow(0);
            }
            if (gamepad2.dpad_up) {
                robot.searchWhiteLine(0.3);
            }
            if (gamepad2.dpad_down) {
                robot.searchWhiteLine(-0.3);
            }
            if (gamepad2.y) {
                robot.liftBallUp();
            } else if (gamepad2.a) {
                robot.liftBallDown();
            } else {
                robot.liftBallStop();
            }
            cancelRequested = gamepad2.dpad_left;
            telemetry.addData("Distance", robot.getDistanceFromWall());
            telemetry.addData("Lift", robot.getStringEncoders());
            telemetry.update();
        }
    }
}
