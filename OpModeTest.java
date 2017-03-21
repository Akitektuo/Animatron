package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.cancelRequested;

@TeleOp(name="OpMode Test", group="OpMode")
public class OpModeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnimatronRobot robot = new AnimatronRobot(this, telemetry);
        waitForStart();
        robot.changeStatus("Start.");
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                robot.captureBall();
            } else if (gamepad1.left_trigger > 0) {
                robot.releaseBall();
            }
            if(gamepad1.dpad_right) {
                robot.releaseLift();
            }
        }
    }
}
