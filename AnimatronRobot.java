package org.firstinspires.ftc.teamro028;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamro028.Constants.MOTOR_BACKWARD_MINIMUM;
import static org.firstinspires.ftc.teamro028.Constants.MOTOR_FORWARD_MAXIMUM;
import static org.firstinspires.ftc.teamro028.Constants.MOTOR_NUMBER;
import static org.firstinspires.ftc.teamro028.Constants.MOTOR_STOP;
import static org.firstinspires.ftc.teamro028.Constants.NUMBER_MOTORS_MOVEMENT;
import static org.firstinspires.ftc.teamro028.Constants.SERVO_LEFT_MAXIMUM;
import static org.firstinspires.ftc.teamro028.Constants.SERVO_LEFT_MINIMUM;
import static org.firstinspires.ftc.teamro028.Constants.SERVO_NUMBER;
import static org.firstinspires.ftc.teamro028.Constants.SERVO_RIGHT_MAXIMUM;
import static org.firstinspires.ftc.teamro028.Constants.SERVO_RIGHT_MINIMUM;
import static org.firstinspires.ftc.teamro028.Constants.TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamro028.Constants.WHITE_LINE_CODE;
import static org.firstinspires.ftc.teamro028.Constants.cancelRequested;

/**
 * Created by Akitektuo on 05.02.2017.
 */

class AnimatronRobot {
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private DcMotor[] motors = new DcMotor[MOTOR_NUMBER];
    private Servo[] servos = new Servo[SERVO_NUMBER];
    private ModernRoboticsI2cGyro gyro;
    private OpticalDistanceSensor bottomSensor;
    private ColorSensor colorSensor;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private CRServoImpl continuousServo;

    AnimatronRobot(LinearOpMode opMode, Telemetry telemetry) throws InterruptedException {
        this.opMode = opMode;
        this.telemetry = telemetry;
        initialize();
    }

    void changeStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }

    void driveUntilLine() throws InterruptedException {
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
            motorMovementLeft(0.3);
            motorMovementRight(0.3);
        }
        Thread.sleep(50);
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
            motorMovementLeft(-0.1);
            motorMovementRight(-0.1);
        }
        motorMovementStop(500);
    }

    int getDistance(int cm) {
        return (int) (cm * TICKS_PER_ROTATION);
    }

    private void initialize() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro) opMode.hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        motors[0] = opMode.hardwareMap.dcMotor.get("motorMovementNW");
        motors[1] = opMode.hardwareMap.dcMotor.get("motorMovementNE");
        motors[2] = opMode.hardwareMap.dcMotor.get("motorMovementSW");
        motors[3] = opMode.hardwareMap.dcMotor.get("motorMovementSE");
        motors[4] = opMode.hardwareMap.dcMotor.get("motorFront");
        motors[5] = opMode.hardwareMap.dcMotor.get("motorLift");
        motors[6] = opMode.hardwareMap.dcMotor.get("motorThrow");
        motors[7] = opMode.hardwareMap.dcMotor.get("motorString");
        for (int i = 0; i < NUMBER_MOTORS_MOVEMENT; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        motors[7].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[4].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[5].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[7].setDirection(DcMotorSimple.Direction.REVERSE);
        motorStop();
        servos[0] = opMode.hardwareMap.servo.get("servoLeft");
        servos[1] = opMode.hardwareMap.servo.get("servoRight");
//        servos[2] = opMode.hardwareMap.servo.get("servoForkBase");
        continuousServo = (CRServoImpl) opMode.hardwareMap.crservo.get("servoForkBase");
        servos[2] = opMode.hardwareMap.servo.get("servoForkUp");
        servos[3] = opMode.hardwareMap.servo.get("servoLock");
        servos[0].setDirection(Servo.Direction.REVERSE);
        servos[2].setDirection(Servo.Direction.REVERSE);
        servos[3].setDirection(Servo.Direction.REVERSE);
        servos[2].setPosition(Servo.MIN_POSITION);
        servos[3].setPosition(Servo.MIN_POSITION);
        continuousServo.setPower(0);
        servoRetract();
        bottomSensor = opMode.hardwareMap.opticalDistanceSensor.get("sensorWhiteLine");
        colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(false);
        rangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        while (!opMode.isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            opMode.idle();
        }
        changeStatus("Initialized");
    }

    void motorBallFrontIn(double power) {
        motors[4].setPower(power);
    }

    void motorBallFrontOut(double power) {
        motors[4].setPower(-power);
    }

    void motorBallFrontStop() {
        motors[4].setPower(MOTOR_STOP);
    }

    void motorMovementBackward(double power, int cm) throws InterruptedException {
        if (power < -0.9) {
            power = -0.9;
        }
        double speedW, speedE, speedMin = -0.2, speedMax = power;
        double target = gyro.getIntegratedZValue();
        for (int i = 0; i < NUMBER_MOTORS_MOVEMENT; i++) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition(-getDistance(cm));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        double startPosition = motors[0].getCurrentPosition();
        int distanceHalf = -getDistance(cm) / 2, motorMaximum;
        while (motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy() && !opMode.isStopRequested()) {
            motorMaximum = Math.max(Math.max(motors[0].getCurrentPosition(), motors[1].getCurrentPosition()), Math.max(motors[2].getCurrentPosition(), motors[3].getCurrentPosition()));
            if (motorMaximum < distanceHalf) {
                power = (-getDistance(cm) - motorMaximum) * speedMax / distanceHalf;
                if (power > speedMin) {
                    power = speedMin;
                    changeStatus("IT WORKS!");
                }
            }

            int zAccumulated = gyro.getIntegratedZValue();
            speedW = power + (zAccumulated - target) / 100;
            speedE = power - (zAccumulated - target) / 100;
            speedW = Range.clip(speedW, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            speedE = Range.clip(speedE, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            motorMovementLeft(speedW);
            motorMovementRight(speedE);
        }
        for (int i = 0; i < NUMBER_MOTORS_MOVEMENT; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        motorMovementStop(50);
    }

    void motorMovementForward(double power, int cm) throws InterruptedException {
        if (power > 0.9) {
            power = 0.9;
        }
        double speedW, speedE, speedMin = 0.2, speedMax = power;
        double target = gyro.getIntegratedZValue();
        for (int i = 0; i < NUMBER_MOTORS_MOVEMENT; i++) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setTargetPosition(getDistance(cm));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        double startPosition = motors[0].getCurrentPosition();
        int distanceHalf = getDistance(cm) / 2, motorMinimum;
        while (motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy() && !opMode.isStopRequested()) {
            motorMinimum = Math.min(Math.min(motors[0].getCurrentPosition(), motors[1].getCurrentPosition()), Math.min(motors[2].getCurrentPosition(), motors[3].getCurrentPosition()));
            if (motorMinimum > distanceHalf) {
                power = (getDistance(cm) - motorMinimum) * speedMax / distanceHalf;
                if (power < speedMin) {
                    power = speedMin;
                }
            }

            int zAccumulated = gyro.getIntegratedZValue();
            speedW = power + (zAccumulated - target) / 100;
            speedE = power - (zAccumulated - target) / 100;
            speedW = Range.clip(speedW, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            speedE = Range.clip(speedE, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            motorMovementLeft(speedW);
            motorMovementRight(speedE);
            changeStatus("Motors encoders are at " + motorMinimum + " and going to " + (getDistance(cm) + startPosition));
        }
        for (int i = 0; i < NUMBER_MOTORS_MOVEMENT; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        motorMovementStop(500);
    }

    void motorMovementLeft(double power) {
        motors[0].setPower(power);
        motors[2].setPower(power);
    }

    void motorMovementRight(double power) {
        motors[1].setPower(power);
        motors[3].setPower(power);
    }

    void motorMovementStop() {
        motorMovementLeft(MOTOR_STOP);
        motorMovementRight(MOTOR_STOP);
        motors[4].setPower(MOTOR_STOP);
    }

    void motorMovementStop(long time) throws InterruptedException {
        motorMovementRight(MOTOR_STOP);
        motorMovementLeft(MOTOR_STOP);
        Thread.sleep(time);
    }

    void motorMovementTurn(int degrees, boolean rightSide) throws InterruptedException {
        double kP = 0.0125, power, minimumPower = 0.05;
        gyro.resetZAxisIntegrator();
        int heading = gyro.getIntegratedZValue();
        degrees = -degrees;
        while (heading != degrees && !opMode.isStopRequested()) {
            power = (degrees - heading) * kP;
            if (power < 0 && power > -minimumPower) {
                power = -minimumPower;
            } else if (power > 0 && power < minimumPower) {
                power = minimumPower;
            }
            if (rightSide) {
                motorMovementRight(power);
            } else {
                motorMovementLeft(-power);
            }
            changeStatus("Target: " + degrees + "; Gyro position is " + heading + " and the speed is " + power * 100 + ".");
            heading = gyro.getIntegratedZValue();
        }
        motorMovementStop(500);
    }

    void motorMovementTurn(int degrees) throws InterruptedException {
        double kP = 0.015, power, minimumPower = 0.05;
        gyro.resetZAxisIntegrator();
        int heading = gyro.getIntegratedZValue();
        degrees = -degrees;
        while (abs(heading - degrees) > 0 && !opMode.isStopRequested()) {
            power = (degrees - heading) * kP;
            if (power < 0 && power > -minimumPower) {
                power = -minimumPower;
            } else if (power > 0 && power < minimumPower) {
                power = minimumPower;
            }
            motorMovementRight(power);
            motorMovementLeft(-power);
            changeStatus("Target: " + degrees + "; Gyro position is " + heading + " and the speed is " + power * 100 + ".");
            heading = gyro.getIntegratedZValue();
        }
        motorMovementStop(50);
    }

    void motorStop() {
        for (int i = 0; i < MOTOR_NUMBER; i++) {
            motors[i].setPower(MOTOR_STOP);
        }
    }

    void servoExtendLeft() {
        servos[0].setPosition(SERVO_LEFT_MAXIMUM);
    }

    void servoExtendRight() {
        servos[1].setPosition(SERVO_RIGHT_MAXIMUM);
    }

    void servoRetract() {
        servoRetractLeft();
        servoRetractRight();
    }

    void servoRetractLeft() {
        servos[0].setPosition(SERVO_LEFT_MINIMUM);
    }

    void servoRetractRight() {
        servos[1].setPosition(SERVO_RIGHT_MINIMUM);
    }

//    void testingSensors() throws InterruptedException {
//        double readingFront, readingRear, optimalReadingFront = 2.30, optimalReadingRear = 1.21, speedLeft, speedRight;
//        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
//            speedLeft = 0.3;
//            speedRight = 0.3;
//            readingFront = rangeSensor[0].getVoltage();
//            readingRear = rangeSensor[1].getVoltage();
//            if (abs(optimalReadingFront - readingFront) > abs(optimalReadingRear - readingRear)) {
//                speedLeft += (optimalReadingFront - readingFront) / 7;
//                speedRight -= (optimalReadingFront - readingFront) / 7;
//            } else {
//                speedLeft -= (optimalReadingRear - readingRear) / 7;
//                speedRight += (optimalReadingRear - readingRear) / 7;
//            }
//            motorMovementLeft(speedLeft);
//            motorMovementRight(speedRight);
//            changeStatus("Left is " + rangeSensor[0].getVoltage() + "\nRight is " + rangeSensor[1].getVoltage());
//        }
//        Thread.sleep(50);
//        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
//            motorMovementLeft(-0.1);
//            motorMovementRight(-0.1);
//        }
//        motorMovementStop(500);
//    }

    void useServoForColor(boolean redTeam) throws InterruptedException {
        if (redTeam && colorSensor.red() > colorSensor.blue()) {
            servoExtendLeft();
        } else if (redTeam && colorSensor.red() < colorSensor.blue()) {
            servoExtendRight();
        } else if (!redTeam && colorSensor.blue() > colorSensor.red()) {
            servoExtendLeft();
        } else {
            servoExtendRight();
        }
        Thread.sleep(750);
        servoRetract();
    }

    void doBeaconFront(double power, boolean redTeam) throws InterruptedException {
        if (power > 0.9) {
            power = 0.9;
        }
        double speedW, speedE;
        double target = gyro.getIntegratedZValue();
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
            int zAccumulated = gyro.getIntegratedZValue();
            speedW = power + (zAccumulated - target) / 100;
            speedE = power - (zAccumulated - target) / 100;
            speedW = Range.clip(speedW, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            speedE = Range.clip(speedE, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            motorMovementLeft(speedW);
            motorMovementRight(speedE);
        }
        motorMovementStop(50);
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
            motorMovementLeft(-0.1);
            motorMovementRight(-0.1);
        }
        motorMovementStop(100);
        useServoForColor(redTeam);
    }

    void doBeaconBack(double power, boolean redTeam) throws InterruptedException {
        if (power < -0.9) {
            power = -0.9;
        }
        double speedW, speedE;
        double target = gyro.getIntegratedZValue();
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
            int zAccumulated = gyro.getIntegratedZValue();
            speedW = power + (zAccumulated - target) / 100;
            speedE = power - (zAccumulated - target) / 100;
            speedW = Range.clip(speedW, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            speedE = Range.clip(speedE, MOTOR_BACKWARD_MINIMUM, MOTOR_FORWARD_MAXIMUM);
            motorMovementLeft(speedW);
            motorMovementRight(speedE);
        }
        motorMovementStop(50);
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
            motorMovementLeft(0.1);
            motorMovementRight(0.1);
        }
        motorMovementStop(100);
        useServoForColor(redTeam);
    }

    String getDistanceFromWall() {
        return "Distance from wall is " + rangeSensor.getDistance(DistanceUnit.CM);
    }

    void searchWhiteLine(double power, boolean redTeam) throws InterruptedException {
        int modifier = 1, target = 12;
        double newPowerLeft, newPowerRight;
        if (power > 0 && power > 0.9) {
            power = 0.9;
        } else if (power < 0 && power < -0.9) {
            power = -0.9;
        }
        if (power < 0) {
            modifier = -modifier;
        }
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
//            newPowerLeft = power - (target - rangeSensor.getDistance(DistanceUnit.CM)) / 50 * modifier;
//            newPowerRight = power + (target - rangeSensor.getDistance(DistanceUnit.CM)) / 50 * modifier;
            motorMovementLeft(power);
            motorMovementRight(power);
//            changeStatus(newPowerLeft + " " + newPowerRight);
        }
        motorMovementStop(50);
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested()) {
            motorMovementLeft(-0.1 * modifier);
            motorMovementRight(-0.1 * modifier);
        }
        motorMovementStop(100);
        useServoForColor(redTeam);
    }

    void searchWhiteLine(double power) throws InterruptedException {
        int modifier = 1;
        if (power > 0 && power > 0.9) {
            power = 0.9;
        } else if (power < 0 && power < -0.9) {
            power = -0.9;
        }
        if (power < 0) {
            modifier = -modifier;
        }
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested() && !cancelRequested) {
//            newPowerLeft = power - (target - rangeSensor.getDistance(DistanceUnit.CM)) / 50 * modifier;
//            newPowerRight = power + (target - rangeSensor.getDistance(DistanceUnit.CM)) / 50 * modifier;
            motorMovementLeft(power);
            motorMovementRight(power);
//            changeStatus(newPowerLeft + " " + newPowerRight);
        }
        motorMovementStop(50);
        while (bottomSensor.getLightDetected() < WHITE_LINE_CODE && !opMode.isStopRequested() && !cancelRequested) {
            motorMovementLeft(-0.1 * modifier);
            motorMovementRight(-0.1 * modifier);
        }
    }

    void ballLift(double power) {
        motors[5].setPower(power);
    }

    void ballThrow(double power) {
        motors[6].setPower(power);
    }

    void throwBalls(long time) throws InterruptedException {
        motors[6].setPower(1);
        motors[5].setPower(1);
        sleep(time);
        motors[5].setPower(0);
        motors[6].setPower(0);
    }

    void liftBallUp() {
        motors[7].setPower(0.7);
    }

    void liftBallDown() {
        motors[7].setPower(-0.7);
    }

    void liftBallStop() {
        motors[7].setPower(0);
    }

    String getStringEncoders() {
        return "Encoders are at " + (motors[7].getCurrentPosition() / 99) + "%";
    }

    void testServo() {
        changeStatus("Servo position " + servos[2].getPosition());
    }

    void captureBall() {
        servos[2].setPosition(Servo.MAX_POSITION);
    }

    void releaseBall() {
        servos[2].setPosition(Servo.MIN_POSITION);
    }

    void releaseLift() {
        servos[3].setPosition(Servo.MAX_POSITION);
    }

    void baseForkLift() {
        continuousServo.setPower(1);
    }

    void baseForkRelease() {
        continuousServo.setPower(-1);
    }

    void baseForkStop() {
        continuousServo.setPower(0);
    }

    void baseForkMove(double power) {
        continuousServo.setPower(power);
    }
}
