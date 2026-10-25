package org.firstinspires.ftc.teamtestcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Car TeleOp", group = "Testing")
public class CarTeleOp extends OpMode {

    private static final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private static final double MAX_POWER = 1.0;
    private static final double SLOW_POWER = 0.35;
    private static final double STEERING_GAIN = 1.0;
    private static final double DEADZONE = 0.05;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
        configureDriveMotor(frontLeft, LEFT_DIRECTION);
        configureDriveMotor(frontRight, RIGHT_DIRECTION);

        backLeft = hardwareMap.tryGet(DcMotor.class, BACK_LEFT_MOTOR_NAME);
        backRight = hardwareMap.tryGet(DcMotor.class, BACK_RIGHT_MOTOR_NAME);
        configureFreeWheel(backLeft);
        configureFreeWheel(backRight);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("RT = gas   LT = reverse   left stick X = steer   RB = slow");
    }

    @Override
    public void loop() {
        double throttle = gamepad1.right_trigger - gamepad1.left_trigger;
        double steer = deadzone(gamepad1.left_stick_x);
        double limit = gamepad1.right_bumper ? SLOW_POWER : MAX_POWER;

        double turn = throttle * steer * STEERING_GAIN;
        double leftPower = throttle + turn;
        double rightPower = throttle - turn;
        double max = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower = leftPower / max * limit;
        rightPower = rightPower / max * limit;

        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);

        telemetry.addData("Throttle", "%.2f", throttle);
        telemetry.addData("Steer", "%.2f", steer);
        telemetry.addData("Left / Right", "%.2f / %.2f", leftPower, rightPower);
        telemetry.addData("Slow mode", gamepad1.right_bumper ? "ON" : "OFF");
        telemetry.addData("Back wheels", backLeft != null && backRight != null ? "free rolling" : "not in config");
        telemetry.addLine("RT = gas   LT = reverse   left stick X = steer   RB = slow");
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    private void configureDriveMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    private void configureFreeWheel(DcMotor motor) {
        if (motor == null) {
            return;
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    private static double deadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
