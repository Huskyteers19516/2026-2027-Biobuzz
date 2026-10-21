package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "Frank Front Wheels TeleOp", group = "Test")
public class frontwheel extends LinearOpMode {



    private static final double NORMAL_POWER = 0.45;
    private static final double SLOW_POWER = 0.25;
    private static final double STEERING_GAIN = 0.75;
    private static final double DEADBAND = 0.05;


    private static final DcMotorSimple.Direction LEFT_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_DIRECTION =
            DcMotorSimple.Direction.REVERSE;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "left_front");
            frontRight = hardwareMap.get(DcMotor.class, "right_front");

            // Rear motors are optional in the hardware map.
            // When present, keep them unpowered and select FLOAT.
//            backLeft = hardwareMap.tryGet(DcMotor.class, "left_back");
//            backRight = hardwareMap.tryGet(DcMotor.class, "right_back");

            configureFront(frontLeft, LEFT_DIRECTION);
            configureFront(frontRight, RIGHT_DIRECTION);
            configureRear(backLeft);
            configureRear(backRight);

            telemetry.addLine("Left stick Y: forward / reverse");
            telemetry.addLine("Right stick X: steer while moving");
            telemetry.addLine("Hold left bumper: slow mode; hold B: stop");
            telemetry.addData("Rear left configured", backLeft != null);
            telemetry.addData("Rear right configured", backRight != null);
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double throttle = applyDeadband(-gamepad1.left_stick_y);
                double steering = applyDeadband(gamepad1.right_stick_x);
                double powerLimit = gamepad1.left_bumper ? SLOW_POWER : NORMAL_POWER;

                // Steering needs throttle, like a car. No stationary spin.
                // Reversing changes the turning direction, like backing a car.
                double turn = throttle * steering * STEERING_GAIN;
                double leftPower = throttle + turn;
                double rightPower = throttle - turn;

                double divisor = Math.max(1.0,
                        Math.max(Math.abs(leftPower), Math.abs(rightPower)));
                leftPower = leftPower / divisor * powerLimit;
                rightPower = rightPower / divisor * powerLimit;

                if (gamepad1.b) {
                    leftPower = 0.0;
                    rightPower = 0.0;
                }

                frontLeft.setPower(leftPower);
                frontRight.setPower(rightPower);

                telemetry.addData("Front left power", "%.2f", leftPower);
                telemetry.addData("Front right power", "%.2f", rightPower);
                telemetry.addData("Rear wheel command", "Zero power / FLOAT");
                telemetry.addData("Stop held", gamepad1.b);
                telemetry.update();
            }
        } finally {
            stopMotor(frontLeft);
            stopMotor(frontRight);
            stopMotor(backLeft);
            stopMotor(backRight);
        }
    }

    private void configureFront(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void configureRear(DcMotor motor) {
        if (motor == null) return;
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) <= DEADBAND) return 0.0;
        double scaled = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
        return Math.copySign(Range.clip(scaled, 0.0, 1.0), value);
    }

    private void stopMotor(DcMotor motor) {
        if (motor != null) motor.setPower(0.0);
    }
}

