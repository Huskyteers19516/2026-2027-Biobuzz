package org.firstinspires.ftc.teamtestcode.autodesign;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Independent, robot-centric mecanum TeleOp.
 * Hardware names and motor directions follow the team's supplied configuration.
 * This OpMode does not run Pedro or require a Pinpoint localizer.
 */
@TeleOp(name = "Frank Drive Control", group = "Test")
public class FrankDriveControl extends LinearOpMode {

    private static final double NORMAL_SPEED = 0.50;
    private static final double SLOW_SPEED = 0.25;
    private static final double STICK_DEADBAND = 0.05;
    private static final double POWER_CHANGE_PER_SECOND = 2.0;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double appliedFrontLeft;
    private double appliedFrontRight;
    private double appliedBackLeft;
    private double appliedBackRight;
    private boolean emergencyStopped;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initializeDrive();

            telemetry.addLine("Left stick: forward / strafe");
            telemetry.addLine("Right stick X: turn");
            telemetry.addLine("Hold left bumper: slow mode");
            telemetry.addLine("B: latch stop; center sticks and press A to reset");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            ElapsedTime cycleTimer = new ElapsedTime();
            cycleTimer.reset();

            while (opModeIsActive()) {
                double dt = Range.clip(cycleTimer.seconds(), 0.0, 0.10);
                cycleTimer.reset();

                double forward = applyDeadband(-gamepad1.left_stick_y);
                double strafe = applyDeadband(gamepad1.left_stick_x);
                double turn = applyDeadband(gamepad1.right_stick_x);
                double speed = gamepad1.left_bumper ? SLOW_SPEED : NORMAL_SPEED;

                // B has priority over A, including when both are held.
                if (gamepad1.b) {
                    emergencyStopped = true;
                } else if (gamepad1.a && sticksCentered()) {
                    emergencyStopped = false;
                }

                if (emergencyStopped) {
                    stopDrive();
                } else {
                    drive(forward, strafe, turn, speed, dt);
                }

                displayStatus(speed);
                idle();
            }
        } finally {

            stopDrive();
        }
    }

    private void initializeDrive() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        configureMotor(frontLeft, DcMotorSimple.Direction.FORWARD);
        configureMotor(backLeft, DcMotorSimple.Direction.FORWARD);
        configureMotor(frontRight, DcMotorSimple.Direction.REVERSE);
        configureMotor(backRight, DcMotorSimple.Direction.REVERSE);

        emergencyStopped = false;
        stopDrive();
    }

    private void configureMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
    }

    private void drive(double forward, double strafe, double turn, double speed, double dt) {
        double frontLeftTarget = forward + strafe + turn;
        double backLeftTarget = forward - strafe + turn;
        double frontRightTarget = forward - strafe - turn;
        double backRightTarget = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftTarget), Math.abs(backLeftTarget)),
                Math.max(Math.abs(frontRightTarget), Math.abs(backRightTarget))));

        double maxChange = POWER_CHANGE_PER_SECOND * dt;
        appliedFrontLeft = rampToward(appliedFrontLeft, frontLeftTarget / max * speed, maxChange);
        appliedBackLeft = rampToward(appliedBackLeft, backLeftTarget / max * speed, maxChange);
        appliedFrontRight = rampToward(appliedFrontRight, frontRightTarget / max * speed, maxChange);
        appliedBackRight = rampToward(appliedBackRight, backRightTarget / max * speed, maxChange);

        frontLeft.setPower(appliedFrontLeft);
        backLeft.setPower(appliedBackLeft);
        frontRight.setPower(appliedFrontRight);
        backRight.setPower(appliedBackRight);
    }

    private double rampToward(double current, double target, double maxChange) {
        return current + Range.clip(target - current, -maxChange, maxChange);
    }

    private void stopDrive() {
        appliedFrontLeft = 0.0;
        appliedFrontRight = 0.0;
        appliedBackLeft = 0.0;
        appliedBackRight = 0.0;
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    private boolean sticksCentered() {
        return applyDeadband(gamepad1.left_stick_x) == 0.0
                && applyDeadband(gamepad1.left_stick_y) == 0.0
                && applyDeadband(gamepad1.right_stick_x) == 0.0;
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) <= STICK_DEADBAND) return 0.0;
        double scaled = (Math.abs(value) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
        return Math.copySign(Range.clip(scaled, 0.0, 1.0), value);
    }

    private void displayStatus(double speed) {
        telemetry.addData("Emergency stop", emergencyStopped ? "LATCHED - center sticks, press A" : "off");
        telemetry.addData("Speed limit", "%.2f", speed);
        telemetry.addData("Front L / R", "%.2f / %.2f", appliedFrontLeft, appliedFrontRight);
        telemetry.addData("Back  L / R", "%.2f / %.2f", appliedBackLeft, appliedBackRight);
        telemetry.update();
    }
}
