package org.firstinspires.ftc.teamtestcode.swervedrivetesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Single Swerve Module", group = "Testing")
public class SingleSwerveTeleOp extends OpMode {

    private static final double SERVO_TRAVEL_DEGREES = 270.0;
    private static final double SERVO_CENTER = 0.5;
    private static final boolean SERVO_REVERSED = false;
    private static final boolean DRIVE_REVERSED = false;

    private static final double STICK_DEADZONE = 0.15;
    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double TRIM_STEP = 0.002;

    private static final double SERVO_SLEW_SECONDS = 0.25;
    private static final double FOLD_HYSTERESIS_DEGREES = 15.0;

    private static final boolean USE_EXTENDED_PWM = false;
    private static final double PWM_LOWER_MICROSECONDS = 500.0;
    private static final double PWM_UPPER_MICROSECONDS = 2500.0;

    private DcMotor driveMotor;
    private Servo steerServo;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double centerTrim = SERVO_CENTER;
    private double commandedAngle = 0.0;
    private double estimatedAngle = 0.0;
    private double commandedPosition = SERVO_CENTER;
    private boolean driveReversedState = false;
    private boolean positionClipped = false;
    private boolean dpadWasPressed = false;
    private boolean extendedPwmApplied = false;

    @Override
    public void init() {
        driveMotor = hardwareMap.get(DcMotor.class, "swerve_motor");
        steerServo = hardwareMap.get(Servo.class, "swerve_servo");

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setDirection(DRIVE_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        steerServo.setDirection(Servo.Direction.FORWARD);

        if (USE_EXTENDED_PWM && steerServo instanceof ServoImplEx) {
            ((ServoImplEx) steerServo).setPwmRange(
                    new PwmControl.PwmRange(PWM_LOWER_MICROSECONDS, PWM_UPPER_MICROSECONDS));
            extendedPwmApplied = true;
        }

        commandedAngle = 0.0;
        estimatedAngle = 0.0;
        driveReversedState = false;
        commandedPosition = positionForAngle(0.0);
        steerServo.setPosition(commandedPosition);
        driveMotor.setPower(0.0);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Single Swerve Module");
        telemetry.addData("Servo position", "%.3f", commandedPosition);
        telemetry.addData("Center trim", "%.3f", centerTrim);
        telemetry.addData("Reachable", "+%.0f / -%.0f deg",
                reachablePositiveDegrees(), reachableNegativeDegrees());
        telemetry.addData("Extended PWM", extendedPwmApplied ? "on" : "off");
        telemetry.addLine("Press START, then hold A to calibrate.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        commandedAngle = 0.0;
        estimatedAngle = 0.0;
        driveReversedState = false;
        commandedPosition = positionForAngle(0.0);
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (gamepad1.a) {
            handleTrim();

            commandedAngle = 0.0;
            driveReversedState = false;
            commandedPosition = positionForAngle(0.0);
            advanceEstimate(dt);
            setOutputs(commandedPosition, 0.0);

            telemetry.addLine(">>> CALIBRATION MODE <<<");
            telemetry.addLine("Wheel is commanded straight forward.");
            telemetry.addData("Servo position", "%.3f", commandedPosition);
            telemetry.addData("Center trim", "%.3f", centerTrim);
            telemetry.addData("Reachable", "+%.0f / -%.0f deg",
                    reachablePositiveDegrees(), reachableNegativeDegrees());
            telemetry.addLine("dpad left/right trims the center.");
            telemetry.addLine("Copy Center trim into SERVO_CENTER.");
            telemetry.update();
            return;
        }

        dpadWasPressed = false;

        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;
        double magnitude = Range.clip(Math.hypot(stickX, stickY), 0.0, 1.0);

        if (magnitude < STICK_DEADZONE) {
            advanceEstimate(dt);
            setOutputs(commandedPosition, 0.0);

            telemetry.addData("Stick", "released");
            telemetry.addData("Held angle", "%.1f deg", commandedAngle);
            telemetry.addData("Servo position", "%.3f", commandedPosition);
            telemetry.addData("Drive power", "%.2f", 0.0);
            telemetry.update();
            return;
        }

        double rawAngle = normalize(Math.toDegrees(Math.atan2(stickX, stickY)));
        double foldedAngle = normalize(rawAngle + 180.0);
        double foldLimit = 90.0 + FOLD_HYSTERESIS_DEGREES;

        double targetAngle;
        boolean reversed;

        if (driveReversedState && Math.abs(foldedAngle) <= foldLimit) {
            targetAngle = foldedAngle;
            reversed = true;
        } else if (!driveReversedState && Math.abs(rawAngle) <= foldLimit) {
            targetAngle = rawAngle;
            reversed = false;
        } else if (Math.abs(rawAngle) <= 90.0) {
            targetAngle = rawAngle;
            reversed = false;
        } else {
            targetAngle = foldedAngle;
            reversed = true;
        }

        driveReversedState = reversed;
        commandedAngle = Range.clip(targetAngle, -90.0, 90.0);
        commandedPosition = positionForAngle(commandedAngle);

        advanceEstimate(dt);

        double ramp = Range.clip(
                1.0 - Math.abs(commandedAngle - estimatedAngle) / 90.0, 0.0, 1.0);

        double drivePower = magnitude * DRIVE_POWER_SCALE * ramp;
        if (reversed) {
            drivePower = -drivePower;
        }

        setOutputs(commandedPosition, drivePower);

        telemetry.addData("Stick", "x %.2f  y %.2f", stickX, stickY);
        telemetry.addData("Target angle", "%.1f deg", commandedAngle);
        telemetry.addData("Servo position", "%.3f", commandedPosition);
        telemetry.addData("Center trim", "%.3f", centerTrim);
        telemetry.addData("Ramp", "%.2f", ramp);
        telemetry.addData("Drive power", "%.2f%s", drivePower, reversed ? "  (reversed)" : "");

        if (positionClipped) {
            telemetry.addLine("!! OUT OF SERVO RANGE - angle not reachable");
            telemetry.addData("Reachable", "+%.0f / -%.0f deg",
                    reachablePositiveDegrees(), reachableNegativeDegrees());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        driveMotor.setPower(0.0);
    }

    private void setOutputs(double servoPosition, double drivePower) {
        steerServo.setPosition(Range.clip(servoPosition, 0.0, 1.0));
        driveMotor.setPower(Range.clip(drivePower, -1.0, 1.0));
    }

    private void advanceEstimate(double seconds) {
        double maxStep = (90.0 / SERVO_SLEW_SECONDS) * Range.clip(seconds, 0.0, 1.0);
        double delta = commandedAngle - estimatedAngle;
        estimatedAngle += Range.clip(delta, -maxStep, maxStep);
    }

    private double positionForAngle(double angleDegrees) {
        double signedAngle = SERVO_REVERSED ? -angleDegrees : angleDegrees;
        double raw = centerTrim + signedAngle / SERVO_TRAVEL_DEGREES;
        positionClipped = raw < 0.0 || raw > 1.0;
        return Range.clip(raw, 0.0, 1.0);
    }

    private double reachablePositiveDegrees() {
        return (SERVO_REVERSED ? centerTrim : 1.0 - centerTrim) * SERVO_TRAVEL_DEGREES;
    }

    private double reachableNegativeDegrees() {
        return (SERVO_REVERSED ? 1.0 - centerTrim : centerTrim) * SERVO_TRAVEL_DEGREES;
    }

    private double normalize(double degrees) {
        degrees %= 360.0;

        if (degrees > 180.0) {
            degrees -= 360.0;
        } else if (degrees < -180.0) {
            degrees += 360.0;
        }

        return degrees;
    }

    private void handleTrim() {
        boolean pressed = gamepad1.dpad_left || gamepad1.dpad_right;

        if (pressed && !dpadWasPressed) {
            centerTrim = Range.clip(centerTrim
                    + (gamepad1.dpad_right ? TRIM_STEP : -TRIM_STEP), 0.0, 1.0);
        }

        dpadWasPressed = pressed;
    }
}
