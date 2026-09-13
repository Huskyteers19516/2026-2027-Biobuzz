package org.firstinspires.ftc.teamtestcode.swervedrivetesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Single Swerve Module", group = "Testing")
public class SingleSwerveTeleOp extends OpMode {

    private static final double ENCODER_OFFSET_DEGREES = 0.0;
    private static final boolean STEER_REVERSED = false;
    private static final boolean DRIVE_REVERSED = false;

    private static final double STEER_KP = 0.012;
    private static final double STEER_KD = 0.0006;
    private static final double STEER_KS = 0.05;
    private static final double STEER_TOLERANCE_DEGREES = 2.0;
    private static final double STEER_MAX_POWER = 1.0;

    private static final double STICK_DEADZONE = 0.15;
    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double TRIM_STEP_DEGREES = 0.5;

    private DcMotor driveMotor;
    private CRServo steerServo;
    private AnalogInput steerEncoder;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double heldAngle = 0.0;
    private double offsetTrim = ENCODER_OFFSET_DEGREES;
    private double previousError = 0.0;
    private boolean dpadWasPressed = false;

    @Override
    public void init() {
        driveMotor = hardwareMap.get(DcMotor.class, "swerve_motor");
        steerServo = hardwareMap.get(CRServo.class, "swerve_servo");
        steerEncoder = hardwareMap.get(AnalogInput.class, "swerve_encoder");

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setDirection(DRIVE_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        steerServo.setDirection(STEER_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        heldAngle = readModuleAngle();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Single Swerve Module");
        telemetry.addData("Module angle", "%.1f deg", readModuleAngle());
        telemetry.addData("Raw encoder", "%.1f deg", readRawAngle());
        telemetry.addLine("Left stick: point and drive. Hold A: calibrate.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        previousError = 0.0;
        heldAngle = readModuleAngle();
    }

    @Override
    public void loop() {
        double moduleAngle = readModuleAngle();

        if (gamepad1.a) {
            handleTrim();
            setOutputs(0.0, 0.0);
            heldAngle = moduleAngle;
            previousError = 0.0;
            loopTimer.reset();

            telemetry.addLine(">>> CALIBRATION MODE <<<");
            telemetry.addLine("Point the wheel straight forward by hand.");
            telemetry.addData("Raw encoder", "%.1f deg", readRawAngle());
            telemetry.addData("Offset", "%.1f deg", offsetTrim);
            telemetry.addData("Module angle", "%.1f deg", moduleAngle);
            telemetry.addLine("dpad left/right trims the offset.");
            telemetry.addLine("Copy Raw encoder into ENCODER_OFFSET_DEGREES.");
            telemetry.update();
            return;
        }

        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;
        double magnitude = Range.clip(Math.hypot(stickX, stickY), 0.0, 1.0);

        if (magnitude < STICK_DEADZONE) {
            setOutputs(0.0, 0.0);
            previousError = 0.0;
            loopTimer.reset();

            telemetry.addData("Stick", "released");
            telemetry.addData("Module angle", "%.1f deg", moduleAngle);
            telemetry.addData("Held angle", "%.1f deg", heldAngle);
            telemetry.update();
            return;
        }

        double targetAngle = Math.toDegrees(Math.atan2(stickX, stickY));
        boolean reversed = false;

        if (Math.abs(normalize(targetAngle - moduleAngle)) > 90.0) {
            targetAngle = normalize(targetAngle + 180.0);
            reversed = true;
        }

        heldAngle = targetAngle;

        double error = normalize(targetAngle - moduleAngle);
        double steerPower = computeSteerPower(error);

        double drivePower = magnitude * DRIVE_POWER_SCALE * Math.cos(Math.toRadians(error));
        if (reversed) {
            drivePower = -drivePower;
        }

        setOutputs(steerPower, drivePower);

        telemetry.addData("Stick", "x %.2f  y %.2f", stickX, stickY);
        telemetry.addData("Module angle", "%.1f deg", moduleAngle);
        telemetry.addData("Target angle", "%.1f deg", targetAngle);
        telemetry.addData("Error", "%.1f deg", error);
        telemetry.addData("Steer power", "%.2f", steerPower);
        telemetry.addData("Drive power", "%.2f%s", drivePower, reversed ? "  (reversed)" : "");
        telemetry.update();
    }

    @Override
    public void stop() {
        setOutputs(0.0, 0.0);
    }

    private void setOutputs(double steerPower, double drivePower) {
        steerServo.setPower(steerPower);
        driveMotor.setPower(Range.clip(drivePower, -1.0, 1.0));
    }

    private double readRawAngle() {
        return steerEncoder.getVoltage() / steerEncoder.getMaxVoltage() * 360.0;
    }

    private double readModuleAngle() {
        return normalize(readRawAngle() - offsetTrim);
    }

    private double computeSteerPower(double error) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double derivative = dt > 1e-4 ? (error - previousError) / dt : 0.0;
        previousError = error;

        if (Math.abs(error) < STEER_TOLERANCE_DEGREES) {
            return 0.0;
        }

        double power = STEER_KP * error
                + STEER_KD * derivative
                + STEER_KS * Math.signum(error);

        return Range.clip(power, -STEER_MAX_POWER, STEER_MAX_POWER);
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
            offsetTrim = normalize(offsetTrim
                    + (gamepad1.dpad_right ? TRIM_STEP_DEGREES : -TRIM_STEP_DEGREES));
        }

        dpadWasPressed = pressed;
    }
}
