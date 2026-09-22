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
    private static final boolean ENCODER_REVERSED = false;
    private static final boolean STEER_REVERSED = false;
    private static final boolean DRIVE_REVERSED = false;

    private static final double ANALOG_MIN_VOLTAGE = 0.0;
    private static final double ANALOG_MAX_VOLTAGE = 3.3;

    private static final double STEER_KP = 0.012;
    private static final double STEER_KD = 0.0006;
    private static final double STEER_KS = 0.05;
    private static final double STEER_TOLERANCE_DEGREES = 2.0;
    private static final double STEER_MAX_POWER = 1.0;

    private static final double STICK_DEADZONE = 0.15;
    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double TRIM_STEP_DEGREES = 0.5;

    private static final double FOLD_HYSTERESIS_DEGREES = 15.0;
    private static final boolean HOLD_ANGLE_ON_RELEASE = true;

    private static final double STALL_POWER_THRESHOLD = 0.25;
    private static final double STALL_TIME_SECONDS = 2.0;
    private static final double STALL_MOVE_DEGREES = 2.0;

    private DcMotor driveMotor;
    private CRServo steerServo;
    private AnalogInput steerEncoder;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();

    private double offsetTrim = ENCODER_OFFSET_DEGREES;
    private double heldAngle = 0.0;
    private double previousAngle = 0.0;
    private boolean driveReversedState = false;
    private boolean dpadWasPressed = false;

    private double lastVoltage = 0.0;
    private double lastRawAngle = 0.0;
    private double lastModuleAngle = 0.0;
    private double minVoltageSeen = Double.MAX_VALUE;
    private double maxVoltageSeen = -Double.MAX_VALUE;

    private boolean steerStalled = false;
    private double stallTargetAngle = 0.0;
    private double stallBestError = 0.0;

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

        setOutputs(0.0, 0.0);

        sampleEncoder();
        heldAngle = lastModuleAngle;
        previousAngle = lastModuleAngle;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;
    }

    @Override
    public void init_loop() {
        sampleEncoder();

        telemetry.addLine("Single Swerve Module");
        telemetry.addLine("Axon MINI in CR mode + analog feedback");
        telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
        telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.addData("Analog range", "%.2f - %.2f V (hub max %.2f V)",
                ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE, steerEncoder.getMaxVoltage());
        telemetry.addData("Encoder reversed", ENCODER_REVERSED ? "true" : "false");
        telemetry.addLine("Turn the wheel by hand now - Raw encoder must change.");
        telemetry.addLine("Left stick: point and drive. Hold A: calibrate.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        stallTimer.reset();
        steerStalled = false;
        driveReversedState = false;

        sampleEncoder();
        heldAngle = lastModuleAngle;
        previousAngle = lastModuleAngle;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;
    }

    @Override
    public void loop() {
        sampleEncoder();
        double moduleAngle = lastModuleAngle;

        if (gamepad1.a) {
            handleTrim();
            setOutputs(0.0, 0.0);

            heldAngle = moduleAngle;
            previousAngle = moduleAngle;
            steerStalled = false;
            stallTargetAngle = moduleAngle;
            stallBestError = 0.0;
            stallTimer.reset();
            loopTimer.reset();

            telemetry.addLine(">>> CALIBRATION MODE <<<");
            telemetry.addLine("All power is off. Turn the module by hand.");
            telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
            telemetry.addData("Module angle", "%.1f deg", moduleAngle);
            telemetry.addData("Offset", "%.1f deg", offsetTrim);
            telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
            telemetry.addData("Volts seen", "%s - %s",
                    formatVoltage(minVoltageSeen), formatVoltage(maxVoltageSeen));
            telemetry.addData("Analog range in use", "%.2f - %.2f V",
                    ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE);
            telemetry.addData("Encoder reversed", ENCODER_REVERSED ? "true" : "false");
            telemetry.addLine("Turn the wheel RIGHT by hand: Module angle must RISE.");
            telemetry.addLine("If it falls, flip ENCODER_REVERSED.");
            telemetry.addLine("dpad left/right trims the offset.");
            telemetry.addLine("Copy Raw encoder into ENCODER_OFFSET_DEGREES.");
            telemetry.update();
            return;
        }

        dpadWasPressed = false;

        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;
        double magnitude = Range.clip(Math.hypot(stickX, stickY), 0.0, 1.0);
        boolean released = magnitude < STICK_DEADZONE;

        double targetAngle;

        if (released) {
            targetAngle = heldAngle;
        } else {
            double stickAngle = normalize(Math.toDegrees(Math.atan2(stickX, stickY)));
            double foldedAngle = normalize(stickAngle + 180.0);
            double foldLimit = 90.0 + FOLD_HYSTERESIS_DEGREES;

            double forwardError = normalize(stickAngle - moduleAngle);
            double reversedError = normalize(foldedAngle - moduleAngle);

            if (driveReversedState) {
                if (Math.abs(reversedError) > foldLimit) {
                    driveReversedState = false;
                }
            } else {
                if (Math.abs(forwardError) > foldLimit) {
                    driveReversedState = true;
                }
            }

            targetAngle = driveReversedState ? foldedAngle : stickAngle;
            heldAngle = targetAngle;
        }

        double error = normalize(targetAngle - moduleAngle);
        double desiredSteerPower = computeSteerPower(error, moduleAngle);

        if (released && !HOLD_ANGLE_ON_RELEASE) {
            desiredSteerPower = 0.0;
        }

        updateStallWatchdog(targetAngle, error, desiredSteerPower);

        double steerPower = steerStalled ? 0.0 : desiredSteerPower;
        double drivePower = 0.0;

        if (!released && !steerStalled) {
            double alignment = Range.clip(Math.cos(Math.toRadians(error)), 0.0, 1.0);
            drivePower = magnitude * DRIVE_POWER_SCALE * alignment;

            if (driveReversedState) {
                drivePower = -drivePower;
            }
        }

        setOutputs(steerPower, drivePower);

        if (steerStalled) {
            telemetry.addLine("!! STALLED / NO FEEDBACK - steering and drive cut");
            telemetry.addLine("Error stopped shrinking while steering was commanded.");
            telemetry.addLine("Check the feedback wire, the analog port, and the horn.");
            telemetry.addLine("Hold A to clear, or turn the module toward the target.");
        }

        telemetry.addData("Stick", released ? "released" : String.format("x %.2f  y %.2f", stickX, stickY));
        telemetry.addData("Module angle", "%.1f deg", moduleAngle);
        telemetry.addData(released ? "Held angle" : "Target angle", "%.1f deg", targetAngle);
        telemetry.addData("Error", "%.1f deg", error);
        telemetry.addData("Steer power", "%.2f", steerPower);
        telemetry.addData("Drive power", "%.2f%s", drivePower,
                driveReversedState && !released ? "  (reversed)" : "");
        telemetry.addData("Offset", "%.1f deg", offsetTrim);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.update();
    }

    @Override
    public void stop() {
        setOutputs(0.0, 0.0);
    }

    private void setOutputs(double steerPower, double drivePower) {
        steerServo.setPower(Range.clip(steerPower, -1.0, 1.0));
        driveMotor.setPower(Range.clip(drivePower, -1.0, 1.0));
    }

    private void sampleEncoder() {
        double voltage = steerEncoder.getVoltage();

        lastVoltage = voltage;

        if (voltage < minVoltageSeen) {
            minVoltageSeen = voltage;
        }

        if (voltage > maxVoltageSeen) {
            maxVoltageSeen = voltage;
        }

        lastRawAngle = rawAngleFromVoltage(voltage);
        lastModuleAngle = normalize(lastRawAngle - offsetTrim);
    }

    private double rawAngleFromVoltage(double voltage) {
        double span = ANALOG_MAX_VOLTAGE - ANALOG_MIN_VOLTAGE;

        if (span <= 0.0) {
            return 0.0;
        }

        double normalized = Range.clip((voltage - ANALOG_MIN_VOLTAGE) / span, 0.0, 1.0);

        if (ENCODER_REVERSED) {
            normalized = 1.0 - normalized;
        }

        return normalized * 360.0;
    }

    private double computeSteerPower(double error, double moduleAngle) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double derivative = dt > 1e-4 ? normalize(moduleAngle - previousAngle) / dt : 0.0;
        previousAngle = moduleAngle;

        if (Math.abs(error) < STEER_TOLERANCE_DEGREES) {
            return 0.0;
        }

        double power = (STEER_KP * error)
                - (STEER_KD * derivative)
                + (STEER_KS * Math.signum(error));

        return Range.clip(power, -STEER_MAX_POWER, STEER_MAX_POWER);
    }

    private void updateStallWatchdog(double targetAngle, double error, double commandedSteerPower) {
        double absError = Math.abs(error);

        if (Math.abs(normalize(targetAngle - stallTargetAngle)) > STALL_MOVE_DEGREES) {
            stallTargetAngle = targetAngle;
            stallBestError = absError;
            stallTimer.reset();
        }

        if (absError < stallBestError - STALL_MOVE_DEGREES) {
            stallBestError = absError;
            stallTimer.reset();
            steerStalled = false;
            return;
        }

        if (Math.abs(commandedSteerPower) < STALL_POWER_THRESHOLD) {
            stallBestError = absError;
            stallTimer.reset();
            return;
        }

        if (stallTimer.seconds() > STALL_TIME_SECONDS) {
            steerStalled = true;
        }
    }

    private String formatVoltage(double voltage) {
        if (voltage == Double.MAX_VALUE || voltage == -Double.MAX_VALUE) {
            return "--";
        }

        return String.format("%.3f V", voltage);
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
            offsetTrim += gamepad1.dpad_right ? TRIM_STEP_DEGREES : -TRIM_STEP_DEGREES;
            offsetTrim = ((offsetTrim % 360.0) + 360.0) % 360.0;
        }

        dpadWasPressed = pressed;
    }
}
