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

    private static final String MOTOR_NAME = "swerve_motor";
    private static final String SERVO_NAME = "swerve_servo";
    private static final String ENCODER_NAME = "swerve_encoder";

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

    private static final double STALL_POWER_THRESHOLD = 0.25;
    private static final double STALL_TIME_SECONDS = 2.0;
    private static final double STALL_MOVE_DEGREES = 2.0;

    private static final double HOME_TOLERANCE_DEGREES = 2.0;
    private static final double HOME_TIMEOUT_SECONDS = 4.0;
    private static final double NOISE_WINDOW_SECONDS = 1.0;

    private DcMotor driveMotor;
    private CRServo steerServo;
    private AnalogInput steerEncoder;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime homeTimer = new ElapsedTime();
    private final ElapsedTime noiseTimer = new ElapsedTime();

    private double loopDt = 0.0;
    private double offsetTrim = ENCODER_OFFSET_DEGREES;
    private double heldAngle = 0.0;
    private double previousAngle = 0.0;
    private double angularVelocity = 0.0;
    private boolean driveReversedState = false;

    private double lastVoltage = 0.0;
    private double lastRawAngle = 0.0;
    private double lastModuleAngle = 0.0;
    private double minVoltageSeen = Double.MAX_VALUE;
    private double maxVoltageSeen = -Double.MAX_VALUE;

    private boolean steerStalled = false;
    private double stallTargetAngle = 0.0;
    private double stallBestError = 0.0;

    private boolean homing = false;
    private String homeResult = "not run";

    private double noiseMinVolts = Double.MAX_VALUE;
    private double noiseMaxVolts = -Double.MAX_VALUE;
    private double noisePeakToPeakVolts = 0.0;

    @Override
    public void init() {
        driveMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        steerServo = hardwareMap.get(CRServo.class, SERVO_NAME);
        steerEncoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setDirection(DRIVE_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        driveMotor.setPower(0.0);

        steerServo.setDirection(STEER_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        loopDt = 0.0;
        noiseTimer.reset();
        sampleEncoder();
    }

    @Override
    public void init_loop() {
        loopDt = 0.0;
        sampleEncoder();
        drainButtonEdges();

        telemetry.addLine("Single Swerve Module - CR servo caster");
        telemetry.addLine("Nothing is powered during INIT.");
        telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
        telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.addData("Analog range in use", "%.2f - %.2f V (hub max %.2f V)",
                ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE, steerEncoder.getMaxVoltage());
        telemetryNoise();
        telemetry.addLine("Turn the wheel by hand now - Raw encoder must change.");
        telemetry.addLine("START drives the module to forward under power.");
        telemetry.addLine("Not sure of ENCODER_REVERSED / STEER_REVERSED yet?");
        telemetry.addLine("Hold A while you press START, then read the first-run steps.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopDt = 0.0;
        sampleEncoder();
        loopTimer.reset();

        heldAngle = lastModuleAngle;
        driveReversedState = false;
        beginHoming();
    }

    @Override
    public void loop() {
        loopDt = loopTimer.seconds();
        loopTimer.reset();

        boolean homeRequested = gamepad1.yWasPressed();
        boolean trimUp = gamepad1.dpadRightWasPressed();
        boolean trimDown = gamepad1.dpadLeftWasPressed();
        boolean calibrating = gamepad1.a;

        if (calibrating) {
            applyEncoderTrim(trimUp, trimDown);
        }

        sampleEncoder();

        if (calibrating) {
            runCalibration();
            return;
        }

        if (gamepad1.left_bumper) {
            runManualOverride();
            return;
        }

        if (homeRequested) {
            beginHoming();
        }

        if (homing) {
            runHoming();
            return;
        }

        runCaster();
    }

    @Override
    public void stop() {
        if (steerServo != null) {
            steerServo.setPower(0.0);
        }

        if (driveMotor != null) {
            driveMotor.setPower(0.0);
        }
    }

    private void runCaster() {
        double moduleAngle = lastModuleAngle;
        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;
        double magnitude = Range.clip(Math.hypot(stickX, stickY), 0.0, 1.0);
        boolean released = magnitude < STICK_DEADZONE;

        if (!released) {
            double stickAngle = normalize(Math.toDegrees(Math.atan2(stickX, stickY)));
            double foldedAngle = normalize(stickAngle + 180.0);
            double foldLimit = 90.0 + FOLD_HYSTERESIS_DEGREES;

            if (driveReversedState) {
                if (Math.abs(normalize(foldedAngle - moduleAngle)) > foldLimit) {
                    driveReversedState = false;
                }
            } else if (Math.abs(normalize(stickAngle - moduleAngle)) > foldLimit) {
                driveReversedState = true;
            }

            heldAngle = driveReversedState ? foldedAngle : stickAngle;
        }

        double targetAngle = heldAngle;
        double error = normalize(targetAngle - moduleAngle);
        double desiredSteerPower = computeSteerPower(error);

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
            telemetry.addLine("Hold LB to steer by hand, or hold A to clear and check the encoder.");
        }

        telemetry.addData("Stick", released ? "released" : String.format("x %.2f  y %.2f", stickX, stickY));
        telemetry.addData("Module angle", "%.1f deg", moduleAngle);
        telemetry.addData(released ? "Held angle" : "Target angle", "%.1f deg", targetAngle);
        telemetry.addData("Error", "%.1f deg", error);
        telemetry.addData("Steer power", "%.2f", steerPower);
        telemetry.addData("Drive power", "%.2f%s", drivePower,
                driveReversedState && !released ? "  (reversed)" : "");
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.addData("Offset", "%.1f deg", offsetTrim);
        telemetry.addData("Homing", homeResult);
        telemetryNoise();
        telemetry.addLine("Y: point forward. LB: manual steer. A: calibrate.");
        telemetry.update();
    }

    private void runHoming() {
        double error = normalize(0.0 - lastModuleAngle);
        double steerPower = computeSteerPower(error);

        updateStallWatchdog(0.0, error, steerPower);

        boolean reached = Math.abs(error) <= HOME_TOLERANCE_DEGREES;
        boolean timedOut = homeTimer.seconds() > HOME_TIMEOUT_SECONDS;

        if (reached || timedOut || steerStalled) {
            steerPower = 0.0;
        }

        setOutputs(steerPower, 0.0);

        if (reached) {
            endHoming(String.format("done in %.1f s", homeTimer.seconds()));
        } else if (steerStalled) {
            endHoming(String.format("STALLED %.1f deg off - check STEER_REVERSED",
                    Math.abs(error)));
        } else if (timedOut) {
            endHoming(String.format("TIMEOUT, still %.1f deg off", Math.abs(error)));
        }

        telemetry.addLine(">>> HOMING TO FORWARD <<<");
        telemetry.addData("Status", homeResult);
        telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
        telemetry.addData("Error to forward", "%.1f deg", error);
        telemetry.addData("Steer power", "%.2f", steerPower);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetryNoise();
        telemetry.addLine("Y homes again. LB steers by hand. A cuts all power.");
        telemetry.update();
    }

    private void runManualOverride() {
        double servoPower = gamepad1.right_stick_x;

        setOutputs(servoPower, 0.0);
        seedHoldFromMeasuredAngle();

        telemetry.addLine(">>> MANUAL STEER OVERRIDE (LB held) <<<");
        telemetry.addLine("No loop, no watchdog. This proves the module can physically turn.");
        telemetry.addData("Servo power", "%.2f  (RIGHT stick x)", servoPower);
        telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
        telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetryNoise();
        telemetry.addLine("Wheel turns but the angle does not: the feedback path is dead.");
        telemetry.addLine("The drive motor is off. Release LB to go back to caster mode.");
        telemetry.update();
    }

    private void runCalibration() {
        setOutputs(0.0, 0.0);
        seedHoldFromMeasuredAngle();

        telemetry.addLine(">>> CALIBRATION MODE <<<");
        telemetry.addLine("All power is off. Turn the module by hand.");
        telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
        telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
        telemetry.addData("Offset", "%.1f deg  (dpad left/right)", offsetTrim);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.addData("Volts seen", "%s - %s",
                formatVoltage(minVoltageSeen), formatVoltage(maxVoltageSeen));
        telemetry.addData("Analog range in use", "%.2f - %.2f V",
                ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE);
        telemetry.addData("Encoder reversed", ENCODER_REVERSED ? "true" : "false");
        telemetryNoise();
        telemetry.addLine("Turn the wheel RIGHT by hand: Module angle must RISE.");
        telemetry.addLine("Copy Raw encoder into ENCODER_OFFSET_DEGREES.");
        telemetry.update();
    }

    private void beginHoming() {
        homing = true;
        homeResult = "running";
        homeTimer.reset();
        driveReversedState = false;
        steerStalled = false;
        stallTargetAngle = 0.0;
        stallBestError = Math.abs(normalize(0.0 - lastModuleAngle));
        stallTimer.reset();
    }

    private void endHoming(String result) {
        homing = false;
        homeResult = result;
        heldAngle = lastModuleAngle;
        driveReversedState = false;
        steerStalled = false;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;
        stallTimer.reset();
    }

    private void seedHoldFromMeasuredAngle() {
        if (homing) {
            homing = false;
            homeResult = "cancelled";
        }

        heldAngle = lastModuleAngle;
        driveReversedState = false;
        steerStalled = false;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;
        stallTimer.reset();
    }

    private void setOutputs(double steerPower, double drivePower) {
        steerServo.setPower(Range.clip(steerPower, -1.0, 1.0));
        driveMotor.setPower(Range.clip(drivePower, -1.0, 1.0));
    }

    private double computeSteerPower(double error) {
        if (Math.abs(error) < STEER_TOLERANCE_DEGREES) {
            return 0.0;
        }

        double power = (STEER_KP * error)
                - (STEER_KD * angularVelocity)
                + (STEER_KS * Math.signum(error));

        return Range.clip(power, -STEER_MAX_POWER, STEER_MAX_POWER);
    }

    private void updateStallWatchdog(double targetAngle, double error, double commandedSteerPower) {
        double absError = Math.abs(error);

        if (Math.abs(normalize(targetAngle - stallTargetAngle)) > STALL_MOVE_DEGREES) {
            stallTargetAngle = targetAngle;
            stallBestError = absError;
            stallTimer.reset();
            steerStalled = false;
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

        double angleDelta = normalize(lastModuleAngle - previousAngle);
        angularVelocity = loopDt > 1e-4 ? angleDelta / loopDt : 0.0;
        previousAngle = lastModuleAngle;

        if (voltage < noiseMinVolts) {
            noiseMinVolts = voltage;
        }

        if (voltage > noiseMaxVolts) {
            noiseMaxVolts = voltage;
        }

        if (noiseTimer.seconds() >= NOISE_WINDOW_SECONDS) {
            noisePeakToPeakVolts = noiseMaxVolts - noiseMinVolts;
            noiseMinVolts = voltage;
            noiseMaxVolts = voltage;
            noiseTimer.reset();
        }
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

    private void telemetryNoise() {
        double windowSpan = noiseMaxVolts - noiseMinVolts;
        double peakToPeak = Math.max(noisePeakToPeakVolts, windowSpan > 0.0 ? windowSpan : 0.0);
        double span = ANALOG_MAX_VOLTAGE - ANALOG_MIN_VOLTAGE;
        double noiseDegrees = span > 0.0 ? peakToPeak / span * 360.0 : 0.0;

        telemetry.addData("Noise p-p", "%.3f V  (%.1f deg)", peakToPeak, noiseDegrees);

        if (Math.abs(lastModuleAngle) > 150.0) {
            telemetry.addLine("Near the +/-180 wrap: flipping between +179 and -180");
            telemetry.addLine("is normal here. Set ENCODER_OFFSET_DEGREES to move it.");
        }
    }

    private void drainButtonEdges() {
        gamepad1.yWasPressed();
        gamepad1.dpadRightWasPressed();
        gamepad1.dpadLeftWasPressed();
    }

    private void applyEncoderTrim(boolean trimUp, boolean trimDown) {
        if (trimUp) {
            offsetTrim += TRIM_STEP_DEGREES;
        }

        if (trimDown) {
            offsetTrim -= TRIM_STEP_DEGREES;
        }

        offsetTrim = ((offsetTrim % 360.0) + 360.0) % 360.0;
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
}
