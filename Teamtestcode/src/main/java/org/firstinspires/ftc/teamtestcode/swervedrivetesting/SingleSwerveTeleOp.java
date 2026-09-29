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

    private static final double STEERING_RATIO = 8.0;
    private static final double RATIO_REFERENCE_DEGREES = 90.0;

    private static final boolean ENCODER_REVERSED = false;
    private static final boolean STEER_REVERSED = false;
    private static final boolean DRIVE_REVERSED = false;

    private static final double ANALOG_MIN_VOLTAGE = 0.0;
    private static final double ANALOG_MAX_VOLTAGE = 3.3;

    private static final double STEER_KP = 0.05;
    private static final double STEER_KD = 0.002;
    private static final double STEER_KS = 0.05;
    private static final double STEER_TOLERANCE_DEGREES = 1.0;
    private static final double STEER_MAX_POWER = 0.6;

    private static final double STICK_DEADZONE = 0.15;
    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double TRIM_STEP_DEGREES = 0.5;
    private static final double FOLD_HYSTERESIS_DEGREES = 15.0;

    private static final double STALL_POWER_THRESHOLD = 0.45;
    private static final double STALL_TIME_SECONDS = 2.0;
    private static final double STALL_MOVE_DEGREES = 1.0;

    private static final double HOME_TOLERANCE_DEGREES = 2.0;
    private static final double HOME_TIMEOUT_SECONDS = 6.0;
    private static final double NOISE_WINDOW_SECONDS = 1.0;
    private static final double ALIAS_WARN_SERVO_DEGREES = 90.0;
    private static final double MAX_SERVO_DEGREES_PER_SECOND = 600.0;

    private DcMotor driveMotor;
    private CRServo steerServo;
    private AnalogInput steerEncoder;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime homeTimer = new ElapsedTime();
    private final ElapsedTime noiseTimer = new ElapsedTime();

    private double steeringRatio = 1.0;
    private boolean ratioValid = true;

    private double loopDt = 0.0;
    private double heldAngle = 0.0;
    private double angularVelocity = 0.0;
    private boolean driveReversedState = false;

    private double lastVoltage = 0.0;
    private double lastRawAngle = 0.0;
    private double previousRawAngle = 0.0;
    private double unwrappedServoAngle = 0.0;
    private double previousUnwrappedServoAngle = 0.0;
    private double zeroServoAngle = 0.0;
    private boolean encoderPrimed = false;
    private int zeroTakenCount = 0;

    private double minVoltageSeen = Double.MAX_VALUE;
    private double maxVoltageSeen = -Double.MAX_VALUE;

    private int aliasWarnings = 0;
    private double worstServoStepDegrees = 0.0;
    private int slowLoopWarnings = 0;
    private double worstLoopSeconds = 0.0;

    private boolean calibrationActive = false;
    private double calibrationStartServoAngle = 0.0;

    private boolean steerStalled = false;
    private double stallTargetAngle = 0.0;
    private double stallBestError = 0.0;
    private double stallStartError = 0.0;
    private double stallEndError = 0.0;
    private double stallModuleStart = 0.0;
    private double stallModuleEnd = 0.0;

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

        ratioValid = STEERING_RATIO > 0.0;
        steeringRatio = ratioValid ? STEERING_RATIO : 1.0;

        loopDt = 0.0;
        noiseTimer.reset();
        sampleEncoder();
        zeroServoAngle = unwrappedServoAngle;
        heldAngle = 0.0;
    }

    @Override
    public void init_loop() {
        loopDt = 0.0;
        sampleEncoder();

        if (gamepad1.bWasPressed()) {
            takeZero();
        }

        drainButtonEdges();

        telemetry.addLine("Single Swerve Module - geared CR servo caster");
        telemetry.addLine("Nothing is powered during INIT.");
        telemetryRatio();
        telemetry.addData("Module angle", "%.1f deg", moduleAngle());
        telemetry.addData("Servo raw", "%.1f deg", lastRawAngle);
        telemetry.addData("Servo unwrapped", "%.1f deg", unwrappedServoAngle);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.addData("Volts seen", "%s - %s",
                formatVoltage(minVoltageSeen), formatVoltage(maxVoltageSeen));
        telemetry.addData("Analog range in use", "%.2f - %.2f V (hub max %.2f V)",
                ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE, steerEncoder.getMaxVoltage());
        telemetryNoise();
        telemetry.addLine("Turn the wheel by hand now - Servo raw must change.");
        telemetry.addLine("POINT THE WHEEL STRAIGHT FORWARD BY HAND BEFORE START.");
        telemetry.addLine("START takes that position as 0 and then homes to it.");
        telemetry.addLine("B re-takes the zero here and at any time later.");
        telemetry.addLine("Hold A across START on a first run, then read the README.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopDt = 0.0;
        sampleEncoder();
        loopTimer.reset();

        takeZero();
        beginHoming();
    }

    @Override
    public void loop() {
        loopDt = loopTimer.seconds();
        loopTimer.reset();

        boolean homeRequested = gamepad1.yWasPressed();
        boolean zeroRequested = gamepad1.bWasPressed();
        boolean measureRestart = gamepad1.xWasPressed();
        boolean trimUp = gamepad1.dpadRightWasPressed();
        boolean trimDown = gamepad1.dpadLeftWasPressed();
        boolean calibrating = gamepad1.a;

        sampleEncoder();

        if (zeroRequested) {
            takeZero();
        }

        if (calibrating) {
            if (!calibrationActive) {
                calibrationActive = true;
                calibrationStartServoAngle = unwrappedServoAngle;
            }

            if (measureRestart) {
                calibrationStartServoAngle = unwrappedServoAngle;
            }

            applyZeroTrim(trimUp, trimDown);
            runCalibration();
            return;
        }

        calibrationActive = false;

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
        double moduleAngle = moduleAngle();
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
            double errorGrowth = stallEndError - stallStartError;
            double moduleMoved = normalize(stallModuleEnd - stallModuleStart);

            telemetry.addLine("!! STALLED - steering and drive cut");
            telemetry.addData("Error went", "%.1f -> %.1f module deg", stallStartError, stallEndError);
            telemetry.addData("Module moved", "%.1f module deg while pushing", moduleMoved);

            if (errorGrowth > STALL_MOVE_DEGREES) {
                telemetry.addLine("DIAGNOSIS: error GREW - the servo drives the wrong way.");
                telemetry.addLine("Flip STEER_REVERSED (or ENCODER_REVERSED if mirrored).");
            } else if (Math.abs(moduleMoved) < STALL_MOVE_DEGREES) {
                telemetry.addLine("DIAGNOSIS: module did NOT move - jammed or no torque.");
                telemetry.addLine("Hold LB and steer by hand power to find the limit.");
            } else {
                telemetry.addLine("DIAGNOSIS: module moved but too slowly to finish.");
                telemetry.addLine("Raise STEER_MAX_POWER / STEER_KP, or check STEERING_RATIO.");
            }

            telemetry.addLine("Error stopped shrinking while steering was commanded.");
            telemetry.addLine("Hold LB to steer by hand, or hold A to clear and check the encoder.");
        }

        telemetryRatio();
        telemetry.addData("Stick", released ? "released" : String.format("x %.2f  y %.2f", stickX, stickY));
        telemetry.addData("Module angle", "%.1f deg", moduleAngle);
        telemetry.addData("Module continuous", "%.1f deg", moduleContinuousAngle());
        telemetry.addData(released ? "Held angle" : "Target angle", "%.1f deg", targetAngle);
        telemetry.addData("Error", "%.1f module deg  (%.0f servo deg)", error, error * steeringRatio);
        telemetry.addData("Module speed", "%.1f module deg/s", angularVelocity);
        telemetry.addData("Steer power", "%.2f", steerPower);
        telemetry.addData("Drive power", "%.2f%s", drivePower,
                driveReversedState && !released ? "  (reversed)" : "");
        telemetry.addData("Servo unwrapped", "%.1f deg  (zero %.1f)", unwrappedServoAngle, zeroServoAngle);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.addData("Homing", homeResult);
        telemetryNoise();
        telemetry.addLine("Y: home to 0. B: re-zero here. LB: manual steer. A: calibrate.");
        telemetry.update();
    }

    private void runHoming() {
        double error = normalize(0.0 - moduleAngle());
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
            endHoming(String.format("STALLED %.1f module deg off - check STEER_REVERSED",
                    Math.abs(error)));
        } else if (timedOut) {
            endHoming(String.format("TIMEOUT, still %.1f module deg off", Math.abs(error)));
        }

        telemetry.addLine(">>> HOMING TO FORWARD <<<");
        telemetryRatio();
        telemetry.addData("Status", homeResult);
        telemetry.addData("Module angle", "%.1f deg", moduleAngle());
        telemetry.addData("Error to forward", "%.1f module deg  (%.0f servo deg)",
                error, error * steeringRatio);
        telemetry.addData("Steer power", "%.2f", steerPower);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetryNoise();
        telemetry.addLine("Y homes again. B re-zeros here. LB steers by hand. A cuts all power.");
        telemetry.update();
    }

    private void runManualOverride() {
        double servoPower = gamepad1.right_stick_x;

        setOutputs(servoPower, 0.0);
        seedHoldFromMeasuredAngle();

        telemetry.addLine(">>> MANUAL STEER OVERRIDE (LB held) <<<");
        telemetry.addLine("No loop, no watchdog. This proves the module can physically turn.");
        telemetryRatio();
        telemetry.addData("Servo power", "%.2f  (RIGHT stick x)", servoPower);
        telemetry.addData("Module angle", "%.1f deg", moduleAngle());
        telemetry.addData("Servo raw", "%.1f deg", lastRawAngle);
        telemetry.addData("Servo unwrapped", "%.1f deg", unwrappedServoAngle);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetryNoise();
        telemetry.addLine("Wheel turns but the angle does not: the feedback path is dead.");
        telemetry.addLine("The drive motor is off. Release LB to go back to caster mode.");
        telemetry.update();
    }

    private void runCalibration() {
        setOutputs(0.0, 0.0);
        seedHoldFromMeasuredAngle();

        double servoMoved = unwrappedServoAngle - calibrationStartServoAngle;
        double reference = RATIO_REFERENCE_DEGREES > 0.0 ? RATIO_REFERENCE_DEGREES : 90.0;
        double impliedRatio = Math.abs(servoMoved) / reference;

        telemetry.addLine(">>> CALIBRATION / RATIO MEASUREMENT <<<");
        telemetry.addLine("All power is off. Turn the module by hand.");
        telemetryRatio();
        telemetry.addData("Servo moved", "%.1f deg since A was pressed", servoMoved);
        telemetry.addData("Implied ratio", "%.2f servo deg per module deg", impliedRatio);
        telemetry.addLine(String.format(
                "ONLY TRUE IF YOU JUST TURNED THE MODULE EXACTLY %.0f DEG.", reference));
        telemetry.addLine("Square the wheel up, press X to restart the measurement,");
        telemetry.addLine(String.format(
                "turn exactly %.0f deg against a square, then read Implied ratio.", reference));
        telemetry.addLine("Put that number into STEERING_RATIO and reinstall.");
        telemetry.addData("Module angle", "%.1f deg", moduleAngle());
        telemetry.addData("Module continuous", "%.1f deg", moduleContinuousAngle());
        telemetry.addData("Servo raw", "%.1f deg", lastRawAngle);
        telemetry.addData("Servo unwrapped", "%.1f deg  (zero %.1f)", unwrappedServoAngle, zeroServoAngle);
        telemetry.addData("Zero taken", "%d times  (B re-zeros)", zeroTakenCount);
        telemetry.addData("Trim", "dpad right/left moves the zero by %.1f module deg",
                TRIM_STEP_DEGREES);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetry.addData("Volts seen", "%s - %s",
                formatVoltage(minVoltageSeen), formatVoltage(maxVoltageSeen));
        telemetry.addData("Analog range in use", "%.2f - %.2f V",
                ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE);
        telemetry.addData("Encoder reversed", ENCODER_REVERSED ? "true" : "false");
        telemetryNoise();
        telemetry.addLine("Turn the wheel RIGHT by hand: Module angle must RISE.");
        telemetry.update();
    }

    private void beginHoming() {
        homing = true;
        homeResult = "running";
        homeTimer.reset();
        driveReversedState = false;
        steerStalled = false;
        stallTargetAngle = 0.0;
        stallBestError = Math.abs(normalize(0.0 - moduleAngle()));
        stallTimer.reset();
    }

    private void endHoming(String result) {
        homing = false;
        homeResult = result;
        heldAngle = moduleAngle();
        driveReversedState = false;
        steerStalled = false;
        stallTargetAngle = heldAngle;
        stallBestError = 0.0;
        stallTimer.reset();
    }

    private void seedHoldFromMeasuredAngle() {
        if (homing) {
            homing = false;
            homeResult = "cancelled";
        }

        heldAngle = moduleAngle();
        driveReversedState = false;
        steerStalled = false;
        stallTargetAngle = heldAngle;
        stallBestError = 0.0;
        stallTimer.reset();
    }

    private void takeZero() {
        zeroServoAngle = unwrappedServoAngle;
        zeroTakenCount++;
        heldAngle = 0.0;
        driveReversedState = false;
        steerStalled = false;
        stallTargetAngle = 0.0;
        stallBestError = 0.0;
        stallTimer.reset();
        aliasWarnings = 0;
        worstServoStepDegrees = 0.0;
        slowLoopWarnings = 0;
        worstLoopSeconds = 0.0;

        if (homing) {
            homing = false;
            homeResult = "cancelled by re-zero";
        }
    }

    private double aliasLoopSeconds() {
        if (MAX_SERVO_DEGREES_PER_SECOND <= 0.0) {
            return Double.MAX_VALUE;
        }

        return 180.0 / MAX_SERVO_DEGREES_PER_SECOND;
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
            stallStartError = absError;
            stallModuleStart = moduleAngle();
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
            if (!steerStalled) {
                stallEndError = absError;
                stallModuleEnd = moduleAngle();
            }

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

        if (!encoderPrimed) {
            encoderPrimed = true;
            previousRawAngle = lastRawAngle;
            unwrappedServoAngle = lastRawAngle;
            previousUnwrappedServoAngle = lastRawAngle;
            zeroServoAngle = lastRawAngle;
        }

        double servoStep = normalize(lastRawAngle - previousRawAngle);
        previousRawAngle = lastRawAngle;
        unwrappedServoAngle += servoStep;

        double absStep = Math.abs(servoStep);

        if (absStep > worstServoStepDegrees) {
            worstServoStepDegrees = absStep;
        }

        if (absStep > ALIAS_WARN_SERVO_DEGREES) {
            aliasWarnings++;
        }

        if (loopDt > worstLoopSeconds) {
            worstLoopSeconds = loopDt;
        }

        if (loopDt > aliasLoopSeconds()) {
            slowLoopWarnings++;
        }

        double servoVelocityStep = unwrappedServoAngle - previousUnwrappedServoAngle;
        previousUnwrappedServoAngle = unwrappedServoAngle;
        angularVelocity = loopDt > 1e-4 ? servoVelocityStep / steeringRatio / loopDt : 0.0;

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

    private double moduleContinuousAngle() {
        return (unwrappedServoAngle - zeroServoAngle) / steeringRatio;
    }

    private double moduleAngle() {
        return normalize(moduleContinuousAngle());
    }

    private void telemetryRatio() {
        if (ratioValid) {
            telemetry.addData("Steering ratio", "%.2f servo deg per module deg", steeringRatio);
        } else {
            telemetry.addLine("!! STEERING_RATIO is not positive - falling back to 1.00");
            telemetry.addLine("Set STEERING_RATIO above zero and reinstall.");
        }
    }

    private void telemetryNoise() {
        double windowSpan = noiseMaxVolts - noiseMinVolts;
        double peakToPeak = Math.max(noisePeakToPeakVolts, windowSpan > 0.0 ? windowSpan : 0.0);
        double span = ANALOG_MAX_VOLTAGE - ANALOG_MIN_VOLTAGE;
        double noiseServoDegrees = span > 0.0 ? peakToPeak / span * 360.0 : 0.0;

        telemetry.addData("Noise p-p", "%.3f V  (%.1f servo deg = %.2f module deg)",
                peakToPeak, noiseServoDegrees, noiseServoDegrees / steeringRatio);
        telemetry.addData("Biggest wrapped servo step", "%.1f deg per loop (max 180)",
                worstServoStepDegrees);
        telemetry.addData("Slowest loop", "%.3f s  (alias risk above %.3f s)",
                worstLoopSeconds, aliasLoopSeconds());

        if (aliasWarnings > 0 || slowLoopWarnings > 0) {
            telemetry.addLine("!! Unwrap may have aliased - module angle may be off by "
                    + String.format("%.0f deg", 360.0 / steeringRatio));
            telemetry.addData("  wrapped steps over limit", "%d loops over %.0f servo deg",
                    aliasWarnings, ALIAS_WARN_SERVO_DEGREES);
            telemetry.addData("  loops slow enough to alias", "%d loops over %.3f s",
                    slowLoopWarnings, aliasLoopSeconds());
            telemetry.addLine("Re-zero with B after pointing the wheel forward by hand.");
        }

        if (Math.abs(moduleAngle()) > 150.0) {
            telemetry.addLine("Near the +/-180 module wrap: flipping between +179 and -180");
            telemetry.addLine("is normal here. Press B facing forward to move the zero.");
        }
    }

    private void drainButtonEdges() {
        gamepad1.yWasPressed();
        gamepad1.xWasPressed();
        gamepad1.dpadRightWasPressed();
        gamepad1.dpadLeftWasPressed();
    }

    private void applyZeroTrim(boolean trimUp, boolean trimDown) {
        if (trimUp) {
            zeroServoAngle -= TRIM_STEP_DEGREES * steeringRatio;
        }

        if (trimDown) {
            zeroServoAngle += TRIM_STEP_DEGREES * steeringRatio;
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
}
