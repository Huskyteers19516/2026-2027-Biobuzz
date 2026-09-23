package org.firstinspires.ftc.teamtestcode.swervedrivetesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    private static final boolean HOLD_ANGLE_ON_RELEASE = true;

    private static final double STALL_POWER_THRESHOLD = 0.25;
    private static final double STALL_TIME_SECONDS = 2.0;
    private static final double STALL_MOVE_DEGREES = 2.0;

    private static final boolean HOME_ON_START = true;
    private static final double HOME_TOLERANCE_DEGREES = 2.0;
    private static final double HOME_TIMEOUT_SECONDS = 4.0;
    private static final double HOME_SETTLE_SECONDS = 0.7;
    private static final double NOISE_WINDOW_SECONDS = 1.0;

    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_TRAVEL_DEGREES = 270.0;
    private static final double SERVO_MIN_POSITION = 0.0;
    private static final double SERVO_MAX_POSITION = 1.0;
    private static final double SERVO_TRIM_STEP = 0.005;
    private static final double SERVO_MANUAL_SPAN = 0.5;
    private static final double SERVO_SETTLE_DEGREES = 10.0;
    private static final double SERVO_JUMP_FULL_CUT_DEGREES = 60.0;
    private static final double DRIVE_RAMP_PER_SECOND = 1.5;
    private static final double IMPLIED_TRAVEL_MIN_SPAN = 0.05;

    private static final int MODE_NONE = 0;
    private static final int MODE_CLOSED_LOOP = 1;
    private static final int MODE_CR_OPEN_LOOP = 2;
    private static final int MODE_POSITION = 3;

    private DcMotor driveMotor;
    private CRServo steerCrServo;
    private Servo steerPositionServo;
    private AnalogInput steerEncoder;

    private int steerMode = MODE_NONE;
    private String detectedServoType = "none";
    private String modeName = "NO STEERING SERVO";
    private String limitLine = "";

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime homeTimer = new ElapsedTime();
    private final ElapsedTime noiseTimer = new ElapsedTime();

    private double loopDt = 0.0;

    private double offsetTrim = ENCODER_OFFSET_DEGREES;
    private double servoCenterTrim = 0.0;
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

    private double lastServoPosition = SERVO_CENTER;
    private double rampedDrivePower = 0.0;
    private double previousCommandAngle = 0.0;
    private double unwrappedAngle = 0.0;

    private boolean haveTravelReference = false;
    private double travelReferencePosition = SERVO_CENTER;
    private double travelReferenceUnwrapped = 0.0;
    private double impliedTravelDegrees = 0.0;
    private boolean haveImpliedTravel = false;

    @Override
    public void init() {
        driveMotor = hardwareMap.tryGet(DcMotor.class, MOTOR_NAME);
        steerCrServo = hardwareMap.tryGet(CRServo.class, SERVO_NAME);

        if (steerCrServo == null) {
            steerPositionServo = hardwareMap.tryGet(Servo.class, SERVO_NAME);
        }

        steerEncoder = hardwareMap.tryGet(AnalogInput.class, ENCODER_NAME);

        if (driveMotor != null) {
            driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveMotor.setDirection(DRIVE_REVERSED
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
            driveMotor.setPower(0.0);
        }

        if (steerCrServo != null) {
            detectedServoType = "CRServo (Continuous Rotation Servo)";
            steerCrServo.setDirection(STEER_REVERSED
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
            steerCrServo.setPower(0.0);
            steerMode = steerEncoder != null ? MODE_CLOSED_LOOP : MODE_CR_OPEN_LOOP;
        } else if (steerPositionServo != null) {
            detectedServoType = "Servo (position mode)";
            steerPositionServo.setDirection(STEER_REVERSED
                    ? Servo.Direction.REVERSE
                    : Servo.Direction.FORWARD);
            steerMode = MODE_POSITION;
        } else {
            detectedServoType = "NOT FOUND";
            steerMode = MODE_NONE;
        }

        if (steerMode == MODE_CLOSED_LOOP) {
            modeName = "CLOSED LOOP CASTER (CR servo + encoder)";
            limitLine = "";
        } else if (steerMode == MODE_CR_OPEN_LOOP) {
            modeName = "OPEN LOOP CR STEER (no encoder)";
            limitLine = "NO ENCODER: heading cannot be held, stick x is raw steer power.";
        } else if (steerMode == MODE_POSITION) {
            modeName = "OPEN LOOP CASTER (position servo)";
            limitLine = steerEncoder != null
                    ? "POSITION MODE: no feedback loop, travel limited by SERVO_TRAVEL_DEGREES."
                    : "POSITION MODE, NO ENCODER: commanded angle only, nothing measures the module.";
        } else {
            modeName = "NO STEERING SERVO";
            limitLine = "NOTHING WILL MOVE: no device named swerve_servo as CRServo or Servo.";
        }

        lastServoPosition = servoPositionForAngle(0.0);
        previousCommandAngle = 0.0;

        sampleEncoder();
        unwrappedAngle = 0.0;
        heldAngle = steerMode == MODE_POSITION ? 0.0 : lastModuleAngle;
        previousAngle = lastModuleAngle;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;
    }

    @Override
    public void init_loop() {
        loopDt = 0.0;
        sampleEncoder();

        telemetry.addLine("Single Swerve Module - caster mode");
        telemetry.addData("Detected swerve_servo", detectedServoType);
        telemetry.addData("Mode", modeName);
        telemetry.addData("Encoder swerve_encoder", steerEncoder != null ? "found" : "NOT FOUND");
        telemetry.addData("Motor swerve_motor", driveMotor != null ? "found" : "NOT FOUND");

        if (!limitLine.isEmpty()) {
            telemetry.addLine(limitLine);
        }

        if (steerEncoder != null) {
            telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
            telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
            telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
            telemetry.addData("Analog range", "%.2f - %.2f V (hub max %.2f V)",
                    ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE, steerEncoder.getMaxVoltage());
            telemetryNoise();
            telemetry.addLine("Turn the wheel by hand now - Raw encoder must change.");
        }

        telemetry.addLine("Nothing is powered during INIT.");
        telemetry.addLine("Left stick: point and drive. LB: manual steer. A: calibrate. Y: home.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        stallTimer.reset();
        steerStalled = false;
        driveReversedState = false;
        rampedDrivePower = 0.0;

        loopDt = 0.0;
        sampleEncoder();
        unwrappedAngle = 0.0;
        heldAngle = steerMode == MODE_POSITION ? 0.0 : lastModuleAngle;
        previousAngle = lastModuleAngle;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;

        previousCommandAngle = 0.0;

        homing = HOME_ON_START && steerMode != MODE_NONE && steerMode != MODE_CR_OPEN_LOOP;
        homeResult = homing ? "running" : (steerMode == MODE_CR_OPEN_LOOP ? "impossible, no encoder" : "off");
        homeTimer.reset();
    }

    @Override
    public void loop() {
        loopDt = loopTimer.seconds();
        loopTimer.reset();
        sampleEncoder();

        if (steerMode == MODE_NONE) {
            stopAll();
            telemetry.addLine(">>> NO STEERING SERVO FOUND <<<");
            telemetry.addLine(limitLine);
            telemetry.addData("Looked for", "CRServo then Servo named %s", SERVO_NAME);
            telemetry.addData("swerve_encoder", steerEncoder != null ? "found" : "NOT FOUND");
            telemetry.addData("swerve_motor", driveMotor != null ? "found" : "NOT FOUND");
            telemetry.addLine("Fix the robot configuration, then re-run.");
            telemetry.update();
            return;
        }

        if (gamepad1.a) {
            runCalibration();
            return;
        }

        if (gamepad1.left_bumper) {
            runManualOverride();
            return;
        }

        if (gamepad1.yWasPressed() && steerMode != MODE_CR_OPEN_LOOP) {
            homing = true;
            homeResult = "running";
            homeTimer.reset();
            steerStalled = false;
            stallTargetAngle = 0.0;
            stallBestError = Math.abs(normalize(0.0 - lastModuleAngle));
            stallTimer.reset();
            rampedDrivePower = 0.0;
        }

        if (homing) {
            runHoming();
            return;
        }

        if (steerMode == MODE_CLOSED_LOOP) {
            runClosedLoopCaster();
        } else if (steerMode == MODE_POSITION) {
            runPositionCaster();
        } else {
            runOpenLoopCrSteer();
        }
    }

    @Override
    public void stop() {
        stopAll();
    }

    private void runClosedLoopCaster() {
        double moduleAngle = lastModuleAngle;

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
        double desiredSteerPower = computeSteerPower(error);

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

        setCrOutputs(steerPower, drivePower);

        telemetry.addData("Mode", modeName);

        if (steerStalled) {
            telemetry.addLine("!! STALLED / NO FEEDBACK - steering and drive cut");
            telemetry.addLine("Error stopped shrinking while steering was commanded.");
            telemetry.addLine("Check the feedback wire, the analog port, and the horn.");
            telemetry.addLine("Hold LB to steer by hand and prove the module can turn.");
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
        telemetry.addData("Homing", homeResult);
        telemetryNoise();
        telemetry.addLine("Y: point forward. LB: manual steer. A: calibrate.");
        telemetry.update();
    }

    private void runPositionCaster() {
        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;
        double magnitude = Range.clip(Math.hypot(stickX, stickY), 0.0, 1.0);
        boolean released = magnitude < STICK_DEADZONE;

        double targetAngle;
        boolean foldFlipped = false;

        if (released) {
            targetAngle = heldAngle;
        } else {
            double stickAngle = normalize(Math.toDegrees(Math.atan2(stickX, stickY)));
            double foldedAngle = normalize(stickAngle + 180.0);
            double foldLimit = 90.0 + FOLD_HYSTERESIS_DEGREES;
            boolean wasReversed = driveReversedState;

            if (driveReversedState) {
                if (Math.abs(foldedAngle) > foldLimit) {
                    driveReversedState = false;
                }
            } else {
                if (Math.abs(stickAngle) > foldLimit) {
                    driveReversedState = true;
                }
            }

            foldFlipped = driveReversedState != wasReversed;
            targetAngle = driveReversedState ? foldedAngle : stickAngle;
            heldAngle = targetAngle;
        }

        double servoPosition = servoPositionForAngle(targetAngle);
        boolean clipped = Math.abs(servoPosition - rawServoPositionForAngle(targetAngle)) > 1e-6;

        double desiredDrive = 0.0;

        if (!released) {
            desiredDrive = magnitude * DRIVE_POWER_SCALE;

            if (driveReversedState) {
                desiredDrive = -desiredDrive;
            }
        }

        double drivePower = rampDrivePower(desiredDrive, targetAngle, foldFlipped);

        setPositionOutputs(servoPosition, drivePower);
        updateImpliedTravel(servoPosition);

        telemetry.addData("Mode", modeName);
        telemetry.addLine(limitLine);
        telemetry.addData("Stick", released ? "released" : String.format("x %.2f  y %.2f", stickX, stickY));
        telemetry.addData(released ? "Held angle" : "Target angle", "%.1f deg", targetAngle);
        telemetry.addData("Servo position", "%.3f%s", servoPosition, clipped ? "  (CLIPPED - out of travel)" : "");
        telemetry.addData("Servo centre", "%.3f", SERVO_CENTER + servoCenterTrim);

        if (steerEncoder != null) {
            telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
            telemetry.addData("Commanded vs measured", "%.1f -> %.1f deg (%.1f off)",
                    targetAngle, lastModuleAngle, normalize(lastModuleAngle - targetAngle));
            telemetry.addData("Implied travel", haveImpliedTravel
                    ? String.format("%.0f deg per full span (set %.0f)",
                            Math.abs(impliedTravelDegrees), SERVO_TRAVEL_DEGREES)
                    : "move away from centre to measure");

            if (haveImpliedTravel && impliedTravelDegrees < 0.0) {
                telemetry.addLine("Encoder counts the OTHER way from the servo.");
                telemetry.addLine("Use the positive number. Try ENCODER_REVERSED = true.");
            }
            telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
            telemetryNoise();
        } else {
            telemetry.addLine("No encoder: the measured angle cannot be shown.");
        }

        telemetry.addData("Drive power", "%.2f%s", drivePower,
                driveReversedState && !released ? "  (reversed)" : "");
        telemetry.addData("Homing", homeResult);
        telemetry.addLine("Y: centre the servo. LB: manual steer. A: calibrate.");
        telemetry.update();
    }

    private void runOpenLoopCrSteer() {
        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;

        double steerPower = Math.abs(stickX) > STICK_DEADZONE ? stickX : 0.0;
        double drivePower = Math.abs(stickY) > STICK_DEADZONE ? stickY * DRIVE_POWER_SCALE : 0.0;

        setCrOutputs(steerPower, drivePower);

        telemetry.addData("Mode", modeName);
        telemetry.addLine(limitLine);
        telemetry.addLine("Closed loop is impossible without swerve_encoder.");
        telemetry.addData("Stick x -> steer power", "%.2f", steerPower);
        telemetry.addData("Stick y -> drive power", "%.2f", drivePower);
        telemetry.addData("Homing", homeResult);
        telemetry.addLine("No angle is measured, so nothing can hold a heading.");
        telemetry.addLine("Plug the Axon feedback wire into an analog port named swerve_encoder.");
        telemetry.addLine("LB: manual steer. A: calibrate.");
        telemetry.update();
    }

    private void runManualOverride() {
        double stickX = gamepad1.left_stick_x;

        telemetry.addLine(">>> MANUAL STEER OVERRIDE (LB held) <<<");
        telemetry.addLine("No feedback at all. This proves the module can physically turn.");
        telemetry.addData("Mode", modeName);
        telemetry.addData("Left stick x", "%.2f", stickX);

        if (steerMode == MODE_POSITION) {
            double position = Range.clip(SERVO_CENTER + servoCenterTrim + stickX * SERVO_MANUAL_SPAN,
                    SERVO_MIN_POSITION, SERVO_MAX_POSITION);
            setPositionOutputs(position, 0.0);
            updateImpliedTravel(position);
            telemetry.addData("Servo position", "%.3f", position);
            telemetry.addLine("Stick x moves the servo position away from centre.");
        } else {
            setCrOutputs(stickX, 0.0);
            telemetry.addData("Servo power", "%.2f", stickX);
            telemetry.addLine("Stick x is raw servo power.");
        }

        if (steerEncoder != null) {
            telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
            telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
            telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
            telemetryNoise();
            telemetry.addLine("If the wheel turns but the angle does not, the feedback path is dead.");
        } else {
            telemetry.addLine("No encoder: watch the wheel itself.");
        }

        telemetry.addLine("The drive motor is off while LB is held.");
        telemetry.addLine("Release LB to go back to caster mode.");
        telemetry.update();

        seedHoldFromOutputs();
        steerStalled = false;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;
        stallTimer.reset();
        rampedDrivePower = 0.0;
        homing = false;
    }

    private void runHoming() {
        driveReversedState = false;
        heldAngle = 0.0;
        previousCommandAngle = 0.0;
        rampedDrivePower = 0.0;

        if (steerMode == MODE_POSITION) {
            double position = servoPositionForAngle(0.0);
            setPositionOutputs(position, 0.0);
            updateImpliedTravel(position);

            if (homeTimer.seconds() > HOME_SETTLE_SECONDS) {
                homing = false;
                homeResult = steerEncoder != null
                        ? String.format("centred, measured %.1f deg off", normalize(lastModuleAngle))
                        : "centred (no encoder to check)";
                captureTravelReference(position);
            }

            telemetry.addLine(">>> HOMING: COMMANDING SERVO CENTRE <<<");
            telemetry.addData("Servo position", "%.3f", position);
            telemetry.addData("Status", homeResult);

            if (steerEncoder != null) {
                telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
                telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
                telemetryNoise();
            }

            telemetry.addLine("Y homes again. LB steers by hand. A cuts the drive motor.");
            telemetry.update();
            return;
        }

        double homeError = normalize(0.0 - lastModuleAngle);
        double homePower = computeSteerPower(homeError);

        if (Math.abs(homeError) <= HOME_TOLERANCE_DEGREES) {
            homing = false;
            homeResult = String.format("done in %.1f s", homeTimer.seconds());
            homePower = 0.0;
        } else if (homeTimer.seconds() > HOME_TIMEOUT_SECONDS) {
            homing = false;
            homeResult = String.format("TIMEOUT, still %.1f deg off", homeError);
            homePower = 0.0;
        }

        setCrOutputs(homePower, 0.0);

        telemetry.addLine(">>> HOMING TO FORWARD <<<");
        telemetry.addData("Status", homeResult);
        telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
        telemetry.addData("Error to forward", "%.1f deg", homeError);
        telemetry.addData("Steer power", "%.2f", homePower);
        telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
        telemetryNoise();
        telemetry.addLine("Y homes again. LB steers by hand. A cuts all power.");
        telemetry.update();
    }

    private void runCalibration() {
        if (driveMotor != null) {
            driveMotor.setPower(0.0);
        }

        if (steerCrServo != null) {
            steerCrServo.setPower(0.0);
        }

        steerStalled = false;
        stallTargetAngle = lastModuleAngle;
        stallBestError = 0.0;
        stallTimer.reset();
        rampedDrivePower = 0.0;
        homing = false;

        telemetry.addLine(">>> CALIBRATION MODE <<<");
        telemetry.addData("Mode", modeName);

        if (steerMode == MODE_POSITION) {
            handleServoCentreTrim();
            setPositionOutputs(servoPositionForAngle(0.0), 0.0);
            telemetry.addLine("The drive motor is off.");
            telemetry.addLine("The servo is held at the trimmed centre while A is held,");
            telemetry.addLine("so dpad left/right moves the wheel as you watch.");
            telemetry.addData("Servo centre", "%.3f  (dpad left/right)", SERVO_CENTER + servoCenterTrim);
            telemetry.addData("Commanded position", "%.3f", lastServoPosition);
            telemetry.addLine("Copy Servo centre into SERVO_CENTER.");
            telemetry.addLine("A position servo keeps holding: unplug it to turn by hand.");
        } else {
            telemetry.addLine("All power is off. Turn the module by hand.");

            if (steerEncoder != null) {
                handleEncoderTrim();
                telemetry.addData("Offset", "%.1f deg  (dpad left/right)", offsetTrim);
                telemetry.addLine("Copy Raw encoder into ENCODER_OFFSET_DEGREES.");
            }
        }

        seedHoldFromOutputs();

        if (steerEncoder != null) {
            telemetry.addData("Raw encoder", "%.1f deg", lastRawAngle);
            telemetry.addData("Module angle", "%.1f deg", lastModuleAngle);
            telemetry.addData("Encoder volts", "%.3f V", lastVoltage);
            telemetry.addData("Volts seen", "%s - %s",
                    formatVoltage(minVoltageSeen), formatVoltage(maxVoltageSeen));
            telemetry.addData("Analog range in use", "%.2f - %.2f V",
                    ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE);
            telemetry.addData("Encoder reversed", ENCODER_REVERSED ? "true" : "false");
            telemetryNoise();
            if (steerMode != MODE_POSITION) {
                telemetry.addLine("Turn the wheel RIGHT by hand: Module angle must RISE.");
            }
        } else {
            telemetry.addLine("NO ENCODER: there is nothing to calibrate a zero against.");
            telemetry.addLine("Plug the feedback wire into an analog port named swerve_encoder.");
        }

        telemetry.update();
    }

    private void stopAll() {
        if (driveMotor != null) {
            driveMotor.setPower(0.0);
        }

        if (steerCrServo != null) {
            steerCrServo.setPower(0.0);
        }
    }

    private void setCrOutputs(double steerPower, double drivePower) {
        if (steerCrServo != null) {
            steerCrServo.setPower(Range.clip(steerPower, -1.0, 1.0));
        }

        if (driveMotor != null) {
            driveMotor.setPower(Range.clip(drivePower, -1.0, 1.0));
        }
    }

    private void setPositionOutputs(double position, double drivePower) {
        double clipped = Range.clip(position, SERVO_MIN_POSITION, SERVO_MAX_POSITION);

        if (steerPositionServo != null) {
            steerPositionServo.setPosition(clipped);
        }

        lastServoPosition = clipped;

        if (driveMotor != null) {
            driveMotor.setPower(Range.clip(drivePower, -1.0, 1.0));
        }
    }

    private double rawServoPositionForAngle(double angleDegrees) {
        double span = SERVO_TRAVEL_DEGREES;

        if (Math.abs(span) < 1e-6) {
            return SERVO_CENTER + servoCenterTrim;
        }

        return SERVO_CENTER + servoCenterTrim + (angleDegrees / span);
    }

    private double angleForServoPosition(double position) {
        double span = SERVO_TRAVEL_DEGREES;

        if (Math.abs(span) < 1e-6) {
            return 0.0;
        }

        return (position - (SERVO_CENTER + servoCenterTrim)) * span;
    }

    private void seedHoldFromOutputs() {
        heldAngle = steerMode == MODE_POSITION
                ? angleForServoPosition(lastServoPosition)
                : lastModuleAngle;
        previousCommandAngle = heldAngle;
        driveReversedState = false;
    }

    private double servoPositionForAngle(double angleDegrees) {
        return Range.clip(rawServoPositionForAngle(angleDegrees), SERVO_MIN_POSITION, SERVO_MAX_POSITION);
    }

    private double rampDrivePower(double desiredDrive, double targetAngle, boolean foldFlipped) {
        double jump = Math.abs(normalize(targetAngle - previousCommandAngle));
        previousCommandAngle = targetAngle;

        if (foldFlipped) {
            rampedDrivePower = 0.0;
        } else if (jump > SERVO_SETTLE_DEGREES) {
            double keep = Range.clip(
                    1.0 - ((jump - SERVO_SETTLE_DEGREES) / SERVO_JUMP_FULL_CUT_DEGREES), 0.0, 1.0);
            rampedDrivePower *= keep;
        }

        double step = DRIVE_RAMP_PER_SECOND * Math.max(loopDt, 0.0);
        double delta = Range.clip(desiredDrive - rampedDrivePower, -step, step);
        rampedDrivePower += delta;

        if (Math.abs(desiredDrive) < 1e-6) {
            rampedDrivePower = 0.0;
        }

        return rampedDrivePower;
    }

    private void captureTravelReference(double position) {
        if (steerEncoder == null) {
            return;
        }

        travelReferencePosition = position;
        travelReferenceUnwrapped = unwrappedAngle;
        haveTravelReference = true;
    }

    private void updateImpliedTravel(double position) {
        if (steerEncoder == null || !haveTravelReference) {
            return;
        }

        double positionSpan = position - travelReferencePosition;

        if (Math.abs(positionSpan) < IMPLIED_TRAVEL_MIN_SPAN) {
            return;
        }

        impliedTravelDegrees = (unwrappedAngle - travelReferenceUnwrapped) / positionSpan;
        haveImpliedTravel = true;
    }

    private void sampleEncoder() {
        if (steerEncoder == null) {
            angularVelocity = 0.0;
            return;
        }

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
        unwrappedAngle += angleDelta;
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

    private double noisePeakToPeakDegrees() {
        double span = ANALOG_MAX_VOLTAGE - ANALOG_MIN_VOLTAGE;
        return span > 0.0 ? noisePeakToPeakVolts / span * 360.0 : 0.0;
    }

    private void telemetryNoise() {
        if (steerEncoder == null) {
            return;
        }

        telemetry.addData("Noise p-p", "%.3f V  (%.1f deg) over %.0f s",
                noisePeakToPeakVolts, noisePeakToPeakDegrees(), NOISE_WINDOW_SECONDS);

        if (Math.abs(lastModuleAngle) > 150.0) {
            telemetry.addLine("Near the +/-180 wrap: flipping between +179 and -180");
            telemetry.addLine("is normal here. Set ENCODER_OFFSET_DEGREES to move it.");
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

    private void handleEncoderTrim() {
        if (gamepad1.dpadRightWasPressed()) {
            offsetTrim += TRIM_STEP_DEGREES;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            offsetTrim -= TRIM_STEP_DEGREES;
        }

        offsetTrim = ((offsetTrim % 360.0) + 360.0) % 360.0;
    }

    private void handleServoCentreTrim() {
        if (gamepad1.dpadRightWasPressed()) {
            servoCenterTrim += SERVO_TRIM_STEP;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            servoCenterTrim -= SERVO_TRIM_STEP;
        }

        servoCenterTrim = Range.clip(servoCenterTrim,
                SERVO_MIN_POSITION - SERVO_CENTER, SERVO_MAX_POSITION - SERVO_CENTER);
    }
}
