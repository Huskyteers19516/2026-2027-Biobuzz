package org.firstinspires.ftc.teamcode.launcher;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleOp.RobotTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

public class Launcher {

    public enum ControlMode { CUSTOM, HUB_PIDF }

    public enum State { OFF, SPINNING_UP, READY, FEEDING, CLOSING, RECOVERING, MANUAL_OPEN }

    public static final double DEFAULT_TARGET_VELOCITY = 1100.0;
    public static final double MIN_TARGET_VELOCITY = 0.0;
    public static final double MAX_TARGET_VELOCITY = 2800.0;

    public static final ControlMode DEFAULT_CONTROL_MODE = ControlMode.CUSTOM;
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double DEFAULT_KS = 0.04;
    public static final double DEFAULT_KV = 1.0 / 2600.0;
    public static final double DEFAULT_KP = 0.0015;
    public static final double DEFAULT_KI = 0.0005;
    public static final double INTEGRAL_ZONE = 100.0;
    public static final double INTEGRAL_LIMIT = 0.15;

    public static final boolean DEFAULT_RECOVERY_BOOST = true;
    public static final double RECOVERY_BOOST_ENTER = 100.0;
    public static final double RECOVERY_BOOST_EXIT = 70.0;

    public static final boolean HUB_OVERRIDE_PIDF = false;
    public static final double HUB_P = 10.0;
    public static final double HUB_I = 3.0;
    public static final double HUB_D = 0.0;
    public static final double HUB_F = 0.0;

    public static final double DEFAULT_READY_TOLERANCE = 40.0;
    public static final double DEFAULT_READY_STABLE_SECONDS = 0.12;
    public static final double DEFAULT_SHOT_DETECT_DROP = 60.0;
    public static final int SHOT_DETECT_SAMPLES = 2;
    public static final double DEFAULT_FEED_MAX_SECONDS = 0.5;
    public static final double MIN_FIRE_VELOCITY = 300.0;
    public static final double TELEOP_QUEUE_EXPIRE_SECONDS = 3.0;
    public static final int BURST_SIZE = 3;
    public static final double DEFAULT_PENDING_STAGING_POWER = 0.4;
    public static final double DEFAULT_BLOCKER_CLOSE_SECONDS = 0.15;
    public static final double DEFAULT_FEED_TRANSFER_POWER = 1.0;
    public static final double DEFAULT_FEED_SETPOINT_BOOST = 0.0;
    public static final double DEFAULT_FEED_SETPOINT_BOOST_RAMP_SECONDS = 0.3;

    public static final double AUTO_READY_TIMEOUT_SECONDS = 2.5;
    public static final double AUTO_SHOT_TIMEOUT_SECONDS = 4.0;
    public static final boolean AUTO_KEEP_SPINNING = false;
    public static final double AUTO_FEED_INTAKE_POWER = 1.0;
    public static final double AUTO_STAGING_TRANSFER_POWER = 0.4;
    public static final int AUTO_EXTRA_ATTEMPTS = 2;

    public static final double BLOCKER_CLOSED_POSITION = RobotTeleOp.BLOCKER_CLOSED_POSITION;
    public static final double BLOCKER_OPEN_POSITION = RobotTeleOp.BLOCKER_OPEN_POSITION;

    private static final double VOLTAGE_READ_SECONDS = 0.1;
    private static final double VOLTAGE_FILTER = 0.3;
    private static final double MIN_VALID_VOLTAGE = 7.0;
    private static final double ENCODER_FAULT_POWER = 0.5;
    private static final double ENCODER_FAULT_VELOCITY = 50.0;
    private static final double ENCODER_FAULT_SECONDS = 1.0;
    private static final double FAULT_SPINUP_SECONDS = 2.0;
    private static final double FAULT_RECOVERY_SECONDS = 1.0;
    private static final double ENCODER_REVERSED_SECONDS = 0.5;
    private static final int SHOT_HISTORY = 50;
    private static final double REWRITE_SECONDS = 0.25;

    public static final class Shot {
        public final int number;
        public final double startTime;
        public final double target;
        public final double preVelocity;
        public final double voltage;
        public final boolean forced;
        public int burstIndex = 0;
        public int burstSize = 0;
        public ControlMode mode;
        public boolean recoveryBoost;
        public double kS;
        public double kV;
        public double kP;
        public double kI;
        public double feedSetpointBoost;
        public double readyTolerance;
        public double shotDetectDrop;
        public double feedMaxSeconds;
        public double feedTransferPower;
        public double pendingStagingPower;
        public boolean detected = false;
        public double detectSeconds = -1.0;
        public double feedSeconds = 0.0;
        public double minVelocity;
        public double endTime = -1.0;
        public double recoverySeconds = -1.0;
        public boolean complete = false;
        public String mark = "";

        Shot(int number, double startTime, double target, double preVelocity, double voltage, boolean forced) {
            this.number = number;
            this.startTime = startTime;
            this.target = target;
            this.preVelocity = preVelocity;
            this.voltage = voltage;
            this.forced = forced;
            this.minVelocity = preVelocity;
        }

        public double dip() {
            return preVelocity - minVelocity;
        }

        public String summary() {
            return String.format("#%d%s pre %.0f dip %.0f %s rec %s%s%s",
                    number, burstIndex > 0 ? String.format(" (%d/%d)", burstIndex, burstSize) : "",
                    preVelocity, dip(),
                    detected ? String.format("det %.2fs", detectSeconds) : "NO DIP",
                    recoverySeconds >= 0 ? String.format("%.2fs", recoverySeconds) : "...",
                    forced ? " FORCED" : "",
                    mark.isEmpty() ? "" : " " + mark);
        }
    }

    private final DcMotorEx outtake;
    private final DcMotor transfer;
    private final Servo blocker;
    private VoltageSensor voltageSensor = null;
    private final ElapsedTime clock = new ElapsedTime();

    public double kS = DEFAULT_KS;
    public double kV = DEFAULT_KV;
    public double kP = DEFAULT_KP;
    public double kI = DEFAULT_KI;
    public boolean recoveryBoost = DEFAULT_RECOVERY_BOOST;
    public double recoveryBoostEnter = RECOVERY_BOOST_ENTER;
    public double recoveryBoostExit = RECOVERY_BOOST_EXIT;
    public double readyTolerance = DEFAULT_READY_TOLERANCE;
    public double readyStableSeconds = DEFAULT_READY_STABLE_SECONDS;
    public double shotDetectDrop = DEFAULT_SHOT_DETECT_DROP;
    public double feedMaxSeconds = DEFAULT_FEED_MAX_SECONDS;
    public double blockerCloseSeconds = DEFAULT_BLOCKER_CLOSE_SECONDS;
    public double feedTransferPower = DEFAULT_FEED_TRANSFER_POWER;
    public double pendingStagingPower = DEFAULT_PENDING_STAGING_POWER;
    public double feedSetpointBoost = DEFAULT_FEED_SETPOINT_BOOST;
    public double feedSetpointBoostRampSeconds = DEFAULT_FEED_SETPOINT_BOOST_RAMP_SECONDS;
    public double readyTimeoutSeconds = Double.POSITIVE_INFINITY;
    public double queueExpireSeconds = Double.POSITIVE_INFINITY;

    private ControlMode controlMode;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean enabled = false;
    private boolean firingHeld = false;
    private boolean manualOpen = false;
    private int queuedShots = 0;
    private double stagingTransferPower = 0.0;
    private Double openLoopPower = null;

    private State state = State.OFF;
    private double now = 0.0;
    private double lastTime = 0.0;
    private double measured = 0.0;
    private double commandedVelocity = 0.0;
    private double power = 0.0;
    private double integral = 0.0;
    private boolean boosting = false;
    private double voltage = NOMINAL_VOLTAGE;
    private double lastVoltageRead = -1.0;
    private double inWindowSince = -1.0;
    private boolean ready = false;
    private double waitingSince = -1.0;
    private double enabledSince = -1.0;
    private double spinUpSeconds = -1.0;
    private double stateSince = 0.0;
    private boolean encoderFault = false;
    private double faultSince = -1.0;
    private double faultReadyAt = Double.POSITIVE_INFINITY;
    private boolean encoderReversed = false;
    private double reversedSince = -1.0;
    private int detectCount = 0;
    private double queuedAt = -1.0;
    private int burstSize = 0;
    private int burstFired = 0;
    private double lastPowerWritten = Double.NaN;
    private double lastVelocityWritten = Double.NaN;
    private double lastBlockerWritten = Double.NaN;
    private double lastTransferWritten = Double.NaN;
    private double powerWrittenAt = -1.0;
    private double velocityWrittenAt = -1.0;
    private double blockerWrittenAt = -1.0;
    private double transferWrittenAt = -1.0;

    private int shotsAttempted = 0;
    private int shotsDetected = 0;
    private Shot activeShot = null;
    private final List<Shot> history = new ArrayList<>();

    public Launcher(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (voltageSensor == null && sensor.getVoltage() > MIN_VALID_VOLTAGE) {
                voltageSensor = sensor;
            }
        }

        outtake = hardwareMap.get(DcMotorEx.class, Constants.OUTTAKE);
        outtake.setDirection(RobotTeleOp.OUTTAKE_DIRECTION);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        transfer = hardwareMap.get(DcMotor.class, Constants.TRANSFER);
        transfer.setDirection(RobotTeleOp.TRANSFER_DIRECTION);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0.0);

        blocker = hardwareMap.get(Servo.class, Constants.OUTTAKE_BLOCKER);

        setControlMode(DEFAULT_CONTROL_MODE);
        outtake.setPower(0.0);
        readVoltage();
        clock.reset();
    }

    public void setControlMode(ControlMode mode) {
        controlMode = mode;
        integral = 0.0;
        boosting = false;
        lastPowerWritten = Double.NaN;
        lastVelocityWritten = Double.NaN;
        if (mode == ControlMode.HUB_PIDF) {
            outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (HUB_OVERRIDE_PIDF) {
                outtake.setVelocityPIDFCoefficients(HUB_P, HUB_I, HUB_D, HUB_F);
            }
        } else {
            outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public ControlMode controlMode() {
        return controlMode;
    }

    public PIDFCoefficients hubCoefficients() {
        return outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTargetVelocity(double ticksPerSecond) {
        targetVelocity = Range.clip(ticksPerSecond, MIN_TARGET_VELOCITY, MAX_TARGET_VELOCITY);
    }

    public double targetVelocity() {
        return targetVelocity;
    }

    public void spinUp() {
        if (!enabled) {
            encoderReversed = false;
            reversedSince = -1.0;
            if (encoderFault) {
                faultReadyAt = now + FAULT_SPINUP_SECONDS;
            }
            enabled = true;
            enabledSince = now;
            spinUpSeconds = -1.0;
            inWindowSince = -1.0;
            ready = false;
        }
    }

    public void stop() {
        enabled = false;
        firingHeld = false;
        queuedShots = 0;
        waitingSince = -1.0;
        if (activeShot != null && !activeShot.complete) {
            if (state == State.FEEDING) {
                finishFeed(false);
            }
            activeShot.complete = true;
        }
        setState(State.OFF);
    }

    public void toggle() {
        if (enabled) {
            stop();
        } else {
            spinUp();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void requestShots(int count) {
        if (count <= 0) {
            return;
        }
        spinUp();
        if (queuedShots == 0) {
            queuedAt = now;
            burstSize = 0;
            burstFired = 0;
        }
        queuedShots += count;
        burstSize += count;
    }

    public void setFiring(boolean held) {
        if (held) {
            spinUp();
        }
        firingHeld = held;
    }

    public void cancelShots() {
        queuedShots = 0;
        firingHeld = false;
        waitingSince = -1.0;
    }

    public void setStagingTransferPower(double power) {
        stagingTransferPower = Range.clip(power, -1.0, 1.0);
    }

    public void setManualBlockerOpen(boolean open) {
        manualOpen = open;
    }

    public void setOpenLoopPower(Double power) {
        openLoopPower = power;
        integral = 0.0;
        boosting = false;
    }

    public void setReadyTimeoutSeconds(double seconds) {
        readyTimeoutSeconds = seconds;
    }

    public void setQueueExpireSeconds(double seconds) {
        queueExpireSeconds = seconds;
    }

    public void sense() {
        now = clock.seconds();
        lastTime = now;
        measured = outtake.getVelocity();
        if (lastVoltageRead < 0 || now - lastVoltageRead >= VOLTAGE_READ_SECONDS) {
            readVoltage();
        }
    }

    public void update() {
        double previous = lastTime;
        sense();
        double dt = Math.max(0.0, now - previous);

        updateReady();
        updateState();
        writeMechanism();
        writeFlywheel(dt);
        updateEncoderFault();
    }

    private void updateReady() {
        if (!enabled || encoderReversed || targetVelocity < MIN_FIRE_VELOCITY) {
            inWindowSince = -1.0;
            ready = false;
            return;
        }
        if (encoderFault) {
            ready = now >= faultReadyAt;
        } else {
            boolean inWindow = Math.abs(measured - targetVelocity) <= readyTolerance;
            if (!inWindow) {
                inWindowSince = -1.0;
            } else if (inWindowSince < 0) {
                inWindowSince = now;
            }
            ready = inWindow && now - inWindowSince >= readyStableSeconds;
        }
        if (ready && spinUpSeconds < 0) {
            spinUpSeconds = now - enabledSince;
        }
    }

    private void updateState() {
        if (activeShot != null && !activeShot.complete) {
            activeShot.minVelocity = Math.min(activeShot.minVelocity, measured);
        }

        if (manualOpen) {
            if (state == State.FEEDING) {
                finishFeed(false);
            }
            setState(State.MANUAL_OPEN);
            return;
        }
        if (!enabled) {
            setState(State.OFF);
            return;
        }

        if (state == State.FEEDING) {
            Shot shot = activeShot;
            double feedTime = now - shot.startTime;
            double reference = Math.min(shot.preVelocity, targetVelocity);
            if (!encoderFault && measured <= reference - shotDetectDrop) {
                detectCount++;
            } else {
                detectCount = 0;
            }
            if (detectCount >= SHOT_DETECT_SAMPLES) {
                shot.detected = true;
                shot.detectSeconds = feedTime;
                finishFeed(true);
            } else if (feedTime >= feedMaxSeconds) {
                finishFeed(false);
            }
            return;
        }
        if (state == State.CLOSING) {
            if (now - stateSince >= blockerCloseSeconds) {
                setState(State.RECOVERING);
            } else {
                return;
            }
        }

        if (activeShot != null && !activeShot.complete && ready) {
            activeShot.recoverySeconds = now - activeShot.endTime;
            activeShot.complete = true;
        }

        if (queuedShots > 0 && now - queuedAt >= queueExpireSeconds) {
            queuedShots = 0;
        }
        boolean wantShot = queuedShots > 0 || firingHeld;
        if (!wantShot) {
            waitingSince = -1.0;
        } else if (waitingSince < 0) {
            waitingSince = now;
        }

        boolean canForce = !encoderReversed && targetVelocity >= MIN_FIRE_VELOCITY
                && measured <= targetVelocity + readyTolerance;
        boolean timedOut = wantShot && canForce && now - waitingSince >= readyTimeoutSeconds;
        if (wantShot && (ready || timedOut)) {
            startFeed(!ready || encoderFault);
            return;
        }

        if (ready) {
            setState(State.READY);
        } else if (activeShot != null && !activeShot.complete) {
            setState(State.RECOVERING);
        } else {
            setState(State.SPINNING_UP);
        }
    }

    private void startFeed(boolean forced) {
        if (activeShot != null && !activeShot.complete) {
            activeShot.complete = true;
        }
        activeShot = new Shot(shotsAttempted + 1, now, targetVelocity, measured, voltage, forced);
        activeShot.mode = controlMode;
        activeShot.recoveryBoost = recoveryBoost;
        activeShot.kS = kS;
        activeShot.kV = kV;
        activeShot.kP = kP;
        activeShot.kI = kI;
        activeShot.feedSetpointBoost = feedSetpointBoost;
        activeShot.readyTolerance = readyTolerance;
        activeShot.shotDetectDrop = shotDetectDrop;
        activeShot.feedMaxSeconds = feedMaxSeconds;
        activeShot.feedTransferPower = feedTransferPower;
        activeShot.pendingStagingPower = pendingStagingPower;
        history.add(activeShot);
        if (history.size() > SHOT_HISTORY) {
            history.remove(0);
        }
        if (queuedShots > 0) {
            queuedShots--;
            burstFired++;
            activeShot.burstIndex = burstFired;
            activeShot.burstSize = burstSize;
        }
        queuedAt = now;
        waitingSince = -1.0;
        detectCount = 0;
        setState(State.FEEDING);
    }

    private void finishFeed(boolean detected) {
        Shot shot = activeShot;
        shot.feedSeconds = now - shot.startTime;
        shot.endTime = now;
        shotsAttempted++;
        if (detected) {
            shotsDetected++;
        }
        if (encoderFault) {
            faultReadyAt = now + FAULT_RECOVERY_SECONDS;
        }
        setState(State.CLOSING);
        if (!detected && encoderFault) {
            shot.complete = true;
        }
    }

    private void setState(State next) {
        if (state != next) {
            state = next;
            stateSince = now;
        }
    }

    private void writeMechanism() {
        boolean open = state == State.FEEDING || state == State.MANUAL_OPEN;
        writeBlocker(open ? BLOCKER_OPEN_POSITION : BLOCKER_CLOSED_POSITION);

        double transferPower;
        if (state == State.FEEDING) {
            transferPower = feedTransferPower;
        } else if (state == State.CLOSING) {
            transferPower = 0.0;
        } else {
            transferPower = stagingTransferPower;
            boolean shotPending = queuedShots > 0 || firingHeld;
            if (shotPending && state != State.MANUAL_OPEN && state != State.OFF && stagingTransferPower >= 0.0) {
                transferPower = Math.max(transferPower, pendingStagingPower);
            }
        }
        writeTransfer(transferPower);
    }

    private void writeFlywheel(double dt) {
        double boost = 0.0;
        if (state == State.FEEDING && feedSetpointBoost != 0.0) {
            double ramp = feedSetpointBoostRampSeconds <= 0 ? 1.0
                    : Range.clip((now - activeShot.startTime) / feedSetpointBoostRampSeconds, 0.0, 1.0);
            boost = feedSetpointBoost * ramp;
        }
        commandedVelocity = enabled ? Range.clip(targetVelocity + boost, MIN_TARGET_VELOCITY, MAX_TARGET_VELOCITY) : 0.0;

        if (openLoopPower != null) {
            commandedVelocity = 0.0;
            power = Range.clip(openLoopPower, 0.0, 1.0);
            if (controlMode == ControlMode.HUB_PIDF) {
                setControlMode(ControlMode.CUSTOM);
            }
            writePower(power);
            return;
        }

        if ((encoderFault || encoderReversed) && controlMode == ControlMode.HUB_PIDF) {
            setControlMode(ControlMode.CUSTOM);
        }

        if (encoderReversed) {
            integral = 0.0;
            boosting = false;
            power = 0.0;
            writePower(0.0);
            return;
        }

        if (controlMode == ControlMode.HUB_PIDF) {
            power = 0.0;
            writeVelocity(commandedVelocity);
            return;
        }

        if (commandedVelocity <= 0.0) {
            integral = 0.0;
            boosting = false;
            power = 0.0;
            writePower(0.0);
            return;
        }

        double error = commandedVelocity - measured;
        double feedforward = (kS + kV * commandedVelocity) * NOMINAL_VOLTAGE / voltage;

        if (encoderFault) {
            integral = 0.0;
            boosting = false;
            power = Range.clip(feedforward, 0.0, 1.0);
            writePower(power);
            return;
        }

        if (Math.abs(error) <= INTEGRAL_ZONE) {
            integral = Range.clip(integral + kI * error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }

        if (recoveryBoost) {
            if (error > recoveryBoostEnter) {
                boosting = true;
            } else if (error < recoveryBoostExit) {
                boosting = false;
            }
        } else {
            boosting = false;
        }

        power = boosting ? 1.0 : Range.clip(feedforward + kP * error + integral, 0.0, 1.0);
        writePower(power);
    }

    private void updateEncoderFault() {
        boolean driving = controlMode == ControlMode.CUSTOM ? power >= ENCODER_FAULT_POWER : commandedVelocity > 0.0;
        if (driving && measured <= -ENCODER_FAULT_VELOCITY) {
            if (reversedSince < 0) {
                reversedSince = now;
            } else if (now - reversedSince >= ENCODER_REVERSED_SECONDS) {
                encoderReversed = true;
            }
        } else {
            reversedSince = -1.0;
        }
        if (driving && Math.abs(measured) < ENCODER_FAULT_VELOCITY) {
            if (faultSince < 0) {
                faultSince = now;
            } else if (now - faultSince >= ENCODER_FAULT_SECONDS && !encoderFault) {
                encoderFault = true;
                faultReadyAt = now + FAULT_SPINUP_SECONDS;
            }
        } else {
            faultSince = -1.0;
            if (Math.abs(measured) >= ENCODER_FAULT_VELOCITY) {
                encoderFault = false;
            }
        }
    }

    private void readVoltage() {
        if (voltageSensor != null) {
            double reading = voltageSensor.getVoltage();
            if (reading > MIN_VALID_VOLTAGE) {
                voltage = lastVoltageRead < 0 ? reading : voltage + VOLTAGE_FILTER * (reading - voltage);
            }
        }
        lastVoltageRead = now;
    }

    private boolean stale(double value, double last, double writtenAt) {
        return value != last || writtenAt < 0 || now - writtenAt >= REWRITE_SECONDS;
    }

    private void writePower(double value) {
        if (stale(value, lastPowerWritten, powerWrittenAt)) {
            outtake.setPower(value);
            lastPowerWritten = value;
            powerWrittenAt = now;
        }
    }

    private void writeVelocity(double value) {
        if (stale(value, lastVelocityWritten, velocityWrittenAt)) {
            outtake.setVelocity(value);
            lastVelocityWritten = value;
            velocityWrittenAt = now;
        }
    }

    private void writeBlocker(double position) {
        if (stale(position, lastBlockerWritten, blockerWrittenAt)) {
            blocker.setPosition(position);
            lastBlockerWritten = position;
            blockerWrittenAt = now;
        }
    }

    private void writeTransfer(double value) {
        if (stale(value, lastTransferWritten, transferWrittenAt)) {
            transfer.setPower(value);
            lastTransferWritten = value;
            transferWrittenAt = now;
        }
    }

    public void shutdown() {
        stop();
        openLoopPower = null;
        outtake.setPower(0.0);
        if (controlMode == ControlMode.HUB_PIDF) {
            outtake.setVelocity(0.0);
        }
        transfer.setPower(0.0);
        blocker.setPosition(BLOCKER_CLOSED_POSITION);
        lastPowerWritten = 0.0;
        lastVelocityWritten = 0.0;
        lastTransferWritten = 0.0;
        lastBlockerWritten = BLOCKER_CLOSED_POSITION;
        powerWrittenAt = now;
        velocityWrittenAt = now;
        transferWrittenAt = now;
        blockerWrittenAt = now;
    }

    public State state() {
        return state;
    }

    public boolean isReady() {
        return ready;
    }

    public boolean isFeeding() {
        return state == State.FEEDING;
    }

    public boolean isBusy() {
        return state == State.FEEDING || state == State.CLOSING;
    }

    public boolean isBoosting() {
        return boosting;
    }

    public boolean encoderFault() {
        return encoderFault;
    }

    public boolean encoderReversed() {
        return encoderReversed;
    }

    public boolean canFire() {
        return enabled && !encoderReversed && targetVelocity >= MIN_FIRE_VELOCITY;
    }

    public double measuredVelocity() {
        return measured;
    }

    public double commandedVelocity() {
        return commandedVelocity;
    }

    public double power() {
        return power;
    }

    public double batteryVoltage() {
        return voltage;
    }

    public double spinUpSeconds() {
        return spinUpSeconds;
    }

    public double time() {
        return now;
    }

    public int queuedShots() {
        return queuedShots;
    }

    public int shotsAttempted() {
        return shotsAttempted;
    }

    public int shotsDetected() {
        return shotsDetected;
    }

    public Shot activeShot() {
        return activeShot;
    }

    public Shot lastShot() {
        return history.isEmpty() ? null : history.get(history.size() - 1);
    }

    public List<Shot> history() {
        return history;
    }

    public void clearHistory() {
        history.clear();
    }

    public String statusText() {
        if (encoderReversed) {
            return "ENCODER REVERSED - MOTOR STOPPED";
        }
        if (enabled && targetVelocity < MIN_FIRE_VELOCITY) {
            return "TARGET TOO LOW TO FIRE";
        }
        if (encoderFault) {
            return "ENCODER FAULT";
        }
        switch (state) {
            case OFF:
                return "OFF";
            case SPINNING_UP:
                return "SPINNING UP - WAIT";
            case READY:
                return "READY TO FIRE";
            case FEEDING:
                return "FIRING";
            case CLOSING:
                return "CLOSING BLOCKER";
            case RECOVERING:
                return "RECOVERING - WAIT";
            case MANUAL_OPEN:
                return "BLOCKER MANUALLY OPEN";
            default:
                return state.toString();
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Launcher", statusText());
        telemetry.addData("Velocity", "target %.0f  measured %.0f  error %+.0f",
                commandedVelocity, measured, commandedVelocity - measured);
        if (controlMode == ControlMode.CUSTOM) {
            telemetry.addData("Motor", "power %.2f%s  battery %.1f V",
                    power, boosting ? "  RECOVERY BOOST" : "", voltage);
        } else {
            telemetry.addData("Motor", "hub PIDF  battery %.1f V", voltage);
        }
        telemetry.addData("Shots", "fired %d  queued %d%s%s",
                shotsAttempted, queuedShots, firingHeld ? "  (holding fire)" : "",
                queuedShots > 0 && burstSize > 1 ? String.format("  burst %d / %d", burstFired, burstSize) : "");
        Shot last = lastShot();
        if (last != null) {
            telemetry.addData("Last shot", last.summary());
        }
        if (encoderFault) {
            telemetry.addLine("!! outtake encoder reads ~0 while powered - running open loop, check the encoder cable");
        }
        if (encoderReversed) {
            telemetry.addLine("!! outtake velocity is NEGATIVE while powered forward - encoder on the wrong port or OUTTAKE_DIRECTION wrong");
        }
    }
}
