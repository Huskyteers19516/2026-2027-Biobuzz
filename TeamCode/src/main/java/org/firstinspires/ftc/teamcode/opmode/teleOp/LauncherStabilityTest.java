package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.launcher.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Launcher Stability Test", group = "Test")
public class LauncherStabilityTest extends OpMode {

    private static final double STEADY_WINDOW_SECONDS = 2.0;
    private static final double PRE_FEED_TRACE_SECONDS = 0.3;
    private static final double LOAD_POWER = 1.0;
    private static final int RECENT_SHOTS_SHOWN = 5;
    private static final int MAX_TRACE_SAMPLES = 800;

    private static final double[] SWEEP_POWERS = {0.2, 0.35, 0.5, 0.65, 0.8, 1.0};
    private static final double SWEEP_STEP_SECONDS = 3.0;
    private static final double SWEEP_SAMPLE_SECONDS = 1.0;
    private static final double SWEEP_MIN_VELOCITY = 150.0;

    private static final String[] MARKS = {"GOOD", "SHORT", "LONG"};

    private static final String SHOTS_FILE = "launcher_shots.csv";
    private static final String TRACE_FILE = "launcher_trace.csv";
    private static final String MARKS_FILE = "launcher_marks.csv";
    private static final String SHOTS_HEADER = "session,shot,burst_index,burst_size,mode,boost,target,pre_velocity,min_velocity,dip,detected,"
            + "detect_s,feed_s,recovery_s,forced,voltage,kP,kI,kV,kS,setpoint_boost,ready_tolerance,detect_drop,"
            + "feed_max_s,feed_power,staging_power\n";
    private static final String TRACE_HEADER = "session,shot,t,commanded,measured,power,state,voltage\n";
    private static final String MARKS_HEADER = "session,shot,mark\n";

    private enum Setting {
        TARGET("target velocity", 50.0),
        KP("kP", 0.0005),
        KI("kI", 0.0001),
        KV("kV", 0.00001),
        KS("kS", 0.01),
        READY_TOLERANCE("ready tolerance", 5.0),
        READY_STABLE("ready stable seconds", 0.02),
        DETECT_DROP("shot detect drop", 10.0),
        FEED_MAX("feed max seconds", 0.05),
        FEED_POWER("feed transfer power", 0.1),
        STAGING_POWER("staging power between shots", 0.05),
        BLOCKER_CLOSE("blocker close seconds", 0.05),
        SETPOINT_BOOST("feed setpoint boost", 25.0),
        BOOST_ENTER("recovery boost enter", 10.0),
        BOOST_EXIT("recovery boost exit", 5.0);

        final String label;
        final double step;

        Setting(String label, double step) {
            this.label = label;
            this.step = step;
        }
    }

    private static final class Sample {
        final double t;
        final double commanded;
        final double measured;
        final double power;
        final String state;
        final double voltage;

        Sample(double t, double commanded, double measured, double power, String state, double voltage) {
            this.t = t;
            this.commanded = commanded;
            this.measured = measured;
            this.power = power;
            this.state = state;
            this.voltage = voltage;
        }
    }

    private Launcher launcher;
    private DcMotor intake;
    private PIDFCoefficients hubCoefficients;
    private ExecutorService writer;

    private int settingIndex = 0;
    private boolean triggerLatched = false;
    private final ArrayDeque<Double> loopTimes = new ArrayDeque<>();
    private final ArrayDeque<double[]> steadySamples = new ArrayDeque<>();
    private final ArrayDeque<Sample> recentSamples = new ArrayDeque<>();

    private boolean sweeping = false;
    private int sweepStep = 0;
    private double sweepStepStart = 0.0;
    private double sweepSumVelocity = 0.0;
    private double sweepSumVoltage = 0.0;
    private int sweepSamples = 0;
    private final List<double[]> sweepPoints = new ArrayList<>();
    private Launcher.ControlMode modeBeforeSweep = Launcher.DEFAULT_CONTROL_MODE;
    private String sweepResult = "not run";

    private final long sessionId = System.currentTimeMillis();
    private int loggedThroughShot = 0;
    private int tracedShot = 0;
    private final StringBuilder traceBuffer = new StringBuilder();
    private int traceSamples = 0;
    private volatile String logStatus = "no shots logged yet";

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        launcher.setQueueExpireSeconds(Launcher.TELEOP_QUEUE_EXPIRE_SECONDS);
        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE);
        intake.setDirection(RobotTeleOp.INTAKE_DIRECTION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setPower(0.0);
        hubCoefficients = launcher.hubCoefficients();
        writer = Executors.newSingleThreadExecutor();

        telemetry.addData("Outtake velocity", "%.0f ticks/s", () -> launcher.measuredVelocity());
    }

    @Override
    public void init_loop() {
        launcher.sense();
        telemetry.addLine("Launcher Stability Test - motors move only after START");
        telemetry.addLine("Logs: " + new File(AppUtil.FIRST_FOLDER, SHOTS_FILE).getPath());
        showControls();
    }

    @Override
    public void loop() {
        handleButtons();
        if (sweeping) {
            runSweep();
        }
        launcher.update();
        recordSamples();
        showTelemetry();
    }

    @Override
    public void stop() {
        launcher.shutdown();
        intake.setPower(0.0);
        logFinishedShots(true);
        writer.shutdown();
        try {
            writer.awaitTermination(500, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void handleButtons() {
        boolean fireHeld = gamepad1.right_trigger >= 0.5;
        boolean firePressed = fireHeld && !triggerLatched;
        if (gamepad1.right_trigger >= 0.5) {
            triggerLatched = true;
        } else if (gamepad1.right_trigger <= 0.3) {
            triggerLatched = false;
        }

        boolean togglePressed = gamepad1.yWasPressed();
        boolean modePressed = gamepad1.xWasPressed();
        boolean boostPressed = gamepad1.bWasPressed() && !gamepad1.start;
        boolean sweepPressed = gamepad1.aWasPressed() && !gamepad1.start;
        boolean goodPressed = gamepad1.rightBumperWasPressed();
        boolean shortPressed = gamepad1.leftBumperWasPressed();
        boolean longPressed = gamepad1.leftStickButtonWasPressed();
        boolean burstPressed = gamepad1.rightStickButtonWasPressed();
        boolean resetPressed = gamepad1.backWasPressed();
        boolean leftPressed = gamepad1.dpadLeftWasPressed();
        boolean rightPressed = gamepad1.dpadRightWasPressed();
        boolean upPressed = gamepad1.dpadUpWasPressed();
        boolean downPressed = gamepad1.dpadDownWasPressed();

        if (sweepPressed) {
            if (sweeping) {
                endSweep("aborted");
            } else {
                startSweep();
            }
        }
        if (sweeping) {
            return;
        }

        if (togglePressed) {
            launcher.toggle();
        }
        if (modePressed) {
            launcher.setControlMode(launcher.controlMode() == Launcher.ControlMode.CUSTOM
                    ? Launcher.ControlMode.HUB_PIDF : Launcher.ControlMode.CUSTOM);
            hubCoefficients = launcher.hubCoefficients();
        }
        if (boostPressed) {
            launcher.recoveryBoost = !launcher.recoveryBoost;
        }
        if (firePressed && launcher.queuedShots() == 0) {
            launcher.requestShots(1);
        } else if (burstPressed && launcher.queuedShots() == 0) {
            launcher.requestShots(Launcher.BURST_SIZE);
        }
        launcher.setFiring(fireHeld);

        boolean loading = gamepad1.left_trigger >= 0.5;
        intake.setPower(loading || launcher.isFeeding() ? LOAD_POWER : 0.0);
        launcher.setStagingTransferPower(loading ? LOAD_POWER : 0.0);

        if (goodPressed) {
            markLastShot(MARKS[0]);
        }
        if (shortPressed) {
            markLastShot(MARKS[1]);
        }
        if (longPressed) {
            markLastShot(MARKS[2]);
        }
        if (resetPressed) {
            Launcher.Shot active = launcher.activeShot();
            if (active != null && !active.complete) {
                logStatus = "wait until the shot has recovered, then press BACK again";
            } else {
                logFinishedShots(true);
                launcher.clearHistory();
                steadySamples.clear();
                logStatus = "stats reset (log files keep everything)";
            }
        }

        if (leftPressed) {
            settingIndex = (settingIndex + Setting.values().length - 1) % Setting.values().length;
        }
        if (rightPressed) {
            settingIndex = (settingIndex + 1) % Setting.values().length;
        }
        if (upPressed) {
            changeSetting(Setting.values()[settingIndex], 1);
        }
        if (downPressed) {
            changeSetting(Setting.values()[settingIndex], -1);
        }
    }

    private void changeSetting(Setting setting, int direction) {
        double delta = setting.step * direction;
        switch (setting) {
            case TARGET:
                launcher.setTargetVelocity(launcher.targetVelocity() + delta);
                break;
            case KP:
                launcher.kP = Math.max(0.0, launcher.kP + delta);
                break;
            case KI:
                launcher.kI = Math.max(0.0, launcher.kI + delta);
                break;
            case KV:
                launcher.kV = Math.max(0.0, launcher.kV + delta);
                break;
            case KS:
                launcher.kS = Math.max(0.0, launcher.kS + delta);
                break;
            case READY_TOLERANCE:
                launcher.readyTolerance = Math.max(15.0, launcher.readyTolerance + delta);
                break;
            case READY_STABLE:
                launcher.readyStableSeconds = Math.max(0.04, launcher.readyStableSeconds + delta);
                break;
            case DETECT_DROP:
                launcher.shotDetectDrop = Math.max(20.0, launcher.shotDetectDrop + delta);
                break;
            case FEED_MAX:
                launcher.feedMaxSeconds = Math.max(0.1, launcher.feedMaxSeconds + delta);
                break;
            case FEED_POWER:
                launcher.feedTransferPower = Math.max(0.1, Math.min(1.0, launcher.feedTransferPower + delta));
                break;
            case STAGING_POWER:
                launcher.pendingStagingPower = Math.max(0.0, Math.min(1.0, launcher.pendingStagingPower + delta));
                break;
            case BLOCKER_CLOSE:
                launcher.blockerCloseSeconds = Math.max(0.05, launcher.blockerCloseSeconds + delta);
                break;
            case SETPOINT_BOOST:
                launcher.feedSetpointBoost = Math.max(0.0, launcher.feedSetpointBoost + delta);
                break;
            case BOOST_ENTER:
                launcher.recoveryBoostEnter = Math.max(launcher.recoveryBoostExit + 5.0, launcher.recoveryBoostEnter + delta);
                break;
            case BOOST_EXIT:
                launcher.recoveryBoostExit = Math.max(0.0, Math.min(launcher.recoveryBoostEnter - 5.0, launcher.recoveryBoostExit + delta));
                break;
            default:
                break;
        }
    }

    private String settingValue(Setting setting) {
        switch (setting) {
            case TARGET:
                return String.format(Locale.US, "%.0f", launcher.targetVelocity());
            case KP:
                return String.format(Locale.US, "%.4f", launcher.kP);
            case KI:
                return String.format(Locale.US, "%.4f", launcher.kI);
            case KV:
                return String.format(Locale.US, "%.6f", launcher.kV);
            case KS:
                return String.format(Locale.US, "%.3f", launcher.kS);
            case READY_TOLERANCE:
                return String.format(Locale.US, "%.0f", launcher.readyTolerance);
            case READY_STABLE:
                return String.format(Locale.US, "%.2f s", launcher.readyStableSeconds);
            case DETECT_DROP:
                return String.format(Locale.US, "%.0f", launcher.shotDetectDrop);
            case FEED_MAX:
                return String.format(Locale.US, "%.2f s", launcher.feedMaxSeconds);
            case FEED_POWER:
                return String.format(Locale.US, "%.1f", launcher.feedTransferPower);
            case STAGING_POWER:
                return String.format(Locale.US, "%.2f", launcher.pendingStagingPower);
            case BLOCKER_CLOSE:
                return String.format(Locale.US, "%.2f s", launcher.blockerCloseSeconds);
            case SETPOINT_BOOST:
                return String.format(Locale.US, "%.0f", launcher.feedSetpointBoost);
            case BOOST_ENTER:
                return String.format(Locale.US, "%.0f below target", launcher.recoveryBoostEnter);
            case BOOST_EXIT:
                return String.format(Locale.US, "%.0f below target", launcher.recoveryBoostExit);
            default:
                return "";
        }
    }

    private void markLastShot(String mark) {
        List<Launcher.Shot> history = launcher.history();
        for (int i = history.size() - 1; i >= 0; i--) {
            Launcher.Shot shot = history.get(i);
            if (shot.endTime < 0) {
                continue;
            }
            shot.mark = mark;
            enqueue(MARKS_FILE, MARKS_HEADER,
                    String.format(Locale.US, "%d,%d,%s\n", sessionId, shot.number, mark));
            logStatus = String.format(Locale.US, "shot %d marked %s", shot.number, mark);
            return;
        }
        logStatus = "no finished shot to mark yet";
    }

    private void startSweep() {
        launcher.stop();
        logFinishedShots(true);
        sweeping = true;
        sweepStep = 0;
        sweepPoints.clear();
        modeBeforeSweep = launcher.controlMode();
        launcher.cancelShots();
        launcher.setStagingTransferPower(0.0);
        intake.setPower(0.0);
        beginSweepStep();
    }

    private void beginSweepStep() {
        sweepStepStart = launcher.time();
        sweepSumVelocity = 0.0;
        sweepSumVoltage = 0.0;
        sweepSamples = 0;
        launcher.setOpenLoopPower(SWEEP_POWERS[sweepStep]);
    }

    private void runSweep() {
        double elapsed = launcher.time() - sweepStepStart;
        if (elapsed >= SWEEP_STEP_SECONDS - SWEEP_SAMPLE_SECONDS) {
            sweepSumVelocity += launcher.measuredVelocity();
            sweepSumVoltage += launcher.batteryVoltage();
            sweepSamples++;
        }
        if (elapsed < SWEEP_STEP_SECONDS) {
            return;
        }
        if (sweepSamples > 0) {
            sweepPoints.add(new double[] {
                    SWEEP_POWERS[sweepStep], sweepSumVelocity / sweepSamples, sweepSumVoltage / sweepSamples
            });
        }
        sweepStep++;
        if (sweepStep >= SWEEP_POWERS.length) {
            endSweep(fitSweep());
        } else {
            beginSweepStep();
        }
    }

    private void endSweep(String result) {
        sweeping = false;
        sweepResult = result;
        launcher.setOpenLoopPower(null);
        launcher.setControlMode(modeBeforeSweep);
        launcher.stop();
    }

    private String fitSweep() {
        double n = 0;
        double sumX = 0;
        double sumY = 0;
        double sumXX = 0;
        double sumXY = 0;
        double sumYY = 0;
        for (double[] point : sweepPoints) {
            double velocity = point[1];
            if (velocity < SWEEP_MIN_VELOCITY) {
                continue;
            }
            double normalizedPower = point[0] * point[2] / Launcher.NOMINAL_VOLTAGE;
            n++;
            sumX += velocity;
            sumY += normalizedPower;
            sumXX += velocity * velocity;
            sumXY += velocity * normalizedPower;
            sumYY += normalizedPower * normalizedPower;
        }
        if (n < 3) {
            return "FAILED - fewer than 3 steps reached " + (int) SWEEP_MIN_VELOCITY
                    + " ticks/s (encoder, direction or motor problem?)";
        }
        double denominator = n * sumXX - sumX * sumX;
        double spreadY = n * sumYY - sumY * sumY;
        if (denominator <= 0 || spreadY <= 0) {
            return "FAILED - the steps gave the same velocity";
        }
        double fitKv = (n * sumXY - sumX * sumY) / denominator;
        double fitKs = (sumY - fitKv * sumX) / n;
        if (fitKv <= 0) {
            return "FAILED - velocity did not rise with power";
        }
        double correlation = (n * sumXY - sumX * sumY) / Math.sqrt(denominator * spreadY);
        launcher.kV = fitKv;
        launcher.kS = Math.max(0.0, fitKs);
        double maxVelocity = (1.0 - launcher.kS) / fitKv;
        return String.format(Locale.US, "kS %.3f  kV %.6f  R^2 %.3f  max %.0f ticks/s at 12 V  hub F %.2f  (applied)",
                fitKs, fitKv, correlation * correlation, maxVelocity, 32767.0 / maxVelocity);
    }

    private void recordSamples() {
        logFinishedShots(false);
        double t = launcher.time();
        loopTimes.addLast(t);
        while (loopTimes.size() > 2 && t - loopTimes.peekFirst() > 1.0) {
            loopTimes.removeFirst();
        }

        if (launcher.state() == Launcher.State.READY && !sweeping) {
            steadySamples.addLast(new double[] {t, launcher.measuredVelocity(), launcher.power()});
        }
        while (!steadySamples.isEmpty() && t - steadySamples.peekFirst()[0] > STEADY_WINDOW_SECONDS) {
            steadySamples.removeFirst();
        }

        Sample sample = new Sample(t, launcher.commandedVelocity(), launcher.measuredVelocity(),
                launcher.power(), launcher.state().toString(), launcher.batteryVoltage());

        Launcher.Shot shot = launcher.activeShot();
        if (shot != null && !shot.complete && !sweeping) {
            if (shot.number != tracedShot) {
                tracedShot = shot.number;
                traceBuffer.setLength(0);
                traceSamples = 0;
                for (Sample earlier : recentSamples) {
                    appendTrace(shot, earlier);
                }
            }
            if (traceSamples < MAX_TRACE_SAMPLES) {
                appendTrace(shot, sample);
            }
        }

        recentSamples.addLast(sample);
        while (!recentSamples.isEmpty() && t - recentSamples.peekFirst().t > PRE_FEED_TRACE_SECONDS) {
            recentSamples.removeFirst();
        }
    }

    private void appendTrace(Launcher.Shot shot, Sample sample) {
        traceBuffer.append(String.format(Locale.US, "%d,%d,%.3f,%.0f,%.0f,%.3f,%s,%.2f\n",
                sessionId, shot.number, sample.t - shot.startTime, sample.commanded,
                sample.measured, sample.power, sample.state, sample.voltage));
        traceSamples++;
    }

    private void logFinishedShots(boolean includeUnfinished) {
        StringBuilder rows = new StringBuilder();
        int highest = loggedThroughShot;
        for (Launcher.Shot shot : launcher.history()) {
            if (shot.number <= loggedThroughShot) {
                continue;
            }
            if (!shot.complete && !(includeUnfinished && shot.endTime >= 0)) {
                break;
            }
            rows.append(String.format(Locale.US,
                    "%d,%d,%d,%d,%s,%s,%.0f,%.0f,%.0f,%.0f,%s,%.3f,%.3f,%.3f,%s,%.2f,%.4f,%.5f,%.6f,%.3f,%.0f,%.0f,%.0f,%.2f,%.2f,%.2f\n",
                    sessionId, shot.number, shot.burstIndex, shot.burstSize, shot.mode, shot.recoveryBoost ? "boost" : "noboost",
                    shot.target, shot.preVelocity, shot.minVelocity, shot.dip(), shot.detected ? "yes" : "no",
                    shot.detectSeconds, shot.feedSeconds, shot.recoverySeconds, shot.forced ? "yes" : "no",
                    shot.voltage, shot.kP, shot.kI, shot.kV, shot.kS, shot.feedSetpointBoost,
                    shot.readyTolerance, shot.shotDetectDrop, shot.feedMaxSeconds, shot.feedTransferPower,
                    shot.pendingStagingPower));
            highest = shot.number;
        }
        if (rows.length() == 0) {
            return;
        }
        enqueue(SHOTS_FILE, SHOTS_HEADER, rows.toString());
        if (traceBuffer.length() > 0 && tracedShot <= highest) {
            enqueue(TRACE_FILE, TRACE_HEADER, traceBuffer.toString());
            traceBuffer.setLength(0);
        }
        loggedThroughShot = highest;
    }

    private void enqueue(final String name, final String header, final String rows) {
        try {
            writer.execute(new Runnable() {
                @Override
                public void run() {
                    try {
                        File file = new File(AppUtil.FIRST_FOLDER, name);
                        boolean isNew = !file.exists();
                        try (FileWriter out = new FileWriter(file, true)) {
                            if (isNew) {
                                out.write(header);
                            }
                            out.write(rows);
                        }
                        logStatus = "written to " + name;
                    } catch (IOException e) {
                        logStatus = "LOG FAILED (" + name + "): " + e.getMessage();
                    }
                }
            });
        } catch (RuntimeException e) {
            logStatus = "LOG FAILED: " + e.getMessage();
        }
    }

    private void showTelemetry() {
        if (sweeping) {
            telemetry.addLine("FEEDFORWARD SWEEP RUNNING - keep balls OUT of the launcher (A = abort)");
            telemetry.addData("Step", "%d / %d  power %.2f  %.1f s",
                    sweepStep + 1, SWEEP_POWERS.length, SWEEP_POWERS[Math.min(sweepStep, SWEEP_POWERS.length - 1)],
                    launcher.time() - sweepStepStart);
            telemetry.addData("Measured", "%.0f ticks/s  battery %.1f V",
                    launcher.measuredVelocity(), launcher.batteryVoltage());
            for (double[] point : sweepPoints) {
                telemetry.addData(String.format(Locale.US, "  power %.2f", point[0]), "%.0f ticks/s at %.1f V", point[1], point[2]);
            }
            return;
        }

        launcher.addTelemetry(telemetry);
        telemetry.addData("Controller", "%s  recovery boost %s",
                launcher.controlMode(), launcher.recoveryBoost ? "ON" : "OFF");
        if (launcher.controlMode() == Launcher.ControlMode.HUB_PIDF && hubCoefficients != null) {
            telemetry.addData("Hub PIDF", "P %.2f  I %.2f  D %.2f  F %.2f  %s",
                    hubCoefficients.p, hubCoefficients.i, hubCoefficients.d, hubCoefficients.f,
                    hubCoefficients.algorithm);
        }
        telemetry.addData("Spin-up", launcher.spinUpSeconds() >= 0
                ? String.format(Locale.US, "%.2f s to READY", launcher.spinUpSeconds()) : "not ready yet");
        if (loopTimes.size() > 2) {
            telemetry.addData("Loop time", "%.1f ms",
                    1000.0 * (loopTimes.peekLast() - loopTimes.peekFirst()) / (loopTimes.size() - 1));
        }

        showSteady();
        showShotStats();

        telemetry.addLine();
        for (int i = 0; i < Setting.values().length; i++) {
            Setting setting = Setting.values()[i];
            telemetry.addData((i == settingIndex ? "> " : "  ") + setting.label, settingValue(setting));
        }
        showCopyBlock();
        telemetry.addData("FF sweep", sweepResult);
        telemetry.addData("Log", logStatus);
        telemetry.addLine();
        showControls();
    }

    private void showSteady() {
        if (steadySamples.size() < 5) {
            telemetry.addData("Steady", "spin up and wait (needs READY, no feeding)");
            return;
        }
        Stats velocity = new Stats();
        Stats power = new Stats();
        double[] values = new double[steadySamples.size()];
        int k = 0;
        for (double[] sample : steadySamples) {
            velocity.add(sample[1]);
            power.add(sample[2]);
            values[k++] = sample[1];
        }
        telemetry.addData("Steady (2 s)", "mean %.0f  std %.1f  min %.0f  max %.0f  power %.2f",
                velocity.mean(), velocity.std(), velocity.min, velocity.max, power.mean());

        Arrays.sort(values);
        double step = Double.POSITIVE_INFINITY;
        for (int i = 1; i < values.length; i++) {
            double gap = values[i] - values[i - 1];
            if (gap > 0.5) {
                step = Math.min(step, gap);
            }
        }
        if (step != Double.POSITIVE_INFINITY) {
            telemetry.addData("Velocity step", "%.0f ticks/s per reading", step);
            if (launcher.shotDetectDrop < 2.5 * step) {
                telemetry.addLine("!! shot detect drop is less than 2.5 velocity steps - raise it to about " + Math.round(2.5 * step));
            }
        }
        if (3.0 * velocity.std() > launcher.readyTolerance) {
            telemetry.addLine("!! noise is large for the ready tolerance - raise ready tolerance to about " + Math.round(3.0 * velocity.std()));
        }
    }

    private void showShotStats() {
        List<Launcher.Shot> history = launcher.history();
        Stats pre = new Stats();
        Stats dip = new Stats();
        Stats recovery = new Stats();
        Stats detect = new Stats();
        Map<String, Stats> errorByMark = new HashMap<>();
        Stats missedDip = new Stats();
        int[] markCounts = new int[MARKS.length];
        Stats[] preByPosition = new Stats[Launcher.BURST_SIZE];
        Stats[] dipByPosition = new Stats[Launcher.BURST_SIZE];
        Stats burstGap = new Stats();
        Launcher.Shot previous = null;
        for (int b = 0; b < Launcher.BURST_SIZE; b++) {
            preByPosition[b] = new Stats();
            dipByPosition[b] = new Stats();
        }
        int detected = 0;
        int forced = 0;
        int complete = 0;
        for (Launcher.Shot shot : history) {
            if (!shot.complete) {
                continue;
            }
            complete++;
            pre.add(shot.preVelocity);
            if (shot.forced) {
                forced++;
            }
            if (shot.detected) {
                detected++;
                dip.add(shot.dip());
                detect.add(shot.detectSeconds);
            }
            if (shot.recoverySeconds >= 0) {
                recovery.add(shot.recoverySeconds);
            }
            if (!shot.mark.isEmpty()) {
                statsFor(errorByMark, shot.mark).add(shot.preVelocity - shot.target);
            }
            if (!shot.detected) {
                missedDip.add(shot.dip());
            }
            if (shot.burstSize == Launcher.BURST_SIZE && shot.burstIndex >= 1) {
                preByPosition[shot.burstIndex - 1].add(shot.preVelocity - shot.target);
                dipByPosition[shot.burstIndex - 1].add(shot.dip());
                if (shot.burstIndex > 1 && previous != null && previous.number == shot.number - 1) {
                    burstGap.add(shot.startTime - previous.startTime);
                }
            }
            previous = shot;
            for (int m = 0; m < MARKS.length; m++) {
                if (MARKS[m].equals(shot.mark)) {
                    markCounts[m]++;
                }
            }
        }

        telemetry.addLine();
        telemetry.addData("Shots", "%d done  %d dip detected  %d forced", complete, detected, forced);
        if (complete == 0) {
            return;
        }
        telemetry.addData("Velocity at feed", pre.text());
        telemetry.addData("Dip (detected)", dip.text());
        telemetry.addData("Recovery", recovery.text() + " s");
        telemetry.addData("Feed to dip", detect.text() + " s");

        telemetry.addData("Marks", "GOOD %d  SHORT %d  LONG %d", markCounts[0], markCounts[1], markCounts[2]);
        if (preByPosition[0].n > 0) {
            for (int b = 0; b < Launcher.BURST_SIZE; b++) {
                telemetry.addData(String.format(Locale.US, "  burst ball %d", b + 1),
                        "%d shots  speed vs target %s  dip %s",
                        preByPosition[b].n, signedMean(preByPosition[b]), dipByPosition[b].meanText());
            }
            telemetry.addData("  time between burst balls", burstGap.text() + " s");
            double spread = 0.0;
            for (int b = 1; b < Launcher.BURST_SIZE; b++) {
                if (preByPosition[b].n > 0) {
                    spread = Math.max(spread, Math.abs(preByPosition[b].mean() - preByPosition[0].mean()));
                }
            }
            telemetry.addLine(spread > launcher.readyTolerance / 2.0
                    ? "-> burst balls leave at different wheel speeds: tighten ready tolerance or raise ready stable seconds"
                    : "-> burst balls leave at the same wheel speed: if heights still differ, look at staging and feeding");
        }

        Stats good = errorByMark.get(MARKS[0]);
        for (int m = 1; m < MARKS.length; m++) {
            Stats bad = errorByMark.get(MARKS[m]);
            if (good == null || bad == null) {
                continue;
            }
            double gap = bad.mean() - good.mean();
            telemetry.addData(MARKS[m] + " vs GOOD", "wheel speed at feed differs by %+.0f ticks/s", gap);
            telemetry.addLine(Math.abs(gap) > launcher.readyTolerance / 2.0
                    ? "-> " + MARKS[m] + " shots left at a different wheel speed: speed control is part of the problem"
                    : "-> same wheel speed as GOOD shots: look at compression, ball height, feeding, double balls");
        }
        if (missedDip.n > 0) {
            telemetry.addLine(String.format(Locale.US,
                    "-> %d NO DIP shots (biggest dip %.0f): no ball, or set shot detect drop below the real dip",
                    missedDip.n, missedDip.max));
        }

        int shown = 0;
        for (int i = history.size() - 1; i >= 0 && shown < RECENT_SHOTS_SHOWN; i--, shown++) {
            Launcher.Shot shot = history.get(i);
            telemetry.addLine("  " + shot.summary());
        }
    }

    private void showCopyBlock() {
        List<String> lines = new ArrayList<>();
        addIfChanged(lines, "DEFAULT_TARGET_VELOCITY", launcher.targetVelocity(), Launcher.DEFAULT_TARGET_VELOCITY, "%.1f");
        addIfChanged(lines, "DEFAULT_KP", launcher.kP, Launcher.DEFAULT_KP, "%.5f");
        addIfChanged(lines, "DEFAULT_KI", launcher.kI, Launcher.DEFAULT_KI, "%.5f");
        addIfChanged(lines, "DEFAULT_KV", launcher.kV, Launcher.DEFAULT_KV, "%.7f");
        addIfChanged(lines, "DEFAULT_KS", launcher.kS, Launcher.DEFAULT_KS, "%.4f");
        addIfChanged(lines, "DEFAULT_READY_TOLERANCE", launcher.readyTolerance, Launcher.DEFAULT_READY_TOLERANCE, "%.1f");
        addIfChanged(lines, "DEFAULT_READY_STABLE_SECONDS", launcher.readyStableSeconds, Launcher.DEFAULT_READY_STABLE_SECONDS, "%.2f");
        addIfChanged(lines, "DEFAULT_SHOT_DETECT_DROP", launcher.shotDetectDrop, Launcher.DEFAULT_SHOT_DETECT_DROP, "%.1f");
        addIfChanged(lines, "DEFAULT_FEED_MAX_SECONDS", launcher.feedMaxSeconds, Launcher.DEFAULT_FEED_MAX_SECONDS, "%.2f");
        addIfChanged(lines, "DEFAULT_FEED_TRANSFER_POWER", launcher.feedTransferPower, Launcher.DEFAULT_FEED_TRANSFER_POWER, "%.2f");
        addIfChanged(lines, "DEFAULT_PENDING_STAGING_POWER", launcher.pendingStagingPower, Launcher.DEFAULT_PENDING_STAGING_POWER, "%.2f");
        addIfChanged(lines, "DEFAULT_BLOCKER_CLOSE_SECONDS", launcher.blockerCloseSeconds, Launcher.DEFAULT_BLOCKER_CLOSE_SECONDS, "%.2f");
        addIfChanged(lines, "DEFAULT_FEED_SETPOINT_BOOST", launcher.feedSetpointBoost, Launcher.DEFAULT_FEED_SETPOINT_BOOST, "%.1f");
        addIfChanged(lines, "RECOVERY_BOOST_ENTER", launcher.recoveryBoostEnter, Launcher.RECOVERY_BOOST_ENTER, "%.1f");
        addIfChanged(lines, "RECOVERY_BOOST_EXIT", launcher.recoveryBoostExit, Launcher.RECOVERY_BOOST_EXIT, "%.1f");
        if (launcher.recoveryBoost != Launcher.DEFAULT_RECOVERY_BOOST) {
            lines.add("DEFAULT_RECOVERY_BOOST = " + launcher.recoveryBoost + ";");
        }
        if (launcher.controlMode() != Launcher.DEFAULT_CONTROL_MODE) {
            lines.add("DEFAULT_CONTROL_MODE = ControlMode." + launcher.controlMode() + ";");
        }
        if (lines.isEmpty()) {
            telemetry.addData("Copy into Launcher.java", "nothing changed from the defaults");
            return;
        }
        telemetry.addLine("Copy into Launcher.java to keep these values:");
        for (String line : lines) {
            telemetry.addLine("  " + line);
        }
    }

    private static void addIfChanged(List<String> lines, String name, double value, double defaultValue, String format) {
        if (Math.abs(value - defaultValue) > 1e-9) {
            lines.add(name + " = " + String.format(Locale.US, format, value) + ";");
        }
    }

    private static String signedMean(Stats stats) {
        return stats.n == 0 ? "-" : String.format(Locale.US, "%+.0f", stats.mean());
    }

    private static Stats statsFor(Map<String, Stats> map, String key) {
        Stats stats = map.get(key);
        if (stats == null) {
            stats = new Stats();
            map.put(key, stats);
        }
        return stats;
    }

    private void showControls() {
        telemetry.addLine("Y flywheel on/off   RT fire (tap = 1, hold = keep firing)   LT hold = load balls");
        telemetry.addLine("R-stick press fire a 3-ball burst   RB mark GOOD   LB mark SHORT   L-stick press mark LONG");
        telemetry.addLine("X hub PIDF / custom   B recovery boost   A feedforward sweep (no balls)   BACK reset");
        telemetry.addLine("dpad left/right pick setting   up/down change it");
    }

    private static final class Stats {
        int n = 0;
        double sum = 0;
        double sumSquares = 0;
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;

        void add(double value) {
            n++;
            sum += value;
            sumSquares += value * value;
            min = Math.min(min, value);
            max = Math.max(max, value);
        }

        double mean() {
            return n == 0 ? 0.0 : sum / n;
        }

        double std() {
            if (n < 2) {
                return 0.0;
            }
            double mean = mean();
            return Math.sqrt(Math.max(0.0, (sumSquares - n * mean * mean) / (n - 1)));
        }

        String meanText() {
            return n == 0 ? "-" : String.format(Locale.US, "%.0f", mean());
        }

        String text() {
            if (n == 0) {
                return "-";
            }
            return String.format(Locale.US, Math.abs(max) < 10 ? "%.2f +/- %.2f  (%.2f..%.2f)" : "%.0f +/- %.0f  (%.0f..%.0f)",
                    mean(), std(), min, max);
        }
    }
}
