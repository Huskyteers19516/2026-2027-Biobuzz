package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.CappedDrivetrain;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeekConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeekEngine;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeekSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeeker;

import java.util.Locale;

@Configurable
@Autonomous(name = "LogiLight Pollen Seek Auto", group = "LogiLight")
public class LogiLightPollenSeekAuto extends LinearOpMode {

    public static double SCAN_SECONDS = 2.5;
    public static boolean SEARCH_TURN_ENABLED = true;
    public static double SEARCH_DELAY_SECONDS = 0.6;
    public static double SEARCH_TURN_DEGREES = 35.0;
    public static double SEARCH_TURN_POWER = 0.15;

    private static final double MISSION_BUDGET_SECONDS = Math.min(25.0, FieldPoints.AUTONOMOUS_BUDGET_SECONDS);
    private static final double MAX_SCAN_SECONDS = 2.5;
    private static final double MAX_SEARCH_TURN_DEGREES = 35.0;
    private static final double MAX_SEARCH_TURN_POWER = 0.25;
    private static final double SEARCH_OVERSHOOT_DEGREES = 10.0;
    private static final double SEARCH_REACHED_DEGREES = 3.0;
    private static final int REZERO_SETTLE_LOOPS = 3;
    private static final double SCAN_DRIFT_ABORT_INCHES = 4.0;

    private LogiLight light;
    private Localizer pinpoint;
    private String pinpointFault = "";
    private PinpointStatusMonitor pinpointStatus;
    private CappedDrivetrain drivetrain;
    private Follower follower;
    private String followerFault = "";
    private PollenSeeker seeker;
    private PushTestGate pushTest;
    private PollenIntakeMotor intake;
    private String intakeFault = "";
    private final ElapsedTime runtime = new ElapsedTime();

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private boolean started;
    private int settleLoops;
    private String phase = "init";
    private String scanNote = "not run";
    private String seekNote = "not run";
    private String stopReason = "mission complete";
    private String scanFault;
    private String message = "";

    @Override
    public void runOpMode() {
        PoseHandoff.clear();
        try {
            setup();
            waitInInit();
            if (isStopRequested()) {
                return;
            }
            runtime.reset();
            started = true;
            try {
                runMission();
            } finally {
                stopAll();
                Pose last = currentPose();
                if (last != null) {
                    PoseHandoff.save(last, alliance);
                }
            }
            while (opModeIsActive()) {
                tick();
                if (intake != null) {
                    intake.stop();
                }
                addSummaryTelemetry();
                telemetry.update();
            }
        } finally {
            stopAll();
            if (seeker != null) {
                try {
                    seeker.releaseCamera();
                } catch (Throwable ignored) {
                }
            }
            if (light != null) {
                try {
                    light.close();
                } catch (Throwable ignored) {
                }
            }
        }
    }

    private void setup() {
        PollenSeekConfig config = PollenSeekSettings.current();
        PollenSeekConfig safe = config.sanitized();
        pushTest = new PushTestGate(safe.pushTestForwardInches, safe.pushTestLateralRatio);
        light = new LogiLight(hardwareMap);
        LogiOpModeTools.PinpointHandle handle = LogiOpModeTools.createPinpoint(hardwareMap);
        pinpoint = handle.localizer;
        pinpointFault = handle.fault;
        pinpointStatus = PinpointStatusMonitor.create(hardwareMap, pinpoint != null);
        if (pinpoint != null) {
            try {
                drivetrain = new CappedDrivetrain(Constants.createDrivetrain(hardwareMap), safe.maxPower);
                follower = new Follower(pinpoint, drivetrain, Constants.createAlgorithm());
            } catch (Throwable t) {
                follower = null;
                drivetrain = null;
                followerFault = "Follower NOT AVAILABLE - " + LogiOpModeTools.describe(t);
            }
        }
        seeker = new PollenSeeker(light, follower, config);
        seeker.setPushTestGate(pushTest);
        seeker.setIntakeOnArrive(safe.runIntakeOnArrive);
        resetToStart();
    }

    private void waitInInit() {
        while (!isStarted() && !isStopRequested()) {
            boolean x1 = gamepad1.xWasPressed();
            boolean b1 = gamepad1.bWasPressed();
            boolean x2 = gamepad2.xWasPressed();
            boolean b2 = gamepad2.bWasPressed();
            boolean back = gamepad1.backWasPressed();
            boolean blue = LogiOpModeTools.pressedWithoutStart(x1, gamepad1) || LogiOpModeTools.pressedWithoutStart(x2, gamepad2);
            boolean red = LogiOpModeTools.pressedWithoutStart(b1, gamepad1) || LogiOpModeTools.pressedWithoutStart(b2, gamepad2);
            GoalScanner.Alliance picked = LogiOpModeTools.allianceFromButtons(alliance, blue, red);
            if (picked != alliance) {
                alliance = picked;
                resetToStart();
                message = "alliance " + alliance + " - pose set to its start pose";
            }
            if (back) {
                resetToStart();
                message = "pose re-zeroed at the " + alliance + " start pose";
            }
            tick();
            updatePushTest();
            addInitTelemetry();
            telemetry.update();
            idle();
        }
    }

    private void runMission() {
        String refusal = refusal();
        if (refusal != null) {
            stopReason = "REFUSING TO DRIVE - " + refusal;
            phase = "refused";
            return;
        }
        StringBuilder fault = new StringBuilder();
        intake = PollenIntakeMotor.create(hardwareMap, fault);
        intakeFault = fault.toString();
        seeker.setIntake(intake);
        boolean seen = scan();
        if (scanFault != null) {
            stopReason = "REFUSING TO SEEK - " + scanFault;
            return;
        }
        if (!canRun()) {
            stopReason = budgetReason("during the scan");
            return;
        }
        if (!seen) {
            stopReason = "no yellow POLLEN confirmed in " + scanNote;
            return;
        }
        seek();
    }

    private String refusal() {
        if (light == null || !light.isAvailable()) {
            return "camera NOT AVAILABLE";
        }
        if (follower == null) {
            return "no Pinpoint / Follower";
        }
        if (PollenSeekSettings.REQUIRE_PUSH_TEST && !pushTest.driveEnabled()) {
            return "Pinpoint push test " + pushTest.shortStatus();
        }
        return seeker.refusalReason();
    }

    private boolean scan() {
        phase = "scan";
        double scanSeconds = clip(SCAN_SECONDS, 0.0, MAX_SCAN_SECONDS);
        double searchDegrees = clip(SEARCH_TURN_DEGREES, 0.0, MAX_SEARCH_TURN_DEGREES);
        double searchPower = clip(SEARCH_TURN_POWER, 0.0, MAX_SEARCH_TURN_POWER);
        Pose start = currentPose();
        double startHeading = start == null ? Double.NaN : start.heading();
        int leg = 0;
        ElapsedTime timer = new ElapsedTime();
        while (canRun() && timer.seconds() < scanSeconds) {
            tick();
            seeker.step(false);
            Pose current = currentPose();
            double drift = start == null || current == null ? 0.0
                    : Math.hypot(current.x() - start.x(), current.y() - start.y());
            if (Double.isNaN(drift) || drift > SCAN_DRIFT_ABORT_INCHES) {
                follower.stop();
                scanFault = String.format(Locale.US,
                        "!! odometry moved %.1f in during the scan / search turn - Y pod reversed or pod offsets wrong - push-test LEFT",
                        drift);
                seeker.reportLocalizerFault(scanFault);
                scanNote = scanFault;
                return false;
            }
            if (seeker.canSeePollen()) {
                follower.stop();
                scanNote = String.format(Locale.US, "POLLEN confirmed after %.2f s", timer.seconds());
                return true;
            }
            Pose here = currentPose();
            boolean searching = SEARCH_TURN_ENABLED && searchPower > 0.0 && searchDegrees > 0.0
                    && timer.seconds() >= SEARCH_DELAY_SECONDS && here != null && !Double.isNaN(startHeading) && leg < 3;
            if (searching) {
                double turned = Math.toDegrees(Angle.normalizeSigned(here.heading() - startHeading));
                if (Math.abs(turned) > searchDegrees + SEARCH_OVERSHOOT_DEGREES) {
                    follower.stop();
                    scanNote = String.format(Locale.US, "!! search turn overshot (%.0f deg) - stopped", turned);
                    leg = 3;
                    continue;
                }
                double goal = leg == 0 ? searchDegrees : (leg == 1 ? -searchDegrees : 0.0);
                double error = goal - turned;
                if (Math.abs(error) <= SEARCH_REACHED_DEGREES) {
                    leg++;
                    follower.stop();
                } else {
                    follower.manual(0.0, 0.0, Math.copySign(searchPower, error));
                }
                addPhaseTelemetry(String.format(Locale.US, "scanning %.1f / %.1f s  search turn %+.0f deg (leg %d)",
                        timer.seconds(), scanSeconds, turned, leg + 1));
            } else {
                follower.stop();
                addPhaseTelemetry(String.format(Locale.US, "scanning %.1f / %.1f s", timer.seconds(), scanSeconds));
            }
        }
        follower.stop();
        scanNote = String.format(Locale.US, "%.1f s scan%s", scanSeconds, SEARCH_TURN_ENABLED ? " with search turn" : "");
        return seeker.canSeePollen();
    }

    private void seek() {
        phase = "seek";
        int lostBefore = seeker.lostCount();
        int attemptsBefore = seeker.attempts();
        ElapsedTime scanTimer = new ElapsedTime();
        while (canRun()) {
            tick();
            PollenSeeker.State state = seeker.step(true);
            if (state == PollenSeeker.State.DONE) {
                seekNote = "DONE - " + seeker.status();
                stopReason = seekNote;
                return;
            }
            if (state == PollenSeeker.State.ABORTED) {
                seekNote = "ABORTED - " + seeker.abortReason();
                stopReason = seekNote;
                return;
            }
            if (seeker.lostCount() > lostBefore) {
                seekNote = "lost the ball - " + seeker.lastEvent();
                stopReason = seekNote;
                return;
            }
            if (state == PollenSeeker.State.SCANNING && seeker.attempts() == attemptsBefore
                    && scanTimer.seconds() > clip(SCAN_SECONDS, 0.0, MAX_SCAN_SECONDS)) {
                seekNote = "no reachable POLLEN - " + seeker.planNote();
                stopReason = seekNote;
                return;
            }
            addPhaseTelemetry(String.format(Locale.US, "%s  %s", state, seeker.status()));
        }
        seekNote = budgetReason("while seeking");
        stopReason = seekNote;
    }

    private void tick() {
        if (follower != null) {
            if (started) {
                follower.update();
            } else {
                try {
                    follower.localizer.update();
                } catch (Throwable t) {
                    pinpointFault = "!! Pinpoint read failed - " + LogiOpModeTools.describe(t);
                }
            }
        }
        pinpointStatus.report(seeker);
        seeker.recordPose();
        light.update();
        seeker.observe();
        draw();
    }

    private void stopAll() {
        if (!started) {
            return;
        }
        if (seeker != null) {
            try {
                seeker.shutdown();
            } catch (Throwable ignored) {
            }
        }
        if (intake != null) {
            try {
                intake.stop();
            } catch (Throwable ignored) {
            }
        }
        if (follower != null) {
            try {
                follower.stop();
                follower.update();
            } catch (Throwable ignored) {
            }
        }
        if (drivetrain != null) {
            try {
                drivetrain.stop();
            } catch (Throwable ignored) {
            }
        }
    }

    private void resetToStart() {
        Pose start = LogiOpModeTools.startPose(alliance);
        if (follower != null) {
            try {
                follower.setPose(start);
            } catch (Throwable t) {
                pinpointFault = "!! Pinpoint setPose failed - " + LogiOpModeTools.describe(t);
            }
        }
        seeker.resetPoseHistory();
        pushTest.rezero(start.x(), start.y(), start.heading());
        settleLoops = REZERO_SETTLE_LOOPS;
    }

    private void updatePushTest() {
        if (settleLoops > 0) {
            settleLoops--;
            return;
        }
        Pose pose = currentPose();
        if (pose != null) {
            pushTest.update(pose.x(), pose.y(), pose.heading());
        }
    }

    private Pose currentPose() {
        if (follower == null) {
            return null;
        }
        try {
            return follower.pose();
        } catch (Throwable t) {
            return null;
        }
    }

    private boolean canRun() {
        return opModeIsActive() && runtime.seconds() < MISSION_BUDGET_SECONDS;
    }

    private String budgetReason(String where) {
        if (!opModeIsActive()) {
            return "stop requested " + where;
        }
        return String.format(Locale.US, "mission budget %.1f s reached %s", MISSION_BUDGET_SECONDS, where);
    }

    private void draw() {
        if (LogiLightSettings.EVENT_MODE || seeker == null) {
            return;
        }
        PollenSeekEngine.FieldBall target = seeker.currentTarget();
        double ballX = target == null || follower == null ? Double.NaN : target.x;
        double ballY = target == null || follower == null ? Double.NaN : target.y;
        LogiLightField.drawSeek(currentPose(), ballX, ballY, seeker.plannedTarget(), seeker.plannedVia());
    }

    private void addInitTelemetry() {
        telemetry.addLine("LogiLight Pollen Seek Auto");
        telemetry.addData("POLLEN VISIBLE", seeker.visibilityLine());
        telemetry.addData("Detections", seeker.rejectLine());
        telemetry.addData("Camera", light.isAvailable() ? light.getStatus().summaryLine() : "!! " + light.fault());
        for (String warning : seeker.warnings()) {
            telemetry.addLine(warning);
        }
        telemetry.addLine(pushTest.liveLine());
        telemetry.addLine(pushTest.instruction());
        String warning = pushTest.warning();
        if (!warning.isEmpty() && !pushTest.instruction().startsWith(warning)) {
            telemetry.addLine(warning);
        }
        telemetry.addData("ALLIANCE", "%s   (X = BLUE, B = RED, gp1 BACK = re-zero)", alliance);
        telemetry.addData("Start pose", LogiLightTelemetry.formatPose(LogiOpModeTools.startPose(alliance)));
        telemetry.addData("Measured pose", LogiLightTelemetry.formatPose(currentPose()));
        if (pinpoint != null) {
            telemetry.addData("Pinpoint status", pinpointStatus.line());
        }
        if (pinpoint == null) {
            telemetry.addLine("!! PINPOINT NOT AVAILABLE - AUTO WILL NOT MOVE  " + pinpointFault);
        } else if (follower == null) {
            telemetry.addLine("!! " + followerFault + " - AUTO WILL NOT MOVE");
        } else if (!pinpointFault.isEmpty()) {
            telemetry.addLine(pinpointFault);
        }
        String refusal = refusal();
        telemetry.addData("Driving", refusal == null ? "allowed after START" : "!! AUTO WILL NOT MOVE - " + refusal);
        telemetry.addData("Cap", seeker.capLine());
        telemetry.addData("Plan", "scan <= %.1f s%s, seek once, intake %s, total <= %.0f s",
                clip(SCAN_SECONDS, 0.0, MAX_SCAN_SECONDS),
                SEARCH_TURN_ENABLED ? String.format(Locale.US, " (search turn +/-%.0f deg)",
                        clip(SEARCH_TURN_DEGREES, 0.0, MAX_SEARCH_TURN_DEGREES)) : "",
                seeker.intakeOnArrive() ? "ON" : "OFF", MISSION_BUDGET_SECONDS);
        if (!message.isEmpty()) {
            telemetry.addData("Note", message);
        }
        telemetry.addLine("Nothing is commanded until START");
    }

    private void addPhaseTelemetry(String detail) {
        telemetry.addData("Phase", "%s  %s", phase, detail);
        telemetry.addData("Runtime", "%.1f / %.1f s", runtime.seconds(), MISSION_BUDGET_SECONDS);
        telemetry.addData("POLLEN VISIBLE", seeker.visibilityLine());
        telemetry.addData("Plan", seeker.planNote());
        PollenSeekEngine.FieldBall target = seeker.currentTarget();
        telemetry.addData("Target", target == null ? "none" : target.describe());
        telemetry.addData("Guards", seeker.guardLine());
        telemetry.addData("Pose", LogiLightTelemetry.formatPose(currentPose()));
        telemetry.addData("Cap", seeker.capLine());
        if (intake == null && seeker.intakeOnArrive()) {
            telemetry.addLine("!! " + intakeFault + " - intake skipped");
        }
        telemetry.update();
    }

    private void addSummaryTelemetry() {
        telemetry.addData("Status", "Finished - motors stopped");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Reason", stopReason);
        telemetry.addData("Scan", scanNote);
        telemetry.addData("Seek", seekNote);
        if (!seeker.abortReason().isEmpty()) {
            telemetry.addData("Last abort", seeker.abortReason());
        }
        telemetry.addData("Last event", seeker.lastEvent());
        telemetry.addData("Final pose", LogiLightTelemetry.formatPose(currentPose()));
        telemetry.addData("Push test", pushTest.shortStatus());
    }

    private static double clip(double value, double low, double high) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return low;
        }
        return Math.max(low, Math.min(high, value));
    }
}
