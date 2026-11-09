package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiBotPose;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFieldMap;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightStore;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiPipeline;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.PipelineType;
import org.firstinspires.ftc.teamcode.vision.logilight.math.HeadingHistory;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;

import java.util.List;
import java.util.Locale;

public final class LogiOpModeTools {

    public static final double STICK_DEADZONE = 0.05;
    public static final String GOALS_PIPELINE_NAME = "goals";
    public static final String POLLEN_PIPELINE_PREFIX = "pollen-";
    public static final String NECTAR_PIPELINE_PREFIX = "nectar-";
    public static final String YELLOW_PROFILE = "yellow";
    public static final String RED_PROFILE = "red";
    public static final String BLUE_PROFILE = "blue";
    public static final double DEFAULT_MAX_RESULT_AGE_MS = 250.0;
    public static final int HEADING_HISTORY_CAPACITY = 400;
    public static final long HEADING_LOOKUP_SLACK_NANOS = 60_000_000L;
    public static final double CONFIRM_WINDOW_SECONDS = 3.0;
    public static final String RESET_CONFIRM_KEY = "reset settings";
    public static final String FIELD_MAP_DELETE_CONFIRM_KEY = "delete field map";

    private static final Object CONFIRM_LOCK = new Object();
    private static String armedConfirmKey;
    private static long armedConfirmNanos;

    private static final String[] KNOWN_GOALS = {
            LogiFieldMap.RED_SCORING, LogiFieldMap.RED_AUDIENCE,
            LogiFieldMap.BLUE_AUDIENCE, LogiFieldMap.BLUE_SCORING
    };

    private LogiOpModeTools() {
    }

    public static final class PinpointHandle {
        public final Localizer localizer;
        public final String fault;

        PinpointHandle(Localizer localizer, String fault) {
            this.localizer = localizer;
            this.fault = fault == null ? "" : fault;
        }

        public boolean isAvailable() {
            return localizer != null;
        }
    }

    public static PinpointHandle createPinpoint(HardwareMap hardwareMap) {
        try {
            Localizer localizer = Constants.createLocalizer(hardwareMap);
            if (localizer == null) {
                return new PinpointHandle(null, "NOT AVAILABLE - createLocalizer returned null");
            }
            return new PinpointHandle(localizer, "");
        } catch (Throwable t) {
            if (t.getCause() instanceof InterruptedException) {
                Thread.currentThread().interrupt();
            }
            return new PinpointHandle(null, "NOT AVAILABLE - " + describe(t));
        }
    }

    public static String settingsCombo(boolean leftBumperHeld, boolean yPressed, boolean xPressed) {
        if (!leftBumperHeld) {
            return null;
        }
        if (yPressed) {
            disarmConfirm();
            LogiLightStore.clearLastError();
            boolean saved = LogiLightStore.saveSettings();
            String note = LogiLightStore.lastError();
            if (!saved) {
                return "!! SETTINGS NOT SAVED - " + note;
            }
            return note.isEmpty()
                    ? "settings saved to " + LogiLightStore.settingsFile().getPath()
                    : "settings saved  !! " + note;
        }
        if (xPressed) {
            if (!confirmTwice(RESET_CONFIRM_KEY)) {
                return resetArmedMessage();
            }
            boolean hadFile = LogiLightStore.hasSavedSettings();
            LogiLightStore.clearLastError();
            boolean deleted = LogiLightStore.deleteSavedSettings();
            LogiLightSettings.resetToCodeDefaults();
            String note = LogiLightStore.lastError();
            if (!note.isEmpty()) {
                return "settings reset to code defaults  !! " + note;
            }
            return deleted && hadFile
                    ? "code defaults restored, old settings moved to " + LogiLightStore.settingsBackupFile().getName()
                    : "code defaults restored";
        }
        return null;
    }

    public static String settingsHint() {
        return "gp1 LB+Y = save settings   LB+X twice = reset settings to code defaults";
    }

    public static String resetArmedMessage() {
        return String.format(Locale.US, "!! press LB+X again within %.0f s to RESET all settings to code defaults",
                CONFIRM_WINDOW_SECONDS);
    }

    public static boolean confirmTwice(String key) {
        long now = System.nanoTime();
        synchronized (CONFIRM_LOCK) {
            boolean confirmed = key != null && key.equals(armedConfirmKey)
                    && now - armedConfirmNanos <= (long) (CONFIRM_WINDOW_SECONDS * 1e9);
            if (confirmed) {
                armedConfirmKey = null;
                armedConfirmNanos = 0L;
                return true;
            }
            armedConfirmKey = key;
            armedConfirmNanos = now;
            return false;
        }
    }

    public static void disarmConfirm() {
        synchronized (CONFIRM_LOCK) {
            armedConfirmKey = null;
            armedConfirmNanos = 0L;
        }
    }

    public static GoalScanner.Alliance allianceFromButtons(GoalScanner.Alliance current, boolean bluePressed,
                                                           boolean redPressed) {
        if (bluePressed) {
            return GoalScanner.Alliance.BLUE;
        }
        if (redPressed) {
            return GoalScanner.Alliance.RED;
        }
        return current;
    }

    public static boolean pressedWithoutStart(boolean pressed, Gamepad gamepad) {
        return pressed && gamepad != null && !gamepad.start;
    }

    public static int goalsPipelineIndex() {
        int named = LogiLightSettings.findPipeline(GOALS_PIPELINE_NAME);
        if (named >= 0) {
            return named;
        }
        int tags = firstPipelineOfType(PipelineType.APRILTAG);
        if (tags >= 0) {
            return tags;
        }
        int combo = firstPipelineOfType(PipelineType.APRILTAG_AND_COLOR);
        return combo >= 0 ? combo : LogiLightSettings.defaultPipelineIndex();
    }

    public static int pollenPipelineIndex(String profileName) {
        if (profileName == null || profileName.trim().isEmpty()) {
            return -1;
        }
        String profile = profileName.trim();
        int named = LogiLightSettings.findPipeline(POLLEN_PIPELINE_PREFIX + profile);
        if (named >= 0) {
            return named;
        }
        int nectar = LogiLightSettings.findPipeline(NECTAR_PIPELINE_PREFIX + profile);
        if (nectar >= 0) {
            return nectar;
        }
        int count = LogiLightSettings.pipelineCount();
        for (int i = 0; i < count; i++) {
            LogiPipeline pipeline = LogiLightSettings.pipelineAt(i);
            if (pipeline != null && pipeline.typeOrDefault() == PipelineType.COLOR && pipeline.usesColorProfile(profile)) {
                return i;
            }
        }
        return -1;
    }

    public static int firstPipelineOfType(PipelineType type) {
        int count = LogiLightSettings.pipelineCount();
        for (int i = 0; i < count; i++) {
            LogiPipeline pipeline = LogiLightSettings.pipelineAt(i);
            if (pipeline != null && pipeline.typeOrDefault() == type) {
                return i;
            }
        }
        return -1;
    }

    public static boolean seesTags(int pipelineIndex) {
        LogiPipeline pipeline = LogiLightSettings.pipelineAt(pipelineIndex);
        if (pipeline == null) {
            return false;
        }
        PipelineType type = pipeline.typeOrDefault();
        return type == PipelineType.APRILTAG || type == PipelineType.APRILTAG_AND_COLOR;
    }

    public static String pipelineLabel(int index) {
        LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
        if (pipeline == null) {
            return index < 0 ? "none" : index + " (missing)";
        }
        return String.format(Locale.US, "%d %s (%s)", index, pipeline.name, pipeline.typeOrDefault());
    }

    public static String allianceProfile(GoalScanner.Alliance alliance) {
        return alliance == GoalScanner.Alliance.BLUE ? BLUE_PROFILE : RED_PROFILE;
    }

    public static boolean isKnownGoal(String name) {
        if (name == null) {
            return false;
        }
        String trimmed = name.trim();
        for (String goal : KNOWN_GOALS) {
            if (goal.equalsIgnoreCase(trimmed)) {
                return true;
            }
        }
        return false;
    }

    public static boolean isAllianceGoal(String name, GoalScanner.Alliance alliance) {
        if (name == null || alliance == null) {
            return false;
        }
        String trimmed = name.trim();
        return alliance.firstCluster.equalsIgnoreCase(trimmed) || alliance.secondCluster.equalsIgnoreCase(trimmed);
    }

    public static boolean isFresh(LogiResult result, int pipelineIndex, double maxAgeMs) {
        if (result == null || result.isExpired() || result.getCaptureNanos() == 0L) {
            return false;
        }
        if (pipelineIndex >= 0 && result.getPipelineIndex() != pipelineIndex) {
            return false;
        }
        double age = result.getStaleness();
        return isFinite(age) && age <= maxAgeMs;
    }

    public static double goalBearingLeftDegrees(LogiFiducial fiducial) {
        if (fiducial == null) {
            return Double.NaN;
        }
        if (isFinite(fiducial.aimBearingLeftDegrees)) {
            return fiducial.aimBearingLeftDegrees;
        }
        if (isFinite(fiducial.bearingLeftDegrees)) {
            return fiducial.bearingLeftDegrees;
        }
        return isFinite(fiducial.tx) ? -fiducial.tx : Double.NaN;
    }

    public static double goalDistanceInches(LogiFiducial fiducial) {
        if (fiducial == null) {
            return Double.NaN;
        }
        if (isFinite(fiducial.aimDistanceInches)) {
            return fiducial.aimDistanceInches;
        }
        return fiducial.robotDistanceInches;
    }

    public static LogiFiducial chooseGoal(LogiResult result, GoalScanner.Alliance alliance, boolean preferNearest) {
        if (result == null) {
            return null;
        }
        List<LogiFiducial> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }
        LogiFiducial best = null;
        for (LogiFiducial fiducial : fiducials) {
            if (fiducial == null || !isAllianceGoal(fiducial.name, alliance) || !isFinite(goalBearingLeftDegrees(fiducial))) {
                continue;
            }
            if (best == null || better(fiducial, best, preferNearest)) {
                best = fiducial;
            }
        }
        if (best != null) {
            return best;
        }
        LogiFiducial primary = result.getPrimaryFiducial();
        if (primary != null && !isKnownGoal(primary.name) && isFinite(goalBearingLeftDegrees(primary))) {
            return primary;
        }
        for (LogiFiducial fiducial : fiducials) {
            if (fiducial == null || isKnownGoal(fiducial.name) || !isFinite(goalBearingLeftDegrees(fiducial))) {
                continue;
            }
            if (best == null || better(fiducial, best, preferNearest)) {
                best = fiducial;
            }
        }
        return best;
    }

    private static boolean better(LogiFiducial candidate, LogiFiducial current, boolean preferNearest) {
        if (preferNearest) {
            double a = goalDistanceInches(candidate);
            double b = goalDistanceInches(current);
            if (isFinite(a) && isFinite(b)) {
                return a < b;
            }
            if (isFinite(a) != isFinite(b)) {
                return isFinite(a);
            }
        }
        return Math.abs(goalBearingLeftDegrees(candidate)) < Math.abs(goalBearingLeftDegrees(current));
    }

    public static double headingChangeSinceDegrees(HeadingHistory history, long captureNanos, double headingNowRadians) {
        if (history == null || captureNanos == 0L || !isFinite(headingNowRadians)) {
            return 0.0;
        }
        Double then = history.headingAt(captureNanos, HEADING_LOOKUP_SLACK_NANOS);
        if (then == null) {
            return 0.0;
        }
        return Math.toDegrees(PlanarPose.normalizeRadians(headingNowRadians - then));
    }

    public static double compensatedTx(ColorTarget target, HeadingHistory history, long captureNanos,
                                       double headingNowRadians) {
        if (target == null) {
            return Double.NaN;
        }
        return target.tx + headingChangeSinceDegrees(history, captureNanos, headingNowRadians);
    }

    public static double robotCenteredTx(ColorTarget target, HeadingHistory history, long captureNanos,
                                         double headingNowRadians) {
        if (target == null) {
            return Double.NaN;
        }
        double raw = target.hasGroundPosition() && isFinite(target.groundBearingLeftDegrees)
                ? -target.groundBearingLeftDegrees
                : target.txNC;
        return raw + headingChangeSinceDegrees(history, captureNanos, headingNowRadians);
    }

    public static PlanarPose toPlanar(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new PlanarPose(pose.x(), pose.y(), pose.heading());
    }

    public static Pose startPose(GoalScanner.Alliance alliance) {
        return GoalAim.startPose(alliance);
    }

    public static boolean isStill(Localizer localizer, double maxSpeedInchesPerSecond, double maxTurnDegreesPerSecond) {
        if (localizer == null) {
            return true;
        }
        try {
            Velocity velocity = localizer.velocity();
            if (velocity == null) {
                return true;
            }
            double speed = Math.hypot(velocity.vx, velocity.vy);
            double turn = Math.abs(Math.toDegrees(velocity.omega));
            return isFinite(speed) && isFinite(turn) && speed <= maxSpeedInchesPerSecond && turn <= maxTurnDegreesPerSecond;
        } catch (Throwable t) {
            return true;
        }
    }

    public static String speedText(Localizer localizer) {
        if (localizer == null) {
            return "no odometry";
        }
        try {
            Velocity velocity = localizer.velocity();
            if (velocity == null) {
                return "unknown";
            }
            return String.format(Locale.US, "%.1f in/s  %.1f deg/s", Math.hypot(velocity.vx, velocity.vy),
                    Math.toDegrees(velocity.omega));
        } catch (Throwable t) {
            return "unknown";
        }
    }

    public static String poseError(LogiBotPose vision, Pose reference) {
        if (vision == null || vision.pose == null || !vision.pose.isPositionFinite() || reference == null) {
            return "none";
        }
        double dx = vision.pose.x - reference.x();
        double dy = vision.pose.y - reference.y();
        String text = String.format(Locale.US, "%.1f in  (dx %+.1f  dy %+.1f)", Math.hypot(dx, dy), dx, dy);
        if (vision.pose.hasHeading() && isFinite(reference.heading())) {
            double dh = Math.toDegrees(PlanarPose.normalizeRadians(vision.pose.heading - reference.heading()));
            text += String.format(Locale.US, "  dh %+.1f deg", dh);
        }
        return text;
    }

    public static double deadzone(double value) {
        if (!isFinite(value) || Math.abs(value) < STICK_DEADZONE) {
            return 0.0;
        }
        return Range.clip(value, -1.0, 1.0);
    }

    public static double clip(double value, double limit) {
        if (!isFinite(value)) {
            return 0.0;
        }
        return Range.clip(value, -Math.abs(limit), Math.abs(limit));
    }

    public static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    public static String describe(Throwable t) {
        if (t == null) {
            return "unknown error";
        }
        String message = t.getMessage();
        String name = t.getClass().getSimpleName();
        return message == null || message.trim().isEmpty() ? name : name + ": " + message.trim();
    }
}
