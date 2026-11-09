package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;

import java.util.Locale;

public final class LogiOdometryFeed {

    public static final String SOURCE_START_POSE = "start pose";
    public static final String SOURCE_AUTO = "end of auto";
    public static final String SOURCE_VISION = "vision MT1";

    private final Localizer localizer;
    private String fault;
    private Pose pose;
    private long poseNanos;
    private long updateCount;
    private String source = "";
    private GoalScanner.Alliance alliance;

    public LogiOdometryFeed(HardwareMap hardwareMap) {
        Localizer created = null;
        String problem = "";
        try {
            created = Constants.createLocalizer(hardwareMap);
        } catch (Throwable t) {
            problem = "NOT AVAILABLE - " + LogiOpModeBase.describe(t);
        }
        localizer = created;
        fault = problem;
        GoalScanner.Alliance handoff = null;
        try {
            handoff = PoseHandoff.alliance();
        } catch (Throwable ignored) {
        }
        alliance = handoff == null ? FieldPoints.DEFAULT_ALLIANCE : handoff;
    }

    public boolean isAvailable() {
        return localizer != null;
    }

    public String fault() {
        return fault;
    }

    public GoalScanner.Alliance alliance() {
        return alliance;
    }

    public void setAlliance(GoalScanner.Alliance newAlliance) {
        if (newAlliance != null) {
            alliance = newAlliance;
        }
    }

    public Pose startPose() {
        return GoalAim.startPose(alliance);
    }

    public boolean resetToStartPose() {
        boolean ok = setPose(startPose());
        source = SOURCE_START_POSE;
        return ok;
    }

    public boolean useAutoHandoffOrStart() {
        Pose handoff = null;
        try {
            handoff = PoseHandoff.pose();
        } catch (Throwable ignored) {
        }
        if (handoff != null && setPose(handoff)) {
            source = SOURCE_AUTO;
            return true;
        }
        return resetToStartPose();
    }

    public boolean setPose(Pose newPose) {
        if (newPose == null || !isFinite(newPose.x()) || !isFinite(newPose.y()) || !isFinite(newPose.heading())) {
            return false;
        }
        if (localizer == null) {
            pose = null;
            return false;
        }
        try {
            localizer.setPose(newPose);
            pose = newPose;
            poseNanos = System.nanoTime();
            return true;
        } catch (Throwable t) {
            fault = "setPose failed - " + LogiOpModeBase.describe(t);
            return false;
        }
    }

    public boolean setPoseFromVision(Pose visionPose) {
        boolean ok = setPose(visionPose);
        if (ok) {
            source = SOURCE_VISION;
        }
        return ok;
    }

    public Pose update() {
        if (localizer == null) {
            return null;
        }
        try {
            long before = System.nanoTime();
            localizer.update();
            long after = System.nanoTime();
            Pose next = localizer.pose();
            if (next != null && isFinite(next.x()) && isFinite(next.y()) && isFinite(next.heading())) {
                pose = next;
                poseNanos = before + (after - before) / 2L;
                updateCount++;
            }
        } catch (Throwable t) {
            fault = "update failed - " + LogiOpModeBase.describe(t);
        }
        return pose;
    }

    public void feed(LogiLight light) {
        Pose current = pose;
        if (light == null || current == null || poseNanos == 0L) {
            return;
        }
        try {
            light.updateRobotOrientation(current.heading(), poseNanos);
        } catch (Throwable ignored) {
        }
    }

    public Pose pose() {
        return pose;
    }

    public PlanarPose planarPose() {
        Pose current = pose;
        if (current == null) {
            return null;
        }
        return new PlanarPose(current.x(), current.y(), current.heading());
    }

    public long poseNanos() {
        return poseNanos;
    }

    public long updateCount() {
        return updateCount;
    }

    public String source() {
        return source;
    }

    public String summary() {
        if (localizer == null) {
            return "!! " + (fault.isEmpty() ? "NOT AVAILABLE" : fault);
        }
        String text = LogiLightTelemetry.formatPose(pose);
        if (!source.isEmpty()) {
            text += "  (" + source + ")";
        }
        if (!fault.isEmpty()) {
            text += "  !! " + fault;
        }
        return text;
    }

    public String headingSummary(LogiLight light) {
        if (localizer == null) {
            return "!! no Pinpoint - MT2 off, MT1 only";
        }
        if (light == null) {
            return "no camera";
        }
        double heading = light.getLastHeadingRadians();
        if (!isFinite(heading)) {
            return "waiting for Pinpoint";
        }
        return String.format(Locale.US, "Pinpoint h %.1f deg  %d samples  (MT2 on)",
                Math.toDegrees(heading), light.getHeadingSampleCount());
    }

    static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
