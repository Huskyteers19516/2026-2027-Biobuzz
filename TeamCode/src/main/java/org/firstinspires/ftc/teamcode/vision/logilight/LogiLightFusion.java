package org.firstinspires.ftc.teamcode.vision.logilight;

import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class LogiLightFusion {

    public static final int MIN_BUFFER_SIZE = 100;
    public static final double JUMP_CONSISTENCY_INCHES = 6.0;
    public static final double MAX_MEASUREMENT_AGE_MS = 500.0;
    public static final double SNAP_VARIANCE_XY = 0.01;
    public static final double SNAP_VARIANCE_HEADING = 0.0001;
    public static final double MIN_VARIANCE = 1e-6;
    public static final double MAX_HEADING_VARIANCE = 6.0;
    public static final double MIN_VARIANCE_SCALE = 0.25;

    private static final double NANOS_PER_MILLI = 1e6;

    private final FusionLocalizer fusion;
    private final List<Pose> pendingJumps = new ArrayList<Pose>();

    private volatile boolean enabled = true;
    private volatile long beforeUpdateNanos = 0L;
    private volatile long lastSetPoseNanos;
    private volatile long lastFusedCaptureNanos = 0L;
    private volatile int acceptedCount = 0;
    private volatile int rejectedCount = 0;
    private volatile int jumpCount = 0;
    private volatile String lastDecision = "no measurements yet";
    private volatile Pose lastMeasurement;
    private volatile long lastAcceptedNanos = 0L;

    public LogiLightFusion(Localizer deadReckoning, Pose startPose) {
        LogiLocalizationConfig loc = config();
        Pose initialCovariance = variancePose(loc.initialVarianceXY, loc.initialVarianceHeading);
        Pose processVariance = variancePose(loc.processVarianceXY, loc.processVarianceHeading);
        Pose measurementVariance = variancePose(loc.measurementVarianceXY, loc.measurementVarianceHeading);
        int bufferSize = Math.max(MIN_BUFFER_SIZE, loc.fusionBufferSize);
        this.fusion = new FusionLocalizer(deadReckoning, initialCovariance, processVariance, measurementVariance, bufferSize);
        if (startPose != null) {
            fusion.setStartPose(startPose);
        }
        this.lastSetPoseNanos = System.nanoTime();
    }

    public FusionLocalizer localizer() {
        return fusion;
    }

    public void beforeUpdate() {
        beforeUpdateNanos = System.nanoTime();
    }

    public void update() {
        beforeUpdate();
        fusion.update();
    }

    public Pose pose() {
        return fusion.pose();
    }

    public void publishHeading(LogiLight light) {
        if (light == null) {
            return;
        }
        try {
            Pose current = fusion.pose();
            if (current != null) {
                light.updateRobotOrientation(current.heading(), System.nanoTime());
            }
        } catch (Throwable ignored) {
        }
    }

    public boolean fuse(LogiResult result) {
        try {
            return fuseInternal(result);
        } catch (Throwable t) {
            return reject("!! fuse failed: " + LogiLight.describe(t));
        }
    }

    private boolean fuseInternal(LogiResult result) {
        if (!enabled) {
            return reject("fusion disabled");
        }
        if (result == null) {
            return reject("no result");
        }
        LogiLocalizationConfig loc = config();
        LogiBotPose mt1 = usable(result.getBotpose());
        LogiBotPose mt2 = usable(result.getBotpose_MT2());
        if (mt1 == null && mt2 == null) {
            return reject("no botpose");
        }
        boolean fuseHeading = loc.fuseMt1Heading && mt1 != null && mt1.hasHeading();
        LogiBotPose source = mt2 != null ? mt2 : mt1;
        long captureNanos = source.captureNanos;
        if (fuseHeading) {
            captureNanos = Math.max(captureNanos, mt1.captureNanos);
        }
        if (captureNanos <= 0L) {
            return reject("no capture time");
        }
        if (captureNanos < lastSetPoseNanos) {
            return reject("frame before last setPose");
        }
        if (captureNanos == lastFusedCaptureNanos) {
            return false;
        }
        long now = System.nanoTime();
        if ((now - captureNanos) / NANOS_PER_MILLI > MAX_MEASUREMENT_AGE_MS) {
            return reject(String.format(Locale.US, "too old %.0f ms", (now - captureNanos) / NANOS_PER_MILLI));
        }
        long before = beforeUpdateNanos;
        long timestamp = before > 0L ? Math.min(captureNanos, before) : captureNanos;

        double x = source.pose.x;
        double y = source.pose.y;
        double heading = fuseHeading ? mt1.pose.heading : Double.NaN;
        Pose measurement = new Pose(x, y, heading);
        lastMeasurement = measurement;

        double baseSigma = Math.max(1e-3, loc.stddevBaseInches);
        double sigmaXY = finiteOr(source.stddevXYInches, baseSigma);
        if (source == mt1 && mt1.hasHeading()) {
            double headingSigma = finiteOr(mt1.stddevHeadingRadians, Math.toRadians(loc.headingStddevBaseDegrees));
            double lever = finiteOr(mt1.avgDistanceInches, 0.0) * headingSigma;
            sigmaXY = Math.sqrt(sigmaXY * sigmaXY + lever * lever);
        }
        double scaleXY = (sigmaXY / baseSigma) * (sigmaXY / baseSigma);
        double varianceXY = Math.max(MIN_VARIANCE, loc.measurementVarianceXY * Math.max(MIN_VARIANCE_SCALE, scaleXY));
        double varianceHeading = Math.max(MIN_VARIANCE, loc.measurementVarianceHeading);
        if (fuseHeading) {
            double baseHeadingSigma = Math.max(1e-4, Math.toRadians(loc.headingStddevBaseDegrees));
            double headingSigma = finiteOr(mt1.stddevHeadingRadians, baseHeadingSigma);
            double scaleHeading = (headingSigma / baseHeadingSigma) * (headingSigma / baseHeadingSigma);
            varianceHeading = Math.max(MIN_VARIANCE, loc.measurementVarianceHeading * Math.max(MIN_VARIANCE_SCALE, scaleHeading));
        }

        Pose current = fusion.pose();
        double jump = current == null ? 0.0 : Math.hypot(x - current.x(), y - current.y());
        if (isFinite(jump) && jump > loc.maxJumpInches) {
            int needed = Math.max(1, loc.jumpConfirmFrames);
            if (!pendingJumps.isEmpty()) {
                Pose latest = pendingJumps.get(pendingJumps.size() - 1);
                if (Math.hypot(x - latest.x(), y - latest.y()) > JUMP_CONSISTENCY_INCHES) {
                    pendingJumps.clear();
                }
            }
            pendingJumps.add(measurement);
            lastFusedCaptureNanos = captureNanos;
            if (pendingJumps.size() < needed) {
                return reject(String.format(Locale.US, "jump %.0f in (%d/%d)", jump, pendingJumps.size(), needed));
            }
            pendingJumps.clear();
            fusion.addMeasurement(measurement, timestamp,
                    variancePose(SNAP_VARIANCE_XY, fuseHeading ? SNAP_VARIANCE_HEADING : varianceHeading));
            jumpCount++;
            return accept(String.format(Locale.US, "relocalized %s jump %.0f in", source.method, jump), now);
        }
        pendingJumps.clear();
        fusion.addMeasurement(measurement, timestamp, variancePose(varianceXY, varianceHeading));
        lastFusedCaptureNanos = captureNanos;
        return accept(String.format(Locale.US, "%s%s  sd %.1f in  var %.1f", source.method,
                fuseHeading ? "+MT1 heading" : "", sigmaXY, varianceXY), now);
    }

    public void setPose(Pose pose) {
        if (pose == null) {
            return;
        }
        fusion.setPose(pose);
        lastSetPoseNanos = System.nanoTime();
        pendingJumps.clear();
        lastDecision = "pose set";
    }

    public void setEnabled(boolean on) {
        enabled = on;
        if (!on) {
            pendingJumps.clear();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public int acceptedCount() {
        return acceptedCount;
    }

    public int rejectedCount() {
        return rejectedCount;
    }

    public int relocalizeCount() {
        return jumpCount;
    }

    public String lastDecision() {
        return lastDecision;
    }

    public Pose lastMeasurement() {
        return lastMeasurement;
    }

    public double msSinceLastAccepted() {
        long accepted = lastAcceptedNanos;
        if (accepted == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - accepted) / NANOS_PER_MILLI;
    }

    public String summary() {
        return String.format(Locale.US, "%s  ok %d  rejected %d  relocalized %d  %s",
                enabled ? "on" : "OFF", acceptedCount, rejectedCount, jumpCount, lastDecision);
    }

    private boolean accept(String decision, long now) {
        acceptedCount++;
        lastAcceptedNanos = now;
        lastDecision = decision;
        return true;
    }

    private boolean reject(String decision) {
        rejectedCount++;
        lastDecision = decision;
        return false;
    }

    private static LogiBotPose usable(LogiBotPose botPose) {
        if (botPose == null || botPose.pose == null || !botPose.pose.isPositionFinite()) {
            return null;
        }
        return botPose;
    }

    private static LogiLocalizationConfig config() {
        LogiLocalizationConfig loc = LogiLightSettings.LOCALIZATION;
        return loc == null ? new LogiLocalizationConfig() : loc;
    }

    private static Pose variancePose(double varianceXY, double varianceHeading) {
        double xy = isFinite(varianceXY) ? Math.max(MIN_VARIANCE, varianceXY) : 1.0;
        double heading = isFinite(varianceHeading) ? Math.max(MIN_VARIANCE, Math.min(MAX_HEADING_VARIANCE, varianceHeading)) : 0.01;
        return new Pose(xy, xy, heading);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double finiteOr(double value, double fallback) {
        return isFinite(value) && value > 0.0 ? value : fallback;
    }
}
