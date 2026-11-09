package org.firstinspires.ftc.teamcode.vision.logilight;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class LogiBotPose {

    public static final String METHOD_MT1 = "MT1";
    public static final String METHOD_MT2 = "MT2";

    private static final double NANOS_PER_MILLI = 1e6;

    public final PlanarPose pose;
    public final String method;
    public final int targetCount;
    public final double avgDistanceInches;
    public final double avgAreaPercent;
    public final double spanInches;
    public final double stddevXYInches;
    public final double stddevHeadingRadians;
    public final long captureNanos;
    public final List<String> targetNames;

    public LogiBotPose(PlanarPose pose, String method, int targetCount, double avgDistanceInches,
                       double avgAreaPercent, double spanInches, double stddevXYInches,
                       double stddevHeadingRadians, long captureNanos, List<String> targetNames) {
        this.pose = pose;
        this.method = method == null ? "" : method;
        this.targetCount = Math.max(0, targetCount);
        this.avgDistanceInches = avgDistanceInches;
        this.avgAreaPercent = avgAreaPercent;
        this.spanInches = spanInches;
        this.stddevXYInches = stddevXYInches;
        this.stddevHeadingRadians = stddevHeadingRadians;
        this.captureNanos = captureNanos;
        List<String> names = new ArrayList<String>();
        if (targetNames != null) {
            for (String name : targetNames) {
                if (name != null) {
                    names.add(name);
                }
            }
        }
        this.targetNames = Collections.unmodifiableList(names);
    }

    public Pose toPedroPose() {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.x, pose.y, pose.heading);
    }

    public Pose toPedroPositionOnly() {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.x, pose.y, Double.NaN);
    }

    public double ageMs() {
        if (captureNanos == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - captureNanos) / NANOS_PER_MILLI;
    }

    public boolean isMegaTag2() {
        return METHOD_MT2.equals(method);
    }

    public boolean hasHeading() {
        return pose != null && pose.hasHeading();
    }

    public boolean isFinite() {
        return pose != null && pose.isPositionFinite();
    }

    public double x() {
        return pose == null ? Double.NaN : pose.x;
    }

    public double y() {
        return pose == null ? Double.NaN : pose.y;
    }

    public double heading() {
        return pose == null ? Double.NaN : pose.heading;
    }

    @Override
    public String toString() {
        if (pose == null) {
            return method + "  none";
        }
        return String.format(Locale.US, "%s  %s  tags %d  d %.1f in  sd %.1f in",
                method, LogiLightTelemetry.formatPose(pose), targetCount, avgDistanceInches, stddevXYInches);
    }
}
