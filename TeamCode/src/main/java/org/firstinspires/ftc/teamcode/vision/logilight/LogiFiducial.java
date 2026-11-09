package org.firstinspires.ftc.teamcode.vision.logilight;

import org.firstinspires.ftc.teamcode.vision.logilight.math.Mat3;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class LogiFiducial {

    public static final int CLUSTER_ID = -1;
    public static final int QUAD_VALUES = 8;

    private static final double NANOS_PER_MILLI = 1e6;

    public final String name;
    public final int id;
    public final boolean isCluster;
    public final int percentClusterFound;
    public final int hamming;
    public final double decisionMargin;

    public final Vec3 targetInOptical;
    public final Mat3 rotationInOptical;
    public final Vec3 targetInRobot;
    public final Mat3 rotationInRobot;

    public final double range;
    public final double bearing;
    public final double elevation;
    public final double yaw;
    public final double pitch;
    public final double roll;

    public final Vec3 aimInRobot;
    public final double tx;
    public final double ty;
    public final double txNC;
    public final double tyNC;
    public final double ta;
    public final double centerX;
    public final double centerY;
    public final List<double[]> corners;

    public final double robotDistanceInches;
    public final double bearingLeftDegrees;
    public final double aimDistanceInches;
    public final double aimBearingLeftDegrees;

    public final String matchedState;
    public final double heightErrorInches;
    public final double tiltMismatchDegrees;
    public final double rotationMismatchDegrees;
    public final double angularVelocityDegPerSec;
    public final PlanarPose mt1;
    public final PlanarPose mt2;
    public final boolean usedForLocalization;
    public final String rejectReason;
    public final long captureNanos;

    private LogiFiducial(Builder b) {
        this.name = b.name == null ? "" : b.name;
        this.id = b.id;
        this.isCluster = b.isCluster;
        this.percentClusterFound = b.percentClusterFound;
        this.hamming = b.hamming;
        this.decisionMargin = b.decisionMargin;
        this.targetInOptical = b.targetInOptical;
        this.rotationInOptical = b.rotationInOptical;
        this.targetInRobot = b.targetInRobot;
        this.rotationInRobot = b.rotationInRobot;
        this.range = b.range;
        this.bearing = b.bearing;
        this.elevation = b.elevation;
        this.yaw = b.yaw;
        this.pitch = b.pitch;
        this.roll = b.roll;
        this.aimInRobot = b.aimInRobot;
        this.tx = b.tx;
        this.ty = b.ty;
        this.txNC = b.txNC;
        this.tyNC = b.tyNC;
        this.ta = b.ta;
        this.centerX = b.centerX;
        this.centerY = b.centerY;
        this.corners = copyQuads(b.corners);
        this.robotDistanceInches = b.robotDistanceInches;
        this.bearingLeftDegrees = b.bearingLeftDegrees;
        this.aimDistanceInches = b.aimDistanceInches;
        this.aimBearingLeftDegrees = b.aimBearingLeftDegrees;
        this.matchedState = b.matchedState == null ? "" : b.matchedState;
        this.heightErrorInches = b.heightErrorInches;
        this.tiltMismatchDegrees = b.tiltMismatchDegrees;
        this.rotationMismatchDegrees = b.rotationMismatchDegrees;
        this.angularVelocityDegPerSec = b.angularVelocityDegPerSec;
        this.mt1 = b.mt1;
        this.mt2 = b.mt2;
        this.usedForLocalization = b.mt1 != null || b.mt2 != null;
        this.rejectReason = b.rejectReason == null ? "" : b.rejectReason;
        this.captureNanos = b.captureNanos;
    }

    public double ageMs() {
        if (captureNanos == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - captureNanos) / NANOS_PER_MILLI;
    }

    public boolean hasPose() {
        return targetInOptical != null && rotationInOptical != null && targetInRobot != null && rotationInRobot != null;
    }

    public boolean hasCenter() {
        return isFinite(centerX) && isFinite(centerY);
    }

    public boolean isFullyAccepted() {
        return usedForLocalization && rejectReason.isEmpty();
    }

    public String localizationTag() {
        if (mt2 != null && rejectReason.isEmpty()) {
            return "[MT2 ok]";
        }
        if (mt2 != null) {
            return "[MT2 ok]  " + rejectReason;
        }
        if (mt1 != null) {
            return rejectReason.isEmpty() ? "[MT1 ok]" : "[MT1 only] " + rejectReason;
        }
        if (!rejectReason.isEmpty()) {
            return "[" + rejectReason + "]";
        }
        return "";
    }

    public String summary() {
        StringBuilder line = new StringBuilder();
        line.append(name);
        if (isCluster) {
            line.append(' ').append(percentClusterFound).append('%');
        }
        if (isFinite(robotDistanceInches)) {
            line.append(String.format(Locale.US, "  d %.1f in  bL %+.1f deg", robotDistanceInches, bearingLeftDegrees));
        }
        line.append(String.format(Locale.US, "  tx %.1f", tx));
        String tag = localizationTag();
        if (!tag.isEmpty()) {
            line.append("  ").append(tag);
        }
        return line.toString();
    }

    @Override
    public String toString() {
        return summary();
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static List<double[]> copyQuads(List<double[]> quads) {
        List<double[]> copy = new ArrayList<double[]>();
        if (quads != null) {
            for (double[] quad : quads) {
                if (quad != null && quad.length >= QUAD_VALUES) {
                    double[] values = new double[QUAD_VALUES];
                    System.arraycopy(quad, 0, values, 0, QUAD_VALUES);
                    copy.add(values);
                }
            }
        }
        return Collections.unmodifiableList(copy);
    }

    public static final class Builder {
        public String name = "";
        public int id = CLUSTER_ID;
        public boolean isCluster = false;
        public int percentClusterFound = 100;
        public int hamming = 0;
        public double decisionMargin = Double.NaN;
        public Vec3 targetInOptical;
        public Mat3 rotationInOptical;
        public Vec3 targetInRobot;
        public Mat3 rotationInRobot;
        public double range = Double.NaN;
        public double bearing = Double.NaN;
        public double elevation = Double.NaN;
        public double yaw = Double.NaN;
        public double pitch = Double.NaN;
        public double roll = Double.NaN;
        public Vec3 aimInRobot;
        public double tx = 0.0;
        public double ty = 0.0;
        public double txNC = 0.0;
        public double tyNC = 0.0;
        public double ta = 0.0;
        public double centerX = Double.NaN;
        public double centerY = Double.NaN;
        public List<double[]> corners = new ArrayList<double[]>();
        public double robotDistanceInches = Double.NaN;
        public double bearingLeftDegrees = Double.NaN;
        public double aimDistanceInches = Double.NaN;
        public double aimBearingLeftDegrees = Double.NaN;
        public String matchedState = "";
        public double heightErrorInches = Double.NaN;
        public double tiltMismatchDegrees = Double.NaN;
        public double rotationMismatchDegrees = Double.NaN;
        public double angularVelocityDegPerSec = Double.NaN;
        public PlanarPose mt1;
        public PlanarPose mt2;
        public String rejectReason = "";
        public long captureNanos = 0L;

        public Builder() {
        }

        public LogiFiducial build() {
            return new LogiFiducial(this);
        }
    }
}
