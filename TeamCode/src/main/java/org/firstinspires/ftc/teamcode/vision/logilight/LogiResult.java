package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class LogiResult {

    public static final int SCRIPT_OUTPUT_LENGTH = 32;
    public static final String PRIMARY_NONE = "none";
    public static final String PRIMARY_FIDUCIAL = "fiducial";
    public static final String PRIMARY_COLOR = "color";
    public static final String PRIMARY_SCRIPT = "script";

    private static final double NANOS_PER_MILLI = 1e6;

    public static final LogiResult EMPTY = new Builder().build();

    private final boolean valid;
    private final double tx;
    private final double ty;
    private final double txNC;
    private final double tyNC;
    private final double ta;
    private final int pipelineIndex;
    private final String pipelineName;
    private final PipelineType pipelineType;
    private final long captureNanos;
    private final double targetingLatencyMs;
    private final List<LogiFiducial> fiducials;
    private final List<ColorTarget> colors;
    private final double[] scriptOutput;
    private final LogiScriptResult scriptResult;
    private final LogiBotPose botpose;
    private final LogiBotPose botposeMt2;
    private final LogiFiducial primaryFiducial;
    private final ColorTarget primaryColor;
    private final String primaryName;
    private final String primaryKind;
    private final double primaryCenterX;
    private final double primaryCenterY;
    private final double primaryDistanceInches;
    private final double primaryBearingLeftDegrees;
    private final long frameNumber;
    private final long buildNanos;
    private final boolean expired;

    private LogiResult(Builder b) {
        this.fiducials = copyList(b.fiducials);
        this.colors = copyList(b.colors);
        this.primaryFiducial = b.primaryFiducial;
        this.primaryColor = b.primaryColor;
        String kind = b.primaryKind == null ? PRIMARY_NONE : b.primaryKind;
        this.valid = !PRIMARY_NONE.equals(kind);
        this.primaryKind = kind;
        this.tx = finiteOrZero(b.tx);
        this.ty = finiteOrZero(b.ty);
        this.txNC = finiteOrZero(b.txNC);
        this.tyNC = finiteOrZero(b.tyNC);
        this.ta = finiteOrZero(b.ta);
        this.pipelineIndex = b.pipelineIndex;
        this.pipelineName = b.pipelineName == null ? "" : b.pipelineName;
        this.pipelineType = b.pipelineType == null ? PipelineType.APRILTAG : b.pipelineType;
        this.captureNanos = b.captureNanos;
        this.targetingLatencyMs = b.targetingLatencyMs;
        this.scriptOutput = padOutputs(b.scriptOutput);
        this.scriptResult = b.scriptResult == null ? LogiScriptResult.EMPTY : b.scriptResult;
        this.botpose = b.botpose;
        this.botposeMt2 = b.botposeMt2;
        this.primaryName = b.primaryName == null ? "" : b.primaryName;
        this.primaryCenterX = b.primaryCenterX;
        this.primaryCenterY = b.primaryCenterY;
        this.primaryDistanceInches = b.primaryDistanceInches;
        this.primaryBearingLeftDegrees = b.primaryBearingLeftDegrees;
        this.frameNumber = b.frameNumber;
        this.buildNanos = b.buildNanos == 0L ? System.nanoTime() : b.buildNanos;
        this.expired = b.expired;
    }

    public boolean isValid() {
        return valid;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTxNC() {
        return txNC;
    }

    public double getTyNC() {
        return tyNC;
    }

    public double getTa() {
        return ta;
    }

    public int getPipelineIndex() {
        return pipelineIndex;
    }

    public String getPipelineName() {
        return pipelineName;
    }

    public PipelineType getPipelineType() {
        return pipelineType;
    }

    public long getCaptureNanos() {
        return captureNanos;
    }

    public double getStaleness() {
        if (captureNanos == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - captureNanos) / NANOS_PER_MILLI;
    }

    public double getTargetingLatency() {
        return targetingLatencyMs;
    }

    public List<LogiFiducial> getFiducialResults() {
        return fiducials;
    }

    public List<ColorTarget> getColorResults() {
        return colors;
    }

    public double[] getScriptOutput() {
        return scriptOutput.clone();
    }

    public LogiScriptResult getScriptResult() {
        return scriptResult;
    }

    public LogiBotPose getBotpose() {
        return botpose;
    }

    public LogiBotPose getBotpose_MT2() {
        return botposeMt2;
    }

    public LogiBotPose getBestBotpose() {
        return botposeMt2 != null ? botposeMt2 : botpose;
    }

    public boolean hasBotpose() {
        return botpose != null || botposeMt2 != null;
    }

    public LogiFiducial getPrimaryFiducial() {
        return primaryFiducial;
    }

    public ColorTarget getPrimaryColor() {
        return primaryColor;
    }

    public LogiFiducial getFiducial(String name) {
        if (name == null) {
            return null;
        }
        String wanted = name.trim();
        for (LogiFiducial fiducial : fiducials) {
            if (fiducial.name.equalsIgnoreCase(wanted)) {
                return fiducial;
            }
        }
        for (LogiFiducial fiducial : fiducials) {
            if (!fiducial.isCluster && String.valueOf(fiducial.id).equals(wanted)) {
                return fiducial;
            }
        }
        return null;
    }

    public List<ColorTarget> getColorResults(String profileName) {
        if (profileName == null) {
            return Collections.emptyList();
        }
        String wanted = profileName.trim();
        List<ColorTarget> matching = new ArrayList<ColorTarget>();
        for (ColorTarget target : colors) {
            if (target.profileName != null && target.profileName.trim().equalsIgnoreCase(wanted)) {
                matching.add(target);
            }
        }
        return Collections.unmodifiableList(matching);
    }

    public String getPrimaryName() {
        return primaryName;
    }

    public String getPrimaryKind() {
        return primaryKind;
    }

    public double getPrimaryCenterX() {
        return primaryCenterX;
    }

    public double getPrimaryCenterY() {
        return primaryCenterY;
    }

    public double getPrimaryDistanceInches() {
        return primaryDistanceInches;
    }

    public double getPrimaryBearingLeftDegrees() {
        return primaryBearingLeftDegrees;
    }

    public long getFrameNumber() {
        return frameNumber;
    }

    public long getBuildNanos() {
        return buildNanos;
    }

    public boolean isExpired() {
        return expired;
    }

    public int getFiducialCount() {
        return fiducials.size();
    }

    public int getColorCount() {
        return colors.size();
    }

    public LogiResult expiredCopy() {
        Builder b = new Builder();
        b.pipelineIndex = pipelineIndex;
        b.pipelineName = pipelineName;
        b.pipelineType = pipelineType;
        b.captureNanos = captureNanos;
        b.targetingLatencyMs = targetingLatencyMs;
        b.frameNumber = frameNumber;
        b.expired = true;
        return b.build();
    }

    public String summary() {
        if (!valid) {
            return String.format(Locale.US, "no target  (pipe %d %s)%s", pipelineIndex, pipelineName, expired ? "  !! stale" : "");
        }
        return String.format(Locale.US, "%s  tx %.1f  ty %.1f  ta %.2f", primaryName, tx, ty, ta);
    }

    @Override
    public String toString() {
        return summary();
    }

    private static double finiteOrZero(double value) {
        return Double.isNaN(value) || Double.isInfinite(value) ? 0.0 : value;
    }

    private static double[] padOutputs(double[] outputs) {
        double[] padded = new double[SCRIPT_OUTPUT_LENGTH];
        if (outputs != null) {
            int count = Math.min(outputs.length, SCRIPT_OUTPUT_LENGTH);
            for (int i = 0; i < count; i++) {
                padded[i] = finiteOrZero(outputs[i]);
            }
        }
        return padded;
    }

    private static <T> List<T> copyList(List<T> source) {
        if (source == null || source.isEmpty()) {
            return Collections.emptyList();
        }
        List<T> copy = new ArrayList<T>(source.size());
        for (T item : source) {
            if (item != null) {
                copy.add(item);
            }
        }
        return Collections.unmodifiableList(copy);
    }

    public static final class Builder {
        public double tx = 0.0;
        public double ty = 0.0;
        public double txNC = 0.0;
        public double tyNC = 0.0;
        public double ta = 0.0;
        public int pipelineIndex = 0;
        public String pipelineName = "";
        public PipelineType pipelineType = PipelineType.APRILTAG;
        public long captureNanos = 0L;
        public double targetingLatencyMs = 0.0;
        public List<LogiFiducial> fiducials = new ArrayList<LogiFiducial>();
        public List<ColorTarget> colors = new ArrayList<ColorTarget>();
        public double[] scriptOutput = new double[SCRIPT_OUTPUT_LENGTH];
        public LogiScriptResult scriptResult = null;
        public LogiBotPose botpose = null;
        public LogiBotPose botposeMt2 = null;
        public LogiFiducial primaryFiducial = null;
        public ColorTarget primaryColor = null;
        public String primaryName = "";
        public String primaryKind = PRIMARY_NONE;
        public double primaryCenterX = Double.NaN;
        public double primaryCenterY = Double.NaN;
        public double primaryDistanceInches = Double.NaN;
        public double primaryBearingLeftDegrees = Double.NaN;
        public long frameNumber = 0L;
        public long buildNanos = 0L;
        public boolean expired = false;

        public Builder() {
        }

        public LogiResult build() {
            return new LogiResult(this);
        }
    }
}
