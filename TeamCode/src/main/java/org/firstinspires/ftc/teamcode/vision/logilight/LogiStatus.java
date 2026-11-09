package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.Locale;

public final class LogiStatus {

    public static final LogiStatus UNKNOWN = new Builder().build();

    public final String cameraState;
    public final String runState;
    public final String webcamName;
    public final double fps;
    public final double portalFps;
    public final long frameCount;
    public final double lastFrameAgeMs;
    public final double averageProcessingMs;
    public final int pipelineIndex;
    public final String pipelineName;
    public final PipelineType pipelineType;
    public final int width;
    public final int height;
    public final String intrinsicsSource;
    public final String exposureSummary;
    public final String controlStatus;
    public final int watchdogRestarts;
    public final String watchdogReason;
    public final boolean panelsStreaming;
    public final boolean driverStationStreaming;
    public final String settingsSource;
    public final String fieldMapSource;
    public final String fault;
    public final String lastProcessorError;
    public final long processorErrorCount;
    public final long readNanos;

    private LogiStatus(Builder b) {
        this.cameraState = text(b.cameraState);
        this.runState = text(b.runState);
        this.webcamName = text(b.webcamName);
        this.fps = b.fps;
        this.portalFps = b.portalFps;
        this.frameCount = b.frameCount;
        this.lastFrameAgeMs = b.lastFrameAgeMs;
        this.averageProcessingMs = b.averageProcessingMs;
        this.pipelineIndex = b.pipelineIndex;
        this.pipelineName = text(b.pipelineName);
        this.pipelineType = b.pipelineType == null ? PipelineType.APRILTAG : b.pipelineType;
        this.width = b.width;
        this.height = b.height;
        this.intrinsicsSource = text(b.intrinsicsSource);
        this.exposureSummary = text(b.exposureSummary);
        this.controlStatus = text(b.controlStatus);
        this.watchdogRestarts = b.watchdogRestarts;
        this.watchdogReason = text(b.watchdogReason);
        this.panelsStreaming = b.panelsStreaming;
        this.driverStationStreaming = b.driverStationStreaming;
        this.settingsSource = text(b.settingsSource);
        this.fieldMapSource = text(b.fieldMapSource);
        this.fault = text(b.fault);
        this.lastProcessorError = text(b.lastProcessorError);
        this.processorErrorCount = b.processorErrorCount;
        this.readNanos = b.readNanos == 0L ? System.nanoTime() : b.readNanos;
    }

    public boolean hasFault() {
        return !fault.isEmpty();
    }

    public boolean isStreaming() {
        return "STREAMING".equals(cameraState);
    }

    public String summaryLine() {
        if (hasFault() && !isStreaming()) {
            return "!! " + fault;
        }
        String age = Double.isNaN(lastFrameAgeMs) || Double.isInfinite(lastFrameAgeMs)
                ? "no frames"
                : String.format(Locale.US, "age %.0f ms", lastFrameAgeMs);
        return String.format(Locale.US, "%s %s  %.0f fps  proc %.1f ms  %s  pipe %d %s  %dx%d",
                cameraState, runState, fps, averageProcessingMs, age, pipelineIndex, pipelineName, width, height);
    }

    @Override
    public String toString() {
        return summaryLine();
    }

    private static String text(String value) {
        return value == null ? "" : value;
    }

    public static final class Builder {
        public String cameraState = "UNKNOWN";
        public String runState = "";
        public String webcamName = "";
        public double fps = 0.0;
        public double portalFps = 0.0;
        public long frameCount = 0L;
        public double lastFrameAgeMs = Double.NaN;
        public double averageProcessingMs = 0.0;
        public int pipelineIndex = 0;
        public String pipelineName = "";
        public PipelineType pipelineType = PipelineType.APRILTAG;
        public int width = 0;
        public int height = 0;
        public String intrinsicsSource = "";
        public String exposureSummary = "";
        public String controlStatus = "";
        public int watchdogRestarts = 0;
        public String watchdogReason = "";
        public boolean panelsStreaming = false;
        public boolean driverStationStreaming = false;
        public String settingsSource = "";
        public String fieldMapSource = "";
        public String fault = "";
        public String lastProcessorError = "";
        public long processorErrorCount = 0L;
        public long readNanos = 0L;

        public Builder() {
        }

        public LogiStatus build() {
            return new LogiStatus(this);
        }
    }
}
