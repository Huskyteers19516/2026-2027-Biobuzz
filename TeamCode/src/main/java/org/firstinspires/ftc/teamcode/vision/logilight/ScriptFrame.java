package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.Locale;

public final class ScriptFrame {

    public static final ScriptFrame EMPTY = new ScriptFrame(0L, 0.0, LogiScriptResult.EMPTY, "");

    public final long captureNanos;
    public final double processingMs;
    public final LogiScriptResult result;
    public final String scriptName;
    public final int pipelineIndex;
    public final String error;

    public ScriptFrame(long captureNanos, double processingMs, LogiScriptResult result, String scriptName) {
        this(captureNanos, processingMs, result, scriptName, -1, "");
    }

    public ScriptFrame(long captureNanos, double processingMs, LogiScriptResult result, String scriptName,
                       int pipelineIndex, String error) {
        this.captureNanos = captureNanos;
        this.processingMs = processingMs;
        this.result = result == null ? LogiScriptResult.EMPTY : result;
        this.scriptName = scriptName == null ? "" : scriptName;
        this.pipelineIndex = pipelineIndex;
        this.error = error == null ? "" : error;
    }

    public boolean hasError() {
        return !error.isEmpty();
    }

    public boolean hasTarget() {
        return result.hasTarget;
    }

    public double ageMs() {
        if (captureNanos == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - captureNanos) / 1.0e6;
    }

    @Override
    public String toString() {
        if (hasError()) {
            return String.format(Locale.US, "%s  !! %s", scriptName, error);
        }
        return String.format(Locale.US, "%s  %s  %.1f ms", scriptName, result.summary(), processingMs);
    }
}
