package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.Locale;

public final class LogiScriptResult {

    public static final int OUTPUT_COUNT = 32;
    public static final LogiScriptResult EMPTY = new LogiScriptResult(new double[OUTPUT_COUNT]);

    public final double[] outputs;
    public final boolean hasTarget;
    public final double targetX;
    public final double targetY;
    public final double targetWidth;
    public final double targetHeight;
    public final String label;

    public LogiScriptResult(double[] outputs) {
        this.outputs = padded(outputs);
        this.hasTarget = false;
        this.targetX = 0.0;
        this.targetY = 0.0;
        this.targetWidth = 0.0;
        this.targetHeight = 0.0;
        this.label = "";
    }

    public LogiScriptResult(double[] outputs, double targetX, double targetY,
                            double targetWidth, double targetHeight, String label) {
        this.outputs = padded(outputs);
        boolean valid = isFinite(targetX) && isFinite(targetY);
        this.hasTarget = valid;
        this.targetX = valid ? targetX : 0.0;
        this.targetY = valid ? targetY : 0.0;
        this.targetWidth = valid && isFinite(targetWidth) ? Math.abs(targetWidth) : 0.0;
        this.targetHeight = valid && isFinite(targetHeight) ? Math.abs(targetHeight) : 0.0;
        this.label = label == null ? "" : label;
    }

    public double output(int index) {
        if (index < 0 || index >= outputs.length) {
            return 0.0;
        }
        return outputs[index];
    }

    public double[] outputsCopy() {
        return outputs.clone();
    }

    public double targetLeft() {
        return targetX - targetWidth / 2.0;
    }

    public double targetTop() {
        return targetY - targetHeight / 2.0;
    }

    public double targetAreaPixels() {
        return targetWidth * targetHeight;
    }

    public String summary() {
        String target = hasTarget
                ? String.format(Locale.US, "target %s (%.0f, %.0f) %.0fx%.0f", label, targetX, targetY, targetWidth, targetHeight)
                : "no target";
        return String.format(Locale.US, "%s  out %.2f %.2f %.2f %.2f", target,
                outputs[0], outputs[1], outputs[2], outputs[3]);
    }

    @Override
    public String toString() {
        return summary();
    }

    private static double[] padded(double[] values) {
        double[] result = new double[OUTPUT_COUNT];
        if (values == null) {
            return result;
        }
        int length = Math.min(values.length, OUTPUT_COUNT);
        for (int i = 0; i < length; i++) {
            result[i] = isFinite(values[i]) ? values[i] : 0.0;
        }
        return result;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
