package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.Locale;

public final class CameraControlInfo {

    private static final double NANOS_PER_MILLI = 1e6;

    public static final CameraControlInfo UNKNOWN = new Builder().build();

    public final boolean available;
    public final boolean exposureSupported;
    public final boolean manualExposureSupported;
    public final String exposureMode;
    public final double minExposureMs;
    public final double maxExposureMs;
    public final double currentExposureMs;
    public final boolean gainSupported;
    public final int minGain;
    public final int maxGain;
    public final int currentGain;
    public final boolean whiteBalanceSupported;
    public final String whiteBalanceMode;
    public final int minKelvin;
    public final int maxKelvin;
    public final int currentKelvin;
    public final boolean focusSupported;
    public final String focusMode;
    public final double minFocus;
    public final double maxFocus;
    public final double currentFocus;
    public final String note;
    public final long readNanos;

    private CameraControlInfo(Builder b) {
        this.available = b.available;
        this.exposureSupported = b.exposureSupported;
        this.manualExposureSupported = b.manualExposureSupported;
        this.exposureMode = b.exposureMode == null ? "" : b.exposureMode;
        this.minExposureMs = b.minExposureMs;
        this.maxExposureMs = b.maxExposureMs;
        this.currentExposureMs = b.currentExposureMs;
        this.gainSupported = b.gainSupported;
        this.minGain = b.minGain;
        this.maxGain = b.maxGain;
        this.currentGain = b.currentGain;
        this.whiteBalanceSupported = b.whiteBalanceSupported;
        this.whiteBalanceMode = b.whiteBalanceMode == null ? "" : b.whiteBalanceMode;
        this.minKelvin = b.minKelvin;
        this.maxKelvin = b.maxKelvin;
        this.currentKelvin = b.currentKelvin;
        this.focusSupported = b.focusSupported;
        this.focusMode = b.focusMode == null ? "" : b.focusMode;
        this.minFocus = b.minFocus;
        this.maxFocus = b.maxFocus;
        this.currentFocus = b.currentFocus;
        this.note = b.note == null ? "" : b.note;
        this.readNanos = b.readNanos;
    }

    public double ageMs() {
        if (readNanos == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - readNanos) / NANOS_PER_MILLI;
    }

    public String exposureLine() {
        if (!exposureSupported) {
            return "exposure n/a";
        }
        return String.format(Locale.US, "exp %.2f ms (%.2f..%.2f) %s", currentExposureMs, minExposureMs, maxExposureMs, exposureMode);
    }

    public String gainLine() {
        if (!gainSupported) {
            return "gain n/a";
        }
        return String.format(Locale.US, "gain %d (%d..%d)", currentGain, minGain, maxGain);
    }

    public String whiteBalanceLine() {
        if (!whiteBalanceSupported) {
            return "wb n/a";
        }
        return String.format(Locale.US, "wb %d K (%d..%d) %s", currentKelvin, minKelvin, maxKelvin, whiteBalanceMode);
    }

    public String focusLine() {
        if (!focusSupported) {
            return "focus n/a";
        }
        return String.format(Locale.US, "focus %.0f (%.0f..%.0f) %s", currentFocus, minFocus, maxFocus, focusMode);
    }

    public String summary() {
        if (!available) {
            return note.isEmpty() ? "camera controls not read yet" : note;
        }
        return exposureLine() + "  " + gainLine() + "  " + whiteBalanceLine() + "  " + focusLine();
    }

    @Override
    public String toString() {
        return summary();
    }

    public static final class Builder {
        public boolean available = false;
        public boolean exposureSupported = false;
        public boolean manualExposureSupported = false;
        public String exposureMode = "";
        public double minExposureMs = 0.0;
        public double maxExposureMs = 0.0;
        public double currentExposureMs = 0.0;
        public boolean gainSupported = false;
        public int minGain = 0;
        public int maxGain = 0;
        public int currentGain = 0;
        public boolean whiteBalanceSupported = false;
        public String whiteBalanceMode = "";
        public int minKelvin = 0;
        public int maxKelvin = 0;
        public int currentKelvin = 0;
        public boolean focusSupported = false;
        public String focusMode = "";
        public double minFocus = 0.0;
        public double maxFocus = 0.0;
        public double currentFocus = 0.0;
        public String note = "";
        public long readNanos = 0L;

        public Builder() {
        }

        public CameraControlInfo build() {
            return new CameraControlInfo(this);
        }
    }
}
