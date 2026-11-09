package org.firstinspires.ftc.teamcode.vision.logilight;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.Locale;

public final class ColorSample {

    public static final int MAX_RADIUS_PX = 40;
    public static final double LOW_PERCENTILE = 0.10;
    public static final double HIGH_PERCENTILE = 0.90;

    private static final double FULL_CHANNEL = 255.0;
    private static final double HUE_RANGE = ColorSpaceMode.HUE_MAX;
    private static final double HALF_HUE_RANGE = HUE_RANGE / 2.0;

    public final ColorSpaceMode colorSpace;
    public final double centerX;
    public final double centerY;
    public final int radiusPx;
    public final int pixelCount;
    public final double median0;
    public final double median1;
    public final double median2;
    public final double low0;
    public final double low1;
    public final double low2;
    public final double high0;
    public final double high1;
    public final double high2;
    public final long captureNanos;

    public ColorSample(ColorSpaceMode colorSpace, double centerX, double centerY, int radiusPx, int pixelCount,
                       double median0, double median1, double median2,
                       double low0, double low1, double low2,
                       double high0, double high1, double high2,
                       long captureNanos) {
        this.colorSpace = colorSpace == null ? ColorSpaceMode.HSV : colorSpace;
        this.centerX = centerX;
        this.centerY = centerY;
        this.radiusPx = radiusPx;
        this.pixelCount = pixelCount;
        this.median0 = median0;
        this.median1 = median1;
        this.median2 = median2;
        this.low0 = low0;
        this.low1 = low1;
        this.low2 = low2;
        this.high0 = high0;
        this.high1 = high1;
        this.high2 = high2;
        this.captureNanos = captureNanos;
    }

    public boolean isValid() {
        return pixelCount > 0;
    }

    public double median(int channel) {
        return channel <= 0 ? median0 : channel == 1 ? median1 : median2;
    }

    public double low(int channel) {
        return channel <= 0 ? low0 : channel == 1 ? low1 : low2;
    }

    public double high(int channel) {
        return channel <= 0 ? high0 : channel == 1 ? high1 : high2;
    }

    public double ageMs() {
        return captureNanos == 0L ? Double.POSITIVE_INFINITY : (System.nanoTime() - captureNanos) / 1.0e6;
    }

    public void applyRange(ColorProfile profile, double margin0, double margin1, double margin2) {
        if (profile == null || !isValid()) {
            return;
        }
        profile.colorSpace = colorSpace;
        if (colorSpace == ColorSpaceMode.HSV) {
            applyHue(profile, median0 - Math.abs(margin0), median0 + Math.abs(margin0));
        } else {
            profile.min0 = clampChannel(median0 - Math.abs(margin0));
            profile.max0 = clampChannel(median0 + Math.abs(margin0));
        }
        profile.min1 = clampChannel(median1 - Math.abs(margin1));
        profile.max1 = clampChannel(median1 + Math.abs(margin1));
        profile.min2 = clampChannel(median2 - Math.abs(margin2));
        profile.max2 = clampChannel(median2 + Math.abs(margin2));
    }

    public void applyPercentileRange(ColorProfile profile, double padding0, double padding1, double padding2) {
        if (profile == null || !isValid()) {
            return;
        }
        profile.colorSpace = colorSpace;
        if (colorSpace == ColorSpaceMode.HSV) {
            double spread = hueOffset(high0, median0) - hueOffset(low0, median0);
            double lowEdge = median0 + hueOffset(low0, median0) - Math.abs(padding0);
            applyHue(profile, lowEdge, lowEdge + Math.max(0.0, spread) + 2.0 * Math.abs(padding0));
        } else {
            profile.min0 = clampChannel(low0 - Math.abs(padding0));
            profile.max0 = clampChannel(high0 + Math.abs(padding0));
        }
        profile.min1 = clampChannel(low1 - Math.abs(padding1));
        profile.max1 = clampChannel(high1 + Math.abs(padding1));
        profile.min2 = clampChannel(low2 - Math.abs(padding2));
        profile.max2 = clampChannel(high2 + Math.abs(padding2));
    }

    public String summary() {
        if (!isValid()) {
            return "no sample";
        }
        return String.format(Locale.US, "%s %s %.0f (%.0f-%.0f)  %s %.0f (%.0f-%.0f)  %s %.0f (%.0f-%.0f)  n %d",
                colorSpace.name(),
                colorSpace.channelName(0), median0, low0, high0,
                colorSpace.channelName(1), median1, low1, high1,
                colorSpace.channelName(2), median2, low2, high2,
                pixelCount);
    }

    @Override
    public String toString() {
        return summary();
    }

    public static ColorSample measure(Mat rgba, ColorSpaceMode space, double centerX, double centerY,
                                      int radiusPx, long captureNanos, Mat scratch) {
        ColorSpaceMode mode = space == null ? ColorSpaceMode.HSV : space;
        int radius = Math.max(1, Math.min(MAX_RADIUS_PX, radiusPx));
        if (rgba == null || rgba.empty()) {
            return empty(mode, centerX, centerY, radius, captureNanos);
        }
        int width = rgba.cols();
        int height = rgba.rows();
        double u = Double.isNaN(centerX) || Double.isInfinite(centerX) ? width / 2.0 : centerX;
        double v = Double.isNaN(centerY) || Double.isInfinite(centerY) ? height / 2.0 : centerY;
        int x0 = clamp((int) Math.floor(u - radius), 0, width - 1);
        int y0 = clamp((int) Math.floor(v - radius), 0, height - 1);
        int x1 = clamp((int) Math.ceil(u + radius), x0 + 1, width);
        int y1 = clamp((int) Math.ceil(v + radius), y0 + 1, height);
        Rect rect = new Rect(x0, y0, x1 - x0, y1 - y0);
        Mat target = scratch == null ? new Mat() : scratch;
        Mat region = rgba.submat(rect);
        try {
            int channels = region.channels();
            if (mode == ColorSpaceMode.HSV) {
                Imgproc.cvtColor(region, target, channels == 1 ? Imgproc.COLOR_GRAY2RGB : Imgproc.COLOR_RGB2HSV);
                if (channels == 1) {
                    Imgproc.cvtColor(target, target, Imgproc.COLOR_RGB2HSV);
                }
            } else if (mode == ColorSpaceMode.YCRCB) {
                Imgproc.cvtColor(region, target, channels == 1 ? Imgproc.COLOR_GRAY2RGB : Imgproc.COLOR_RGB2YCrCb);
                if (channels == 1) {
                    Imgproc.cvtColor(target, target, Imgproc.COLOR_RGB2YCrCb);
                }
            } else if (channels == 4) {
                Imgproc.cvtColor(region, target, Imgproc.COLOR_RGBA2RGB);
            } else if (channels == 1) {
                Imgproc.cvtColor(region, target, Imgproc.COLOR_GRAY2RGB);
            } else {
                region.copyTo(target);
            }
            int count = target.rows() * target.cols();
            if (count <= 0 || target.channels() != 3) {
                return empty(mode, u, v, radius, captureNanos);
            }
            byte[] pixels = new byte[count * 3];
            target.get(0, 0, pixels);
            double[] channel0 = new double[count];
            double[] channel1 = new double[count];
            double[] channel2 = new double[count];
            for (int i = 0; i < count; i++) {
                channel0[i] = pixels[3 * i] & 0xFF;
                channel1[i] = pixels[3 * i + 1] & 0xFF;
                channel2[i] = pixels[3 * i + 2] & 0xFF;
            }
            double median0;
            double low0;
            double high0;
            if (mode == ColorSpaceMode.HSV) {
                double[] hue = circularStats(channel0);
                median0 = hue[0];
                low0 = hue[1];
                high0 = hue[2];
            } else {
                Arrays.sort(channel0);
                median0 = percentile(channel0, 0.5);
                low0 = percentile(channel0, LOW_PERCENTILE);
                high0 = percentile(channel0, HIGH_PERCENTILE);
            }
            Arrays.sort(channel1);
            Arrays.sort(channel2);
            return new ColorSample(mode, u, v, radius, count,
                    median0, percentile(channel1, 0.5), percentile(channel2, 0.5),
                    low0, percentile(channel1, LOW_PERCENTILE), percentile(channel2, LOW_PERCENTILE),
                    high0, percentile(channel1, HIGH_PERCENTILE), percentile(channel2, HIGH_PERCENTILE),
                    captureNanos);
        } finally {
            region.release();
            if (scratch == null) {
                target.release();
            }
        }
    }

    private static ColorSample empty(ColorSpaceMode mode, double u, double v, int radius, long captureNanos) {
        return new ColorSample(mode, u, v, radius, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, captureNanos);
    }

    private static double[] circularStats(double[] hues) {
        double sumCos = 0.0;
        double sumSin = 0.0;
        for (double hue : hues) {
            double angle = hue / HUE_RANGE * 2.0 * Math.PI;
            sumCos += Math.cos(angle);
            sumSin += Math.sin(angle);
        }
        double mean = wrapHue(Math.atan2(sumSin, sumCos) / (2.0 * Math.PI) * HUE_RANGE);
        double[] offsets = new double[hues.length];
        for (int i = 0; i < hues.length; i++) {
            offsets[i] = hueOffset(hues[i], mean);
        }
        Arrays.sort(offsets);
        return new double[]{
                wrapHue(mean + percentile(offsets, 0.5)),
                wrapHue(mean + percentile(offsets, LOW_PERCENTILE)),
                wrapHue(mean + percentile(offsets, HIGH_PERCENTILE))};
    }

    private static double hueOffset(double hue, double reference) {
        double offset = (hue - reference) % HUE_RANGE;
        if (offset < -HALF_HUE_RANGE) {
            offset += HUE_RANGE;
        } else if (offset >= HALF_HUE_RANGE) {
            offset -= HUE_RANGE;
        }
        return offset;
    }

    private static void applyHue(ColorProfile profile, double low, double high) {
        if (high - low >= HUE_RANGE) {
            profile.min0 = 0.0;
            profile.max0 = HUE_RANGE;
            return;
        }
        if (low < 0.0) {
            profile.min0 = low + HUE_RANGE;
            profile.max0 = high;
        } else if (high >= HUE_RANGE) {
            profile.min0 = low;
            profile.max0 = high - HUE_RANGE;
        } else {
            profile.min0 = low;
            profile.max0 = high;
        }
    }

    private static double wrapHue(double hue) {
        double wrapped = hue % HUE_RANGE;
        return wrapped < 0.0 ? wrapped + HUE_RANGE : wrapped;
    }

    private static double percentile(double[] sorted, double fraction) {
        if (sorted.length == 0) {
            return 0.0;
        }
        double position = Math.max(0.0, Math.min(1.0, fraction)) * (sorted.length - 1);
        int lower = (int) Math.floor(position);
        int upper = Math.min(sorted.length - 1, lower + 1);
        double weight = position - lower;
        return sorted[lower] + (sorted[upper] - sorted[lower]) * weight;
    }

    private static double clampChannel(double value) {
        if (Double.isNaN(value)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(FULL_CHANNEL, value));
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
