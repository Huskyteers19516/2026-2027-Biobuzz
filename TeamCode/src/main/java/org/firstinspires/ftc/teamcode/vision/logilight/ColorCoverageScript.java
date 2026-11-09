package org.firstinspires.ftc.teamcode.vision.logilight;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public final class ColorCoverageScript implements LogiScript {

    public static final String NAME = "color-coverage";
    public static final double DEFAULT_ROI_LEFT = 0.25;
    public static final double DEFAULT_ROI_TOP = 0.5;
    public static final double DEFAULT_ROI_RIGHT = 0.75;
    public static final double DEFAULT_ROI_BOTTOM = 1.0;
    public static final double TARGET_MIN_PERCENT = 5.0;
    public static final int MAX_PROFILE_OUTPUTS = 30;
    public static final int DOMINANT_PROFILE_OUTPUT = 30;
    public static final int ROI_PIXEL_COUNT_OUTPUT = 31;
    public static final int MAX_PROCESS_PIXELS = 40000;

    private static final double FULL_CHANNEL = 255.0;

    private final Scalar lowerBound = new Scalar(0.0, 0.0, 0.0, 0.0);
    private final Scalar upperBound = new Scalar(0.0, 0.0, 0.0, 0.0);
    private Mat scaled;
    private Mat rgb;
    private Mat hsv;
    private Mat ycrcb;
    private Mat mask;
    private Mat wrapMask;
    private int frameWidth;
    private int frameHeight;

    @Override
    public String name() {
        return NAME;
    }

    @Override
    public synchronized void init(int width, int height) {
        frameWidth = width;
        frameHeight = height;
    }

    @Override
    public synchronized LogiScriptResult run(Mat rgba, long captureNanos, double[] inputs) {
        double[] outputs = new double[LogiScriptResult.OUTPUT_COUNT];
        outputs[DOMINANT_PROFILE_OUTPUT] = -1.0;
        if (rgba == null || rgba.empty()) {
            return new LogiScriptResult(outputs);
        }
        int width = rgba.cols();
        int height = rgba.rows();
        frameWidth = width;
        frameHeight = height;
        Rect roi = roiRect(inputs, width, height);
        double roiPixels = (double) roi.width * (double) roi.height;
        outputs[ROI_PIXEL_COUNT_OUTPUT] = roiPixels;

        Mat region = rgba.submat(roi);
        try {
            Mat source = region;
            double scale = Math.min(1.0, Math.sqrt(MAX_PROCESS_PIXELS / Math.max(1.0, roiPixels)));
            if (scale < 1.0) {
                int scaledWidth = Math.max(1, (int) Math.round(roi.width * scale));
                int scaledHeight = Math.max(1, (int) Math.round(roi.height * scale));
                if (scaled == null) {
                    scaled = new Mat();
                }
                Imgproc.resize(region, scaled, new Size(scaledWidth, scaledHeight), 0.0, 0.0, Imgproc.INTER_AREA);
                source = scaled;
            }
            ensureMats();
            boolean hsvReady = false;
            boolean ycrcbReady = false;
            boolean rgbReady = false;
            double total = (double) source.rows() * (double) source.cols();
            ColorProfile[] profiles = LogiLightSettings.COLOR_PROFILES;
            int profileCount = profiles == null ? 0 : Math.min(profiles.length, MAX_PROFILE_OUTPUTS);
            int dominant = -1;
            double dominantPercent = 0.0;
            for (int i = 0; i < profileCount; i++) {
                ColorProfile profile = profiles[i];
                if (profile == null || !profile.enabled || total <= 0.0) {
                    continue;
                }
                ColorSpaceMode space = profile.colorSpace == null ? ColorSpaceMode.HSV : profile.colorSpace;
                Mat image;
                if (space == ColorSpaceMode.HSV) {
                    if (!hsvReady) {
                        Imgproc.cvtColor(source, hsv, Imgproc.COLOR_RGB2HSV);
                        hsvReady = true;
                    }
                    image = hsv;
                } else if (space == ColorSpaceMode.YCRCB) {
                    if (!ycrcbReady) {
                        Imgproc.cvtColor(source, ycrcb, Imgproc.COLOR_RGB2YCrCb);
                        ycrcbReady = true;
                    }
                    image = ycrcb;
                } else {
                    if (!rgbReady) {
                        if (source.channels() == 4) {
                            Imgproc.cvtColor(source, rgb, Imgproc.COLOR_RGBA2RGB);
                        } else {
                            source.copyTo(rgb);
                        }
                        rgbReady = true;
                    }
                    image = rgb;
                }
                threshold(image, profile, space);
                double percent = Core.countNonZero(mask) / total * 100.0;
                outputs[i] = percent;
                if (percent > dominantPercent) {
                    dominantPercent = percent;
                    dominant = i;
                }
            }
            outputs[DOMINANT_PROFILE_OUTPUT] = dominant;
            if (dominant >= 0 && dominantPercent > TARGET_MIN_PERCENT) {
                String label = profiles[dominant].name == null ? "" : profiles[dominant].name;
                return new LogiScriptResult(outputs,
                        roi.x + roi.width / 2.0, roi.y + roi.height / 2.0,
                        roi.width, roi.height, label);
            }
            return new LogiScriptResult(outputs);
        } finally {
            region.release();
        }
    }

    public synchronized int frameWidth() {
        return frameWidth;
    }

    public synchronized int frameHeight() {
        return frameHeight;
    }

    public static Rect roiRect(double[] inputs, int width, int height) {
        double left = DEFAULT_ROI_LEFT;
        double top = DEFAULT_ROI_TOP;
        double right = DEFAULT_ROI_RIGHT;
        double bottom = DEFAULT_ROI_BOTTOM;
        if (inputs != null && inputs.length >= 4) {
            boolean allZero = true;
            for (int i = 0; i < 4; i++) {
                if (inputs[i] != 0.0) {
                    allZero = false;
                    break;
                }
            }
            if (!allZero) {
                left = unit(inputs[0], DEFAULT_ROI_LEFT);
                top = unit(inputs[1], DEFAULT_ROI_TOP);
                right = unit(inputs[2], DEFAULT_ROI_RIGHT);
                bottom = unit(inputs[3], DEFAULT_ROI_BOTTOM);
            }
        }
        if (right < left) {
            double swap = left;
            left = right;
            right = swap;
        }
        if (bottom < top) {
            double swap = top;
            top = bottom;
            bottom = swap;
        }
        int safeWidth = Math.max(1, width);
        int safeHeight = Math.max(1, height);
        int x0 = clamp((int) Math.floor(left * safeWidth), 0, safeWidth - 1);
        int y0 = clamp((int) Math.floor(top * safeHeight), 0, safeHeight - 1);
        int x1 = clamp((int) Math.ceil(right * safeWidth), x0 + 1, safeWidth);
        int y1 = clamp((int) Math.ceil(bottom * safeHeight), y0 + 1, safeHeight);
        return new Rect(x0, y0, x1 - x0, y1 - y0);
    }

    private void threshold(Mat image, ColorProfile profile, ColorSpaceMode space) {
        double min0 = finiteOr(profile.min0, 0.0);
        double min1 = finiteOr(profile.min1, 0.0);
        double min2 = finiteOr(profile.min2, 0.0);
        double max0 = finiteOr(profile.max0, FULL_CHANNEL);
        double max1 = finiteOr(profile.max1, FULL_CHANNEL);
        double max2 = finiteOr(profile.max2, FULL_CHANNEL);
        if (space == ColorSpaceMode.HSV && min0 > max0) {
            setBounds(min0, min1, min2, ColorSpaceMode.HUE_MAX, max1, max2);
            Core.inRange(image, lowerBound, upperBound, mask);
            setBounds(0.0, min1, min2, max0, max1, max2);
            Core.inRange(image, lowerBound, upperBound, wrapMask);
            Core.bitwise_or(mask, wrapMask, mask);
        } else {
            setBounds(min0, min1, min2, max0, max1, max2);
            Core.inRange(image, lowerBound, upperBound, mask);
        }
    }

    private void setBounds(double min0, double min1, double min2, double max0, double max1, double max2) {
        lowerBound.val[0] = min0;
        lowerBound.val[1] = min1;
        lowerBound.val[2] = min2;
        lowerBound.val[3] = 0.0;
        upperBound.val[0] = max0;
        upperBound.val[1] = max1;
        upperBound.val[2] = max2;
        upperBound.val[3] = FULL_CHANNEL;
    }

    private void ensureMats() {
        if (rgb == null) {
            rgb = new Mat();
        }
        if (hsv == null) {
            hsv = new Mat();
        }
        if (ycrcb == null) {
            ycrcb = new Mat();
        }
        if (mask == null) {
            mask = new Mat();
        }
        if (wrapMask == null) {
            wrapMask = new Mat();
        }
    }

    private static double unit(double value, double fallback) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return fallback;
        }
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double finiteOr(double value, double fallback) {
        return Double.isNaN(value) || Double.isInfinite(value) ? fallback : value;
    }
}
