package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.Locale;

public class LogiCameraConfig {

    public String webcamName = "Webcam 1";
    public int width = 640;
    public int height = 480;
    public boolean mjpeg = false;

    public double forwardInches = 0.0;
    public double leftInches = 0.0;
    public double upInches = 10.0;
    public double yawLeftDegrees = 0.0;
    public double tiltUpDegrees = 30.0;
    public double rollClockwiseDegrees = 0.0;

    public double fx = 0.0;
    public double fy = 0.0;
    public double cx = 0.0;
    public double cy = 0.0;

    public boolean manualExposure = true;
    public double exposureMs = 8.0;
    public int gain = 120;
    public boolean autoWhiteBalance = true;
    public int whiteBalanceKelvin = 4600;
    public boolean autoFocus = false;
    public double focusLength = 0.0;

    public double decimation = 2.0;
    public int aprilTagThreads = 3;

    public double crosshairX = 0.0;
    public double crosshairY = 0.0;

    public int streamMaxFps = 10;
    public int streamWidth = 320;
    public double colorProcessScale = 0.5;

    public boolean watchdogEnabled = true;
    public double watchdogTimeoutSeconds = 3.0;
    public int watchdogMaxRestarts = 3;

    public LogiCameraConfig() {
    }

    public LogiCameraConfig copy() {
        LogiCameraConfig copy = new LogiCameraConfig();
        copy.webcamName = webcamName;
        copy.width = width;
        copy.height = height;
        copy.mjpeg = mjpeg;
        copy.forwardInches = forwardInches;
        copy.leftInches = leftInches;
        copy.upInches = upInches;
        copy.yawLeftDegrees = yawLeftDegrees;
        copy.tiltUpDegrees = tiltUpDegrees;
        copy.rollClockwiseDegrees = rollClockwiseDegrees;
        copy.fx = fx;
        copy.fy = fy;
        copy.cx = cx;
        copy.cy = cy;
        copy.manualExposure = manualExposure;
        copy.exposureMs = exposureMs;
        copy.gain = gain;
        copy.autoWhiteBalance = autoWhiteBalance;
        copy.whiteBalanceKelvin = whiteBalanceKelvin;
        copy.autoFocus = autoFocus;
        copy.focusLength = focusLength;
        copy.decimation = decimation;
        copy.aprilTagThreads = aprilTagThreads;
        copy.crosshairX = crosshairX;
        copy.crosshairY = crosshairY;
        copy.streamMaxFps = streamMaxFps;
        copy.streamWidth = streamWidth;
        copy.colorProcessScale = colorProcessScale;
        copy.watchdogEnabled = watchdogEnabled;
        copy.watchdogTimeoutSeconds = watchdogTimeoutSeconds;
        copy.watchdogMaxRestarts = watchdogMaxRestarts;
        return copy;
    }

    public static LogiCameraConfig floorCameraDefaults() {
        LogiCameraConfig config = new LogiCameraConfig();
        config.webcamName = "Webcam 2";
        config.tiltUpDegrees = -30.0;
        config.upInches = 8.0;
        config.forwardInches = 6.0;
        return config;
    }

    public LogiCameraConfig withWebcamName(String name) {
        LogiCameraConfig copy = copy();
        if (name != null && !name.trim().isEmpty()) {
            copy.webcamName = name.trim();
        }
        return copy;
    }

    public boolean hasManualIntrinsics() {
        return isPositiveFinite(fx) && isPositiveFinite(fy) && isPositiveFinite(cx) && isPositiveFinite(cy);
    }

    public String mountSummary() {
        return String.format(Locale.US, "fwd %.1f  left %.1f  up %.1f in  yaw %.1f  tilt %.1f  roll %.1f deg",
                forwardInches, leftInches, upInches, yawLeftDegrees, tiltUpDegrees, rollClockwiseDegrees);
    }

    public String exposureSummary() {
        String exposure = manualExposure
                ? String.format(Locale.US, "exp %.2f ms  gain %d", exposureMs, gain)
                : String.format(Locale.US, "exp auto  gain %d", gain);
        String whiteBalance = autoWhiteBalance ? "wb auto" : String.format(Locale.US, "wb %d K", whiteBalanceKelvin);
        String focus = autoFocus ? "focus auto" : String.format(Locale.US, "focus %.0f", focusLength);
        return exposure + "  " + whiteBalance + "  " + focus;
    }

    private static boolean isPositiveFinite(double value) {
        return value > 0.0 && !Double.isInfinite(value) && !Double.isNaN(value);
    }
}
