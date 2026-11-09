package org.firstinspires.ftc.teamcode.vision.logilight;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public final class LogiFrameContext {

    public static final String INTRINSICS_CONFIG = "config";
    public static final String INTRINSICS_SDK_CALIBRATION = "SDK calibration";
    public static final String INTRINSICS_FALLBACK = "fallback (uncalibrated)";
    public static final String INTRINSICS_EDITED_WARNING = "  !! fx/fy/cx/cy changed - restart the OpMode to apply";
    public static final String FALLBACK_WIDTH_WARNING = "  !! SDK fallback f is for 640 wide - range/tx wrong";
    public static final String LOG_TAG = "LogiLight";
    public static final long RECENT_ERROR_MS = 5000L;
    public static final int MAX_SCRIPT_INPUTS = 64;

    private static final double STATS_SMOOTHING = 0.1;
    private static final long MAX_FRAME_INTERVAL_NANOS = 2000000000L;
    private static final long ERROR_LOG_INTERVAL_NANOS = 1000000000L;
    private static final double NANOS_PER_MS = 1.0e6;
    private static final double NANOS_PER_SECOND = 1.0e9;
    private static final int CONFIG_VALUE_COUNT = 12;

    public volatile StreamView streamView;

    private final LogiCameraConfig camera;
    private final double[] lockedIntrinsics;
    private final Object modelLock = new Object();
    private final Object statsLock = new Object();
    private final Object errorLock = new Object();

    private volatile CameraModel cameraModel;
    private volatile String intrinsicsSource = INTRINSICS_FALLBACK;
    private volatile String intrinsicsDetail = "";
    private volatile int frameWidth;
    private volatile int frameHeight;
    private volatile boolean initialized;
    private volatile double[] builtConfigValues = new double[0];
    private CameraCalibration calibration;

    private final AtomicReference<ActiveState> active = new AtomicReference<ActiveState>();
    private volatile ColorProfile[] colorProfileOverride;
    private volatile double[] scriptInputs = new double[0];

    private volatile long lastCaptureNanos;
    private volatile long frameStartNanos;
    private volatile long frameCount;
    private volatile long lastProcessedAtNanos;
    private volatile long lastProcessedCaptureNanos;
    private volatile long processedCount;
    private volatile double averageLatencyMs;
    private volatile double lastLatencyMs;
    private volatile double frameIntervalNanos;

    private volatile String lastError = "";
    private volatile long lastErrorNanos;
    private final AtomicLong errorCount = new AtomicLong();
    private long lastLoggedNanos;
    private String lastLoggedMessage = "";

    private volatile StreamView lastSettingsStreamView;
    private final AtomicReference<ColorSampleRequest> colorSampleRequest = new AtomicReference<ColorSampleRequest>();
    private volatile ColorSample latestColorSample;

    public LogiFrameContext(LogiCameraConfig camera) {
        this(camera, null);
    }

    public LogiFrameContext(LogiCameraConfig camera, LogiCameraConfig build) {
        this.camera = camera == null ? new LogiCameraConfig() : camera;
        this.lockedIntrinsics = lockIntrinsics(build != null ? build : this.camera);
        StreamView initialView = LogiLightSettings.STREAM_VIEW;
        this.streamView = initialView == null ? StreamView.ANNOTATED : initialView;
        this.lastSettingsStreamView = initialView;
        try {
            int index = LogiLightSettings.defaultPipelineIndex();
            LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
            setActivePipeline(index, pipeline, null, null);
        } catch (Throwable t) {
            active.set(new ActiveState(0, null, new ColorProfile[0], null, null, 0L, System.nanoTime()));
        }
    }

    public LogiCameraConfig camera() {
        return camera;
    }

    public void onInit(int width, int height, CameraCalibration calibration) {
        synchronized (modelLock) {
            int resolvedWidth = width > 0 ? width : (camera.width > 0 ? camera.width : CameraModel.DEFAULT_WIDTH);
            int resolvedHeight = height > 0 ? height : (camera.height > 0 ? camera.height : CameraModel.DEFAULT_HEIGHT);
            boolean sameSize = initialized && resolvedWidth == frameWidth && resolvedHeight == frameHeight;
            boolean usable = isUsableCalibration(calibration, resolvedWidth, resolvedHeight);
            if (sameSize && (!usable || this.calibration != null)) {
                return;
            }
            if (usable) {
                this.calibration = calibration;
            } else if (!sameSize) {
                this.calibration = null;
            }
            frameWidth = resolvedWidth;
            frameHeight = resolvedHeight;
            initialized = true;
            rebuildModelLocked();
        }
    }

    public boolean isInitialized() {
        return initialized;
    }

    public CameraModel cameraModel() {
        return cameraModel;
    }

    public CameraModel cameraModelOrFallback() {
        CameraModel model = cameraModel;
        if (model != null) {
            return model;
        }
        int width = camera.width > 0 ? camera.width : CameraModel.DEFAULT_WIDTH;
        int height = camera.height > 0 ? camera.height : CameraModel.DEFAULT_HEIGHT;
        double[] locked = lockedIntrinsics;
        double[] intrinsics = locked != null
                ? locked.clone()
                : CameraModel.fallbackIntrinsics(width, height);
        return buildModel(width, height, intrinsics, camera);
    }

    public boolean hasLockedIntrinsics() {
        return lockedIntrinsics != null;
    }

    public double[] lockedIntrinsics() {
        double[] locked = lockedIntrinsics;
        return locked == null ? null : locked.clone();
    }

    public boolean intrinsicsEdited() {
        LogiCameraConfig config = camera;
        double[] locked = lockedIntrinsics;
        boolean manual = config.hasManualIntrinsics();
        if (locked == null) {
            return manual;
        }
        return !manual
                || Double.compare(config.fx, locked[0]) != 0
                || Double.compare(config.fy, locked[1]) != 0
                || Double.compare(config.cx, locked[2]) != 0
                || Double.compare(config.cy, locked[3]) != 0;
    }

    private static double[] lockIntrinsics(LogiCameraConfig lens) {
        try {
            if (lens != null && lens.hasManualIntrinsics()) {
                return new double[]{lens.fx, lens.fy, lens.cx, lens.cy};
            }
        } catch (Throwable ignored) {
        }
        return null;
    }

    CameraModel ensureModel(int width, int height) {
        CameraModel model = cameraModel;
        if (model == null || !initialized) {
            onInit(width, height, null);
            model = cameraModel;
        }
        return model;
    }

    public void setCrosshair(double x, double y) {
        synchronized (modelLock) {
            camera.crosshairX = clampUnit(x);
            camera.crosshairY = clampUnit(y);
            if (initialized) {
                rebuildModelLocked();
            }
        }
    }

    public void refreshMountFromConfig() {
        synchronized (modelLock) {
            if (initialized) {
                rebuildModelLocked();
            }
        }
    }

    public String intrinsicsSource() {
        return intrinsicsSource;
    }

    public String intrinsicsDetail() {
        return intrinsicsDetail;
    }

    public int frameWidth() {
        return frameWidth;
    }

    public int frameHeight() {
        return frameHeight;
    }

    public void setActivePipeline(int index, LogiPipeline pipeline, ColorProfile[] profiles, LogiScript script) {
        ColorProfile[] resolved = profiles == null
                ? LogiLightSettings.resolveColorProfiles(pipeline)
                : profiles.clone();
        String profileNames = pipeline == null ? null : pipeline.colorProfiles;
        active.set(new ActiveState(index, pipeline, resolved, script, profileNames,
                LogiLightSettings.revision(), System.nanoTime()));
    }

    public int activePipelineIndex() {
        ActiveState state = active.get();
        return state == null ? 0 : state.index;
    }

    public LogiPipeline activePipeline() {
        ActiveState state = active.get();
        return state == null ? null : state.pipeline;
    }

    public PipelineType activePipelineType() {
        LogiPipeline pipeline = activePipeline();
        return pipeline == null ? PipelineType.DRIVER : pipeline.typeOrDefault();
    }

    public String activePipelineName() {
        LogiPipeline pipeline = activePipeline();
        if (pipeline == null || pipeline.name == null) {
            return "";
        }
        return pipeline.name;
    }

    public long activePipelineSetNanos() {
        ActiveState state = active.get();
        return state == null ? 0L : state.setNanos;
    }

    public ColorProfile[] activeColorProfiles() {
        return profilesForFrame().clone();
    }

    public LogiScript activeScript() {
        ActiveState state = active.get();
        return state == null ? null : state.script;
    }

    public void setColorProfileOverride(ColorProfile... profiles) {
        colorProfileOverride = profiles == null ? null : profiles.clone();
    }

    public void clearColorProfileOverride() {
        colorProfileOverride = null;
    }

    public ColorProfile[] colorProfileOverride() {
        ColorProfile[] override = colorProfileOverride;
        return override == null ? null : override.clone();
    }

    ColorProfile[] profilesForFrame() {
        ColorProfile[] override = colorProfileOverride;
        if (override != null) {
            return override;
        }
        ActiveState state = active.get();
        if (state == null) {
            return new ColorProfile[0];
        }
        LogiPipeline pipeline = state.pipeline;
        if (pipeline != null) {
            String names = pipeline.colorProfiles;
            long revision = LogiLightSettings.revision();
            if (revision != state.revision || !sameText(names, state.profileNames)) {
                ColorProfile[] resolved = LogiLightSettings.resolveColorProfiles(pipeline);
                active.compareAndSet(state, state.withProfiles(resolved, names, revision));
                return resolved;
            }
        }
        return state.profiles;
    }

    public void setScriptInputs(double[] inputs) {
        if (inputs == null) {
            scriptInputs = new double[0];
            return;
        }
        int length = Math.min(inputs.length, MAX_SCRIPT_INPUTS);
        double[] copy = new double[length];
        System.arraycopy(inputs, 0, copy, 0, length);
        scriptInputs = copy;
    }

    public double[] scriptInputs() {
        return scriptInputs.clone();
    }

    public void markFrame(long captureNanos) {
        if (captureNanos == lastCaptureNanos && frameCount > 0) {
            return;
        }
        boolean newFrame = false;
        synchronized (statsLock) {
            if (captureNanos != lastCaptureNanos || frameCount == 0) {
                lastCaptureNanos = captureNanos;
                frameStartNanos = System.nanoTime();
                frameCount = frameCount + 1;
                newFrame = true;
            }
        }
        if (newFrame) {
            refreshModelIfConfigChanged();
        }
    }

    public void markProcessingDone(long captureNanos) {
        long now = System.nanoTime();
        synchronized (statsLock) {
            if (processedCount > 0 && captureNanos == lastProcessedCaptureNanos) {
                return;
            }
            long reference = captureNanos > 0L && captureNanos <= now ? captureNanos : frameStartNanos;
            double latency = reference > 0L && reference <= now ? (now - reference) / NANOS_PER_MS : 0.0;
            lastLatencyMs = latency;
            averageLatencyMs = processedCount == 0 ? latency : averageLatencyMs + STATS_SMOOTHING * (latency - averageLatencyMs);
            long previous = lastProcessedAtNanos;
            if (previous > 0L) {
                long interval = now - previous;
                if (interval > 0L && interval < MAX_FRAME_INTERVAL_NANOS) {
                    frameIntervalNanos = frameIntervalNanos <= 0.0
                            ? interval
                            : frameIntervalNanos + STATS_SMOOTHING * (interval - frameIntervalNanos);
                }
            }
            lastProcessedAtNanos = now;
            lastProcessedCaptureNanos = captureNanos;
            processedCount = processedCount + 1;
        }
    }

    public long lastCaptureNanos() {
        return lastCaptureNanos;
    }

    public long lastProcessedAtNanos() {
        return lastProcessedAtNanos;
    }

    public long lastProcessedCaptureNanos() {
        return lastProcessedCaptureNanos;
    }

    public long frameCount() {
        return frameCount;
    }

    public long processedFrameCount() {
        return processedCount;
    }

    public double averageProcessingMs() {
        return averageLatencyMs;
    }

    public double lastProcessingMs() {
        return lastLatencyMs;
    }

    public double measuredFps() {
        long last = lastProcessedAtNanos;
        double interval = frameIntervalNanos;
        if (last <= 0L || interval <= 0.0) {
            return 0.0;
        }
        double sinceLast = System.nanoTime() - last;
        double effective = Math.max(interval, sinceLast);
        return effective > 0.0 ? NANOS_PER_SECOND / effective : 0.0;
    }

    public double frameAgeMs() {
        long last = lastProcessedAtNanos;
        if (last <= 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - last) / NANOS_PER_MS;
    }

    public void reportError(String where, Throwable t) {
        String message = describe(where, t);
        recordError(message, t);
    }

    public void reportError(String where, String message) {
        String text = (where == null || where.isEmpty() ? "" : where + ": ") + (message == null ? "error" : message);
        recordError(text, null);
    }

    public String lastError() {
        return lastError;
    }

    public long errorCount() {
        return errorCount.get();
    }

    public double lastErrorAgeMs() {
        long at = lastErrorNanos;
        if (at == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - at) / NANOS_PER_MS;
    }

    public boolean hasRecentError() {
        return !lastError.isEmpty() && lastErrorAgeMs() <= RECENT_ERROR_MS;
    }

    public void clearLastError() {
        lastError = "";
        lastErrorNanos = 0L;
    }

    public void setStreamView(StreamView view) {
        streamView = view == null ? StreamView.ANNOTATED : view;
    }

    public StreamView getStreamView() {
        StreamView view = streamView;
        return view == null ? StreamView.ANNOTATED : view;
    }

    public StreamView streamView() {
        return getStreamView();
    }

    public StreamView effectiveStreamView() {
        StreamView settingsView = LogiLightSettings.STREAM_VIEW;
        if (settingsView != null && settingsView != lastSettingsStreamView) {
            lastSettingsStreamView = settingsView;
            streamView = settingsView;
        }
        return getStreamView();
    }

    public void requestColorSample(ColorSpaceMode space, int radiusPx) {
        requestColorSample(space, Double.NaN, Double.NaN, radiusPx);
    }

    public void requestColorSample(ColorSpaceMode space, double u, double v, int radiusPx) {
        ColorSpaceMode resolved = space == null ? ColorSpaceMode.HSV : space;
        int radius = Math.max(1, Math.min(ColorSample.MAX_RADIUS_PX, radiusPx));
        colorSampleRequest.set(new ColorSampleRequest(resolved, u, v, radius, System.nanoTime()));
    }

    public boolean isColorSamplePending() {
        return colorSampleRequest.get() != null;
    }

    public ColorSample latestColorSample() {
        return latestColorSample;
    }

    public void clearColorSample() {
        latestColorSample = null;
    }

    ColorSampleRequest takeColorSampleRequest() {
        return colorSampleRequest.getAndSet(null);
    }

    void publishColorSample(ColorSample sample) {
        if (sample != null) {
            latestColorSample = sample;
        }
    }

    public String statsSummary() {
        return String.format(Locale.US, "fps %.1f  lat %.1f ms  frames %d  errors %d",
                measuredFps(), averageProcessingMs(), frameCount(), errorCount());
    }

    private void refreshModelIfConfigChanged() {
        if (!initialized) {
            return;
        }
        if (!configDiffers()) {
            return;
        }
        synchronized (modelLock) {
            if (initialized && configDiffers()) {
                rebuildModelLocked();
            }
        }
    }

    private boolean configDiffers() {
        double[] built = builtConfigValues;
        double[] current = configValues(camera);
        if (built.length != current.length) {
            return true;
        }
        for (int i = 0; i < current.length; i++) {
            if (Double.doubleToLongBits(built[i]) != Double.doubleToLongBits(current[i])) {
                return true;
            }
        }
        return false;
    }

    private static double[] configValues(LogiCameraConfig config) {
        double[] values = new double[CONFIG_VALUE_COUNT];
        values[0] = config.forwardInches;
        values[1] = config.leftInches;
        values[2] = config.upInches;
        values[3] = config.yawLeftDegrees;
        values[4] = config.tiltUpDegrees;
        values[5] = config.rollClockwiseDegrees;
        values[6] = config.crosshairX;
        values[7] = config.crosshairY;
        values[8] = config.fx;
        values[9] = config.fy;
        values[10] = config.cx;
        values[11] = config.cy;
        return values;
    }

    private void rebuildModelLocked() {
        LogiCameraConfig config = camera;
        double[] snapshot = configValues(config);
        int width = frameWidth > 0 ? frameWidth : CameraModel.DEFAULT_WIDTH;
        int height = frameHeight > 0 ? frameHeight : CameraModel.DEFAULT_HEIGHT;
        double[] intrinsics;
        String source;
        String detail;
        CameraCalibration cal = calibration;
        double[] locked = lockedIntrinsics;
        if (locked != null) {
            intrinsics = locked.clone();
            source = INTRINSICS_CONFIG;
            detail = String.format(Locale.US, "config  fx %.1f  fy %.1f  cx %.1f  cy %.1f",
                    intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
        } else if (cal != null) {
            intrinsics = new double[]{cal.focalLengthX, cal.focalLengthY, cal.principalPointX, cal.principalPointY};
            source = INTRINSICS_SDK_CALIBRATION;
            String scaled = "";
            if (cal.resolutionScaledFrom != null) {
                scaled = String.format(Locale.US, "  scaled from %dx%d",
                        cal.resolutionScaledFrom.getWidth(), cal.resolutionScaledFrom.getHeight());
            }
            detail = String.format(Locale.US, "SDK calibration  fx %.1f  fy %.1f  cx %.1f  cy %.1f%s",
                    intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3], scaled);
        } else {
            intrinsics = CameraModel.fallbackIntrinsics(width, height);
            source = INTRINSICS_FALLBACK;
            detail = String.format(Locale.US, "!! uncalibrated fallback  f %.1f  cx %.1f  cy %.1f",
                    intrinsics[0], intrinsics[2], intrinsics[3]);
            if (width != CameraModel.DEFAULT_WIDTH) {
                detail = detail + FALLBACK_WIDTH_WARNING;
            }
        }
        if (intrinsicsEdited()) {
            source = source + INTRINSICS_EDITED_WARNING;
            detail = detail + INTRINSICS_EDITED_WARNING;
        }
        cameraModel = buildModel(width, height, intrinsics, config);
        intrinsicsSource = source;
        intrinsicsDetail = detail;
        builtConfigValues = snapshot;
    }

    private static CameraModel buildModel(int width, int height, double[] intrinsics, LogiCameraConfig config) {
        return new CameraModel(width, height,
                intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
                config.forwardInches, config.leftInches, config.upInches,
                config.yawLeftDegrees, config.tiltUpDegrees, config.rollClockwiseDegrees,
                config.crosshairX, config.crosshairY);
    }

    private static boolean isUsableCalibration(CameraCalibration calibration, int width, int height) {
        if (calibration == null) {
            return false;
        }
        try {
            if (calibration.isFake() || calibration.isDegenerate()) {
                return false;
            }
            double fx = calibration.focalLengthX;
            double fy = calibration.focalLengthY;
            double cx = calibration.principalPointX;
            double cy = calibration.principalPointY;
            if (!(fx > 0.0) || !(fy > 0.0) || Double.isInfinite(fx) || Double.isInfinite(fy)) {
                return false;
            }
            if (Double.isNaN(cx) || Double.isNaN(cy) || Double.isInfinite(cx) || Double.isInfinite(cy)) {
                return false;
            }
            return cx > 0.0 && cy > 0.0 && cx < width && cy < height;
        } catch (Throwable t) {
            return false;
        }
    }

    private void recordError(String message, Throwable t) {
        long now = System.nanoTime();
        lastError = message;
        lastErrorNanos = now;
        errorCount.incrementAndGet();
        boolean log;
        synchronized (errorLock) {
            log = !message.equals(lastLoggedMessage) || now - lastLoggedNanos >= ERROR_LOG_INTERVAL_NANOS;
            if (log) {
                lastLoggedMessage = message;
                lastLoggedNanos = now;
            }
        }
        if (!log) {
            return;
        }
        try {
            if (t != null) {
                RobotLog.ee(LOG_TAG, t, message);
            } else {
                RobotLog.ee(LOG_TAG, message);
            }
        } catch (Throwable ignored) {
        }
    }

    static String describe(String where, Throwable t) {
        String prefix = where == null || where.isEmpty() ? "" : where + ": ";
        if (t == null) {
            return prefix + "unknown error";
        }
        String text = t.getMessage();
        String name = t.getClass().getSimpleName();
        return prefix + (text == null || text.isEmpty() ? name : name + ": " + text);
    }

    private static boolean sameText(String a, String b) {
        if (a == null) {
            return b == null;
        }
        return a.equals(b);
    }

    private static double clampUnit(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return 0.0;
        }
        return Math.max(-1.0, Math.min(1.0, value));
    }

    static final class ColorSampleRequest {
        final ColorSpaceMode space;
        final double u;
        final double v;
        final int radiusPx;
        final long requestedNanos;

        ColorSampleRequest(ColorSpaceMode space, double u, double v, int radiusPx, long requestedNanos) {
            this.space = space;
            this.u = u;
            this.v = v;
            this.radiusPx = radiusPx;
            this.requestedNanos = requestedNanos;
        }
    }

    private static final class ActiveState {
        final int index;
        final LogiPipeline pipeline;
        final ColorProfile[] profiles;
        final LogiScript script;
        final String profileNames;
        final long revision;
        final long setNanos;

        ActiveState(int index, LogiPipeline pipeline, ColorProfile[] profiles, LogiScript script,
                    String profileNames, long revision, long setNanos) {
            this.index = index;
            this.pipeline = pipeline;
            this.profiles = profiles == null ? new ColorProfile[0] : profiles;
            this.script = script;
            this.profileNames = profileNames;
            this.revision = revision;
            this.setNanos = setNanos;
        }

        ActiveState withProfiles(ColorProfile[] newProfiles, String newNames, long newRevision) {
            return new ActiveState(index, pipeline, newProfiles, script, newNames, newRevision, setNanos);
        }
    }
}
