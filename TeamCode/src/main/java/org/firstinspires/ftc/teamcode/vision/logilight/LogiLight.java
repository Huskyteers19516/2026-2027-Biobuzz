package org.firstinspires.ftc.teamcode.vision.logilight;

import android.graphics.Bitmap;
import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;
import org.firstinspires.ftc.teamcode.vision.logilight.math.ClusterLayout;
import org.firstinspires.ftc.teamcode.vision.logilight.math.FieldTargetState;
import org.firstinspires.ftc.teamcode.vision.logilight.math.HeadingHistory;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Mat3;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.TargetGeometry;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.core.Point;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;

public final class LogiLight implements LogiOverlaySource {

    public enum RunState {
        RUNNING,
        PAUSED,
        STOPPED,
        CLOSED
    }

    public static final int HEADING_HISTORY_CAPACITY = 400;
    public static final double CONNECTED_THRESHOLD_MS = 500.0;
    public static final double RESULT_EXPIRY_MS = 500.0;
    public static final double CONTROL_INFO_REFRESH_MS = 1000.0;
    public static final double CONTROL_INFO_INTEREST_MS = 3000.0;
    public static final double SETTINGS_POLL_MS = 250.0;
    public static final long PORTAL_CLOSE_WAIT_MS = 3000L;
    public static final long PORTAL_CLOSE_POLL_MS = 50L;
    public static final long PORTAL_REOPEN_WAIT_MS = 1000L;
    public static final long RESULT_LOCK_WAIT_MS = 200L;
    public static final double WATCHDOG_BACKOFF_BASE_SECONDS = 1.0;
    public static final double WATCHDOG_BACKOFF_MAX_SECONDS = 5.0;
    public static final double WATCHDOG_HEALTHY_RESET_SECONDS = 5.0;
    public static final double WATCHDOG_FRESH_FRAME_SECONDS = 0.5;
    public static final double WATCHDOG_SLOW_RETRY_SECONDS = 10.0;
    public static final long EXPOSURE_MODE_SETTLE_MS = 50L;
    public static final long CONTROL_SETTLE_MS = 20L;
    public static final double MIN_DECIMATION = 1.0;
    public static final double MAX_DECIMATION = 8.0;
    public static final double DEFAULT_DECIMATION = 2.0;
    public static final double MIN_WATCHDOG_TIMEOUT_SECONDS = 0.5;
    public static final double MIN_STDDEV_INCHES = 0.05;
    public static final double MIN_STDDEV_RADIANS = 1e-4;
    public static final int MAX_SCRIPT_INPUTS = 32;
    public static final int DEFAULT_WIDTH = 640;
    public static final int DEFAULT_HEIGHT = 480;

    private static final double NANOS_PER_MILLI = 1e6;
    private static final double NANOS_PER_SECOND = 1e9;
    private static final double MICROS_PER_MILLI = 1000.0;

    private static final AtomicReference<Object> DRIVER_STATION_OWNER = new AtomicReference<Object>();
    private static final AtomicReference<Object> PANELS_OWNER = new AtomicReference<Object>();
    private static final AtomicInteger INSTANCE_COUNTER = new AtomicInteger();

    private final Object streamOwnerToken = new Object();
    private final Object stateLock = new Object();
    private final ReentrantLock resultLock = new ReentrantLock();
    private final AtomicBoolean restartInProgress = new AtomicBoolean(false);
    private final AtomicBoolean controlTaskRunning = new AtomicBoolean(false);
    private final AtomicBoolean controlInfoReadRunning = new AtomicBoolean(false);
    private final AtomicInteger generationCounter = new AtomicInteger(0);
    private final AtomicReference<ControlRequest> pendingControls = new AtomicReference<ControlRequest>();
    private final HeadingHistory headingHistory = new HeadingHistory(HEADING_HISTORY_CAPACITY);
    private final long createdNanos = System.nanoTime();
    private final OpModeManagerNotifier.Notifications opModeEndListener = new OpModeManagerNotifier.Notifications() {
        @Override
        public void onOpModePreInit(OpMode opMode) {
        }

        @Override
        public void onOpModePreStart(OpMode opMode) {
        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
            try {
                close();
            } catch (Throwable ignored) {
            }
        }
    };

    private final HardwareMap hardwareMap;
    private final LogiCameraConfig buildCamera;
    private final boolean followsGlobalCamera;
    private final LogiFrameContext context;
    private final String settingsLoadMessage;
    private final String workerName;

    private volatile LogiCameraConfig camera;
    private volatile ExecutorService worker;
    private volatile Rig rig;
    private volatile RunState runState = RunState.RUNNING;
    private volatile ActivePipeline active;
    private volatile LogiFieldMap fieldMap;
    private volatile String fault = "";
    private volatile int watchdogRestarts = 0;
    private volatile String watchdogReason = "";
    private volatile boolean panelsStreaming = false;
    private volatile String panelsMessage = "";
    private volatile boolean driverStationStreaming = false;
    private volatile ControlRequest lastQueuedControls;
    private volatile ControlRequest appliedControls;
    private volatile int appliedControlsGeneration = -1;
    private volatile String controlStatus = "camera controls not applied yet";
    private volatile CameraControlInfo controlInfo = CameraControlInfo.UNKNOWN;
    private volatile long lastControlInfoRequestNanos = 0L;
    private volatile String lastSnapshotPath = "";
    private volatile LogiResult cachedResult = LogiResult.EMPTY;
    private volatile LogiResult expiredSource;
    private volatile LogiResult expiredResult;
    private volatile int configEpoch = 0;
    private volatile double lastHeadingRadians = Double.NaN;
    private volatile VisionPortal closingPortal;
    private volatile long lastRestartNanos = 0L;
    private volatile long healthySinceNanos = 0L;

    private long lastSettingsPollNanos = 0L;
    private long syncedSettingsRevision;
    private double[] syncedMount;
    private volatile boolean syncedPanelsWanted;

    private Rig keyRig;
    private List<AprilTagDetection> keyDetections;
    private ColorFrame keyColor;
    private ScriptFrame keyScript;
    private ActivePipeline keyActive;
    private LogiFieldMap keyFieldMap;
    private long keyProcessedAt = -1L;
    private int keyEpoch = -1;

    public LogiLight(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public LogiLight(HardwareMap hardwareMap, LogiCameraConfig cameraConfig) {
        this.hardwareMap = hardwareMap;
        LogiCameraConfig globalBefore = LogiLightSettings.CAMERA;
        this.settingsLoadMessage = loadSettingsSafely();
        LogiCameraConfig chosen = cameraConfig == null || cameraConfig == globalBefore ? LogiLightSettings.CAMERA : cameraConfig;
        if (chosen == null) {
            chosen = new LogiCameraConfig();
        }
        this.followsGlobalCamera = chosen == LogiLightSettings.CAMERA;
        this.camera = chosen;
        this.buildCamera = copyOf(chosen);
        this.workerName = "LogiLight-" + INSTANCE_COUNTER.incrementAndGet() + " " + buildCamera.webcamName;
        this.context = newContext(chosen, buildCamera);
        this.worker = newWorker(workerName);
        this.fieldMap = loadFieldMapSafely();
        this.syncedSettingsRevision = LogiLightSettings.revision();
        this.syncedMount = mountValues(chosen);
        this.active = resolveActive(LogiLightSettings.defaultPipelineIndex(), System.nanoTime(), null);
        watchOpModeEnd();
        initialize();
    }

    public static LogiLight forWebcam(HardwareMap hardwareMap, String webcamName) {
        loadSettingsSafely();
        LogiCameraConfig global = LogiLightSettings.CAMERA;
        if (global == null) {
            LogiCameraConfig fresh = new LogiCameraConfig();
            if (webcamName != null && !webcamName.trim().isEmpty()) {
                fresh.webcamName = webcamName.trim();
            }
            return new LogiLight(hardwareMap, fresh);
        }
        String wanted = webcamName == null ? "" : webcamName.trim();
        String current = global.webcamName == null ? "" : global.webcamName.trim();
        if (wanted.isEmpty() || wanted.equals(current)) {
            return new LogiLight(hardwareMap, global);
        }
        return new LogiLight(hardwareMap, global.withWebcamName(wanted));
    }

    private void initialize() {
        try {
            if (hardwareMap == null) {
                fault = "NOT AVAILABLE - hardwareMap is null";
                return;
            }
            Rig built = buildRig();
            synchronized (stateLock) {
                rig = built;
                fault = "";
                applyPipelineToRig(built, active);
                queueDesiredControls(true);
            }
            attachStreams(built);
        } catch (Throwable t) {
            rig = null;
            fault = "NOT AVAILABLE - " + describe(t);
        }
    }

    public boolean isAvailable() {
        return rig != null;
    }

    public String fault() {
        return fault;
    }

    public void start() {
        try {
            boolean rebuild = false;
            synchronized (stateLock) {
                RunState previous = runState;
                if (previous == RunState.RUNNING && rig != null) {
                    return;
                }
                if (worker == null || worker.isShutdown()) {
                    restartInProgress.set(false);
                    controlTaskRunning.set(false);
                    controlInfoReadRunning.set(false);
                    worker = newWorker(workerName);
                }
                runState = RunState.RUNNING;
                if (rig == null) {
                    rebuild = true;
                } else {
                    rig.streamingSinceNanos = 0L;
                    healthySinceNanos = 0L;
                    applyPipelineToRig(rig, active);
                    queueDesiredControls(true);
                }
            }
            if (rebuild) {
                watchOpModeEnd();
                awaitClosingPortal();
                initialize();
            }
        } catch (Throwable t) {
            recordError("start", t);
        }
    }

    public void pause() {
        try {
            synchronized (stateLock) {
                if (runState != RunState.RUNNING) {
                    return;
                }
                runState = RunState.PAUSED;
                applyPipelineToRig(rig, active);
            }
        } catch (Throwable t) {
            recordError("pause", t);
        }
    }

    public void stop() {
        try {
            synchronized (stateLock) {
                if (runState == RunState.CLOSED || runState == RunState.STOPPED) {
                    return;
                }
                runState = RunState.STOPPED;
                applyPipelineToRig(rig, active);
            }
        } catch (Throwable t) {
            recordError("stop", t);
        }
    }

    public void close() {
        Rig old;
        ExecutorService oldWorker;
        synchronized (stateLock) {
            runState = RunState.CLOSED;
            old = rig;
            rig = null;
            oldWorker = worker;
        }
        unwatchOpModeEnd();
        detachStreams();
        if (old != null) {
            closingPortal = old.portal;
            closeStreamQuietly(old);
            closePortalAsync(old);
        }
        if (oldWorker != null) {
            try {
                oldWorker.shutdownNow();
            } catch (Throwable ignored) {
            }
        }
        clearResultCache();
    }

    private void clearResultCache() {
        boolean locked = false;
        try {
            locked = resultLock.tryLock(RESULT_LOCK_WAIT_MS, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Throwable ignored) {
        }
        if (!locked) {
            return;
        }
        try {
            keyRig = null;
            keyDetections = null;
            keyColor = null;
            keyScript = null;
            cachedResult = LogiResult.EMPTY;
        } finally {
            resultLock.unlock();
        }
    }

    private void watchOpModeEnd() {
        try {
            OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
            if (manager != null) {
                manager.registerListener(opModeEndListener);
            }
        } catch (Throwable ignored) {
        }
    }

    private void unwatchOpModeEnd() {
        try {
            OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
            if (manager != null) {
                manager.unregisterListener(opModeEndListener);
            }
        } catch (Throwable ignored) {
        }
    }

    private void awaitClosingPortal() {
        VisionPortal closing = closingPortal;
        if (closing == null) {
            return;
        }
        long deadline = System.nanoTime() + (long) (PORTAL_REOPEN_WAIT_MS * NANOS_PER_MILLI);
        while (System.nanoTime() < deadline) {
            VisionPortal.CameraState state;
            try {
                state = closing.getCameraState();
            } catch (Throwable t) {
                break;
            }
            if (state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
                break;
            }
            try {
                Thread.sleep(PORTAL_CLOSE_POLL_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        closingPortal = null;
    }

    public boolean isRunning() {
        return runState == RunState.RUNNING && rig != null;
    }

    public RunState getRunState() {
        return runState;
    }

    public boolean isConnected() {
        Rig current = rig;
        if (current == null) {
            return false;
        }
        return cameraState(current) == VisionPortal.CameraState.STREAMING
                && getTimeSinceLastUpdate() <= CONNECTED_THRESHOLD_MS;
    }

    public long getTimeSinceLastUpdate() {
        long processedAt = context.lastProcessedAtNanos();
        long reference = processedAt > 0L ? processedAt : createdNanos;
        return Math.max(0L, (long) ((System.nanoTime() - reference) / NANOS_PER_MILLI));
    }

    public void update() {
        try {
            long now = System.nanoTime();
            if (now - lastSettingsPollNanos >= SETTINGS_POLL_MS * NANOS_PER_MILLI) {
                lastSettingsPollNanos = now;
                pollSettings(now);
            }
            Rig current = rig;
            if (current == null) {
                return;
            }
            VisionPortal.CameraState state = cameraState(current);
            trackStreaming(current, state, now);
            runWatchdog(current, state, now);
            dispatchControls(current, state);
            refreshControlInfoIfDue(current, state, now);
        } catch (Throwable t) {
            recordError("update", t);
        }
    }

    public boolean pipelineSwitch(int index) {
        try {
            LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
            if (pipeline == null) {
                return false;
            }
            Rig current;
            synchronized (stateLock) {
                ActivePipeline now = active;
                if (now != null && now.index == index && now.pipeline == pipeline && now.type == pipeline.typeOrDefault()) {
                    return true;
                }
                ActivePipeline next = resolveActive(index, System.nanoTime(), now);
                active = next;
                configEpoch++;
                current = rig;
                applyPipelineToRig(current, next);
                applyDecimation(current, next, false);
                queueDesiredControls(false);
            }
            if (current != null) {
                dispatchControls(current, cameraState(current));
            }
            return true;
        } catch (Throwable t) {
            recordError("pipeline switch", t);
            return false;
        }
    }

    public boolean pipelineSwitch(String name) {
        int index = LogiLightSettings.findPipeline(name);
        return index >= 0 && pipelineSwitch(index);
    }

    public int getPipelineIndex() {
        ActivePipeline a = active;
        return a == null ? 0 : a.index;
    }

    public LogiPipeline getPipeline() {
        ActivePipeline a = active;
        return a == null ? null : a.pipeline;
    }

    public void updateRobotOrientation(double headingRadians) {
        updateRobotOrientation(headingRadians, System.nanoTime());
    }

    public void updateRobotOrientation(double headingRadians, long timeNanos) {
        if (Double.isNaN(headingRadians) || Double.isInfinite(headingRadians)) {
            return;
        }
        headingHistory.add(timeNanos, headingRadians);
        lastHeadingRadians = PlanarPose.normalizeRadians(headingRadians);
    }

    public double getLastHeadingRadians() {
        return lastHeadingRadians;
    }

    public int getHeadingSampleCount() {
        return headingHistory.size();
    }

    public void updateScriptInputs(double... inputs) {
        try {
            double[] copy;
            if (inputs == null) {
                copy = new double[0];
            } else {
                copy = Arrays.copyOf(inputs, Math.min(inputs.length, MAX_SCRIPT_INPUTS));
            }
            context.setScriptInputs(copy);
        } catch (Throwable t) {
            recordError("script inputs", t);
        }
    }

    public LogiResult getLatestResult() {
        LogiResult result = currentResult(true);
        if (result == null) {
            return LogiResult.EMPTY;
        }
        if ((result.isValid() || result.hasBotpose()) && result.getStaleness() > RESULT_EXPIRY_MS) {
            LogiResult expired = expiredResult;
            if (expired == null || expiredSource != result) {
                expired = result.expiredCopy();
                expiredResult = expired;
                expiredSource = result;
            }
            return expired;
        }
        return result;
    }

    @Override
    public LogiResult overlayResult() {
        try {
            LogiResult result = currentResult(false);
            return result == null ? LogiResult.EMPTY : result;
        } catch (Throwable t) {
            return LogiResult.EMPTY;
        }
    }

    public LogiStatus getStatus() {
        LogiStatus.Builder b = new LogiStatus.Builder();
        try {
            long now = System.nanoTime();
            Rig current = rig;
            ActivePipeline a = active;
            if (current == null) {
                b.cameraState = fault.isEmpty() ? (runState == RunState.CLOSED ? "CLOSED" : "NO CAMERA") : "FAULT";
                b.portalFps = 0.0;
            } else {
                VisionPortal.CameraState state = cameraState(current);
                String stateName = state == null ? "UNKNOWN" : state.name();
                b.cameraState = restartInProgress.get() ? "RESTARTING" : stateName;
                b.portalFps = portalFps(current.portal);
            }
            b.runState = runState.name();
            b.webcamName = buildCamera.webcamName;
            b.fps = context.measuredFps();
            b.frameCount = context.frameCount();
            long lastCapture = context.lastCaptureNanos();
            b.lastFrameAgeMs = lastCapture == 0L ? Double.NaN : (now - lastCapture) / NANOS_PER_MILLI;
            b.averageProcessingMs = context.averageProcessingMs();
            if (a != null) {
                b.pipelineIndex = a.index;
                b.pipelineName = a.pipeline.name;
                b.pipelineType = a.type;
            }
            int width = context.frameWidth();
            int height = context.frameHeight();
            b.width = width > 0 ? width : buildCamera.width;
            b.height = height > 0 ? height : buildCamera.height;
            String intrinsics = context.intrinsicsSource();
            b.intrinsicsSource = intrinsics == null || intrinsics.isEmpty() ? "not initialized yet" : intrinsics;
            b.exposureSummary = exposureSummary();
            b.controlStatus = controlStatus;
            b.watchdogRestarts = watchdogRestarts;
            b.watchdogReason = watchdogReason;
            b.panelsStreaming = panelsStreaming;
            b.driverStationStreaming = driverStationStreaming;
            b.settingsSource = LogiLightStore.settingsSource();
            LogiFieldMap map = fieldMap;
            b.fieldMapSource = map == null ? "none" : map.summary();
            b.fault = fault;
            String processorError = context.lastError();
            b.lastProcessorError = processorError == null ? "" : processorError;
            b.processorErrorCount = context.errorCount();
            b.readNanos = now;
        } catch (Throwable t) {
            b.fault = "status failed: " + describe(t);
        }
        return b.build();
    }

    public CameraControlInfo getCameraControlInfo() {
        long now = System.nanoTime();
        lastControlInfoRequestNanos = now;
        try {
            Rig current = rig;
            if (current != null) {
                refreshControlInfoIfDue(current, cameraState(current), now);
            }
        } catch (Throwable t) {
            recordError("control info", t);
        }
        return controlInfo;
    }

    public String getControlStatus() {
        return controlStatus;
    }

    public void setExposure(double exposureMs, int gain) {
        LogiCameraConfig cfg = camera;
        if (isFinite(exposureMs) && exposureMs > 0.0) {
            cfg.exposureMs = exposureMs;
            cfg.manualExposure = true;
        } else if (isFinite(exposureMs)) {
            cfg.manualExposure = false;
        }
        if (gain >= 0) {
            cfg.gain = gain;
        }
        requestControls(true);
    }

    public void setWhiteBalance(boolean auto, int kelvin) {
        LogiCameraConfig cfg = camera;
        cfg.autoWhiteBalance = auto;
        if (kelvin > 0) {
            cfg.whiteBalanceKelvin = kelvin;
        }
        requestControls(true);
    }

    public void setFocus(boolean auto, double focusLength) {
        LogiCameraConfig cfg = camera;
        cfg.autoFocus = auto;
        if (isFinite(focusLength) && focusLength >= 0.0) {
            cfg.focusLength = focusLength;
        }
        requestControls(true);
    }

    public void setDecimation(double decimation) {
        if (!isFinite(decimation)) {
            return;
        }
        camera.decimation = clamp(decimation, MIN_DECIMATION, MAX_DECIMATION);
        try {
            applyDecimation(rig, active, true);
        } catch (Throwable t) {
            recordError("decimation", t);
        }
    }

    public double getDecimation() {
        return desiredDecimation(active);
    }

    public void applyCameraControls() {
        try {
            applyDecimation(rig, active, true);
        } catch (Throwable t) {
            recordError("decimation", t);
        }
        requestControls(true);
    }

    public void setCrosshair(double x, double y) {
        if (!isFinite(x) || !isFinite(y)) {
            return;
        }
        double clampedX = clamp(x, -1.0, 1.0);
        double clampedY = clamp(y, -1.0, 1.0);
        LogiCameraConfig cfg = camera;
        cfg.crosshairX = clampedX;
        cfg.crosshairY = clampedY;
        try {
            context.setCrosshair(clampedX, clampedY);
            syncedMount = mountValues(cfg);
            configEpoch++;
        } catch (Throwable t) {
            recordError("crosshair", t);
        }
    }

    public void refreshMount() {
        try {
            LogiCameraConfig cfg = camera;
            context.refreshMountFromConfig();
            context.setCrosshair(cfg.crosshairX, cfg.crosshairY);
            syncedMount = mountValues(cfg);
            configEpoch++;
        } catch (Throwable t) {
            recordError("mount", t);
        }
    }

    public void setStreamView(StreamView view) {
        try {
            context.setStreamView(view == null ? StreamView.ANNOTATED : view);
        } catch (Throwable t) {
            recordError("stream view", t);
        }
    }

    public StreamView getStreamView() {
        try {
            return context.effectiveStreamView();
        } catch (Throwable t) {
            return StreamView.ANNOTATED;
        }
    }

    public boolean captureSnapshot(String name) {
        Rig current = rig;
        if (current == null) {
            return false;
        }
        try {
            current.stream.requestSnapshot(snapshotName(name));
            return true;
        } catch (Throwable t) {
            recordError("snapshot", t);
            return false;
        }
    }

    public String lastSnapshotPath() {
        Rig current = rig;
        if (current != null) {
            try {
                String path = current.stream.lastSnapshotPath();
                if (path != null && !path.isEmpty()) {
                    lastSnapshotPath = path;
                }
            } catch (Throwable t) {
                recordError("snapshot path", t);
            }
        }
        return lastSnapshotPath;
    }

    public Bitmap latestStreamBitmap() {
        Rig current = rig;
        if (current == null) {
            return null;
        }
        try {
            return current.stream.latestBitmap();
        } catch (Throwable t) {
            return null;
        }
    }

    public CameraModel getCameraModel() {
        try {
            CameraModel model = context.cameraModelOrFallback();
            if (model != null) {
                return model;
            }
        } catch (Throwable ignored) {
        }
        LogiCameraConfig cfg = camera;
        LogiCameraConfig lens = buildCamera;
        int width = cfg.width > 0 ? cfg.width : DEFAULT_WIDTH;
        int height = cfg.height > 0 ? cfg.height : DEFAULT_HEIGHT;
        double[] intrinsics = lens.hasManualIntrinsics()
                ? new double[]{lens.fx, lens.fy, lens.cx, lens.cy}
                : CameraModel.fallbackIntrinsics(width, height);
        return new CameraModel(width, height, intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
                cfg.forwardInches, cfg.leftInches, cfg.upInches,
                cfg.yawLeftDegrees, cfg.tiltUpDegrees, cfg.rollClockwiseDegrees,
                cfg.crosshairX, cfg.crosshairY);
    }

    public LogiFieldMap getFieldMap() {
        return fieldMap;
    }

    public void setFieldMap(LogiFieldMap map) {
        fieldMap = map != null ? map : loadFieldMapSafely();
    }

    public AprilTagProcessor getAprilTagProcessor() {
        Rig current = rig;
        return current == null ? null : current.aprilTag;
    }

    public VisionPortal getVisionPortal() {
        Rig current = rig;
        return current == null ? null : current.portal;
    }

    public LogiFrameContext getFrameContext() {
        return context;
    }

    public LogiCameraConfig getCameraConfig() {
        return camera;
    }

    public String getWebcamName() {
        return buildCamera.webcamName;
    }

    public String getSettingsSource() {
        return LogiLightStore.settingsSource();
    }

    public String getSettingsLoadMessage() {
        try {
            String first = LogiLightStore.firstLoadMessage();
            if (first != null && !first.isEmpty()) {
                return first;
            }
        } catch (Throwable ignored) {
        }
        return settingsLoadMessage;
    }

    public int getWatchdogRestarts() {
        return watchdogRestarts;
    }

    public String getWatchdogReason() {
        return watchdogReason;
    }

    public boolean isPanelsStreaming() {
        return panelsStreaming;
    }

    public String getPanelsMessage() {
        return panelsMessage;
    }

    private LogiResult currentResult(boolean blocking) {
        if (blocking) {
            resultLock.lock();
        } else if (!resultLock.tryLock()) {
            return cachedResult;
        }
        try {
            return refreshResultLocked();
        } catch (Throwable t) {
            recordError("result", t);
            return cachedResult;
        } finally {
            resultLock.unlock();
        }
    }

    private LogiResult refreshResultLocked() {
        Rig current = rig;
        ActivePipeline a = active;
        LogiFieldMap map = fieldMap;
        List<AprilTagDetection> detections = null;
        ColorFrame colorFrame = ColorFrame.EMPTY;
        ScriptFrame scriptFrame = ScriptFrame.EMPTY;
        if (current != null) {
            detections = current.aprilTag.getDetections();
            ColorFrame latestColor = current.color.latest();
            colorFrame = latestColor == null ? ColorFrame.EMPTY : latestColor;
            ScriptFrame latestScript = current.script.latest();
            scriptFrame = latestScript == null ? ScriptFrame.EMPTY : latestScript;
        }
        long processedAt = context.lastProcessedAtNanos();
        int epoch = configEpoch;
        LogiResult cached = cachedResult;
        if (cached != null && current == keyRig && detections == keyDetections && colorFrame == keyColor
                && scriptFrame == keyScript && a == keyActive && map == keyFieldMap
                && processedAt == keyProcessedAt && epoch == keyEpoch) {
            return cached;
        }
        LogiResult built = buildResult(a, map, detections, colorFrame, scriptFrame, processedAt);
        keyRig = current;
        keyDetections = detections;
        keyColor = colorFrame;
        keyScript = scriptFrame;
        keyActive = a;
        keyFieldMap = map;
        keyProcessedAt = processedAt;
        keyEpoch = epoch;
        cachedResult = built;
        return built;
    }

    private LogiResult buildResult(ActivePipeline a, LogiFieldMap map, List<AprilTagDetection> detections,
                                   ColorFrame colorFrame, ScriptFrame scriptFrame, long processedAt) {
        LogiResult.Builder out = new LogiResult.Builder();
        out.buildNanos = System.nanoTime();
        out.frameNumber = context.frameCount();
        if (a == null) {
            return out.build();
        }
        long frameCapture = context.lastCaptureNanos();
        frameCapture = Math.max(frameCapture, newestDetectionNanos(detections));
        frameCapture = Math.max(frameCapture, colorFrame.captureNanos);
        frameCapture = Math.max(frameCapture, scriptFrame.captureNanos);
        ActivePipeline snapshot = a;
        if (a.previous != null && frameCapture != 0L && frameCapture < a.sinceNanos) {
            snapshot = a.previous;
        }
        PipelineType type = snapshot.type;
        LogiPipeline pipeline = snapshot.pipeline;
        out.pipelineIndex = snapshot.index;
        out.pipelineName = pipeline.name;
        out.pipelineType = type;
        out.captureNanos = frameCapture;

        CameraModel model = context.cameraModel();
        LogiLocalizationConfig loc = LogiLightSettings.LOCALIZATION == null ? new LogiLocalizationConfig() : LogiLightSettings.LOCALIZATION;

        FiducialBatch batch = new FiducialBatch();
        if (type.usesAprilTag() && model != null && detections != null) {
            boolean localize = pipeline.localization && loc.enabled;
            for (AprilTagDetection detection : detections) {
                if (detection == null || detection.frameAcquisitionNanoTime < snapshot.sinceNanos) {
                    continue;
                }
                LogiFiducial fiducial = buildFiducial(detection, pipeline, model, map, loc, localize, batch);
                if (fiducial != null) {
                    batch.fiducials.add(fiducial);
                }
            }
        }
        LogiFiducial primaryFiducial = sortFiducials(batch.fiducials, pipeline);
        out.fiducials = batch.fiducials;
        out.botpose = combine(batch.mt1, LogiBotPose.METHOD_MT1);
        out.botposeMt2 = combine(batch.mt2, LogiBotPose.METHOD_MT2);

        List<ColorTarget> colors = Collections.emptyList();
        if (type.usesColor() && colorFrame.captureNanos >= snapshot.sinceNanos && colorFrame.targets != null) {
            colors = colorFrame.targets;
        }
        out.colors = colors;
        ColorTarget primaryColor = colors.isEmpty() ? null : colors.get(0);

        ScriptFrame script = ScriptFrame.EMPTY;
        if (type.usesScript() && scriptFrame.captureNanos >= snapshot.sinceNanos) {
            script = scriptFrame;
        }
        LogiScriptResult scriptResult = script.result == null ? LogiScriptResult.EMPTY : script.result;
        out.scriptResult = scriptResult;
        out.scriptOutput = scriptResult.outputs;

        if (type == PipelineType.APRILTAG && primaryFiducial != null) {
            applyPrimaryFiducial(out, primaryFiducial);
        } else if (type == PipelineType.COLOR && primaryColor != null) {
            applyPrimaryColor(out, primaryColor, colorFrame.captureNanos);
        } else if (type == PipelineType.APRILTAG_AND_COLOR) {
            if (primaryFiducial != null) {
                applyPrimaryFiducial(out, primaryFiducial);
            } else if (primaryColor != null) {
                applyPrimaryColor(out, primaryColor, colorFrame.captureNanos);
            }
        } else if (type == PipelineType.SCRIPT && scriptResult.hasTarget) {
            applyPrimaryScript(out, script, scriptResult, model);
        }

        if (out.primaryFiducial == null && out.primaryColor == null && !LogiResult.PRIMARY_SCRIPT.equals(out.primaryKind)) {
            out.primaryKind = LogiResult.PRIMARY_NONE;
        }
        if (processedAt > 0L && out.captureNanos > 0L && processedAt >= out.captureNanos) {
            out.targetingLatencyMs = (processedAt - out.captureNanos) / NANOS_PER_MILLI;
        } else {
            out.targetingLatencyMs = context.averageProcessingMs();
        }
        return out.build();
    }

    private void applyPrimaryFiducial(LogiResult.Builder out, LogiFiducial f) {
        out.primaryKind = LogiResult.PRIMARY_FIDUCIAL;
        out.primaryFiducial = f;
        out.tx = f.tx;
        out.ty = f.ty;
        out.txNC = f.txNC;
        out.tyNC = f.tyNC;
        out.ta = f.ta;
        out.primaryName = f.name;
        out.primaryCenterX = f.centerX;
        out.primaryCenterY = f.centerY;
        out.primaryDistanceInches = isFinite(f.aimDistanceInches) ? f.aimDistanceInches : f.robotDistanceInches;
        out.primaryBearingLeftDegrees = isFinite(f.aimBearingLeftDegrees) ? f.aimBearingLeftDegrees : f.bearingLeftDegrees;
        if (f.captureNanos > 0L) {
            out.captureNanos = f.captureNanos;
        }
    }

    private void applyPrimaryColor(LogiResult.Builder out, ColorTarget t, long captureNanos) {
        out.primaryKind = LogiResult.PRIMARY_COLOR;
        out.primaryColor = t;
        out.tx = t.tx;
        out.ty = t.ty;
        out.txNC = t.txNC;
        out.tyNC = t.tyNC;
        out.ta = t.ta;
        out.primaryName = t.profileName == null ? "" : t.profileName;
        out.primaryCenterX = t.centerX;
        out.primaryCenterY = t.centerY;
        out.primaryDistanceInches = t.groundDistanceInches;
        out.primaryBearingLeftDegrees = t.groundBearingLeftDegrees;
        if (captureNanos > 0L) {
            out.captureNanos = captureNanos;
        }
    }

    private void applyPrimaryScript(LogiResult.Builder out, ScriptFrame frame, LogiScriptResult result, CameraModel model) {
        if (model == null || !isFinite(result.targetX) || !isFinite(result.targetY)) {
            return;
        }
        double[] angles = model.targetAngles(result.targetX, result.targetY);
        out.primaryKind = LogiResult.PRIMARY_SCRIPT;
        out.tx = angles[0];
        out.ty = angles[1];
        out.txNC = angles[2];
        out.tyNC = angles[3];
        double imageArea = model.width * (double) model.height;
        double boxArea = Math.abs(result.targetWidth * result.targetHeight);
        out.ta = imageArea > 0.0 && isFinite(boxArea) ? 100.0 * boxArea / imageArea : 0.0;
        String label = result.label == null || result.label.isEmpty() ? frame.scriptName : result.label;
        out.primaryName = label == null ? "" : label;
        out.primaryCenterX = result.targetX;
        out.primaryCenterY = result.targetY;
        if (frame.captureNanos > 0L) {
            out.captureNanos = frame.captureNanos;
        }
    }

    private LogiFiducial buildFiducial(AprilTagDetection detection, LogiPipeline pipeline, CameraModel model,
                                       LogiFieldMap map, LogiLocalizationConfig loc, boolean localize,
                                       FiducialBatch batch) {
        LogiFiducial.Builder f = new LogiFiducial.Builder();
        f.captureNanos = detection.frameAcquisitionNanoTime;
        double[] pixelCenter = null;
        List<double[]> quads = new ArrayList<double[]>();
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            f.isCluster = true;
            f.id = LogiFiducial.CLUSTER_ID;
            f.percentClusterFound = cluster.percentClusterFound;
            f.name = clusterName(cluster.metadata);
        } else if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            f.isCluster = false;
            f.id = single.id;
            f.hamming = single.hamming;
            f.decisionMargin = single.decisionMargin;
            f.percentClusterFound = 100;
            String metadataName = single.metadata == null || single.metadata.name == null ? "" : single.metadata.name.trim();
            f.name = metadataName.isEmpty() ? "tag " + single.id : metadataName;
            double[] quad = quadFromPoints(single.corners);
            if (quad != null) {
                quads.add(quad);
            }
            if (single.center != null && isFinite(single.center.x) && isFinite(single.center.y)) {
                pixelCenter = new double[]{single.center.x, single.center.y};
            }
        } else {
            return null;
        }

        double unitToInches = inchesPerUnit(detection.distanceUnit);
        Vec3 targetInOptical = null;
        Mat3 rotationInOptical = null;
        AprilTagPoseRaw raw = detection.rawPose;
        if (raw != null && raw.R != null) {
            Vec3 position = new Vec3(raw.x * unitToInches, raw.y * unitToInches, raw.z * unitToInches);
            Mat3 rotation = matrixFrom(raw.R);
            if (position.isFinite() && rotation != null && rotation.isFinite()) {
                targetInOptical = position;
                rotationInOptical = rotation;
            }
        }
        AprilTagPoseFtc ftc = detection.ftcPose;
        if (ftc != null) {
            f.range = ftc.range * unitToInches;
            f.bearing = ftc.bearing;
            f.elevation = ftc.elevation;
            f.yaw = ftc.yaw;
            f.pitch = ftc.pitch;
            f.roll = ftc.roll;
        }

        if (targetInOptical != null) {
            f.targetInOptical = targetInOptical;
            f.rotationInOptical = rotationInOptical;
            f.targetInRobot = model.opticalPointToRobot(targetInOptical);
            f.rotationInRobot = model.opticalRotationToRobot(rotationInOptical);
            Vec3 aimOffset = new Vec3(pipeline.aimOffsetX, pipeline.aimOffsetY, pipeline.aimOffsetZ);
            Vec3 aimInOptical = aimOffset.isFinite()
                    ? targetInOptical.plus(rotationInOptical.times(aimOffset))
                    : targetInOptical;
            f.aimInRobot = model.opticalPointToRobot(aimInOptical);
            f.robotDistanceInches = TargetGeometry.horizontalDistanceInches(f.targetInRobot);
            f.bearingLeftDegrees = TargetGeometry.horizontalBearingLeftDegrees(f.targetInRobot);
            f.aimDistanceInches = TargetGeometry.horizontalDistanceInches(f.aimInRobot);
            f.aimBearingLeftDegrees = TargetGeometry.horizontalBearingLeftDegrees(f.aimInRobot);
            if (f.isCluster) {
                quads = projectClusterQuads(f.name, targetInOptical, rotationInOptical, model);
            }
            boolean useAimPoint = f.isCluster || pipeline.hasAimOffset() || pixelCenter == null;
            if (useAimPoint) {
                double[] projected = model.project(aimInOptical);
                if (projected != null) {
                    pixelCenter = projected;
                }
            }
        } else if (f.isCluster) {
            return null;
        }

        if (pixelCenter != null) {
            f.centerX = pixelCenter[0];
            f.centerY = pixelCenter[1];
            double[] angles = model.targetAngles(pixelCenter[0], pixelCenter[1]);
            f.tx = angles[0];
            f.ty = angles[1];
            f.txNC = angles[2];
            f.tyNC = angles[3];
        } else if (ftc != null) {
            f.tx = -ftc.bearing;
            f.ty = ftc.elevation;
            f.txNC = f.tx;
            f.tyNC = f.ty;
        } else {
            return null;
        }

        double imageArea = model.width * (double) model.height;
        double quadArea = 0.0;
        for (double[] quad : quads) {
            quadArea += TargetGeometry.polygonArea(quad);
        }
        if (f.isCluster) {
            quadArea *= clamp(f.percentClusterFound, 0.0, 100.0) / 100.0;
        }
        f.ta = imageArea > 0.0 ? 100.0 * quadArea / imageArea : 0.0;
        f.corners = quads;

        if (localize) {
            if (f.targetInRobot == null) {
                f.rejectReason = "no pose";
            } else {
                localizeFiducial(f, map, loc, batch);
            }
        }
        return f.build();
    }

    private void localizeFiducial(LogiFiducial.Builder f, LogiFieldMap map, LogiLocalizationConfig loc, FiducialBatch batch) {
        LogiFieldMap.Target target = map == null ? null : map.find(f.name);
        if (target == null) {
            f.rejectReason = "not in field map";
            return;
        }
        if (!target.surveyed && !loc.useUnsurveyedTargets) {
            f.rejectReason = "not surveyed";
            return;
        }
        if (f.isCluster && f.percentClusterFound < loc.minPercentClusterFound) {
            f.rejectReason = String.format(Locale.US, "only %d%% of cluster", f.percentClusterFound);
            return;
        }
        if (!isFinite(f.robotDistanceInches) || f.robotDistanceInches > loc.maxTargetDistanceInches) {
            f.rejectReason = String.format(Locale.US, "too far %.0f in", f.robotDistanceInches);
            return;
        }
        FieldTargetState best = null;
        double bestTilt = Double.POSITIVE_INFINITY;
        for (FieldTargetState state : target.states) {
            double tilt = TargetGeometry.tiltMismatchRadians(f.rotationInRobot, state.rotationField);
            if (isFinite(tilt) && tilt < bestTilt) {
                bestTilt = tilt;
                best = state;
            }
        }
        if (best == null) {
            f.rejectReason = "no field states";
            return;
        }
        f.matchedState = best.label;
        f.tiltMismatchDegrees = Math.toDegrees(bestTilt);
        if (f.tiltMismatchDegrees > loc.maxTiltMismatchDegrees) {
            f.rejectReason = String.format(Locale.US, "tilt %.0f deg off %s", f.tiltMismatchDegrees, best.label);
            return;
        }
        f.heightErrorInches = TargetGeometry.heightErrorInches(f.targetInRobot, best.originField);
        if (!isFinite(f.heightErrorInches) || Math.abs(f.heightErrorInches) > loc.maxHeightErrorInches) {
            f.rejectReason = String.format(Locale.US, "height err %+.1f in", f.heightErrorInches);
            return;
        }
        boolean yawKnown = target.surveyed;
        if (yawKnown) {
            PlanarPose mt1 = TargetGeometry.solveFull(f.targetInRobot, f.rotationInRobot, best);
            if (mt1 != null && mt1.isFinite()) {
                f.mt1 = mt1;
            }
        }
        String mt2Reason = "";
        long extrapolateNanos = (long) (Math.max(0.0, loc.headingMaxExtrapolateMs) * NANOS_PER_MILLI);
        Double heading = headingHistory.headingAt(f.captureNanos, extrapolateNanos);
        if (heading == null) {
            mt2Reason = headingHistory.size() == 0
                    ? "MT2: no heading (call updateRobotOrientation)"
                    : "MT2: no heading at frame time";
        } else {
            double omega = Math.abs(Math.toDegrees(headingHistory.angularVelocityAt(f.captureNanos)));
            f.angularVelocityDegPerSec = omega;
            if (omega > loc.maxAngularVelocityDegPerSec) {
                mt2Reason = String.format(Locale.US, "MT2: turning %.0f deg/s", omega);
            } else {
                double mismatch = Double.NaN;
                if (yawKnown) {
                    mismatch = Math.toDegrees(TargetGeometry.rotationMismatchRadians(f.rotationInRobot, heading, best.rotationField));
                    f.rotationMismatchDegrees = mismatch;
                }
                if (yawKnown && (!isFinite(mismatch) || mismatch > loc.maxRotationMismatchDegrees)) {
                    mt2Reason = String.format(Locale.US, "MT2: rotation off %.0f deg", mismatch);
                } else {
                    PlanarPose mt2 = TargetGeometry.solveKnownHeading(f.targetInRobot, best.originField, heading);
                    if (mt2 != null && mt2.isPositionFinite()) {
                        f.mt2 = mt2;
                    } else {
                        mt2Reason = "MT2: solve failed";
                    }
                }
            }
        }
        if (yawKnown) {
            f.rejectReason = mt2Reason;
        } else {
            f.rejectReason = mt2Reason.isEmpty()
                    ? "MT1 off: unsurveyed, yaw unknown"
                    : "MT1 off: unsurveyed, yaw unknown; " + mt2Reason;
        }
        if (f.mt1 == null && f.mt2 == null && mt2Reason.isEmpty() && yawKnown) {
            f.rejectReason = "pose solve failed";
        }
        double[] sigma = TargetGeometry.stddevXYH(f.robotDistanceInches, loc.stddevBaseInches, loc.stddevPerInchSquared,
                loc.headingStddevBaseDegrees, loc.headingStddevPerInchDegrees);
        double sigmaXY = Math.max(MIN_STDDEV_INCHES, isFinite(sigma[0]) ? sigma[0] : 1.0);
        double sigmaHeading = Math.max(MIN_STDDEV_RADIANS, isFinite(sigma[2]) ? sigma[2] : Math.toRadians(3.0));
        if (f.mt1 != null) {
            batch.mt1.add(new AcceptedPose(f.mt1, f.name, f.robotDistanceInches, f.ta, best.originField, f.captureNanos, sigmaXY, sigmaHeading));
        }
        if (f.mt2 != null) {
            batch.mt2.add(new AcceptedPose(f.mt2, f.name, f.robotDistanceInches, f.ta, best.originField, f.captureNanos, sigmaXY, sigmaHeading));
        }
    }

    private static LogiBotPose combine(List<AcceptedPose> accepted, String method) {
        if (accepted == null || accepted.isEmpty()) {
            return null;
        }
        List<PlanarPose> poses = new ArrayList<PlanarPose>();
        List<Double> weights = new ArrayList<Double>();
        List<String> names = new ArrayList<String>();
        double sumWeightXY = 0.0;
        double sumWeightHeading = 0.0;
        double sumDistance = 0.0;
        double sumArea = 0.0;
        long capture = 0L;
        for (AcceptedPose a : accepted) {
            double weight = 1.0 / (a.sigmaXY * a.sigmaXY);
            poses.add(a.pose);
            weights.add(weight);
            names.add(a.name);
            sumWeightXY += weight;
            sumWeightHeading += 1.0 / (a.sigmaHeading * a.sigmaHeading);
            sumDistance += a.distance;
            sumArea += a.area;
            capture = Math.max(capture, a.captureNanos);
        }
        PlanarPose average = TargetGeometry.weightedAverage(poses, weights);
        if (average == null || !average.isPositionFinite()) {
            return null;
        }
        double span = 0.0;
        for (int i = 0; i < accepted.size(); i++) {
            for (int j = i + 1; j < accepted.size(); j++) {
                double d = accepted.get(i).origin.distanceTo(accepted.get(j).origin);
                if (isFinite(d)) {
                    span = Math.max(span, d);
                }
            }
        }
        int count = accepted.size();
        double stddevXY = sumWeightXY > 0.0 ? Math.sqrt(1.0 / sumWeightXY) : Double.NaN;
        boolean megaTag2 = LogiBotPose.METHOD_MT2.equals(method);
        double stddevHeading = megaTag2 ? 0.0 : (sumWeightHeading > 0.0 ? Math.sqrt(1.0 / sumWeightHeading) : Double.NaN);
        return new LogiBotPose(average, method, count, sumDistance / count, sumArea / count, span,
                stddevXY, stddevHeading, capture, names);
    }

    private static LogiFiducial sortFiducials(List<LogiFiducial> fiducials, LogiPipeline pipeline) {
        if (fiducials.isEmpty()) {
            return null;
        }
        final TargetSort sort = pipeline.fiducialSort == null ? TargetSort.CLOSEST_TO_CROSSHAIR : pipeline.fiducialSort;
        Collections.sort(fiducials, new Comparator<LogiFiducial>() {
            @Override
            public int compare(LogiFiducial a, LogiFiducial b) {
                return Double.compare(sortKey(a, sort), sortKey(b, sort));
            }
        });
        for (String wanted : pipeline.priorityTargetNames()) {
            for (int i = 0; i < fiducials.size(); i++) {
                LogiFiducial candidate = fiducials.get(i);
                boolean nameMatch = candidate.name.equalsIgnoreCase(wanted);
                boolean idMatch = !candidate.isCluster && String.valueOf(candidate.id).equals(wanted);
                if (nameMatch || idMatch) {
                    if (i > 0) {
                        fiducials.remove(i);
                        fiducials.add(0, candidate);
                    }
                    return candidate;
                }
            }
        }
        return fiducials.get(0);
    }

    private static double sortKey(LogiFiducial f, TargetSort sort) {
        double key;
        switch (sort) {
            case LARGEST:
                key = -f.ta;
                break;
            case SMALLEST:
                key = f.ta;
                break;
            case LEFTMOST:
                key = f.tx;
                break;
            case RIGHTMOST:
                key = -f.tx;
                break;
            case HIGHEST:
                key = -f.ty;
                break;
            case LOWEST:
                key = f.ty;
                break;
            case NEAREST_GROUND_DISTANCE:
                key = f.robotDistanceInches;
                break;
            case CLOSEST_TO_CROSSHAIR:
            default:
                key = Math.hypot(f.tx, f.ty);
                break;
        }
        return isFinite(key) ? key : Double.POSITIVE_INFINITY;
    }

    private static List<double[]> projectClusterQuads(String clusterName, Vec3 targetInOptical, Mat3 rotationInOptical, CameraModel model) {
        List<double[]> quads = new ArrayList<double[]>();
        ClusterLayout layout = ClusterLayout.forName(clusterName);
        if (layout == null) {
            return quads;
        }
        for (Vec3[] memberCorners : layout.memberCorners()) {
            double[] quad = new double[LogiFiducial.QUAD_VALUES];
            boolean visible = true;
            for (int k = 0; k < memberCorners.length && k < 4; k++) {
                Vec3 cornerInOptical = targetInOptical.plus(rotationInOptical.times(memberCorners[k]));
                double[] pixel = model.project(cornerInOptical);
                if (pixel == null) {
                    visible = false;
                    break;
                }
                quad[2 * k] = pixel[0];
                quad[2 * k + 1] = pixel[1];
            }
            if (visible) {
                quads.add(quad);
            }
        }
        return quads;
    }

    private static double[] quadFromPoints(Point[] corners) {
        if (corners == null || corners.length < 4) {
            return null;
        }
        double[] quad = new double[LogiFiducial.QUAD_VALUES];
        for (int k = 0; k < 4; k++) {
            Point p = corners[k];
            if (p == null || !isFinite(p.x) || !isFinite(p.y)) {
                return null;
            }
            quad[2 * k] = p.x;
            quad[2 * k + 1] = p.y;
        }
        return quad;
    }

    private static String clusterName(AprilTagClusterMetadata metadata) {
        if (metadata == null) {
            return "cluster";
        }
        String shortName = metadata.shortName == null ? "" : metadata.shortName.trim();
        if (!shortName.isEmpty()) {
            return shortName;
        }
        String longName = metadata.name == null ? "" : metadata.name.trim();
        return longName.isEmpty() ? "cluster" : longName;
    }

    private static Mat3 matrixFrom(MatrixF matrix) {
        if (matrix == null || matrix.numRows() < 3 || matrix.numCols() < 3) {
            return null;
        }
        return new Mat3(
                matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2),
                matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2),
                matrix.get(2, 0), matrix.get(2, 1), matrix.get(2, 2));
    }

    private static double inchesPerUnit(DistanceUnit unit) {
        if (unit == null || unit == DistanceUnit.INCH) {
            return 1.0;
        }
        return unit.toInches(1.0);
    }

    private static long newestDetectionNanos(List<AprilTagDetection> detections) {
        long newest = 0L;
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection != null) {
                    newest = Math.max(newest, detection.frameAcquisitionNanoTime);
                }
            }
        }
        return newest;
    }

    private Rig buildRig() {
        LogiCameraConfig cfg = buildCamera;
        String name = cfg.webcamName == null ? "" : cfg.webcamName.trim();
        WebcamName webcam;
        try {
            webcam = hardwareMap.get(WebcamName.class, name);
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException("no webcam named \"" + name + "\" in the active robot configuration");
        }
        int width = cfg.width > 0 ? cfg.width : DEFAULT_WIDTH;
        int height = cfg.height > 0 ? cfg.height : DEFAULT_HEIGHT;
        int threads = cfg.aprilTagThreads > 0 ? cfg.aprilTagThreads : AprilTagProcessor.THREADS_DEFAULT;
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setDrawTagID(false)
                .setNumThreads(threads)
                .setSuppressCalibrationWarnings(false);
        if (cfg.hasManualIntrinsics()) {
            tagBuilder.setLensIntrinsics(cfg.fx, cfg.fy, cfg.cx, cfg.cy);
        }
        AprilTagProcessor aprilTag = tagBuilder.build();
        double decimation = desiredDecimation(active);
        aprilTag.setDecimation((float) decimation);
        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE);
        ColorTargetProcessor color = new ColorTargetProcessor(context);
        ScriptProcessor script = new ScriptProcessor(context);
        LogiLightStream stream = new LogiLightStream(context, color, script, this);
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(width, height))
                .setStreamFormat(cfg.mjpeg ? VisionPortal.StreamFormat.MJPEG : VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false)
                .setShowStatsOverlay(false)
                .setAutoStopLiveView(true)
                .addProcessors(aprilTag, color, script, stream)
                .build();
        Rig built = new Rig(generationCounter.incrementAndGet(), portal, aprilTag, color, script, stream);
        built.appliedDecimation = decimation;
        return built;
    }

    private void applyPipelineToRig(Rig target, ActivePipeline a) {
        if (a == null) {
            return;
        }
        try {
            context.setActivePipeline(a.index, a.pipeline, a.profiles, a.script);
        } catch (Throwable t) {
            recordError("set pipeline", t);
        }
        if (target == null) {
            return;
        }
        boolean on = runState == RunState.RUNNING;
        setProcessorEnabled(target, target.aprilTag, on && a.type.usesAprilTag());
        setProcessorEnabled(target, target.color, on && a.type.usesColor());
        setProcessorEnabled(target, target.script, on && a.type.usesScript());
        setProcessorEnabled(target, target.stream, on);
    }

    private void setProcessorEnabled(Rig target, VisionProcessor processor, boolean enabled) {
        try {
            if (target.portal.getProcessorEnabled(processor) != enabled) {
                target.portal.setProcessorEnabled(processor, enabled);
            }
        } catch (Throwable t) {
            recordError("processor enable", t);
        }
    }

    private ActivePipeline resolveActive(int requestedIndex, long sinceNanos, ActivePipeline previous) {
        int index = requestedIndex;
        LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
        if (pipeline == null) {
            index = LogiLightSettings.defaultPipelineIndex();
            pipeline = LogiLightSettings.pipelineAt(index);
        }
        if (pipeline == null) {
            index = 0;
            pipeline = LogiPipeline.goals();
        }
        return new ActivePipeline(index, pipeline, resolveProfiles(pipeline), resolveScript(pipeline), sinceNanos,
                previous == null ? null : previous.detached());
    }

    private static ColorProfile[] resolveProfiles(LogiPipeline pipeline) {
        try {
            return LogiLightSettings.resolveColorProfiles(pipeline);
        } catch (Throwable t) {
            return new ColorProfile[0];
        }
    }

    private static LogiScript resolveScript(LogiPipeline pipeline) {
        if (pipeline == null || pipeline.scriptName == null || pipeline.scriptName.trim().isEmpty()) {
            return null;
        }
        try {
            return ScriptProcessor.find(pipeline.scriptName.trim());
        } catch (Throwable t) {
            return null;
        }
    }

    private void pollSettings(long now) {
        long revision = LogiLightSettings.revision();
        boolean replaced = revision != syncedSettingsRevision;
        syncedSettingsRevision = revision;
        if (replaced && followsGlobalCamera) {
            adoptGlobalCamera();
        }
        LogiCameraConfig cfg = camera;
        double[] mount = mountValues(cfg);
        if (replaced || !Arrays.equals(mount, syncedMount)) {
            syncedMount = mount;
            try {
                context.refreshMountFromConfig();
                context.setCrosshair(cfg.crosshairX, cfg.crosshairY);
                configEpoch++;
            } catch (Throwable t) {
                recordError("mount", t);
            }
        }
        Rig current;
        synchronized (stateLock) {
            current = rig;
            ActivePipeline a = active;
            if (a != null) {
                int index = a.index;
                LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
                if (pipeline == null) {
                    index = LogiLightSettings.defaultPipelineIndex();
                    pipeline = LogiLightSettings.pipelineAt(index);
                }
                if (pipeline != null) {
                    ColorProfile[] profiles = resolveProfiles(pipeline);
                    LogiScript script = resolveScript(pipeline);
                    boolean identityChanged = index != a.index || pipeline != a.pipeline || pipeline.typeOrDefault() != a.type;
                    boolean resolvedChanged = !sameElements(profiles, a.profiles) || script != a.script;
                    if (identityChanged || resolvedChanged) {
                        ActivePipeline next = identityChanged
                                ? new ActivePipeline(index, pipeline, profiles, script, now, a.detached())
                                : new ActivePipeline(index, pipeline, profiles, script, a.sinceNanos, a.previous);
                        active = next;
                        configEpoch++;
                        applyPipelineToRig(current, next);
                    }
                }
            }
            applyDecimation(current, active, false);
            queueDesiredControls(false);
        }
        boolean panelsWanted = LogiLightSettings.panelsStreamEnabled();
        if (panelsWanted != syncedPanelsWanted && current != null && runState != RunState.CLOSED) {
            syncedPanelsWanted = panelsWanted;
            if (panelsWanted) {
                startPanels(current);
            } else {
                stopPanels();
            }
        }
    }

    private void adoptGlobalCamera() {
        LogiCameraConfig global = LogiLightSettings.CAMERA;
        LogiCameraConfig mine = camera;
        if (global == null || global == mine) {
            return;
        }
        copyPublicFields(global, mine);
        LogiLightSettings.CAMERA = mine;
    }

    private static void copyPublicFields(Object from, Object to) {
        for (Field field : to.getClass().getFields()) {
            int modifiers = field.getModifiers();
            if (Modifier.isStatic(modifiers) || Modifier.isFinal(modifiers)) {
                continue;
            }
            try {
                field.set(to, field.get(from));
            } catch (Throwable ignored) {
            }
        }
    }

    private void trackStreaming(Rig current, VisionPortal.CameraState state, long now) {
        if (state == VisionPortal.CameraState.STREAMING) {
            if (current.streamingSinceNanos == 0L) {
                current.streamingSinceNanos = now;
                requestControls(true);
            }
        } else {
            current.streamingSinceNanos = 0L;
        }
    }

    private void runWatchdog(Rig current, VisionPortal.CameraState state, long now) {
        if (runState != RunState.RUNNING || restartInProgress.get()) {
            return;
        }
        LogiCameraConfig cfg = camera;
        if (!cfg.watchdogEnabled) {
            return;
        }
        String reason = null;
        if (state == VisionPortal.CameraState.ERROR) {
            reason = "camera ERROR";
        } else if (state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            reason = "camera closed unexpectedly";
        } else if (state == VisionPortal.CameraState.STREAMING && current.streamingSinceNanos != 0L) {
            long lastFrame = Math.max(context.lastCaptureNanos(), current.streamingSinceNanos);
            double timeoutSeconds = Math.max(MIN_WATCHDOG_TIMEOUT_SECONDS, cfg.watchdogTimeoutSeconds);
            if (now - lastFrame > timeoutSeconds * NANOS_PER_SECOND) {
                reason = String.format(Locale.US, "no frames for %.1f s", (now - lastFrame) / NANOS_PER_SECOND);
            }
        }
        if (reason == null) {
            trackHealthy(state, now);
            return;
        }
        healthySinceNanos = 0L;
        int limit = Math.max(0, cfg.watchdogMaxRestarts);
        long last = lastRestartNanos;
        long sinceRestart = now - last;
        if (watchdogRestarts >= limit) {
            if (limit == 0 || (last != 0L && sinceRestart < WATCHDOG_SLOW_RETRY_SECONDS * NANOS_PER_SECOND)) {
                watchdogReason = "!! " + reason + " - restart limit reached"
                        + (limit == 0 ? "" : String.format(Locale.US, ", retry every %.0f s", WATCHDOG_SLOW_RETRY_SECONDS));
                return;
            }
        } else if (last != 0L && sinceRestart < watchdogBackoffNanos()) {
            return;
        }
        restartAsync(current, reason);
    }

    private long watchdogBackoffNanos() {
        int doublings = Math.max(0, Math.min(watchdogRestarts, 3));
        double seconds = Math.min(WATCHDOG_BACKOFF_MAX_SECONDS, WATCHDOG_BACKOFF_BASE_SECONDS * (1 << doublings));
        return (long) (seconds * NANOS_PER_SECOND);
    }

    private void trackHealthy(VisionPortal.CameraState state, long now) {
        long lastCapture = context.lastCaptureNanos();
        boolean fresh = state == VisionPortal.CameraState.STREAMING && lastCapture != 0L
                && now - lastCapture < WATCHDOG_FRESH_FRAME_SECONDS * NANOS_PER_SECOND;
        if (!fresh) {
            healthySinceNanos = 0L;
            return;
        }
        long since = healthySinceNanos;
        if (since == 0L) {
            healthySinceNanos = now;
        } else if (watchdogRestarts != 0 && now - since > WATCHDOG_HEALTHY_RESET_SECONDS * NANOS_PER_SECOND) {
            watchdogRestarts = 0;
        }
    }

    private void restartAsync(final Rig old, final String reason) {
        if (!restartInProgress.compareAndSet(false, true)) {
            return;
        }
        lastRestartNanos = System.nanoTime();
        healthySinceNanos = 0L;
        watchdogRestarts++;
        final int attempt = watchdogRestarts;
        watchdogReason = reason + " - restarting (" + attempt + ")";
        boolean submitted = submit(new Runnable() {
            @Override
            public void run() {
                performRestart(old, reason, attempt);
            }
        });
        if (!submitted) {
            restartInProgress.set(false);
        }
    }

    private void performRestart(Rig old, String reason, int attempt) {
        try {
            if (runState == RunState.CLOSED || rig != old) {
                return;
            }
            detachStreams();
            closeStreamQuietly(old);
            if (closePortalAndWait(old.portal)) {
                releaseRig(old);
            }
            if (runState == RunState.CLOSED || rig != old) {
                return;
            }
            Rig fresh = buildRig();
            boolean adopted = false;
            synchronized (stateLock) {
                if (runState != RunState.CLOSED && rig == old) {
                    rig = fresh;
                    adopted = true;
                    fault = "";
                    applyPipelineToRig(fresh, active);
                    queueDesiredControls(true);
                }
            }
            if (adopted) {
                attachStreams(fresh);
                watchdogReason = reason + " - restarted (" + attempt + ")";
            } else {
                closeStreamQuietly(fresh);
                closePortalAsync(fresh);
            }
        } catch (Throwable t) {
            watchdogReason = "!! " + reason + " - restart failed: " + describe(t);
            recordError("watchdog restart", t);
        } finally {
            restartInProgress.set(false);
        }
    }

    private static void closeStreamQuietly(Rig target) {
        try {
            target.stream.close();
        } catch (Throwable ignored) {
        }
    }

    private static boolean closePortalAndWait(VisionPortal portal) {
        if (portal == null) {
            return false;
        }
        try {
            portal.close();
        } catch (Throwable ignored) {
        }
        long deadline = System.nanoTime() + (long) (PORTAL_CLOSE_WAIT_MS * NANOS_PER_MILLI);
        while (System.nanoTime() < deadline) {
            VisionPortal.CameraState state;
            try {
                state = portal.getCameraState();
            } catch (Throwable t) {
                return false;
            }
            if (state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
                return true;
            }
            if (state == VisionPortal.CameraState.ERROR) {
                return false;
            }
            try {
                Thread.sleep(PORTAL_CLOSE_POLL_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        return false;
    }

    private static boolean waitForPortalClosed(VisionPortal portal) {
        long deadline = System.nanoTime() + (long) (PORTAL_CLOSE_WAIT_MS * NANOS_PER_MILLI);
        while (true) {
            VisionPortal.CameraState state;
            try {
                state = portal.getCameraState();
            } catch (Throwable t) {
                return false;
            }
            if (state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
                return true;
            }
            if (System.nanoTime() >= deadline) {
                return false;
            }
            try {
                Thread.sleep(PORTAL_CLOSE_POLL_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
    }

    private static void releaseRig(Rig target) {
        if (target == null) {
            return;
        }
        try {
            target.color.release();
        } catch (Throwable ignored) {
        }
        try {
            target.stream.release();
        } catch (Throwable ignored) {
        }
    }

    private void closePortalAsync(final Rig target) {
        if (target == null || target.portal == null) {
            return;
        }
        final VisionPortal portal = target.portal;
        try {
            Thread closer = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        portal.close();
                    } catch (Throwable ignored) {
                    }
                    try {
                        if (waitForPortalClosed(portal)) {
                            releaseRig(target);
                        }
                    } catch (Throwable ignored) {
                    }
                }
            }, workerName + " close");
            closer.setDaemon(true);
            closer.start();
        } catch (Throwable t) {
            try {
                portal.close();
            } catch (Throwable ignored) {
            }
        }
    }

    private void attachStreams(Rig target) {
        if (target == null) {
            return;
        }
        if (LogiLightSettings.DRIVER_STATION_STREAM) {
            try {
                CameraStreamServer.getInstance().setSource(target.stream);
                DRIVER_STATION_OWNER.set(streamOwnerToken);
                driverStationStreaming = true;
            } catch (Throwable t) {
                driverStationStreaming = false;
                recordError("driver station stream", t);
            }
        }
        syncedPanelsWanted = LogiLightSettings.panelsStreamEnabled();
        if (syncedPanelsWanted) {
            startPanels(target);
        } else {
            panelsStreaming = false;
            panelsMessage = LogiLightSettings.EVENT_MODE ? "off (EVENT_MODE)" : "off (PANELS_STREAM = false)";
        }
    }

    private void startPanels(Rig target) {
        try {
            int fps = Math.max(1, camera.streamMaxFps);
            PanelsCameraStream.INSTANCE.startStream(target.stream, Integer.valueOf(fps));
            PANELS_OWNER.set(streamOwnerToken);
            panelsStreaming = true;
            panelsMessage = "streaming at up to " + fps + " fps";
        } catch (Throwable t) {
            panelsStreaming = false;
            panelsMessage = "!! Panels stream unavailable: " + describe(t);
        }
    }

    private void stopPanels() {
        if (PANELS_OWNER.compareAndSet(streamOwnerToken, null)) {
            try {
                PanelsCameraStream.INSTANCE.stopStream();
            } catch (Throwable ignored) {
            }
        }
        panelsStreaming = false;
        panelsMessage = "stopped";
    }

    private void detachStreams() {
        stopPanels();
        if (DRIVER_STATION_OWNER.compareAndSet(streamOwnerToken, null)) {
            try {
                CameraStreamServer.getInstance().setSource(null);
            } catch (Throwable ignored) {
            }
        }
        driverStationStreaming = false;
    }

    private void requestControls(boolean force) {
        try {
            queueDesiredControls(force);
            Rig current = rig;
            if (current != null) {
                dispatchControls(current, cameraState(current));
            }
        } catch (Throwable t) {
            recordError("camera controls", t);
        }
    }

    private void queueDesiredControls(boolean force) {
        ControlRequest desired = desiredControls();
        if (force || !desired.equals(lastQueuedControls)) {
            lastQueuedControls = desired;
            pendingControls.set(desired);
        }
    }

    private ControlRequest desiredControls() {
        LogiCameraConfig cfg = camera;
        ActivePipeline a = active;
        LogiPipeline pipeline = a == null ? null : a.pipeline;
        boolean overridesExposure = pipeline != null && pipeline.overridesExposure();
        boolean overridesGain = pipeline != null && pipeline.overridesGain();
        boolean manual = cfg.manualExposure || overridesExposure;
        double exposureMs = overridesExposure ? pipeline.exposureMs : cfg.exposureMs;
        int gain = overridesGain ? pipeline.gain : cfg.gain;
        return new ControlRequest(manual, exposureMs, gain, cfg.autoWhiteBalance, cfg.whiteBalanceKelvin,
                cfg.autoFocus, cfg.focusLength);
    }

    private String exposureSummary() {
        ControlRequest desired = desiredControls();
        ControlRequest applied = appliedControls;
        Rig current = rig;
        boolean appliedHere = applied != null && current != null && appliedControlsGeneration == current.generation;
        String state = appliedHere && desired.equals(applied) ? "applied" : "pending";
        return desired.summary() + "  (" + state + ")";
    }

    private void dispatchControls(final Rig target, VisionPortal.CameraState state) {
        if (target == null || state != VisionPortal.CameraState.STREAMING || runState == RunState.CLOSED) {
            return;
        }
        final ControlRequest request = pendingControls.get();
        if (request == null) {
            return;
        }
        if (!controlTaskRunning.compareAndSet(false, true)) {
            return;
        }
        boolean submitted = submit(new Runnable() {
            @Override
            public void run() {
                try {
                    applyControls(target, request);
                } finally {
                    controlTaskRunning.set(false);
                }
            }
        });
        if (!submitted) {
            controlTaskRunning.set(false);
        }
    }

    private void applyControls(Rig target, ControlRequest request) {
        if (rig != target) {
            return;
        }
        try {
            ControlOutcome outcome = applyControlsNow(target.portal, request);
            if (outcome.streaming) {
                appliedControls = request;
                appliedControlsGeneration = target.generation;
                pendingControls.compareAndSet(request, null);
                controlInfo = readControlInfo(target.portal);
            }
            controlStatus = outcome.message;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            controlStatus = "camera controls interrupted";
        } catch (Throwable t) {
            controlStatus = "!! camera controls failed: " + describe(t);
        }
    }

    private static ControlOutcome applyControlsNow(VisionPortal portal, ControlRequest request) throws InterruptedException {
        List<String> parts = new ArrayList<String>();
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure == null) {
                parts.add("exposure n/a");
            } else if (request.manualExposure) {
                if (exposure.getMode() != ExposureControl.Mode.Manual) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    Thread.sleep(EXPOSURE_MODE_SETTLE_MS);
                }
                long micros = Math.max(1L, Math.round(request.exposureMs * MICROS_PER_MILLI));
                long minMicros = exposure.getMinExposure(TimeUnit.MICROSECONDS);
                long maxMicros = exposure.getMaxExposure(TimeUnit.MICROSECONDS);
                if (maxMicros > 0L && maxMicros >= minMicros) {
                    micros = Math.max(Math.max(1L, minMicros), Math.min(maxMicros, micros));
                }
                boolean ok = exposure.setExposure(micros, TimeUnit.MICROSECONDS);
                Thread.sleep(CONTROL_SETTLE_MS);
                parts.add(ok ? String.format(Locale.US, "exp %.2f ms", micros / MICROS_PER_MILLI) : "!! exposure rejected");
            } else {
                ExposureControl.Mode autoMode = pickAutoExposureMode(exposure);
                if (autoMode != null && exposure.getMode() != autoMode) {
                    exposure.setMode(autoMode);
                    Thread.sleep(EXPOSURE_MODE_SETTLE_MS);
                }
                parts.add(autoMode == null ? "!! auto exposure unsupported" : "exp auto");
            }
        } catch (IllegalStateException e) {
            return new ControlOutcome(false, "waiting for camera STREAMING");
        } catch (InterruptedException e) {
            throw e;
        } catch (Throwable t) {
            parts.add("!! exposure " + describe(t));
        }
        try {
            GainControl gain = portal.getCameraControl(GainControl.class);
            if (gain == null) {
                parts.add("gain n/a");
            } else if (request.gain >= 0) {
                int minGain = gain.getMinGain();
                int maxGain = gain.getMaxGain();
                int wanted = maxGain >= minGain ? Math.max(minGain, Math.min(maxGain, request.gain)) : request.gain;
                boolean ok = gain.setGain(wanted);
                Thread.sleep(CONTROL_SETTLE_MS);
                parts.add(ok ? "gain " + wanted : "!! gain rejected");
            }
        } catch (InterruptedException e) {
            throw e;
        } catch (Throwable t) {
            parts.add("!! gain " + describe(t));
        }
        try {
            WhiteBalanceControl whiteBalance = portal.getCameraControl(WhiteBalanceControl.class);
            if (whiteBalance == null) {
                parts.add("wb n/a");
            } else if (request.autoWhiteBalance) {
                if (whiteBalance.getMode() != WhiteBalanceControl.Mode.AUTO) {
                    whiteBalance.setMode(WhiteBalanceControl.Mode.AUTO);
                    Thread.sleep(CONTROL_SETTLE_MS);
                }
                parts.add("wb auto");
            } else {
                if (whiteBalance.getMode() != WhiteBalanceControl.Mode.MANUAL) {
                    whiteBalance.setMode(WhiteBalanceControl.Mode.MANUAL);
                    Thread.sleep(CONTROL_SETTLE_MS);
                }
                int minKelvin = whiteBalance.getMinWhiteBalanceTemperature();
                int maxKelvin = whiteBalance.getMaxWhiteBalanceTemperature();
                int kelvin = maxKelvin >= minKelvin && maxKelvin > 0
                        ? Math.max(minKelvin, Math.min(maxKelvin, request.kelvin))
                        : request.kelvin;
                boolean ok = whiteBalance.setWhiteBalanceTemperature(kelvin);
                Thread.sleep(CONTROL_SETTLE_MS);
                parts.add(ok ? "wb " + kelvin + " K" : "!! wb rejected");
            }
        } catch (InterruptedException e) {
            throw e;
        } catch (Throwable t) {
            parts.add("!! wb " + describe(t));
        }
        try {
            FocusControl focus = portal.getCameraControl(FocusControl.class);
            if (focus == null) {
                parts.add("focus n/a");
            } else if (!request.autoFocus) {
                if (focus.isModeSupported(FocusControl.Mode.Fixed) && focus.getMode() != FocusControl.Mode.Fixed) {
                    focus.setMode(FocusControl.Mode.Fixed);
                    Thread.sleep(CONTROL_SETTLE_MS);
                }
                if (focus.isFocusLengthSupported()) {
                    double minFocus = focus.getMinFocusLength();
                    double maxFocus = focus.getMaxFocusLength();
                    double length = maxFocus > minFocus
                            ? Math.max(minFocus, Math.min(maxFocus, request.focusLength))
                            : request.focusLength;
                    boolean ok = focus.setFocusLength(length);
                    Thread.sleep(CONTROL_SETTLE_MS);
                    parts.add(ok ? String.format(Locale.US, "focus %.0f", length) : "!! focus rejected");
                } else {
                    parts.add("focus fixed");
                }
            } else {
                FocusControl.Mode autoMode = focus.isModeSupported(FocusControl.Mode.ContinuousAuto)
                        ? FocusControl.Mode.ContinuousAuto
                        : focus.isModeSupported(FocusControl.Mode.Auto) ? FocusControl.Mode.Auto : null;
                if (autoMode != null && focus.getMode() != autoMode) {
                    focus.setMode(autoMode);
                    Thread.sleep(CONTROL_SETTLE_MS);
                }
                parts.add(autoMode == null ? "focus auto n/a" : "focus auto");
            }
        } catch (InterruptedException e) {
            throw e;
        } catch (Throwable t) {
            parts.add("!! focus " + describe(t));
        }
        StringBuilder message = new StringBuilder();
        for (String part : parts) {
            if (message.length() > 0) {
                message.append("  ");
            }
            message.append(part);
        }
        return new ControlOutcome(true, message.toString());
    }

    private static ExposureControl.Mode pickAutoExposureMode(ExposureControl exposure) {
        ExposureControl.Mode[] preference = {
                ExposureControl.Mode.AperturePriority,
                ExposureControl.Mode.ContinuousAuto,
                ExposureControl.Mode.Auto};
        for (ExposureControl.Mode mode : preference) {
            try {
                if (exposure.isModeSupported(mode)) {
                    return mode;
                }
            } catch (Throwable ignored) {
            }
        }
        return null;
    }

    private void refreshControlInfoIfDue(final Rig target, VisionPortal.CameraState state, long now) {
        if (state != VisionPortal.CameraState.STREAMING || runState == RunState.CLOSED) {
            return;
        }
        if (now - lastControlInfoRequestNanos > CONTROL_INFO_INTEREST_MS * NANOS_PER_MILLI) {
            return;
        }
        CameraControlInfo current = controlInfo;
        if (current.readNanos != 0L && now - current.readNanos < CONTROL_INFO_REFRESH_MS * NANOS_PER_MILLI) {
            return;
        }
        if (!controlInfoReadRunning.compareAndSet(false, true)) {
            return;
        }
        boolean submitted = submit(new Runnable() {
            @Override
            public void run() {
                try {
                    if (rig == target) {
                        controlInfo = readControlInfo(target.portal);
                    }
                } catch (Throwable t) {
                    recordError("control info", t);
                } finally {
                    controlInfoReadRunning.set(false);
                }
            }
        });
        if (!submitted) {
            controlInfoReadRunning.set(false);
        }
    }

    private static CameraControlInfo readControlInfo(VisionPortal portal) {
        CameraControlInfo.Builder b = new CameraControlInfo.Builder();
        b.readNanos = System.nanoTime();
        b.available = true;
        List<String> notes = new ArrayList<String>();
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure != null) {
                b.exposureSupported = exposure.isExposureSupported();
                b.manualExposureSupported = exposure.isModeSupported(ExposureControl.Mode.Manual);
                ExposureControl.Mode mode = exposure.getMode();
                b.exposureMode = mode == null ? "" : mode.name();
                b.minExposureMs = exposure.getMinExposure(TimeUnit.MICROSECONDS) / MICROS_PER_MILLI;
                b.maxExposureMs = exposure.getMaxExposure(TimeUnit.MICROSECONDS) / MICROS_PER_MILLI;
                b.currentExposureMs = exposure.getExposure(TimeUnit.MICROSECONDS) / MICROS_PER_MILLI;
            }
        } catch (IllegalStateException e) {
            b.available = false;
            b.note = "camera not streaming";
            return b.build();
        } catch (Throwable t) {
            notes.add("exposure " + describe(t));
        }
        try {
            GainControl gain = portal.getCameraControl(GainControl.class);
            if (gain != null) {
                b.gainSupported = true;
                b.minGain = gain.getMinGain();
                b.maxGain = gain.getMaxGain();
                b.currentGain = gain.getGain();
            }
        } catch (Throwable t) {
            notes.add("gain " + describe(t));
        }
        try {
            WhiteBalanceControl whiteBalance = portal.getCameraControl(WhiteBalanceControl.class);
            if (whiteBalance != null) {
                b.whiteBalanceSupported = true;
                WhiteBalanceControl.Mode mode = whiteBalance.getMode();
                b.whiteBalanceMode = mode == null ? "" : mode.name();
                b.minKelvin = whiteBalance.getMinWhiteBalanceTemperature();
                b.maxKelvin = whiteBalance.getMaxWhiteBalanceTemperature();
                b.currentKelvin = whiteBalance.getWhiteBalanceTemperature();
            }
        } catch (Throwable t) {
            notes.add("wb " + describe(t));
        }
        try {
            FocusControl focus = portal.getCameraControl(FocusControl.class);
            if (focus != null) {
                b.focusSupported = focus.isFocusLengthSupported() || focus.isModeSupported(FocusControl.Mode.Fixed);
                FocusControl.Mode mode = focus.getMode();
                b.focusMode = mode == null ? "" : mode.name();
                b.minFocus = focus.getMinFocusLength();
                b.maxFocus = focus.getMaxFocusLength();
                b.currentFocus = focus.getFocusLength();
            }
        } catch (Throwable t) {
            notes.add("focus " + describe(t));
        }
        if (!notes.isEmpty()) {
            StringBuilder note = new StringBuilder("!! ");
            for (String part : notes) {
                if (note.length() > 3) {
                    note.append("  ");
                }
                note.append(part);
            }
            b.note = note.toString();
        }
        return b.build();
    }

    private void applyDecimation(Rig target, ActivePipeline a, boolean force) {
        if (target == null) {
            return;
        }
        double wanted = desiredDecimation(a);
        if (force || Double.compare(wanted, target.appliedDecimation) != 0) {
            try {
                target.aprilTag.setDecimation((float) wanted);
                target.appliedDecimation = wanted;
            } catch (Throwable t) {
                recordError("decimation", t);
            }
        }
    }

    private double desiredDecimation(ActivePipeline a) {
        LogiPipeline pipeline = a == null ? null : a.pipeline;
        double value = pipeline != null && pipeline.overridesDecimation() ? pipeline.decimation : camera.decimation;
        if (!isFinite(value) || value <= 0.0) {
            value = DEFAULT_DECIMATION;
        }
        return clamp(value, MIN_DECIMATION, MAX_DECIMATION);
    }

    private boolean submit(Runnable task) {
        ExecutorService executor = worker;
        if (executor == null || executor.isShutdown()) {
            return false;
        }
        try {
            executor.execute(task);
            return true;
        } catch (Throwable t) {
            return false;
        }
    }

    private void recordError(String where, Throwable t) {
        try {
            context.reportError(where, t);
        } catch (Throwable ignored) {
        }
    }

    private static VisionPortal.CameraState cameraState(Rig target) {
        if (target == null) {
            return null;
        }
        try {
            return target.portal.getCameraState();
        } catch (Throwable t) {
            return VisionPortal.CameraState.ERROR;
        }
    }

    private static double portalFps(VisionPortal portal) {
        try {
            return portal.getFps();
        } catch (Throwable t) {
            return 0.0;
        }
    }

    private static String loadSettingsSafely() {
        try {
            if (!LogiLightSettings.LOAD_SAVED_SETTINGS) {
                return "using code values (LOAD_SAVED_SETTINGS = false)";
            }
            return LogiLightStore.loadSettingsOnce();
        } catch (Throwable t) {
            return "!! settings not loaded: " + describe(t);
        }
    }

    private static LogiFieldMap loadFieldMapSafely() {
        try {
            LogiFieldMap loaded = LogiLightStore.loadFieldMap();
            if (loaded != null) {
                return loaded;
            }
        } catch (Throwable ignored) {
        }
        try {
            return LogiFieldMap.bioBuzzDefaults();
        } catch (Throwable t) {
            return new LogiFieldMap(null, "!! no field map: " + describe(t));
        }
    }

    private static LogiFrameContext newContext(LogiCameraConfig config, LogiCameraConfig build) {
        try {
            return new LogiFrameContext(config, build);
        } catch (RuntimeException e) {
            return new LogiFrameContext(new LogiCameraConfig(), build);
        }
    }

    private static LogiCameraConfig copyOf(LogiCameraConfig config) {
        try {
            return config.copy();
        } catch (Throwable t) {
            return new LogiCameraConfig();
        }
    }

    private static ExecutorService newWorker(final String name) {
        return Executors.newSingleThreadExecutor(new ThreadFactory() {
            @Override
            public Thread newThread(Runnable runnable) {
                Thread thread = new Thread(runnable, name + " camera");
                thread.setDaemon(true);
                return thread;
            }
        });
    }

    private static double[] mountValues(LogiCameraConfig cfg) {
        return new double[]{cfg.forwardInches, cfg.leftInches, cfg.upInches,
                cfg.yawLeftDegrees, cfg.tiltUpDegrees, cfg.rollClockwiseDegrees,
                cfg.crosshairX, cfg.crosshairY};
    }

    private static boolean sameElements(Object[] a, Object[] b) {
        if (a == b) {
            return true;
        }
        if (a == null || b == null || a.length != b.length) {
            return false;
        }
        for (int i = 0; i < a.length; i++) {
            if (a[i] != b[i]) {
                return false;
            }
        }
        return true;
    }

    private static String snapshotName(String name) {
        String base = name == null ? "" : name.trim().replaceAll("[^A-Za-z0-9_-]+", "_");
        return base.isEmpty() ? "snap" : base;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    static String describe(Throwable t) {
        if (t == null) {
            return "unknown error";
        }
        String message = t.getMessage();
        String type = t.getClass().getSimpleName();
        return message == null || message.isEmpty() ? type : type + ": " + message;
    }

    private static final class Rig {
        final int generation;
        final VisionPortal portal;
        final AprilTagProcessor aprilTag;
        final ColorTargetProcessor color;
        final ScriptProcessor script;
        final LogiLightStream stream;
        volatile long streamingSinceNanos = 0L;
        volatile double appliedDecimation = Double.NaN;

        Rig(int generation, VisionPortal portal, AprilTagProcessor aprilTag, ColorTargetProcessor color,
            ScriptProcessor script, LogiLightStream stream) {
            this.generation = generation;
            this.portal = portal;
            this.aprilTag = aprilTag;
            this.color = color;
            this.script = script;
            this.stream = stream;
        }
    }

    private static final class ActivePipeline {
        final int index;
        final LogiPipeline pipeline;
        final PipelineType type;
        final ColorProfile[] profiles;
        final LogiScript script;
        final long sinceNanos;
        final ActivePipeline previous;

        ActivePipeline(int index, LogiPipeline pipeline, ColorProfile[] profiles, LogiScript script,
                       long sinceNanos, ActivePipeline previous) {
            this.index = index;
            this.pipeline = pipeline;
            this.type = pipeline.typeOrDefault();
            this.profiles = profiles == null ? new ColorProfile[0] : profiles;
            this.script = script;
            this.sinceNanos = sinceNanos;
            this.previous = previous;
        }

        ActivePipeline detached() {
            return previous == null ? this : new ActivePipeline(index, pipeline, profiles, script, sinceNanos, null);
        }
    }

    private static final class ControlRequest {
        final boolean manualExposure;
        final double exposureMs;
        final int gain;
        final boolean autoWhiteBalance;
        final int kelvin;
        final boolean autoFocus;
        final double focusLength;

        ControlRequest(boolean manualExposure, double exposureMs, int gain, boolean autoWhiteBalance, int kelvin,
                       boolean autoFocus, double focusLength) {
            this.manualExposure = manualExposure;
            this.exposureMs = exposureMs;
            this.gain = gain;
            this.autoWhiteBalance = autoWhiteBalance;
            this.kelvin = kelvin;
            this.autoFocus = autoFocus;
            this.focusLength = focusLength;
        }

        String summary() {
            String exposure = manualExposure
                    ? String.format(Locale.US, "exp %.2f ms  gain %d", exposureMs, gain)
                    : String.format(Locale.US, "exp auto  gain %d", gain);
            String whiteBalance = autoWhiteBalance ? "wb auto" : "wb " + kelvin + " K";
            String focus = autoFocus ? "focus auto" : String.format(Locale.US, "focus %.0f", focusLength);
            return exposure + "  " + whiteBalance + "  " + focus;
        }

        @Override
        public boolean equals(Object other) {
            if (this == other) {
                return true;
            }
            if (!(other instanceof ControlRequest)) {
                return false;
            }
            ControlRequest o = (ControlRequest) other;
            return manualExposure == o.manualExposure
                    && Double.compare(exposureMs, o.exposureMs) == 0
                    && gain == o.gain
                    && autoWhiteBalance == o.autoWhiteBalance
                    && kelvin == o.kelvin
                    && autoFocus == o.autoFocus
                    && Double.compare(focusLength, o.focusLength) == 0;
        }

        @Override
        public int hashCode() {
            long bits = Double.doubleToLongBits(exposureMs) * 31L + Double.doubleToLongBits(focusLength);
            int result = (int) (bits ^ (bits >>> 32));
            result = 31 * result + gain;
            result = 31 * result + kelvin;
            result = 31 * result + (manualExposure ? 1 : 0);
            result = 31 * result + (autoWhiteBalance ? 1 : 0);
            result = 31 * result + (autoFocus ? 1 : 0);
            return result;
        }
    }

    private static final class ControlOutcome {
        final boolean streaming;
        final String message;

        ControlOutcome(boolean streaming, String message) {
            this.streaming = streaming;
            this.message = message;
        }
    }

    private static final class AcceptedPose {
        final PlanarPose pose;
        final String name;
        final double distance;
        final double area;
        final Vec3 origin;
        final long captureNanos;
        final double sigmaXY;
        final double sigmaHeading;

        AcceptedPose(PlanarPose pose, String name, double distance, double area, Vec3 origin, long captureNanos,
                     double sigmaXY, double sigmaHeading) {
            this.pose = pose;
            this.name = name;
            this.distance = distance;
            this.area = area;
            this.origin = origin;
            this.captureNanos = captureNanos;
            this.sigmaXY = sigmaXY;
            this.sigmaHeading = sigmaHeading;
        }
    }

    private static final class FiducialBatch {
        final List<LogiFiducial> fiducials = new ArrayList<LogiFiducial>();
        final List<AcceptedPose> mt1 = new ArrayList<AcceptedPose>();
        final List<AcceptedPose> mt2 = new ArrayList<AcceptedPose>();
    }
}
