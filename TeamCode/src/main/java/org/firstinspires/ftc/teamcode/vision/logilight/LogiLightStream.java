package org.firstinspires.ftc.teamcode.vision.logilight;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public final class LogiLightStream implements VisionProcessor, CameraStreamSource {

    public static final int PLACEHOLDER_WIDTH = 160;
    public static final int PLACEHOLDER_HEIGHT = 120;
    public static final int MIN_STREAM_WIDTH = 64;
    public static final int MAX_STREAM_FPS = 30;
    public static final long VIEWER_TIMEOUT_MS = 3000L;
    public static final long IDLE_RENDER_INTERVAL_MS = 300L;
    public static final long SCRIPT_TARGET_MAX_AGE_MS = 1000L;
    public static final String SNAPSHOT_EXTENSION = ".png";
    public static final String SNAPSHOT_TIME_PATTERN = "yyyyMMdd_HHmmss";
    public static final String DEFAULT_SNAPSHOT_NAME = "snapshot";
    public static final int SNAPSHOT_QUEUE_LIMIT = 3;
    public static final int MAX_SNAPSHOT_NAME_CHARS = 48;
    public static final int MAX_ERROR_TEXT_CHARS = 60;

    private static final int COLOR_CROSSHAIR = 0xFF00FF00;
    private static final int COLOR_TARGET = 0xFFFFFF00;
    private static final int COLOR_PRIMARY = 0xFF00FFFF;
    private static final int COLOR_FIDUCIAL = 0xFFFF00FF;
    private static final int COLOR_LOCALIZED = 0xFF00FF00;
    private static final int COLOR_TEXT = 0xFFFFFFFF;
    private static final int COLOR_SHADOW = 0xFF000000;
    private static final int COLOR_BLACK = 0xFF000000;
    private static final double PRIMARY_MATCH_DEGREES = 0.05;
    private static final double FPS_INTERVAL_TOLERANCE = 0.85;
    private static final long NANOS_PER_MS = 1000000L;
    private static final double NANOS_PER_SECOND = 1.0e9;
    private static final long SNAPSHOT_THREAD_KEEP_ALIVE_SECONDS = 10L;
    private static final int MAX_UNIQUE_SUFFIX = 100;

    private static final ThreadFactory SNAPSHOT_THREADS = new ThreadFactory() {
        private final AtomicInteger counter = new AtomicInteger();

        @Override
        public Thread newThread(Runnable runnable) {
            Thread thread = new Thread(runnable, "LogiLight-snapshot-" + counter.incrementAndGet());
            thread.setDaemon(true);
            thread.setPriority(Thread.NORM_PRIORITY - 1);
            return thread;
        }
    };

    private final LogiFrameContext context;
    private final ColorTargetProcessor color;
    private final ScriptProcessor script;
    private volatile LogiOverlaySource overlay;

    private final AtomicReference<Bitmap> latestBitmap = new AtomicReference<Bitmap>();
    private final AtomicReference<SnapshotRequest> pendingSnapshot = new AtomicReference<SnapshotRequest>();
    private final AtomicInteger snapshotCount = new AtomicInteger();
    private final Object executorLock = new Object();
    private final Object placeholderLock = new Object();
    private final Paint linePaint;
    private final Paint fillPaint;
    private final Paint textPaint;
    private final Path polygon = new Path();
    private final List<String> textLines = new ArrayList<String>();

    private volatile long lastViewerRequestNanos;
    private volatile String lastSnapshotPath = "";
    private volatile String lastSnapshotError = "";
    private volatile boolean closed;
    private long lastRenderNanos;
    private StreamView lastRenderedView;
    private ThreadPoolExecutor snapshotExecutor;
    private Bitmap placeholder;
    private Mat streamMat;
    private Mat sampleScratch;

    public LogiLightStream(LogiFrameContext context, ColorTargetProcessor color, ScriptProcessor script,
                           LogiOverlaySource overlay) {
        this.context = context == null ? new LogiFrameContext(null) : context;
        this.color = color;
        this.script = script;
        this.overlay = overlay;
        this.linePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.linePaint.setStyle(Paint.Style.STROKE);
        this.fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.fillPaint.setStyle(Paint.Style.FILL);
        this.textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.textPaint.setStyle(Paint.Style.FILL);
    }

    public LogiFrameContext context() {
        return context;
    }

    public void requestSnapshot(String name) {
        requestSnapshot(name, true);
    }

    public void requestSnapshot(String name, boolean annotated) {
        if (closed) {
            lastSnapshotError = "stream closed";
            return;
        }
        pendingSnapshot.set(new SnapshotRequest(sanitizeName(name), annotated));
    }

    public boolean isSnapshotPending() {
        return pendingSnapshot.get() != null;
    }

    public String lastSnapshotPath() {
        return lastSnapshotPath;
    }

    public String lastSnapshotError() {
        return lastSnapshotError;
    }

    public int snapshotCount() {
        return snapshotCount.get();
    }

    public Bitmap latestBitmap() {
        lastViewerRequestNanos = System.nanoTime();
        return latestBitmap.get();
    }

    public boolean hasViewers() {
        long last = lastViewerRequestNanos;
        return last != 0L && System.nanoTime() - last <= VIEWER_TIMEOUT_MS * NANOS_PER_MS;
    }

    public void close() {
        closed = true;
        overlay = null;
        pendingSnapshot.set(null);
        synchronized (executorLock) {
            if (snapshotExecutor != null) {
                snapshotExecutor.shutdown();
                snapshotExecutor = null;
            }
        }
        latestBitmap.set(null);
        synchronized (placeholderLock) {
            placeholder = null;
        }
    }

    void release() {
        Mat stream = streamMat;
        streamMat = null;
        if (stream != null) {
            try {
                stream.release();
            } catch (Throwable ignored) {
            }
        }
        Mat scratch = sampleScratch;
        sampleScratch = null;
        if (scratch != null) {
            try {
                scratch.release();
            } catch (Throwable ignored) {
            }
        }
        latestBitmap.set(null);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        if (continuation == null) {
            return;
        }
        try {
            lastViewerRequestNanos = System.nanoTime();
            Bitmap current = latestBitmap.get();
            final Bitmap bitmap = current != null ? current : placeholder();
            continuation.dispatch(new ContinuationResult<Consumer<Bitmap>>() {
                @Override
                public void handle(Consumer<Bitmap> consumer) {
                    if (consumer != null) {
                        consumer.accept(bitmap);
                    }
                }
            });
        } catch (Throwable t) {
            context.reportError("stream request", t);
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        try {
            context.onInit(width, height, calibration);
        } catch (Throwable t) {
            context.reportError("stream init", t);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        try {
            context.markFrame(captureTimeNanos);
            context.markProcessingDone(captureTimeNanos);
            if (frame == null || frame.empty() || closed) {
                return null;
            }
            serviceColorSample(frame, captureTimeNanos);
            SnapshotRequest snapshot = pendingSnapshot.getAndSet(null);
            long now = System.nanoTime();
            boolean streamDue = isStreamDue(now);
            if (!streamDue && snapshot == null) {
                return null;
            }
            LogiResult result = overlayResult();
            if (streamDue) {
                lastRenderNanos = now;
                renderStream(frame, captureTimeNanos, result);
            }
            if (snapshot != null) {
                renderSnapshot(frame, result, snapshot);
            }
        } catch (Throwable t) {
            context.reportError("stream", t);
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    private boolean isStreamDue(long now) {
        if (latestBitmap.get() == null) {
            return true;
        }
        if (context.effectiveStreamView() != lastRenderedView) {
            return true;
        }
        long sinceRender = now - lastRenderNanos;
        long lastViewer = lastViewerRequestNanos;
        boolean watched = lastViewer != 0L && now - lastViewer <= VIEWER_TIMEOUT_MS * NANOS_PER_MS;
        if (!watched) {
            return sinceRender >= IDLE_RENDER_INTERVAL_MS * NANOS_PER_MS;
        }
        int fps = Math.max(1, Math.min(MAX_STREAM_FPS, context.camera().streamMaxFps));
        double interval = NANOS_PER_SECOND / fps;
        return sinceRender >= interval * FPS_INTERVAL_TOLERANCE;
    }

    private LogiResult overlayResult() {
        LogiOverlaySource source = overlay;
        if (source == null) {
            return null;
        }
        try {
            return source.overlayResult();
        } catch (Throwable t) {
            context.reportError("overlay result", t);
            return null;
        }
    }

    private void renderStream(Mat frame, long captureNanos, LogiResult result) {
        int frameWidth = frame.cols();
        int frameHeight = frame.rows();
        int width = streamWidth(frameWidth);
        int height = Math.max(1, (int) Math.round(width * (double) frameHeight / frameWidth));
        StreamView view = context.effectiveStreamView();
        lastRenderedView = view;
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        boolean thresholdShown = false;
        if (view == StreamView.THRESHOLD) {
            Mat threshold = color == null ? null : color.latestThreshold(captureNanos);
            if (threshold != null) {
                copyScaled(threshold, bitmap, width, height, Imgproc.INTER_NEAREST);
                thresholdShown = true;
            } else {
                bitmap.eraseColor(COLOR_BLACK);
            }
        } else {
            copyScaled(frame, bitmap, width, height, Imgproc.INTER_AREA);
        }
        if (view != StreamView.RAW) {
            Canvas canvas = new Canvas(bitmap);
            drawOverlays(canvas, result, width / (double) frameWidth, height / (double) frameHeight, view, thresholdShown);
        }
        latestBitmap.set(bitmap);
    }

    private void copyScaled(Mat source, Bitmap bitmap, int width, int height, int interpolation) {
        if (source.cols() == width && source.rows() == height) {
            Utils.matToBitmap(source, bitmap);
            return;
        }
        if (streamMat == null) {
            streamMat = new Mat();
        }
        Imgproc.resize(source, streamMat, new Size(width, height), 0.0, 0.0, interpolation);
        Utils.matToBitmap(streamMat, bitmap);
    }

    private int streamWidth(int frameWidth) {
        int configured = context.camera().streamWidth;
        if (configured <= 0) {
            return frameWidth;
        }
        int minimum = Math.min(MIN_STREAM_WIDTH, frameWidth);
        return Math.max(minimum, Math.min(frameWidth, configured));
    }

    private void renderSnapshot(Mat frame, LogiResult result, SnapshotRequest request) {
        Bitmap bitmap = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, bitmap);
        if (request.annotated) {
            drawOverlays(new Canvas(bitmap), result, 1.0, 1.0, StreamView.ANNOTATED, false);
        }
        String stamp = new SimpleDateFormat(SNAPSHOT_TIME_PATTERN, Locale.US).format(new Date());
        submitSnapshot(bitmap, request.name, stamp);
    }

    private void submitSnapshot(final Bitmap bitmap, final String name, final String stamp) {
        ThreadPoolExecutor executor = snapshotExecutor();
        if (executor == null) {
            bitmap.recycle();
            lastSnapshotError = "stream closed";
            return;
        }
        try {
            executor.execute(new Runnable() {
                @Override
                public void run() {
                    writeSnapshot(bitmap, name, stamp);
                }
            });
        } catch (RejectedExecutionException e) {
            bitmap.recycle();
            lastSnapshotError = "snapshot queue full, dropped " + name;
            context.reportError("snapshot", lastSnapshotError);
        }
    }

    private ThreadPoolExecutor snapshotExecutor() {
        synchronized (executorLock) {
            if (closed) {
                return null;
            }
            if (snapshotExecutor == null || snapshotExecutor.isShutdown()) {
                ThreadPoolExecutor executor = new ThreadPoolExecutor(1, 1,
                        SNAPSHOT_THREAD_KEEP_ALIVE_SECONDS, TimeUnit.SECONDS,
                        new ArrayBlockingQueue<Runnable>(SNAPSHOT_QUEUE_LIMIT), SNAPSHOT_THREADS);
                executor.allowCoreThreadTimeOut(true);
                snapshotExecutor = executor;
            }
            return snapshotExecutor;
        }
    }

    private void writeSnapshot(Bitmap bitmap, String name, String stamp) {
        File file = null;
        try {
            File directory = LogiLightStore.snapshotDirectory();
            if (!directory.isDirectory() && !directory.mkdirs() && !directory.isDirectory()) {
                lastSnapshotError = "cannot create " + directory.getPath();
                context.reportError("snapshot", lastSnapshotError);
                return;
            }
            file = uniqueFile(directory, name + "_" + stamp);
            boolean encoded;
            OutputStream output = new BufferedOutputStream(new FileOutputStream(file));
            try {
                encoded = bitmap.compress(Bitmap.CompressFormat.PNG, 100, output);
                output.flush();
            } finally {
                output.close();
            }
            if (!encoded) {
                lastSnapshotError = "PNG encoding failed for " + file.getName();
                context.reportError("snapshot", lastSnapshotError);
                if (!file.delete()) {
                    file.deleteOnExit();
                }
                return;
            }
            lastSnapshotPath = file.getAbsolutePath();
            lastSnapshotError = "";
            snapshotCount.incrementAndGet();
        } catch (Throwable t) {
            lastSnapshotError = LogiFrameContext.describe(file == null ? "snapshot" : file.getName(), t);
            context.reportError("snapshot", t);
        } finally {
            bitmap.recycle();
        }
    }

    private static File uniqueFile(File directory, String baseName) {
        File candidate = new File(directory, baseName + SNAPSHOT_EXTENSION);
        for (int suffix = 2; candidate.exists() && suffix <= MAX_UNIQUE_SUFFIX; suffix++) {
            candidate = new File(directory, baseName + "_" + suffix + SNAPSHOT_EXTENSION);
        }
        return candidate;
    }

    static String sanitizeName(String name) {
        String trimmed = name == null ? "" : name.trim();
        if (trimmed.isEmpty()) {
            return DEFAULT_SNAPSHOT_NAME;
        }
        StringBuilder builder = new StringBuilder(Math.min(trimmed.length(), MAX_SNAPSHOT_NAME_CHARS));
        for (int i = 0; i < trimmed.length() && builder.length() < MAX_SNAPSHOT_NAME_CHARS; i++) {
            char c = trimmed.charAt(i);
            boolean safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
            builder.append(safe ? c : '_');
        }
        return builder.toString();
    }

    private void serviceColorSample(Mat frame, long captureNanos) {
        LogiFrameContext.ColorSampleRequest request = context.takeColorSampleRequest();
        if (request == null) {
            return;
        }
        try {
            double u = request.u;
            double v = request.v;
            if (!isFinite(u) || !isFinite(v)) {
                CameraModel model = context.cameraModel();
                if (model != null) {
                    double[] crosshair = model.crosshairPixel();
                    u = crosshair[0];
                    v = crosshair[1];
                } else {
                    u = frame.cols() / 2.0;
                    v = frame.rows() / 2.0;
                }
            }
            if (sampleScratch == null) {
                sampleScratch = new Mat();
            }
            context.publishColorSample(ColorSample.measure(frame, request.space, u, v, request.radiusPx,
                    captureNanos, sampleScratch));
        } catch (Throwable t) {
            context.reportError("color sample", t);
        }
    }

    private void drawOverlays(Canvas canvas, LogiResult result, double scaleX, double scaleY,
                              StreamView view, boolean thresholdShown) {
        int width = canvas.getWidth();
        float stroke = Math.max(1.0f, width / 240.0f);
        float textSize = Math.max(9.0f, width / 26.0f);
        List<ColorTarget> colorTargets = colorTargets(result);
        List<LogiFiducial> fiducials = fiducials(result);
        Object primary = findPrimary(result, colorTargets, fiducials);
        if (view == StreamView.ANNOTATED) {
            drawCrosshair(canvas, scaleX, scaleY, stroke);
        }
        for (int i = 0; i < fiducials.size(); i++) {
            LogiFiducial fiducial = fiducials.get(i);
            if (fiducial != null) {
                drawFiducial(canvas, fiducial, fiducial == primary, scaleX, scaleY, stroke, textSize);
            }
        }
        for (int i = 0; i < colorTargets.size(); i++) {
            ColorTarget target = colorTargets.get(i);
            if (target != null) {
                drawColorTarget(canvas, target, target == primary, scaleX, scaleY, stroke);
            }
        }
        drawScriptTarget(canvas, scaleX, scaleY, stroke);
        drawTextLines(canvas, result, view, thresholdShown, textSize);
    }

    private List<ColorTarget> colorTargets(LogiResult result) {
        if (result != null) {
            List<ColorTarget> targets = result.getColorResults();
            return targets == null ? Collections.<ColorTarget>emptyList() : targets;
        }
        if (color != null && context.activePipelineType().usesColor()) {
            return color.latest().targets;
        }
        return Collections.emptyList();
    }

    private static List<LogiFiducial> fiducials(LogiResult result) {
        if (result == null) {
            return Collections.emptyList();
        }
        List<LogiFiducial> fiducials = result.getFiducialResults();
        return fiducials == null ? Collections.<LogiFiducial>emptyList() : fiducials;
    }

    private static Object findPrimary(LogiResult result, List<ColorTarget> colorTargets, List<LogiFiducial> fiducials) {
        if (result == null) {
            return colorTargets.isEmpty() ? null : colorTargets.get(0);
        }
        if (!result.isValid()) {
            return null;
        }
        double tx = result.getTx();
        double ty = result.getTy();
        Object best = null;
        double bestError = PRIMARY_MATCH_DEGREES;
        for (int i = 0; i < fiducials.size(); i++) {
            LogiFiducial fiducial = fiducials.get(i);
            if (fiducial == null) {
                continue;
            }
            double error = Math.abs(fiducial.tx - tx) + Math.abs(fiducial.ty - ty);
            if (error <= bestError) {
                best = fiducial;
                bestError = error;
            }
        }
        for (int i = 0; i < colorTargets.size(); i++) {
            ColorTarget target = colorTargets.get(i);
            if (target == null) {
                continue;
            }
            double error = Math.abs(target.tx - tx) + Math.abs(target.ty - ty);
            if (error < bestError || (best == null && error <= bestError)) {
                best = target;
                bestError = error;
            }
        }
        return best;
    }

    private void drawCrosshair(Canvas canvas, double scaleX, double scaleY, float stroke) {
        CameraModel model = context.cameraModel();
        if (model == null) {
            return;
        }
        double[] crosshair = model.crosshairPixel();
        float x = toCanvas(crosshair[0], scaleX);
        float y = toCanvas(crosshair[1], scaleY);
        float arm = Math.max(6.0f, canvas.getWidth() / 24.0f);
        linePaint.setColor(COLOR_CROSSHAIR);
        linePaint.setStrokeWidth(stroke);
        canvas.drawLine(x - arm, y, x + arm, y, linePaint);
        canvas.drawLine(x, y - arm, x, y + arm, linePaint);
    }

    private void drawColorTarget(Canvas canvas, ColorTarget target, boolean primary,
                                 double scaleX, double scaleY, float stroke) {
        int lineColor = primary ? COLOR_PRIMARY : COLOR_TARGET;
        float lineWidth = primary ? stroke * 2.0f : stroke;
        drawQuad(canvas, target.corners, scaleX, scaleY, lineColor, lineWidth);
        fillPaint.setColor(lineColor);
        canvas.drawCircle(toCanvas(target.centerX, scaleX), toCanvas(target.centerY, scaleY), lineWidth * 1.5f, fillPaint);
    }

    private void drawFiducial(Canvas canvas, LogiFiducial fiducial, boolean primary,
                              double scaleX, double scaleY, float stroke, float textSize) {
        int lineColor = fiducial.usedForLocalization ? COLOR_LOCALIZED : COLOR_FIDUCIAL;
        float lineWidth = primary ? stroke * 2.0f : stroke;
        List<double[]> quads = fiducial.corners;
        if (quads != null) {
            for (int i = 0; i < quads.size(); i++) {
                drawQuad(canvas, quads.get(i), scaleX, scaleY, lineColor, lineWidth);
            }
        }
        if (!isFinite(fiducial.centerX) || !isFinite(fiducial.centerY)) {
            return;
        }
        float x = toCanvas(fiducial.centerX, scaleX);
        float y = toCanvas(fiducial.centerY, scaleY);
        fillPaint.setColor(lineColor);
        canvas.drawCircle(x, y, lineWidth * 1.5f, fillPaint);
        if (primary) {
            linePaint.setColor(COLOR_PRIMARY);
            linePaint.setStrokeWidth(lineWidth);
            canvas.drawCircle(x, y, Math.max(5.0f, textSize * 0.8f), linePaint);
        }
        if (fiducial.name != null && !fiducial.name.isEmpty()) {
            drawShadowedText(canvas, fiducial.name, x + textSize * 0.6f, y - textSize * 0.6f, textSize * 0.9f, lineColor);
        }
    }

    private void drawScriptTarget(Canvas canvas, double scaleX, double scaleY, float stroke) {
        if (script == null || !context.activePipelineType().usesScript()) {
            return;
        }
        ScriptFrame frame = script.latest();
        LogiScriptResult result = frame.result;
        if (!result.hasTarget || frame.ageMs() > SCRIPT_TARGET_MAX_AGE_MS) {
            return;
        }
        float left = toCanvas(result.targetLeft(), scaleX);
        float top = toCanvas(result.targetTop(), scaleY);
        float right = toCanvas(result.targetLeft() + result.targetWidth, scaleX);
        float bottom = toCanvas(result.targetTop() + result.targetHeight, scaleY);
        linePaint.setColor(COLOR_PRIMARY);
        linePaint.setStrokeWidth(stroke * 2.0f);
        canvas.drawRect(left, top, right, bottom, linePaint);
        fillPaint.setColor(COLOR_PRIMARY);
        canvas.drawCircle(toCanvas(result.targetX, scaleX), toCanvas(result.targetY, scaleY), stroke * 3.0f, fillPaint);
    }

    private void drawQuad(Canvas canvas, double[] corners, double scaleX, double scaleY, int lineColor, float lineWidth) {
        if (corners == null || corners.length < 8) {
            return;
        }
        for (int i = 0; i < 8; i++) {
            if (!isFinite(corners[i])) {
                return;
            }
        }
        polygon.reset();
        polygon.moveTo(toCanvas(corners[0], scaleX), toCanvas(corners[1], scaleY));
        polygon.lineTo(toCanvas(corners[2], scaleX), toCanvas(corners[3], scaleY));
        polygon.lineTo(toCanvas(corners[4], scaleX), toCanvas(corners[5], scaleY));
        polygon.lineTo(toCanvas(corners[6], scaleX), toCanvas(corners[7], scaleY));
        polygon.close();
        linePaint.setColor(lineColor);
        linePaint.setStrokeWidth(lineWidth);
        canvas.drawPath(polygon, linePaint);
    }

    private void drawTextLines(Canvas canvas, LogiResult result, StreamView view, boolean thresholdShown, float textSize) {
        textLines.clear();
        int pipelineIndex = result != null ? result.getPipelineIndex() : context.activePipelineIndex();
        String viewTag = view == StreamView.ANNOTATED ? "" : "  [" + view.name() + "]";
        textLines.add(String.format(Locale.US, "P%d %s%s", pipelineIndex, context.activePipelineName(), viewTag));
        textLines.add(String.format(Locale.US, "fps %.0f  lat %.0f ms", context.measuredFps(), context.averageProcessingMs()));
        if (result != null && result.isValid()) {
            textLines.add(String.format(Locale.US, "tx %.1f ty %.1f ta %.2f", result.getTx(), result.getTy(), result.getTa()));
        } else {
            textLines.add("no target");
        }
        if (result != null) {
            LogiBotPose megaTag2 = result.getBotpose_MT2();
            if (megaTag2 != null && megaTag2.pose != null) {
                textLines.add("MT2 " + formatPose(megaTag2.pose));
            }
            LogiBotPose megaTag1 = result.getBotpose();
            if (megaTag1 != null && megaTag1.pose != null) {
                textLines.add("MT1 " + formatPose(megaTag1.pose));
            }
        }
        if (view == StreamView.THRESHOLD && !thresholdShown) {
            textLines.add("no threshold: pick a color pipeline");
        }
        if (script != null && context.activePipelineType().usesScript()) {
            ScriptFrame frame = script.latest();
            if (frame.hasError()) {
                textLines.add("!! " + truncate(frame.error));
            } else {
                LogiScriptResult scriptResult = frame.result;
                textLines.add(String.format(Locale.US, "out %.1f %.1f %.1f %.1f",
                        scriptResult.output(0), scriptResult.output(1), scriptResult.output(2), scriptResult.output(3)));
            }
        }
        if (context.hasRecentError()) {
            textLines.add("!! " + truncate(context.lastError()));
        }
        float lineHeight = textSize * 1.2f;
        float x = Math.max(2.0f, textSize * 0.3f);
        for (int i = 0; i < textLines.size(); i++) {
            drawShadowedText(canvas, textLines.get(i), x, lineHeight * (i + 1), textSize, COLOR_TEXT);
        }
    }

    private void drawShadowedText(Canvas canvas, String text, float x, float y, float size, int textColor) {
        float offset = Math.max(1.0f, size / 12.0f);
        textPaint.setTextSize(size);
        textPaint.setColor(COLOR_SHADOW);
        canvas.drawText(text, x + offset, y + offset, textPaint);
        textPaint.setColor(textColor);
        canvas.drawText(text, x, y, textPaint);
    }

    private Bitmap placeholder() {
        synchronized (placeholderLock) {
            if (placeholder == null) {
                Bitmap bitmap = Bitmap.createBitmap(PLACEHOLDER_WIDTH, PLACEHOLDER_HEIGHT, Bitmap.Config.ARGB_8888);
                bitmap.eraseColor(COLOR_BLACK);
                try {
                    Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
                    paint.setColor(COLOR_TEXT);
                    paint.setTextSize(12.0f);
                    Canvas canvas = new Canvas(bitmap);
                    canvas.drawText("LogiLight", 8.0f, 52.0f, paint);
                    canvas.drawText("waiting for camera", 8.0f, 70.0f, paint);
                } catch (Throwable ignored) {
                }
                placeholder = bitmap;
            }
            return placeholder;
        }
    }

    private static String formatPose(PlanarPose pose) {
        String heading = pose.hasHeading() ? String.format(Locale.US, "%.1f", pose.headingDegrees()) : "--";
        return String.format(Locale.US, "x %.1f  y %.1f  h %s", pose.x, pose.y, heading);
    }

    private static String truncate(String text) {
        if (text == null) {
            return "";
        }
        return text.length() <= MAX_ERROR_TEXT_CHARS ? text : text.substring(0, MAX_ERROR_TEXT_CHARS - 3) + "...";
    }

    private static float toCanvas(double framePixel, double scale) {
        return (float) ((framePixel + 0.5) * scale);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static final class SnapshotRequest {
        final String name;
        final boolean annotated;

        SnapshotRequest(String name, boolean annotated) {
            this.name = name;
            this.annotated = annotated;
        }
    }
}
