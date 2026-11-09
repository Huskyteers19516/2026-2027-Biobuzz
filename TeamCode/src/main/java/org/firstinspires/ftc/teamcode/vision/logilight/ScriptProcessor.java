package org.firstinspires.ftc.teamcode.vision.logilight;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

public final class ScriptProcessor implements VisionProcessor {

    private static final double NANOS_PER_MS = 1.0e6;
    private static final CopyOnWriteArrayList<LogiScript> REGISTRY = new CopyOnWriteArrayList<LogiScript>();

    static {
        register(new ColorCoverageScript());
    }

    private final LogiFrameContext context;
    private final List<LogiScript> initializedScripts = new ArrayList<LogiScript>();
    private volatile ScriptFrame latest = ScriptFrame.EMPTY;
    private int initializedWidth;
    private int initializedHeight;

    public ScriptProcessor(LogiFrameContext context) {
        this.context = context == null ? new LogiFrameContext(null) : context;
    }

    public LogiFrameContext context() {
        return context;
    }

    public ScriptFrame latest() {
        ScriptFrame frame = latest;
        return frame == null ? ScriptFrame.EMPTY : frame;
    }

    public static void register(LogiScript script) {
        String name = safeName(script);
        if (name.isEmpty()) {
            return;
        }
        synchronized (REGISTRY) {
            for (int i = 0; i < REGISTRY.size(); i++) {
                if (name.equalsIgnoreCase(safeName(REGISTRY.get(i)))) {
                    REGISTRY.set(i, script);
                    return;
                }
            }
            REGISTRY.add(script);
        }
    }

    public static boolean unregister(String name) {
        String wanted = name == null ? "" : name.trim();
        if (wanted.isEmpty()) {
            return false;
        }
        synchronized (REGISTRY) {
            for (int i = 0; i < REGISTRY.size(); i++) {
                if (wanted.equalsIgnoreCase(safeName(REGISTRY.get(i)))) {
                    REGISTRY.remove(i);
                    return true;
                }
            }
        }
        return false;
    }

    public static LogiScript find(String name) {
        String wanted = name == null ? "" : name.trim();
        if (wanted.isEmpty()) {
            return null;
        }
        for (LogiScript script : REGISTRY) {
            if (wanted.equalsIgnoreCase(safeName(script))) {
                return script;
            }
        }
        return null;
    }

    public static List<String> names() {
        List<String> result = new ArrayList<String>();
        for (LogiScript script : REGISTRY) {
            String name = safeName(script);
            if (!name.isEmpty()) {
                result.add(name);
            }
        }
        return Collections.unmodifiableList(result);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        try {
            context.onInit(width, height, calibration);
            synchronized (initializedScripts) {
                initializedScripts.clear();
                initializedWidth = width;
                initializedHeight = height;
            }
        } catch (Throwable t) {
            context.reportError("script init", t);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long startNanos = System.nanoTime();
        String scriptName = "";
        int pipelineIndex = -1;
        try {
            context.markFrame(captureTimeNanos);
            LogiPipeline pipeline = context.activePipeline();
            if (pipeline == null || !pipeline.typeOrDefault().usesScript() || frame == null || frame.empty()) {
                return null;
            }
            pipelineIndex = context.activePipelineIndex();
            LogiScript script = resolveScript(pipeline);
            scriptName = script != null ? safeName(script) : (pipeline.scriptName == null ? "" : pipeline.scriptName.trim());
            if (script == null) {
                ScriptFrame missing = new ScriptFrame(captureTimeNanos, elapsedMs(startNanos), LogiScriptResult.EMPTY,
                        scriptName, pipelineIndex, "script not found: " + (scriptName.isEmpty() ? "(none)" : scriptName));
                latest = missing;
                return missing;
            }
            LogiScriptResult result;
            try {
                ensureInitialized(script, frame.cols(), frame.rows());
                result = script.run(frame, captureTimeNanos, context.scriptInputs());
            } catch (Throwable scriptError) {
                context.reportError("script " + scriptName, scriptError);
                ScriptFrame failed = new ScriptFrame(captureTimeNanos, elapsedMs(startNanos), LogiScriptResult.EMPTY,
                        scriptName, pipelineIndex, LogiFrameContext.describe("", scriptError));
                latest = failed;
                return failed;
            }
            ScriptFrame done = new ScriptFrame(captureTimeNanos, elapsedMs(startNanos),
                    result == null ? LogiScriptResult.EMPTY : result, scriptName, pipelineIndex, "");
            latest = done;
            return done;
        } catch (Throwable t) {
            context.reportError("script processor", t);
            try {
                latest = new ScriptFrame(captureTimeNanos, elapsedMs(startNanos), LogiScriptResult.EMPTY,
                        scriptName, pipelineIndex, LogiFrameContext.describe("", t));
            } catch (Throwable ignored) {
            }
            return null;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    private LogiScript resolveScript(LogiPipeline pipeline) {
        LogiScript assigned = context.activeScript();
        String wanted = pipeline.scriptName == null ? "" : pipeline.scriptName.trim();
        if (wanted.isEmpty()) {
            return assigned;
        }
        if (assigned != null && wanted.equalsIgnoreCase(safeName(assigned))) {
            return assigned;
        }
        LogiScript registered = find(wanted);
        return registered != null ? registered : assigned;
    }

    private void ensureInitialized(LogiScript script, int width, int height) {
        synchronized (initializedScripts) {
            if (width != initializedWidth || height != initializedHeight) {
                initializedScripts.clear();
                initializedWidth = width;
                initializedHeight = height;
            }
            for (int i = 0; i < initializedScripts.size(); i++) {
                if (initializedScripts.get(i) == script) {
                    return;
                }
            }
            script.init(width, height);
            initializedScripts.add(script);
        }
    }

    private static double elapsedMs(long startNanos) {
        return (System.nanoTime() - startNanos) / NANOS_PER_MS;
    }

    private static String safeName(LogiScript script) {
        if (script == null) {
            return "";
        }
        try {
            String name = script.name();
            return name == null ? "" : name.trim();
        } catch (Throwable t) {
            return "";
        }
    }
}
