package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.logilight.CameraControlInfo;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiCameraConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiPipeline;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiStatus;
import org.firstinspires.ftc.teamcode.vision.logilight.StreamView;

import java.util.Locale;

@TeleOp(name = "LogiLight Camera Tuner", group = "LogiLight")
public class LogiLightCameraTuner extends LogiOpModeBase {

    private static final double EXPOSURE_STEP_MS = 0.5;
    private static final double FINE_EXPOSURE_STEP_MS = 0.1;
    private static final double FINE_EXPOSURE_BELOW_MS = 1.0;
    private static final double EXPOSURE_ROUNDING = 100.0;
    private static final int GAIN_STEP = 10;
    private static final int WHITE_BALANCE_STEP_KELVIN = 200;
    private static final double FOCUS_STEP = 5.0;
    private static final double DECIMATION_STEP = 0.5;
    private static final double FALLBACK_MIN_EXPOSURE_MS = 0.05;
    private static final double FALLBACK_MAX_EXPOSURE_MS = 100.0;
    private static final int FALLBACK_MIN_GAIN = 0;
    private static final int FALLBACK_MAX_GAIN = 255;
    private static final int FALLBACK_MIN_KELVIN = 2000;
    private static final int FALLBACK_MAX_KELVIN = 6500;
    private static final double FALLBACK_MIN_FOCUS = 0.0;
    private static final double FALLBACK_MAX_FOCUS = 255.0;
    private static final double EPSILON = 1e-9;

    private boolean editPipeline = false;

    @Override
    protected void onLoop() {
        LogiCameraConfig cfg = camera();
        LogiPipeline pipeline = ll.getPipeline();
        CameraControlInfo info = ll.getCameraControlInfo();
        handleButtons(cfg, pipeline, info);
        LogiStatus status = ll.getStatus();
        LogiResult result = ll.getLatestResult();
        addSettingsTelemetry(cfg, pipeline);
        addCameraReport(info, status);
        addPerformance(status, result);
        telemetry.addLine();
        addHint("dpad U/D = exposure +-0.5 ms   dpad R/L = gain +-10   BACK = auto/manual exposure");
        addHint("RB / LB tap = white balance +-200 K   R-stick btn = auto white balance");
        addHint("RT / LT = focus +-5   L-stick btn = autofocus");
        addHint("Y / A = decimation +-0.5   X = next pipeline   B = edit camera / this pipeline");
        addHint("LB + A = stream view   LB + B = clear this pipeline's overrides");
        addHint(SETTINGS_HINT);
        addHint("Tip: lowest exposure that still detects tags at your longest shot = least motion blur. Raise gain to brighten.");
    }

    private void handleButtons(LogiCameraConfig cfg, LogiPipeline pipeline, CameraControlInfo info) {
        boolean shift = comboHeld();
        boolean pipelineTarget = editPipeline && pipeline != null;
        if (shift) {
            if (pad1.a()) {
                StreamView next = ll.getStreamView().next();
                ll.setStreamView(next);
                showMessage("stream view " + next);
            }
            if (pad1.b() && pipeline != null) {
                pipeline.exposureMs = -1.0;
                pipeline.gain = -1;
                pipeline.decimation = -1.0;
                ll.applyCameraControls();
                showMessage("cleared exposure/gain/decimation overrides of pipeline " + pipeline.name);
            }
        } else {
            if (pad1.x()) {
                switchPipelineBy(1);
            }
            if (pad1.b()) {
                editPipeline = !editPipeline;
                showMessage(editPipeline ? "now editing the active pipeline's override" : "now editing the camera (all pipelines)");
            }
            if (pad1.y()) {
                adjustDecimation(cfg, pipeline, pipelineTarget, DECIMATION_STEP);
            }
            if (pad1.a()) {
                adjustDecimation(cfg, pipeline, pipelineTarget, -DECIMATION_STEP);
            }
        }
        if (pad1.dpadUp()) {
            adjustExposure(cfg, pipeline, pipelineTarget, info, 1);
        }
        if (pad1.dpadDown()) {
            adjustExposure(cfg, pipeline, pipelineTarget, info, -1);
        }
        if (pad1.dpadRight()) {
            adjustGain(cfg, pipeline, pipelineTarget, info, GAIN_STEP);
        }
        if (pad1.dpadLeft()) {
            adjustGain(cfg, pipeline, pipelineTarget, info, -GAIN_STEP);
        }
        if (pad1.rightBumper()) {
            adjustWhiteBalance(cfg, info, WHITE_BALANCE_STEP_KELVIN);
        }
        if (pad1.leftBumperTap()) {
            adjustWhiteBalance(cfg, info, -WHITE_BALANCE_STEP_KELVIN);
        }
        if (pad1.rightTrigger()) {
            adjustFocus(cfg, info, FOCUS_STEP);
        }
        if (pad1.leftTrigger()) {
            adjustFocus(cfg, info, -FOCUS_STEP);
        }
        if (pad1.back()) {
            toggleAutoExposure(cfg, pipeline, pipelineTarget);
        }
        if (pad1.leftStickButton()) {
            ll.setFocus(!cfg.autoFocus, cfg.focusLength);
            showMessage(cfg.autoFocus ? "autofocus ON" : "autofocus OFF (fixed focus)");
        }
        if (pad1.rightStickButton()) {
            ll.setWhiteBalance(!cfg.autoWhiteBalance, cfg.whiteBalanceKelvin);
            showMessage(cfg.autoWhiteBalance ? "auto white balance ON" : "manual white balance");
        }
    }

    private void adjustExposure(LogiCameraConfig cfg, LogiPipeline pipeline, boolean pipelineTarget,
                                CameraControlInfo info, int direction) {
        double current = pipelineTarget && pipeline.overridesExposure() ? pipeline.exposureMs : cfg.exposureMs;
        if (!(current > 0.0) || Double.isInfinite(current)) {
            current = FINE_EXPOSURE_BELOW_MS;
        }
        boolean fine = current < FINE_EXPOSURE_BELOW_MS - EPSILON
                || (direction < 0 && current <= FINE_EXPOSURE_BELOW_MS + EPSILON);
        double step = fine ? FINE_EXPOSURE_STEP_MS : EXPOSURE_STEP_MS;
        double min = FALLBACK_MIN_EXPOSURE_MS;
        double max = FALLBACK_MAX_EXPOSURE_MS;
        if (info.available && info.exposureSupported && info.maxExposureMs > info.minExposureMs) {
            min = Math.max(info.minExposureMs, 0.01);
            max = info.maxExposureMs;
        }
        double next = Math.round((current + direction * step) * EXPOSURE_ROUNDING) / EXPOSURE_ROUNDING;
        next = clamp(next, min, max);
        if (pipelineTarget) {
            pipeline.exposureMs = next;
            ll.applyCameraControls();
            showMessage(String.format(Locale.US, "pipeline %s exposure %.2f ms", pipeline.name, next));
        } else {
            ll.setExposure(next, -1);
            showMessage(String.format(Locale.US, "camera exposure %.2f ms (manual)", next));
        }
    }

    private void adjustGain(LogiCameraConfig cfg, LogiPipeline pipeline, boolean pipelineTarget,
                            CameraControlInfo info, int delta) {
        int current = pipelineTarget && pipeline.overridesGain() ? pipeline.gain : cfg.gain;
        int min = FALLBACK_MIN_GAIN;
        int max = FALLBACK_MAX_GAIN;
        if (info.available && info.gainSupported && info.maxGain > info.minGain) {
            min = info.minGain;
            max = info.maxGain;
        }
        int next = Math.max(min, Math.min(max, current + delta));
        if (pipelineTarget) {
            pipeline.gain = next;
            ll.applyCameraControls();
            showMessage("pipeline " + pipeline.name + " gain " + next);
        } else {
            ll.setExposure(Double.NaN, next);
            showMessage("camera gain " + next);
        }
    }

    private void adjustWhiteBalance(LogiCameraConfig cfg, CameraControlInfo info, int delta) {
        int min = FALLBACK_MIN_KELVIN;
        int max = FALLBACK_MAX_KELVIN;
        if (info.available && info.whiteBalanceSupported && info.maxKelvin > info.minKelvin) {
            min = info.minKelvin;
            max = info.maxKelvin;
        }
        int start = cfg.whiteBalanceKelvin;
        if (cfg.autoWhiteBalance && info.available && info.whiteBalanceSupported && info.currentKelvin > 0) {
            start = info.currentKelvin;
        }
        int next = Math.max(min, Math.min(max, start + delta));
        ll.setWhiteBalance(false, next);
        showMessage("white balance " + next + " K (manual)");
    }

    private void adjustFocus(LogiCameraConfig cfg, CameraControlInfo info, double delta) {
        double min = FALLBACK_MIN_FOCUS;
        double max = FALLBACK_MAX_FOCUS;
        if (info.available && info.focusSupported && info.maxFocus > info.minFocus) {
            min = info.minFocus;
            max = info.maxFocus;
        }
        double next = clamp(cfg.focusLength + delta, min, max);
        ll.setFocus(false, next);
        showMessage(String.format(Locale.US, "focus %.0f (fixed)%s", next,
                info.available && !info.focusSupported ? "  !! this camera has no focus control" : ""));
    }

    private void adjustDecimation(LogiCameraConfig cfg, LogiPipeline pipeline, boolean pipelineTarget, double delta) {
        double current;
        if (pipelineTarget) {
            current = pipeline.overridesDecimation() ? pipeline.decimation : ll.getDecimation();
        } else {
            current = cfg.decimation > 0.0 && !Double.isInfinite(cfg.decimation) ? cfg.decimation : LogiLight.DEFAULT_DECIMATION;
        }
        double next = clamp(current + delta, LogiLight.MIN_DECIMATION, LogiLight.MAX_DECIMATION);
        if (pipelineTarget) {
            pipeline.decimation = next;
            ll.applyCameraControls();
            showMessage(String.format(Locale.US, "pipeline %s decimation %.1f", pipeline.name, next));
        } else {
            ll.setDecimation(next);
            showMessage(String.format(Locale.US, "camera decimation %.1f%s", cfg.decimation,
                    pipeline != null && pipeline.overridesDecimation() ? "  !! this pipeline overrides it" : ""));
        }
    }

    private void toggleAutoExposure(LogiCameraConfig cfg, LogiPipeline pipeline, boolean pipelineTarget) {
        if (pipelineTarget && pipeline.overridesExposure()) {
            pipeline.exposureMs = -1.0;
            ll.applyCameraControls();
            showMessage("pipeline " + pipeline.name + " exposure override cleared - camera setting used");
            return;
        }
        cfg.manualExposure = !cfg.manualExposure;
        ll.applyCameraControls();
        showMessage(cfg.manualExposure
                ? String.format(Locale.US, "manual exposure %.2f ms", cfg.exposureMs)
                : "auto exposure (bad for AprilTags while moving)");
    }

    private void addSettingsTelemetry(LogiCameraConfig cfg, LogiPipeline pipeline) {
        addPipelineLine();
        String pipelineName = pipeline == null ? "?" : pipeline.name;
        telemetry.addData("Editing", editPipeline ? ">> PIPELINE " + pipelineName + " override (only this pipeline)"
                : ">> CAMERA (all pipelines)");
        boolean exposureOverride = pipeline != null && pipeline.overridesExposure();
        boolean gainOverride = pipeline != null && pipeline.overridesGain();
        boolean decimationOverride = pipeline != null && pipeline.overridesDecimation();
        String exposure;
        if (exposureOverride) {
            exposure = String.format(Locale.US, "%.2f ms (pipeline override)  camera %s", pipeline.exposureMs,
                    cfg.manualExposure ? String.format(Locale.US, "%.2f ms", cfg.exposureMs) : "auto");
        } else if (cfg.manualExposure) {
            exposure = String.format(Locale.US, "%.2f ms manual (camera)", cfg.exposureMs);
        } else {
            exposure = "AUTO (camera)  !! tags blur while moving";
        }
        telemetry.addData("Exposure", exposure);
        telemetry.addData("Gain", gainOverride
                ? String.format(Locale.US, "%d (pipeline override)  camera %d", pipeline.gain, cfg.gain)
                : String.format(Locale.US, "%d (camera)", cfg.gain));
        telemetry.addData("White balance", cfg.autoWhiteBalance ? "auto" : cfg.whiteBalanceKelvin + " K manual");
        telemetry.addData("Focus", cfg.autoFocus ? "auto" : String.format(Locale.US, "%.0f fixed", cfg.focusLength));
        telemetry.addData("Decimation", decimationOverride
                ? String.format(Locale.US, "%.1f (pipeline override)  camera %.1f", pipeline.decimation, cfg.decimation)
                : String.format(Locale.US, "%.1f (camera)", cfg.decimation));
        telemetry.addData("Stream", "%s  (LB + A)", ll.getStreamView());
    }

    private void addCameraReport(CameraControlInfo info, LogiStatus status) {
        telemetry.addData("Wanted", status.exposureSummary);
        if (!info.available) {
            telemetry.addData("Camera reports", info.summary());
        } else {
            telemetry.addData("Camera exposure", info.exposureLine());
            telemetry.addData("Camera gain", info.gainLine());
            telemetry.addData("Camera wb", info.whiteBalanceLine());
            telemetry.addData("Camera focus", info.focusLine());
            if (info.available && !info.manualExposureSupported) {
                telemetry.addLine("!! this camera does not support manual exposure");
            }
        }
        String controls = ll.getControlStatus();
        if (controls != null && !controls.isEmpty()) {
            telemetry.addData("Controls", controls);
        }
    }

    private void addPerformance(LogiStatus status, LogiResult result) {
        telemetry.addData("Rate", "%.0f fps  proc %.1f ms  latency %.0f ms  %s", status.fps, status.averageProcessingMs,
                result.getTargetingLatency(), status.cameraState);
        LogiFiducial primary = result.getPrimaryFiducial();
        if (primary != null) {
            telemetry.addData("Primary tag", "%s  d %.1f in  ta %.3f%s", primary.name, primary.robotDistanceInches, primary.ta,
                    primary.isCluster ? String.format(Locale.US, "  %d%% found", primary.percentClusterFound)
                            : String.format(Locale.US, "  margin %.0f", primary.decisionMargin));
        }
        telemetry.addData("Seen", "%d tags/clusters  %d color blobs", result.getFiducialCount(), result.getColorCount());
    }

    private static double clamp(double value, double min, double max) {
        if (Double.isNaN(value)) {
            return min;
        }
        return Math.max(min, Math.min(max, value));
    }
}
