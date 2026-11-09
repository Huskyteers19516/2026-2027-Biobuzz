package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.logilight.LogiCameraConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Locale;

@TeleOp(name = "LogiLight Crosshair Calibration", group = "LogiLight")
public class LogiLightCrosshairCalibration extends LogiOpModeBase {

    private static final int MAX_SAMPLES = 10;
    private static final double MAX_SAMPLE_AGE_MS = 1000.0;
    private static final double NUDGE_STEP = 0.01;
    private static final double NANOS_PER_MILLI = 1e6;

    private static final class CenterSample {
        final double u;
        final double v;
        final long captureNanos;

        CenterSample(double u, double v, long captureNanos) {
            this.u = u;
            this.v = v;
            this.captureNanos = captureNanos;
        }
    }

    private final Deque<CenterSample> samples = new ArrayDeque<CenterSample>();
    private long lastCaptureNanos = 0L;
    private String lastPrimaryName = "";

    @Override
    protected void onLoop() {
        LogiResult result = ll.getLatestResult();
        collect(result);
        handleButtons();
        addTelemetry(result);
    }

    private void collect(LogiResult result) {
        long now = System.nanoTime();
        while (!samples.isEmpty() && (now - samples.peekFirst().captureNanos) / NANOS_PER_MILLI > MAX_SAMPLE_AGE_MS) {
            samples.removeFirst();
        }
        if (!result.isValid() || result.isExpired()) {
            return;
        }
        long capture = result.getCaptureNanos();
        double u = result.getPrimaryCenterX();
        double v = result.getPrimaryCenterY();
        if (capture == 0L || capture == lastCaptureNanos || !isFinite(u) || !isFinite(v)) {
            return;
        }
        String name = result.getPrimaryName() == null ? "" : result.getPrimaryName();
        if (!name.equals(lastPrimaryName)) {
            samples.clear();
            lastPrimaryName = name;
        }
        lastCaptureNanos = capture;
        samples.addLast(new CenterSample(u, v, capture));
        while (samples.size() > MAX_SAMPLES) {
            samples.removeFirst();
        }
    }

    private void handleButtons() {
        boolean shift = comboHeld();
        LogiCameraConfig cfg = camera();
        if (pad1.a() && !shift) {
            setFromTarget();
        }
        if (pad1.b() && !shift) {
            ll.setCrosshair(0.0, 0.0);
            showMessage("crosshair reset to 0, 0 (image center)");
        }
        if (pad1.rightBumper()) {
            switchPipelineBy(1);
            samples.clear();
        }
        double dx = 0.0;
        double dy = 0.0;
        if (pad1.dpadRight()) {
            dx += NUDGE_STEP;
        }
        if (pad1.dpadLeft()) {
            dx -= NUDGE_STEP;
        }
        if (pad1.dpadUp()) {
            dy += NUDGE_STEP;
        }
        if (pad1.dpadDown()) {
            dy -= NUDGE_STEP;
        }
        if (dx != 0.0 || dy != 0.0) {
            ll.setCrosshair(cfg.crosshairX + dx, cfg.crosshairY + dy);
            showMessage(String.format(Locale.US, "crosshair nudged to x %.3f  y %.3f", cfg.crosshairX, cfg.crosshairY));
        }
    }

    private void setFromTarget() {
        if (samples.isEmpty()) {
            showMessage("!! no target in view - nothing to center on");
            return;
        }
        double sumU = 0.0;
        double sumV = 0.0;
        for (CenterSample sample : samples) {
            sumU += sample.u;
            sumV += sample.v;
        }
        double u = sumU / samples.size();
        double v = sumV / samples.size();
        CameraModel model = ll.getCameraModel();
        double[] crosshair = model.crosshairFromPixel(u, v);
        ll.setCrosshair(crosshair[0], crosshair[1]);
        showMessage(String.format(Locale.US, "crosshair set on %s from %d frames: x %.4f  y %.4f",
                lastPrimaryName, samples.size(), crosshair[0], crosshair[1]));
    }

    private void addTelemetry(LogiResult result) {
        LogiCameraConfig cfg = camera();
        CameraModel model = ll.getCameraModel();
        double[] pixel = model.crosshairPixel();
        addPipelineLine();
        telemetry.addData("Crosshair", "x %.4f  y %.4f  (pixel u %.1f  v %.1f of %dx%d)", cfg.crosshairX, cfg.crosshairY,
                pixel[0], pixel[1], model.width, model.height);
        if (result.isValid()) {
            telemetry.addData("Target", "%s  tx %.2f  ty %.2f  ta %.3f", result.getPrimaryName(), result.getTx(),
                    result.getTy(), result.getTa());
            telemetry.addData("No-crosshair", "txnc %.2f  tync %.2f", result.getTxNC(), result.getTyNC());
            if (isFinite(result.getPrimaryCenterX())) {
                telemetry.addData("Target pixel", "u %.1f  v %.1f   averaging %d frames", result.getPrimaryCenterX(),
                        result.getPrimaryCenterY(), samples.size());
            } else {
                telemetry.addLine("!! this target has no pixel center (cluster without a pose) - A cannot use it");
            }
            boolean centered = Math.abs(result.getTx()) < 0.5 && Math.abs(result.getTy()) < 0.5;
            telemetry.addData("Centered", centered ? "YES (|tx|,|ty| < 0.5 deg)" : "no");
        } else {
            telemetry.addData("Target", "none in view%s", result.isExpired() ? "  !! stale" : "");
        }
        telemetry.addData(">> crosshairX", "%.4f", cfg.crosshairX);
        telemetry.addData(">> crosshairY", "%.4f", cfg.crosshairY);
        telemetry.addLine();
        addHint("1. Put the robot where it scores and aim the shooter exactly at the target (test shots).");
        addHint("2. Press A: the crosshair moves onto the target, so tx = ty = 0 means 'on target'.");
        addHint("3. Hold LB + press Y to save. The crosshair is shared by all pipelines.");
        addHint("The crosshair only changes tx/ty: Limelight-style code, LogiLimelight3A, CLOSEST_TO_CROSSHAIR sort, Color Tuner wand.");
        addHint("LogiLight Aim Assist / Auto Example aim at goals with geometry + Constants.SHOOTER_HEADING_OFFSET_DEGREES,");
        addHint("and chase balls on the robot-centred ground bearing - tune SHOOTER_HEADING_OFFSET_DEGREES for them.");
        addHint("Goals: the target point is the cluster origin moved by the pipeline's aimOffsetX/Y/Z.");
        addHint("gp1 A = set from target   B = reset 0,0   dpad = nudge 0.01   RB = next pipeline");
        addHint(SETTINGS_HINT);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
