package org.firstinspires.ftc.teamcode.vision.logilight;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;

import java.util.List;
import java.util.Locale;

public final class LogiLightTelemetry {

    public static final int MAX_FIDUCIAL_LINES = 6;
    public static final int MAX_COLOR_LINES = 4;
    public static final int SCRIPT_OUTPUTS_SHOWN = 4;
    public static final String NO_POSE = "none";

    private LogiLightTelemetry() {
    }

    public static void addStatus(Telemetry telemetry, LogiLight light) {
        if (telemetry == null) {
            return;
        }
        if (light == null) {
            telemetry.addData("LogiLight", "!! not created");
            return;
        }
        try {
            LogiStatus status = light.getStatus();
            if (!light.isAvailable()) {
                telemetry.addData("LogiLight", status.fault.isEmpty() ? "NOT AVAILABLE" : status.fault);
                telemetry.addData("Settings", status.settingsSource);
                return;
            }
            telemetry.addData("LogiLight", status.summaryLine());
            telemetry.addData("Camera", "%s  %s  %dx%d", status.webcamName, status.cameraState, status.width, status.height);
            telemetry.addData("Rate", "%.0f fps  proc %.1f ms  frames %d", status.fps, status.averageProcessingMs, status.frameCount);
            telemetry.addData("Pipeline", "%d %s (%s)", status.pipelineIndex, status.pipelineName, status.pipelineType);
            telemetry.addData("Intrinsics", status.intrinsicsSource);
            telemetry.addData("Exposure", status.exposureSummary);
            if (!status.controlStatus.isEmpty()) {
                telemetry.addData("Controls", status.controlStatus);
            }
            telemetry.addData("Streams", "DS %s  Panels %s", status.driverStationStreaming ? "on" : "off",
                    status.panelsStreaming ? "on" : "off");
            telemetry.addData("Settings", status.settingsSource);
            telemetry.addData("Field map", status.fieldMapSource);
            if (status.watchdogRestarts > 0 || !status.watchdogReason.isEmpty()) {
                telemetry.addData("Watchdog", "%d restarts  %s", status.watchdogRestarts, status.watchdogReason);
            }
            if (status.hasFault()) {
                telemetry.addData("!! Fault", status.fault);
            }
            if (status.processorErrorCount > 0) {
                telemetry.addData("!! Processor errors", "%d  %s", status.processorErrorCount, status.lastProcessorError);
            }
        } catch (Throwable t) {
            telemetry.addData("!! LogiLight telemetry", LogiLight.describe(t));
        }
    }

    public static void addResult(Telemetry telemetry, LogiResult result) {
        if (telemetry == null) {
            return;
        }
        LogiResult r = result == null ? LogiResult.EMPTY : result;
        try {
            if (r.isValid()) {
                telemetry.addData("Target", "%s  tx %.1f  ty %.1f  ta %.2f", r.getPrimaryName(), r.getTx(), r.getTy(), r.getTa());
                if (isFinite(r.getPrimaryDistanceInches())) {
                    telemetry.addData("Target range", "%.1f in  bL %+.1f deg", r.getPrimaryDistanceInches(), r.getPrimaryBearingLeftDegrees());
                }
            } else {
                telemetry.addData("Target", "no target  (pipe %d %s)%s", r.getPipelineIndex(), r.getPipelineName(),
                        r.isExpired() ? "  !! stale" : "");
            }
            telemetry.addData("Latency", "proc %.0f ms  age %.0f ms  frame %d", r.getTargetingLatency(),
                    finiteOr(r.getStaleness(), -1.0), r.getFrameNumber());
            List<LogiFiducial> fiducials = r.getFiducialResults();
            int shown = 0;
            for (LogiFiducial fiducial : fiducials) {
                if (shown >= MAX_FIDUCIAL_LINES) {
                    telemetry.addLine(String.format(Locale.US, "... %d more tags", fiducials.size() - shown));
                    break;
                }
                telemetry.addLine(fiducialLine(fiducial));
                shown++;
            }
            List<ColorTarget> colors = r.getColorResults();
            shown = 0;
            for (ColorTarget target : colors) {
                if (shown >= MAX_COLOR_LINES) {
                    telemetry.addLine(String.format(Locale.US, "... %d more blobs", colors.size() - shown));
                    break;
                }
                telemetry.addLine(colorLine(target));
                shown++;
            }
            if (r.getPipelineType() == PipelineType.SCRIPT) {
                telemetry.addData("Script", scriptLine(r));
            }
            LogiBotPose mt1 = r.getBotpose();
            LogiBotPose mt2 = r.getBotpose_MT2();
            if (mt1 != null) {
                telemetry.addData("Botpose MT1", botPoseLine(mt1));
            }
            if (mt2 != null) {
                telemetry.addData("Botpose MT2", botPoseLine(mt2));
            }
            if (mt1 == null && mt2 == null && !fiducials.isEmpty()) {
                telemetry.addData("Botpose", "none (see tag lines)");
            }
        } catch (Throwable t) {
            telemetry.addData("!! LogiLight result", LogiLight.describe(t));
        }
    }

    public static void addCompact(Telemetry telemetry, LogiResult result) {
        if (telemetry == null) {
            return;
        }
        LogiResult r = result == null ? LogiResult.EMPTY : result;
        try {
            if (r.isValid()) {
                telemetry.addData("Vision", "%s  tx %.1f  ty %.1f  ta %.2f  %.0f ms", r.getPrimaryName(), r.getTx(), r.getTy(),
                        r.getTa(), finiteOr(r.getStaleness(), -1.0));
            } else {
                telemetry.addData("Vision", "no target  pipe %d %s", r.getPipelineIndex(), r.getPipelineName());
            }
            LogiBotPose best = r.getBestBotpose();
            telemetry.addData("Vision pose", best == null ? NO_POSE : best.method + "  " + formatPose(best.pose)
                    + String.format(Locale.US, "  tags %d", best.targetCount));
        } catch (Throwable t) {
            telemetry.addData("!! LogiLight", LogiLight.describe(t));
        }
    }

    public static String formatPose(PlanarPose pose) {
        if (pose == null || !pose.isPositionFinite()) {
            return NO_POSE;
        }
        if (!pose.hasHeading()) {
            return String.format(Locale.US, "x %.1f  y %.1f  h --", pose.x, pose.y);
        }
        return String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg", pose.x, pose.y, pose.headingDegrees());
    }

    public static String formatPose(Pose pose) {
        if (pose == null || !isFinite(pose.x()) || !isFinite(pose.y())) {
            return NO_POSE;
        }
        if (!isFinite(pose.heading())) {
            return String.format(Locale.US, "x %.1f  y %.1f  h --", pose.x(), pose.y());
        }
        return String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    public static String fiducialLine(LogiFiducial fiducial) {
        if (fiducial == null) {
            return "";
        }
        return fiducial.summary();
    }

    public static String colorLine(ColorTarget target) {
        if (target == null) {
            return "";
        }
        StringBuilder line = new StringBuilder();
        line.append(target.profileName == null ? "color" : target.profileName);
        line.append(String.format(Locale.US, "  tx %.1f  ty %.1f  ta %.2f", target.tx, target.ty, target.ta));
        if (target.hasGroundPosition()) {
            line.append(String.format(Locale.US, "  ground %.1f in @ %+.1f deg",
                    target.groundDistanceInches, target.groundBearingLeftDegrees));
        }
        return line.toString();
    }

    public static String botPoseLine(LogiBotPose botPose) {
        if (botPose == null) {
            return NO_POSE;
        }
        return String.format(Locale.US, "%s  tags %d  d %.0f in  sd %.1f in  age %.0f ms",
                formatPose(botPose.pose), botPose.targetCount, botPose.avgDistanceInches,
                botPose.stddevXYInches, finiteOr(botPose.ageMs(), -1.0));
    }

    public static String scriptLine(LogiResult result) {
        double[] outputs = result.getScriptOutput();
        StringBuilder line = new StringBuilder();
        LogiScriptResult scriptResult = result.getScriptResult();
        if (scriptResult != null && scriptResult.label != null && !scriptResult.label.isEmpty()) {
            line.append(scriptResult.label).append("  ");
        }
        for (int i = 0; i < SCRIPT_OUTPUTS_SHOWN && i < outputs.length; i++) {
            if (i > 0) {
                line.append("  ");
            }
            line.append(String.format(Locale.US, "o%d %.2f", i, outputs[i]));
        }
        return line.toString();
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double finiteOr(double value, double fallback) {
        return isFinite(value) ? value : fallback;
    }
}
