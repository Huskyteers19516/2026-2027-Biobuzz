package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.vision.logilight.CameraControlInfo;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiBotPose;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiCameraConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFieldMap;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightStore;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLocalizationConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiScriptResult;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiStatus;
import org.firstinspires.ftc.teamcode.vision.logilight.PipelineType;
import org.firstinspires.ftc.teamcode.vision.logilight.StreamView;
import org.firstinspires.ftc.teamcode.vision.logilight.math.FieldTargetState;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "LogiLight Viewer", group = "LogiLight")
public class LogiLightViewer extends LogiOpModeBase {

    private static final String[] PAGE_NAMES = {"OVERVIEW", "TARGETS", "SYSTEM"};
    private static final int PAGE_OVERVIEW = 0;
    private static final int PAGE_TARGETS = 1;
    private static final int PAGE_SYSTEM = 2;
    private static final int MAX_DETAIL_FIDUCIALS = 4;
    private static final int MAX_DETAIL_COLORS = 4;
    private static final int SCRIPT_OUTPUTS_SHOWN = 8;
    private static final String SNAPSHOT_NAME = "viewer";

    private int page = PAGE_OVERVIEW;

    @Override
    protected boolean usesOdometry() {
        return true;
    }

    @Override
    protected void onInit() {
        odometry.useAutoHandoffOrStart();
    }

    @Override
    protected void onStart() {
        PoseHandoff.clear();
    }

    @Override
    protected void onLoop() {
        handleButtons();
        LogiResult result = ll.getLatestResult();
        LogiLightField.drawResult(odometry.pose(), result, null, ll.getFieldMap());
        telemetry.addData("Page", "%s  %d/%d  (gp1 X = next page)", PAGE_NAMES[page], page + 1, PAGE_NAMES.length);
        if (page == PAGE_TARGETS) {
            addTargetsPage(result);
        } else if (page == PAGE_SYSTEM) {
            addSystemPage();
        } else {
            addOverviewPage(result);
        }
    }

    private void handleButtons() {
        boolean shift = comboHeld();
        if (pad1.dpadRight()) {
            switchPipelineBy(1);
        }
        if (pad1.dpadLeft()) {
            switchPipelineBy(-1);
        }
        if (pad1.a() && !shift) {
            boolean ok = ll.captureSnapshot(SNAPSHOT_NAME);
            showMessage(ok ? "snapshot requested - saved in " + LogiLightStore.snapshotDirectory().getPath()
                    : "!! snapshot failed - camera not available");
        }
        if (pad1.b() && !shift) {
            StreamView next = ll.getStreamView().next();
            ll.setStreamView(next);
            showMessage("stream view " + next);
        }
        if (pad1.x() && !shift) {
            page = (page + 1) % PAGE_NAMES.length;
        }
        if (pad1.y() && !shift) {
            relocalizeFromVision();
        }
        if (pad1.back()) {
            boolean ok = odometry.resetToStartPose();
            showMessage(ok ? "odometry re-zeroed at " + odometry.alliance() + " start pose" : "!! no Pinpoint to re-zero");
        }
        if (pad1.dpadUp()) {
            GoalScanner.Alliance next = odometry.alliance() == GoalScanner.Alliance.RED
                    ? GoalScanner.Alliance.BLUE : GoalScanner.Alliance.RED;
            odometry.setAlliance(next);
            boolean ok = odometry.resetToStartPose();
            showMessage("alliance " + next + (ok ? " - odometry at its start pose" : "  !! no Pinpoint"));
        }
    }

    private void relocalizeFromVision() {
        if (!odometry.isAvailable()) {
            showMessage("!! no Pinpoint - nothing to relocalize");
            return;
        }
        LogiBotPose mt1 = ll.getLatestResult().getBotpose();
        if (mt1 == null || !mt1.hasHeading() || !mt1.isFinite()) {
            showMessage("!! no MT1 pose - needs a surveyed goal in view (run LogiLight Field Survey)");
            return;
        }
        boolean ok = odometry.setPoseFromVision(mt1.toPedroPose());
        showMessage(ok ? "odometry set to vision MT1 " + LogiLightTelemetry.formatPose(mt1.pose)
                : "!! could not set the Pinpoint pose");
    }

    private void addOverviewPage(LogiResult result) {
        LogiStatus status = ll.getStatus();
        telemetry.addData("LogiLight", status.summaryLine());
        addPipelineLine();
        LogiLightTelemetry.addResult(telemetry, result);
        telemetry.addData("Odometry", odometry.summary());
        telemetry.addData("Heading feed", odometry.headingSummary(ll));
        addVisionVersusOdometry(result);
        telemetry.addData("Stream", "%s  DS %s  Panels %s", ll.getStreamView(),
                status.driverStationStreaming ? "on" : "off", status.panelsStreaming ? "on" : "off");
        String snapshot = ll.lastSnapshotPath();
        if (snapshot != null && !snapshot.isEmpty()) {
            telemetry.addData("Snapshot", snapshot);
        }
        telemetry.addLine();
        addHint("gp1 dpad L/R = pipeline   A = snapshot   B = stream view   X = page");
        addHint("gp1 Y = set odometry to vision MT1   BACK = re-zero at start pose   dpad up = alliance "
                + odometry.alliance());
        addHint(SETTINGS_HINT);
        addHint(STREAM_HINT);
    }

    private void addVisionVersusOdometry(LogiResult result) {
        PlanarPose odo = odometry.planarPose();
        LogiBotPose best = result.getBestBotpose();
        if (odo == null || best == null || best.pose == null || !best.pose.isPositionFinite()) {
            return;
        }
        double distance = odo.distanceTo(best.pose);
        if (best.hasHeading()) {
            telemetry.addData("Vision - odometry", "%.1f in  h %+.1f deg  (%s)", distance,
                    Math.toDegrees(odo.headingErrorTo(best.pose)), best.method);
        } else {
            telemetry.addData("Vision - odometry", "%.1f in  (%s)", distance, best.method);
        }
    }

    private void addTargetsPage(LogiResult result) {
        addPipelineLine();
        if (result.isValid()) {
            telemetry.addData("Primary", "%s  tx %.2f  ty %.2f  ta %.3f  txnc %.2f  tync %.2f", result.getPrimaryName(),
                    result.getTx(), result.getTy(), result.getTa(), result.getTxNC(), result.getTyNC());
            if (isFinite(result.getPrimaryCenterX())) {
                telemetry.addData("Primary pixel", "u %.1f  v %.1f", result.getPrimaryCenterX(), result.getPrimaryCenterY());
            }
        } else {
            telemetry.addData("Primary", "no target%s", result.isExpired() ? "  !! stale result" : "");
        }
        telemetry.addData("Latency", "capture->done %.0f ms  age %.0f ms  frame %d",
                result.getTargetingLatency(), finiteOr(result.getStaleness(), -1.0), result.getFrameNumber());
        List<LogiFiducial> fiducials = result.getFiducialResults();
        int shown = 0;
        for (LogiFiducial fiducial : fiducials) {
            if (shown >= MAX_DETAIL_FIDUCIALS) {
                telemetry.addLine(String.format(Locale.US, "... %d more tags", fiducials.size() - shown));
                break;
            }
            addFiducialDetail(fiducial);
            shown++;
        }
        if (fiducials.isEmpty() && result.getPipelineType().usesAprilTag()) {
            telemetry.addLine("no AprilTags / clusters in view");
        }
        List<ColorTarget> colors = result.getColorResults();
        shown = 0;
        for (ColorTarget target : colors) {
            if (shown >= MAX_DETAIL_COLORS) {
                telemetry.addLine(String.format(Locale.US, "... %d more blobs", colors.size() - shown));
                break;
            }
            addColorDetail(target);
            shown++;
        }
        if (colors.isEmpty() && result.getPipelineType().usesColor()) {
            telemetry.addLine("no color blobs pass the filters (tune with LogiLight Color Tuner)");
        }
        if (result.getPipelineType() == PipelineType.SCRIPT) {
            addScriptDetail(result);
        }
        LogiBotPose mt1 = result.getBotpose();
        LogiBotPose mt2 = result.getBotpose_MT2();
        telemetry.addData("Botpose MT1", mt1 == null ? "none" : LogiLightTelemetry.botPoseLine(mt1));
        telemetry.addData("Botpose MT2", mt2 == null ? "none" : LogiLightTelemetry.botPoseLine(mt2));
        telemetry.addLine();
        addHint("gp1 dpad L/R = pipeline   B = stream view   X = page");
    }

    private void addFiducialDetail(LogiFiducial f) {
        telemetry.addLine(f.summary());
        if (f.hasPose()) {
            telemetry.addLine(String.format(Locale.US, "   range %.1f in  bearing %+.1f  elev %+.1f  yaw %.1f  pitch %.1f  roll %.1f",
                    f.range, f.bearing, f.elevation, f.yaw, f.pitch, f.roll));
            telemetry.addLine(String.format(Locale.US, "   robot d %.1f in  bL %+.1f  aim d %.1f  aim bL %+.1f  ty %.1f  ta %.3f",
                    f.robotDistanceInches, f.bearingLeftDegrees, f.aimDistanceInches, f.aimBearingLeftDegrees, f.ty, f.ta));
        } else {
            telemetry.addLine(String.format(Locale.US, "   no pose  tx %.1f  ty %.1f  ta %.3f", f.tx, f.ty, f.ta));
        }
        if (!f.isCluster) {
            telemetry.addLine(String.format(Locale.US, "   id %d  margin %.0f  hamming %d", f.id, f.decisionMargin, f.hamming));
        }
        if (!f.matchedState.isEmpty()) {
            telemetry.addLine(String.format(Locale.US, "   state %s  height err %+.1f in  tilt %.1f  rot %.1f deg",
                    f.matchedState, f.heightErrorInches, f.tiltMismatchDegrees, f.rotationMismatchDegrees));
        }
        if (f.mt1 != null || f.mt2 != null) {
            telemetry.addLine("   MT1 " + LogiLightTelemetry.formatPose(f.mt1) + "   MT2 " + LogiLightTelemetry.formatPose(f.mt2));
        }
        if (!f.rejectReason.isEmpty()) {
            telemetry.addLine("   rejected: " + f.rejectReason);
        }
    }

    private void addColorDetail(ColorTarget t) {
        String ground = t.hasGroundPosition()
                ? String.format(Locale.US, "  ground %.1f in @ %+.1f deg (fwd %.1f left %.1f)", t.groundDistanceInches,
                t.groundBearingLeftDegrees, t.groundForwardInches, t.groundLeftInches)
                : "  no ground fix";
        telemetry.addLine(String.format(Locale.US, "%s  tx %.1f  ty %.1f  ta %.3f  px (%.0f, %.0f)", t.profileName,
                t.tx, t.ty, t.ta, t.centerX, t.centerY));
        telemetry.addLine(String.format(Locale.US, "   full %.2f  aspect %.2f  circ %.2f  box %.0fx%.0f%s",
                t.fullness, t.aspectRatio, t.circularity, t.boxWidth, t.boxHeight, ground));
    }

    private void addScriptDetail(LogiResult result) {
        LogiScriptResult script = result.getScriptResult();
        double[] outputs = result.getScriptOutput();
        StringBuilder line = new StringBuilder();
        for (int i = 0; i < SCRIPT_OUTPUTS_SHOWN && i < outputs.length; i++) {
            if (i > 0) {
                line.append("  ");
            }
            line.append(String.format(Locale.US, "o%d %.2f", i, outputs[i]));
        }
        telemetry.addData("Script out", line.toString());
        if (script != null && script.hasTarget) {
            telemetry.addData("Script target", "%s  u %.0f  v %.0f  %.0fx%.0f", script.label == null ? "" : script.label,
                    script.targetX, script.targetY, script.targetWidth, script.targetHeight);
        }
    }

    private void addSystemPage() {
        LogiLightTelemetry.addStatus(telemetry, ll);
        CameraControlInfo info = ll.getCameraControlInfo();
        telemetry.addData("Camera reports", info.summary());
        LogiCameraConfig cfg = camera();
        telemetry.addData("Mount", cfg.mountSummary());
        telemetry.addData("Crosshair", "x %.3f  y %.3f", cfg.crosshairX, cfg.crosshairY);
        String detail = ll.getFrameContext().intrinsicsDetail();
        if (detail != null && !detail.isEmpty()) {
            telemetry.addData("Lens", detail);
        }
        telemetry.addData("Stream cfg", "%d px wide  max %d fps  color scale %.2f", cfg.streamWidth, cfg.streamMaxFps,
                cfg.colorProcessScale);
        telemetry.addData("Settings load", ll.getSettingsLoadMessage());
        String panels = ll.getPanelsMessage();
        if (panels != null && !panels.isEmpty()) {
            telemetry.addData("Panels", panels);
        }
        telemetry.addData("Heading feed", odometry.headingSummary(ll));
        addFieldMap(ll.getFieldMap());
        LogiLocalizationConfig localization = LogiLightSettings.LOCALIZATION;
        if (localization != null) {
            telemetry.addData("Localization", "%s  unsurveyed %s  max d %.0f in  min cluster %d%%",
                    localization.enabled ? "on" : "OFF",
                    localization.useUnsurveyedTargets ? "used" : "ignored",
                    localization.maxTargetDistanceInches, localization.minPercentClusterFound);
        }
        telemetry.addLine();
        addHint("gp1 X = page   " + SETTINGS_HINT);
    }

    private void addFieldMap(LogiFieldMap map) {
        if (map == null) {
            telemetry.addData("Field map", "none");
            return;
        }
        telemetry.addData("Field map", map.summary());
        for (LogiFieldMap.Target target : map.targets()) {
            FieldTargetState state = target.firstState();
            String where = state == null ? "no states" : String.format(Locale.US, "(%.1f, %.1f, %.1f)",
                    state.originField.x, state.originField.y, state.originField.z);
            telemetry.addLine(String.format(Locale.US, "   %s  %s  states %s  %s", target.name,
                    target.surveyed ? "surveyed" : "!! placeholder", target.stateLabels(), where));
        }
        if (map.surveyedCount() == 0) {
            telemetry.addLine("!! no surveyed goals - botpose stays empty until LogiLight Field Survey is run");
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double finiteOr(double value, double fallback) {
        return isFinite(value) ? value : fallback;
    }

    @Override
    protected void onSettingsReset() {
        ll.setStreamView(LogiLightSettings.STREAM_VIEW);
    }
}
