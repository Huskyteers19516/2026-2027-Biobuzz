package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.logilight.ColorProfile;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiCameraConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFrameContext;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightStore;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiPipeline;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiStatus;
import org.firstinspires.ftc.teamcode.vision.logilight.PipelineType;
import org.firstinspires.ftc.teamcode.vision.logilight.StreamView;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "LogiLight Ball Test", group = "LogiLight")
public class LogiLightBallTest extends OpMode {

    private static final String YELLOW = "yellow";
    private static final String RED = "red";
    private static final String BLUE = "blue";
    private static final String[] ALL_COLORS = {YELLOW, RED, BLUE};
    private static final int MAX_BALLS_SHOWN = 5;
    private static final String STREAMING_STATE = "STREAMING";
    private static final String MISSING_WEBCAM_TEXT = "no webcam named";

    private enum Mode {
        ALL("all balls", ALL_COLORS),
        YELLOW_ONLY("yellow POLLEN", new String[]{YELLOW}),
        RED_ONLY("red NECTAR", new String[]{RED}),
        BLUE_ONLY("blue NECTAR", new String[]{BLUE});

        final String label;
        final String[] colors;

        Mode(String label, String[] colors) {
            this.label = label;
            this.colors = colors;
        }
    }

    private LogiLight camera;
    private Mode mode = Mode.ALL;
    private String pipelineNote = "";
    private String profileNote = "";
    private boolean snapshotPending = false;
    private String snapshotPathBefore = "";
    private String snapshotMessage = "";

    @Override
    public void init() {
        camera = new LogiLight(hardwareMap);
        selectColorPipeline();
        applyMode();
    }

    @Override
    public void init_loop() {
        runCameraTest();
    }

    @Override
    public void loop() {
        runCameraTest();
    }

    @Override
    public void stop() {
        if (camera != null) {
            camera.close();
        }
    }

    private void runCameraTest() {
        handleButtons();
        applyMode();
        camera.update();

        if (!camera.isAvailable()) {
            showUnavailable();
            return;
        }

        LogiStatus status = camera.getStatus();
        LogiFrameContext context = camera.getFrameContext();
        telemetry.addData("Camera", "%s  %s  %.0f fps  frame age %s",
                camera.getWebcamName(), status.cameraState, status.fps, formatAge(status.lastFrameAgeMs));
        if (!status.watchdogReason.isEmpty()) {
            telemetry.addData("Watchdog", status.watchdogReason);
        }
        if (context.hasRecentError()) {
            telemetry.addData("!! Error", context.lastError());
        }
        telemetry.addData("Looking for", "%s   (A all  Y yellow  B red  X blue)", mode.label);
        telemetry.addData("Stream view", "%s   (dpad L/R)", camera.getStreamView());
        if (!pipelineNote.isEmpty()) {
            telemetry.addLine(pipelineNote);
        }
        if (!profileNote.isEmpty()) {
            telemetry.addLine(profileNote);
        }

        boolean fresh = STREAMING_STATE.equals(status.cameraState)
                && !Double.isNaN(status.lastFrameAgeMs)
                && status.lastFrameAgeMs < LogiLight.RESULT_EXPIRY_MS;
        if (!fresh) {
            telemetry.addData("Balls", "no fresh frames - camera %s, wait for STREAMING or check the USB cable",
                    status.cameraState);
        } else {
            showBalls(camera.getLatestResult());
        }

        showSnapshotStatus();
        telemetry.addLine(videoHint(status));
        telemetry.addLine("Video title shows the carrier pipeline \"" + pipelineName()
                + "\"; colors come from A/Y/B/X and every ball box is drawn yellow - use the list above for colors.");
    }

    private void showUnavailable() {
        String fault = camera.fault();
        telemetry.addData("!! Camera", fault);
        if (fault != null && fault.contains(MISSING_WEBCAM_TEXT)) {
            telemetry.addLine("Configure Robot: name the webcam \"" + camera.getWebcamName()
                    + "\" exactly, save, then restart the robot.");
        } else {
            telemetry.addLine("Check the USB cable (use the Control Hub USB 3.0 port), then restart this OpMode.");
        }
    }

    private void showBalls(LogiResult result) {
        telemetry.addData("Counts", "yellow %d   red %d   blue %d",
                result.getColorResults(YELLOW).size(),
                result.getColorResults(RED).size(),
                result.getColorResults(BLUE).size());

        List<ColorTarget> balls = new ArrayList<ColorTarget>(result.getColorResults());
        if (balls.isEmpty()) {
            telemetry.addData("Balls", "none seen");
            return;
        }
        boolean anyGround = false;
        int shown = Math.min(MAX_BALLS_SHOWN, balls.size());
        for (int i = 0; i < balls.size(); i++) {
            ColorTarget ball = balls.get(i);
            if (ball.hasGroundPosition()) {
                anyGround = true;
            }
            if (i < shown) {
                telemetry.addData("#" + (i + 1), describe(ball));
            }
        }
        if (balls.size() > shown) {
            telemetry.addData("More", "%d more not shown", balls.size() - shown);
        }
        LogiCameraConfig config = camera.getCameraConfig();
        if (!anyGround && config != null) {
            telemetry.addLine(String.format(Locale.US,
                    "No ground distance: mount says up %.1f in, tilt %+.0f deg (negative = down). "
                            + "Set them in Panels or LogiLight Mount Calibration once the camera is mounted.",
                    config.upInches, config.tiltUpDegrees));
        } else if (anyGround) {
            telemetry.addLine("Ground distances use the mount values in LogiLightSettings.CAMERA - only right once the camera is mounted and measured.");
        }
    }

    private void showSnapshotStatus() {
        if (snapshotPending) {
            String path = camera.lastSnapshotPath();
            if (path != null && !path.isEmpty() && !path.equals(snapshotPathBefore)) {
                snapshotPending = false;
                snapshotMessage = "snapshot saved: " + path;
            } else {
                snapshotMessage = "snapshot queued (needs a live frame)";
            }
        }
        if (!snapshotMessage.isEmpty()) {
            telemetry.addLine(snapshotMessage);
        }
    }

    private void handleButtons() {
        boolean start = gamepad1.start;
        boolean a = gamepad1.aWasPressed();
        boolean y = gamepad1.yWasPressed();
        boolean b = gamepad1.bWasPressed();
        boolean x = gamepad1.xWasPressed();
        boolean left = gamepad1.dpadLeftWasPressed();
        boolean right = gamepad1.dpadRightWasPressed();
        boolean up = gamepad1.dpadUpWasPressed();
        if (start) {
            return;
        }
        if (a) {
            mode = Mode.ALL;
        } else if (y) {
            mode = Mode.YELLOW_ONLY;
        } else if (b) {
            mode = Mode.RED_ONLY;
        } else if (x) {
            mode = Mode.BLUE_ONLY;
        }
        if (left || right) {
            StreamView view = camera.getStreamView();
            camera.setStreamView(left ? previous(view) : view.next());
        }
        if (up) {
            String before = camera.lastSnapshotPath();
            if (camera.captureSnapshot("balltest")) {
                snapshotPending = true;
                snapshotPathBefore = before == null ? "" : before;
                snapshotMessage = "snapshot queued (needs a live frame)";
            } else {
                snapshotPending = false;
                snapshotMessage = "!! snapshot not queued - camera not ready";
            }
        }
    }

    private void applyMode() {
        List<ColorProfile> profiles = new ArrayList<ColorProfile>();
        List<String> missing = new ArrayList<String>();
        List<String> disabled = new ArrayList<String>();
        for (String name : mode.colors) {
            ColorProfile profile = LogiLightSettings.findColorProfile(name);
            if (profile == null) {
                missing.add(name);
                continue;
            }
            if (!profile.enabled) {
                disabled.add(name);
            }
            profiles.add(profile);
        }
        camera.getFrameContext().setColorProfileOverride(profiles.toArray(new ColorProfile[0]));
        StringBuilder note = new StringBuilder();
        if (!missing.isEmpty()) {
            note.append("!! no color profile named ").append(missing).append(" in LogiLightSettings.COLOR_PROFILES");
        }
        if (!disabled.isEmpty()) {
            if (note.length() > 0) {
                note.append("   ");
            }
            note.append("!! color profile ").append(disabled)
                    .append(" is disabled (enabled = NO) - turn it on in Panels or LogiLight Color Tuner");
        }
        profileNote = note.toString();
    }

    private void selectColorPipeline() {
        int count = LogiLightSettings.pipelineCount();
        int fallback = -1;
        for (int i = 0; i < count; i++) {
            LogiPipeline pipeline = LogiLightSettings.pipelineAt(i);
            if (pipeline == null) {
                continue;
            }
            PipelineType type = pipeline.typeOrDefault();
            if (type == PipelineType.COLOR) {
                camera.pipelineSwitch(i);
                return;
            }
            if (fallback < 0 && type.usesColor()) {
                fallback = i;
            }
        }
        if (fallback >= 0) {
            camera.pipelineSwitch(fallback);
            pipelineNote = "no COLOR-only pipeline - using " + LogiLightSettings.pipelineAt(fallback).name;
        } else {
            pipelineNote = "!! no pipeline that detects color in LogiLightSettings.PIPELINES - balls cannot be seen";
        }
    }

    private String pipelineName() {
        LogiPipeline pipeline = camera.getPipeline();
        return pipeline == null || pipeline.name == null ? "?" : pipeline.name;
    }

    private static String videoHint(LogiStatus status) {
        if (status.driverStationStreaming && status.panelsStreaming) {
            return "dpad up = snapshot   Video: DS menu > Camera Stream during INIT, or Panels (http://192.168.43.1:8001)";
        }
        if (status.driverStationStreaming) {
            return "dpad up = snapshot   Video: DS menu > Camera Stream during INIT";
        }
        if (status.panelsStreaming) {
            return "dpad up = snapshot   Video: Panels (http://192.168.43.1:8001)";
        }
        return "dpad up = snapshot   !! DS and Panels streams are off (DRIVER_STATION_STREAM / PANELS_STREAM in LogiLightSettings)";
    }

    private static String formatAge(double ageMs) {
        return Double.isNaN(ageMs) || Double.isInfinite(ageMs) ? "--" : String.format(Locale.US, "%.0f ms", ageMs);
    }

    private static String describe(ColorTarget ball) {
        StringBuilder line = new StringBuilder();
        line.append(ball.profileName == null ? "?" : ball.profileName);
        line.append(String.format(Locale.US, "  tx %+.1f  ty %+.1f  ta %.2f%%  %.0fx%.0f px",
                ball.tx, ball.ty, ball.ta, ball.boxWidth, ball.boxHeight));
        if (ball.hasGroundPosition()) {
            line.append(String.format(Locale.US, "  %.1f in ahead  %+.1f in left",
                    ball.groundForwardInches, ball.groundLeftInches));
        }
        return line.toString();
    }

    private static StreamView previous(StreamView view) {
        StreamView[] views = StreamView.values();
        int index = (view.ordinal() - 1 + views.length) % views.length;
        return views[index];
    }
}
