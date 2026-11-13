package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.logilight.ColorProfile;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
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
    private String message = "";

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
            telemetry.addData("!! Camera", "NOT AVAILABLE - %s", camera.fault());
            telemetry.addLine("Check Configure Robot: the webcam must be named \""
                    + LogiLightSettings.CAMERA.webcamName + "\".");
            return;
        }

        LogiStatus status = camera.getStatus();
        telemetry.addData("Camera", "%s  %.0f fps  frame age %.0f ms",
                status.cameraState, status.fps, status.lastFrameAgeMs);
        telemetry.addData("Looking for", "%s   (A all  Y yellow  B red  X blue)", mode.label);
        telemetry.addData("Stream view", "%s   (dpad L/R)", camera.getStreamView());
        if (!pipelineNote.isEmpty()) {
            telemetry.addLine(pipelineNote);
        }

        LogiResult result = camera.getLatestResult();
        telemetry.addData("Counts", "yellow %d   red %d   blue %d",
                result.getColorResults(YELLOW).size(),
                result.getColorResults(RED).size(),
                result.getColorResults(BLUE).size());

        List<ColorTarget> balls = new ArrayList<ColorTarget>(result.getColorResults());
        if (balls.isEmpty()) {
            telemetry.addData("Balls", "none seen");
        } else {
            int shown = Math.min(MAX_BALLS_SHOWN, balls.size());
            for (int i = 0; i < shown; i++) {
                telemetry.addData("#" + (i + 1), describe(balls.get(i)));
            }
            if (balls.size() > shown) {
                telemetry.addData("More", "%d more not shown", balls.size() - shown);
            }
        }

        if (LogiLightSettings.CAMERA.tiltUpDegrees >= 0.0) {
            telemetry.addLine("Ground distance needs a downward camera: set LogiLightSettings.CAMERA "
                    + "tiltUpDegrees (negative = down) and upInches.");
        }
        if (!message.isEmpty()) {
            telemetry.addLine(message);
        }
        telemetry.addLine("dpad up = snapshot   Video: DS menu > Camera Stream during INIT, or Panels");
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
            boolean queued = camera.captureSnapshot("balltest");
            message = queued ? "snapshot queued - saved under /sdcard/FIRST/logilight/snapshots"
                    : "!! snapshot failed";
        }
    }

    private void applyMode() {
        List<ColorProfile> profiles = new ArrayList<ColorProfile>();
        List<String> missing = new ArrayList<String>();
        for (String name : mode.colors) {
            ColorProfile profile = LogiLightSettings.findColorProfile(name);
            if (profile == null) {
                missing.add(name);
            } else {
                profiles.add(profile);
            }
        }
        camera.getFrameContext().setColorProfileOverride(profiles.toArray(new ColorProfile[0]));
        if (!missing.isEmpty()) {
            pipelineNote = "!! no color profile named " + missing + " in LogiLightSettings.COLOR_PROFILES";
        } else if (pipelineNote.startsWith("!! no color profile")) {
            pipelineNote = "";
        }
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
            pipelineNote = "!! no pipeline that detects color in LogiLightSettings.PIPELINES";
        }
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
