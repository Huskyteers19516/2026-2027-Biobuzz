package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.logilight.LogiCameraConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightStore;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiPipeline;
import org.firstinspires.ftc.teamcode.vision.logilight.PipelineType;

public abstract class LogiOpModeBase extends OpMode {

    public static final int TELEMETRY_INTERVAL_MS = 100;
    public static final double MESSAGE_SHOW_SECONDS = 8.0;
    public static final double MAX_LOOP_SECONDS = 0.25;
    public static final int SAVE_RUMBLE_MS = 250;
    public static final int RESET_RUMBLE_BLIPS = 2;
    public static final String SETTINGS_HINT = "gp1 LB hold + Y = save settings   LB hold + X twice = reset to code defaults";
    public static final String STREAM_HINT = "Video: Panels camera stream, or DS menu > Camera Stream during INIT";

    protected LogiLight ll;
    protected LogiOdometryFeed odometry;
    protected final LogiPadEdges pad1 = new LogiPadEdges();
    protected final LogiPadEdges pad2 = new LogiPadEdges();
    protected double loopSeconds;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime messageTimer = new ElapsedTime();
    private String createFault = "";
    private String message = "";
    private String loopError = "";
    private boolean started;

    protected boolean usesOdometry() {
        return false;
    }

    protected void onInit() {
    }

    protected void onStart() {
    }

    protected abstract void onLoop();

    protected void onStop() {
    }

    protected void onSettingsReset() {
    }

    @Override
    public final void init() {
        try {
            telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        } catch (Throwable ignored) {
        }
        ll = createLight();
        if (usesOdometry()) {
            odometry = new LogiOdometryFeed(hardwareMap);
        }
        if (ll != null) {
            try {
                onInit();
            } catch (RuntimeException e) {
                loopError = "init: " + describe(e);
            }
        }
        loopTimer.reset();
        if (ll == null) {
            telemetry.addLine("!! LogiLight could not be created - " + createFault);
        } else if (!ll.isAvailable()) {
            telemetry.addLine("!! CAMERA NOT AVAILABLE - " + ll.fault());
        } else {
            telemetry.addData("LogiLight", "camera opening ...");
        }
        if (odometry != null && !odometry.isAvailable()) {
            telemetry.addData("Pinpoint", odometry.summary());
        }
    }

    @Override
    public final void init_loop() {
        tick();
    }

    @Override
    public final void start() {
        started = true;
        loopTimer.reset();
        if (ll != null) {
            try {
                onStart();
            } catch (RuntimeException e) {
                loopError = "start: " + describe(e);
            }
        }
    }

    @Override
    public final void loop() {
        tick();
    }

    @Override
    public final void stop() {
        try {
            if (ll != null) {
                onStop();
            }
        } catch (RuntimeException ignored) {
        }
        if (ll != null) {
            try {
                ll.close();
            } catch (Throwable ignored) {
            }
        }
    }

    protected final boolean isStarted() {
        return started;
    }

    protected final boolean comboHeld() {
        return gamepad1 != null && gamepad1.left_bumper;
    }

    protected final void showMessage(String text) {
        message = text == null ? "" : text;
        messageTimer.reset();
    }

    protected final LogiCameraConfig camera() {
        LogiCameraConfig config = ll == null ? null : ll.getCameraConfig();
        return config == null ? LogiLightSettings.CAMERA : config;
    }

    protected final boolean switchPipelineBy(int delta) {
        int count = LogiLightSettings.pipelineCount();
        if (ll == null || count <= 0) {
            return false;
        }
        int next = ((ll.getPipelineIndex() + delta) % count + count) % count;
        boolean ok = ll.pipelineSwitch(next);
        showMessage(ok ? "pipeline " + pipelineLabel(next) : "!! pipeline switch failed");
        return ok;
    }

    protected final boolean ensurePipelineOfType(PipelineType wanted) {
        if (ll == null || wanted == null) {
            return false;
        }
        LogiPipeline current = ll.getPipeline();
        if (current != null && current.typeOrDefault() == wanted) {
            return true;
        }
        int index = findPipelineOfType(wanted, 0);
        return index >= 0 && ll.pipelineSwitch(index);
    }

    protected final boolean cyclePipelineOfType(PipelineType wanted) {
        if (ll == null || wanted == null) {
            return false;
        }
        int index = findPipelineOfType(wanted, ll.getPipelineIndex() + 1);
        if (index < 0) {
            showMessage("!! no " + wanted + " pipeline in LogiLightSettings.PIPELINES");
            return false;
        }
        boolean ok = ll.pipelineSwitch(index);
        showMessage(ok ? "pipeline " + pipelineLabel(index) : "!! pipeline switch failed");
        return ok;
    }

    protected static int findPipelineOfType(PipelineType wanted, int startIndex) {
        int count = LogiLightSettings.pipelineCount();
        for (int offset = 0; offset < count; offset++) {
            int index = ((startIndex + offset) % count + count) % count;
            LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
            if (pipeline != null && pipeline.typeOrDefault() == wanted) {
                return index;
            }
        }
        return -1;
    }

    protected static String pipelineLabel(int index) {
        LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
        if (pipeline == null) {
            return index + " (missing)";
        }
        return index + "/" + LogiLightSettings.pipelineCount() + " " + pipeline.name + " (" + pipeline.typeOrDefault() + ")";
    }

    protected final void addPipelineLine() {
        if (ll == null) {
            return;
        }
        telemetry.addData("Pipeline", pipelineLabel(ll.getPipelineIndex()));
    }

    protected final void addHint(String line) {
        telemetry.addLine(line);
    }

    static String describe(Throwable t) {
        if (t == null) {
            return "unknown error";
        }
        String text = t.getMessage();
        String name = t.getClass().getSimpleName();
        return text == null || text.isEmpty() ? name : name + ": " + text;
    }

    private LogiLight createLight() {
        try {
            return new LogiLight(hardwareMap);
        } catch (Throwable first) {
            createFault = describe(first);
            try {
                return new LogiLight(null);
            } catch (Throwable second) {
                createFault = createFault + " / " + describe(second);
                return null;
            }
        }
    }

    private void tick() {
        loopSeconds = Math.max(0.0, Math.min(MAX_LOOP_SECONDS, loopTimer.seconds()));
        loopTimer.reset();
        pad1.read(gamepad1);
        pad2.read(gamepad2);
        if (odometry != null) {
            odometry.update();
            odometry.feed(ll);
        }
        if (ll == null) {
            telemetry.addLine("!! LogiLight could not be created - " + createFault);
            telemetry.addLine("Nothing works in this OpMode until that is fixed.");
            return;
        }
        ll.update();
        handleSettingsCombo();
        if (!ll.isAvailable()) {
            telemetry.addLine("!! CAMERA NOT AVAILABLE - " + ll.fault());
            telemetry.addLine("!! Check the robot configuration has a webcam named \"" + camera().webcamName + "\"");
        }
        try {
            onLoop();
            loopError = "";
        } catch (RuntimeException e) {
            loopError = describe(e);
        }
        if (!loopError.isEmpty()) {
            telemetry.addData("!! OpMode error", loopError);
        }
        if (!message.isEmpty() && messageTimer.seconds() <= MESSAGE_SHOW_SECONDS) {
            telemetry.addData("Last action", message);
        }
    }

    private void handleSettingsCombo() {
        if (!comboHeld()) {
            return;
        }
        if (pad1.y()) {
            LogiOpModeTools.disarmConfirm();
            saveSettings();
        } else if (pad1.x()) {
            if (LogiOpModeTools.confirmTwice(LogiOpModeTools.RESET_CONFIRM_KEY)) {
                resetSettings();
            } else {
                showMessage(LogiOpModeTools.resetArmedMessage());
                rumble();
            }
        }
    }

    private void saveSettings() {
        LogiLightStore.clearLastError();
        boolean ok = LogiLightStore.saveSettings();
        String problem = LogiLightStore.lastError();
        if (ok) {
            String text = "SAVED - " + LogiLightStore.settingsSource();
            if (problem != null && !problem.isEmpty()) {
                text += "  !! " + problem;
            }
            showMessage(text);
            rumble();
        } else {
            showMessage("!! SAVE FAILED - " + problem);
        }
    }

    private void resetSettings() {
        boolean hadFile = LogiLightStore.hasSavedSettings();
        LogiLightStore.clearLastError();
        boolean deleted = !hadFile || LogiLightStore.deleteSavedSettings();
        LogiLightSettings.resetToCodeDefaults();
        try {
            ll.refreshMount();
        } catch (Throwable ignored) {
        }
        String text = "RESET to code defaults";
        if (hadFile) {
            text += deleted ? " - old " + LogiLightStore.SETTINGS_FILE_NAME + " moved to "
                    + LogiLightStore.settingsBackupFile().getName()
                    : "  !! could not move " + LogiLightStore.SETTINGS_FILE_NAME + ": " + LogiLightStore.lastError();
        }
        showMessage(text);
        try {
            if (gamepad1 != null) {
                gamepad1.rumbleBlips(RESET_RUMBLE_BLIPS);
            }
        } catch (Throwable ignored) {
        }
        try {
            onSettingsReset();
        } catch (RuntimeException e) {
            loopError = "reset: " + describe(e);
        }
    }

    private void rumble() {
        try {
            if (gamepad1 != null) {
                gamepad1.rumble(SAVE_RUMBLE_MS);
            }
        } catch (Throwable ignored) {
        }
    }
}
