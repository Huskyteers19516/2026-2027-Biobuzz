package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.compat.LogiLimelight3A;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "LogiLight Limelight Compat Demo", group = "LogiLight")
public class LogiLightLimelightCompatDemo extends OpMode {

    public static final String LIMELIGHT_NAME = "limelight";
    public static final int POLL_RATE_HZ = 100;
    public static final int MAX_PIPELINE_INDEX = 9;
    public static final int MAX_FIDUCIAL_LINES = 6;
    public static final int MAX_COLOR_LINES = 4;
    public static final int PYTHON_OUTPUTS_SHOWN = 4;
    private static final double[] DEFAULT_PYTHON_INPUTS = {0.25, 0.5, 0.75, 1.0, 0.0, 0.0, 0.0, 0.0};
    private static final String PYTHON_PIPELINE_TYPE = "pipe_python";

    private LogiLimelight3A installed;
    private Limelight3A limelight;
    private String setupNote = "";
    private Localizer localizer;
    private String localizerFault = "";
    private int pipeline = 0;
    private String message = "";

    @Override
    public void init() {
        try {
            installed = LogiLimelight3A.install(hardwareMap, LIMELIGHT_NAME, Constants.WEBCAM);
            Limelight3A found = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);
            if (found instanceof LogiLimelight3A) {
                limelight = found;
            } else {
                limelight = installed;
                setupNote = installed.installWarning().isEmpty()
                        ? "using the installed LogiLight device directly"
                        : installed.installWarning() + " - using the installed LogiLight device directly";
            }
        } catch (Throwable t) {
            installed = null;
            limelight = null;
            setupNote = "!! LogiLight install failed - " + LogiOpModeTools.describe(t);
        }

        LogiOpModeTools.PinpointHandle handle = LogiOpModeTools.createPinpoint(hardwareMap);
        localizer = handle.localizer;
        localizerFault = handle.fault;
        if (localizer != null) {
            Pose handoff = PoseHandoff.pose();
            try {
                localizer.setPose(handoff != null ? handoff : LogiOpModeTools.startPose(PoseHandoff.alliance()));
            } catch (Throwable t) {
                localizerFault = "!! Pinpoint setPose failed - " + LogiOpModeTools.describe(t);
            }
        }

        if (limelight != null) {
            limelight.setPollRateHz(POLL_RATE_HZ);
            limelight.pipelineSwitch(pipeline);
            limelight.updatePythonInputs(DEFAULT_PYTHON_INPUTS);
            limelight.start();
        }
    }

    @Override
    public void init_loop() {
        handleButtons();
        showLimelight();
    }

    @Override
    public void start() {
        PoseHandoff.clear();
    }

    @Override
    public void loop() {
        handleButtons();
        showLimelight();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
        if (installed != null) {
            installed.uninstall();
        }
    }

    private void handleButtons() {
        boolean leftPressed = gamepad1.dpadLeftWasPressed();
        boolean rightPressed = gamepad1.dpadRightWasPressed();
        boolean aPressed = gamepad1.aWasPressed();
        boolean xPressed = gamepad1.xWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean leftBumper = gamepad1.left_bumper;

        String combo = LogiOpModeTools.settingsCombo(leftBumper, yPressed, xPressed);
        if (combo != null) {
            message = combo;
        }
        if (limelight == null) {
            return;
        }
        if (rightPressed) {
            switchPipeline(pipeline >= MAX_PIPELINE_INDEX ? 0 : pipeline + 1, true);
        }
        if (leftPressed) {
            switchPipeline(pipeline <= 0 ? MAX_PIPELINE_INDEX : pipeline - 1, false);
        }
        if (aPressed) {
            String name = String.format(Locale.US, "compat_%d", System.currentTimeMillis() / 1000L);
            message = limelight.captureSnapshot(name) ? "snapshot requested: " + name : "!! snapshot failed";
        }
    }

    private void switchPipeline(int wanted, boolean upward) {
        int candidate = wanted;
        for (int attempt = 0; attempt <= MAX_PIPELINE_INDEX; attempt++) {
            if (limelight.pipelineSwitch(candidate)) {
                pipeline = candidate;
                message = "pipeline " + candidate;
                return;
            }
            if (upward) {
                candidate = candidate >= MAX_PIPELINE_INDEX ? 0 : candidate + 1;
            } else {
                candidate = candidate <= 0 ? MAX_PIPELINE_INDEX : candidate - 1;
            }
        }
        message = "!! no pipeline accepted the switch";
    }

    private void showLimelight() {
        telemetry.addLine("LogiLight Limelight Compat Demo (Limelight3A API only)");
        if (!setupNote.isEmpty()) {
            telemetry.addLine(setupNote);
        }
        if (limelight == null) {
            telemetry.addData("Limelight", "NOT AVAILABLE");
            return;
        }

        if (localizer != null) {
            try {
                localizer.update();
                Pose pose = localizer.pose();
                if (pose != null) {
                    limelight.updateRobotOrientation(Math.toDegrees(pose.heading()));
                    telemetry.addData("Robot yaw", "%.1f deg (Pinpoint, sent to updateRobotOrientation)",
                            Math.toDegrees(pose.heading()));
                }
            } catch (Throwable t) {
                localizerFault = "!! Pinpoint read failed - " + LogiOpModeTools.describe(t);
            }
        }
        if (!localizerFault.isEmpty()) {
            telemetry.addData("Pinpoint", "%s  (MegaTag2 needs a heading)", localizerFault);
        }

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData("Connected", "%s  (%d ms since last update)", limelight.isConnected() ? "yes" : "NO",
                limelight.getTimeSinceLastUpdate());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("tx", "%.2f", result.getTx());
            telemetry.addData("ty", "%.2f", result.getTy());
            telemetry.addData("ta", "%.2f", result.getTa());
            telemetry.addData("txnc / tync", "%.2f / %.2f", result.getTxNC(), result.getTyNC());
            telemetry.addData("Latency", "capture %.0f ms  targeting %.0f ms  staleness %d ms",
                    result.getCaptureLatency(), result.getTargetingLatency(), result.getStaleness());
            telemetry.addData("Pipeline result", "pID %d  %s", result.getPipelineIndex(), result.getPipelineType());

            if (result.getBotposeTagCount() > 0) {
                telemetry.addData("Botpose MT1", formatBotpose(result.getBotpose()));
                telemetry.addData("Botpose MT2", formatBotpose(result.getBotpose_MT2()));
                telemetry.addData("Botpose stats", "tags %d  span %.2f m  avg dist %.2f m  avg area %.2f",
                        result.getBotposeTagCount(), result.getBotposeSpan(), result.getBotposeAvgDist(),
                        result.getBotposeAvgArea());
            }

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            int shown = 0;
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (shown++ >= MAX_FIDUCIAL_LINES) {
                    break;
                }
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f, ta %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees(),
                        fr.getTargetArea());
                Pose3D fieldPose = fr.getRobotPoseFieldSpace();
                Pose3D targetPose = fr.getTargetPoseRobotSpace();
                telemetry.addData("  robot (field)", formatBotpose(fieldPose));
                telemetry.addData("  target (robot)", formatPosition(targetPose));
            }

            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            shown = 0;
            for (LLResultTypes.ColorResult cr : colorResults) {
                if (shown++ >= MAX_COLOR_LINES) {
                    break;
                }
                telemetry.addData("Color", "X: %.2f, Y: %.2f, ta %.2f, px (%.0f, %.0f)", cr.getTargetXDegrees(),
                        cr.getTargetYDegrees(), cr.getTargetArea(), cr.getTargetXPixels(), cr.getTargetYPixels());
            }

            if (PYTHON_PIPELINE_TYPE.equals(result.getPipelineType())) {
                double[] outputs = result.getPythonOutput();
                StringBuilder line = new StringBuilder();
                for (int i = 0; i < Math.min(PYTHON_OUTPUTS_SHOWN, outputs.length); i++) {
                    line.append(String.format(Locale.US, "%s%.2f", i == 0 ? "" : "  ", outputs[i]));
                }
                telemetry.addData("Python out", line.toString());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        if (!message.isEmpty()) {
            telemetry.addData("Note", message);
        }
        telemetry.addLine("gp1 dpad L/R = pipeline   A = snapshot");
        telemetry.addLine("LogiLimelight3A.install() runs in every OpMode; the device leaves hardwareMap when the OpMode stops");
        telemetry.addLine(LogiOpModeTools.settingsHint());
    }

    private static String formatBotpose(Pose3D pose) {
        if (pose == null) {
            return "none";
        }
        Position meters = pose.getPosition().toUnit(DistanceUnit.METER);
        Position inches = pose.getPosition().toUnit(DistanceUnit.INCH);
        double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
        return String.format(Locale.US, "x %.3f m  y %.3f m  yaw %.1f deg   (Pedro x %.1f  y %.1f in)",
                meters.x, meters.y, yaw,
                inches.x + LogiLimelight3A.FIELD_CENTER_INCHES, inches.y + LogiLimelight3A.FIELD_CENTER_INCHES);
    }

    private static String formatPosition(Pose3D pose) {
        if (pose == null) {
            return "none";
        }
        Position inches = pose.getPosition().toUnit(DistanceUnit.INCH);
        return String.format(Locale.US, "x %.1f  y %.1f  z %.1f in", inches.x, inches.y, inches.z);
    }
}
