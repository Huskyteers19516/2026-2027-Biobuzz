package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiBotPose;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFieldMap;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightFusion;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLocalizationConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;

import java.util.Locale;

@Configurable
@TeleOp(name = "LogiLight Localization Test", group = "LogiLight")
public class LogiLightLocalizationTest extends OpMode {

    public static double SLOW_SCALE = 0.35;
    public static double MAX_SNAP_AGE_MS = 300.0;

    private LogiLight light;
    private LogiMecanumDrive drive;
    private Localizer pinpoint;
    private String pinpointFault = "";
    private LogiLightFusion fusion;
    private String fusionFault = "";

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private boolean slowMode = false;
    private Pose odometryPose;
    private Pose fusedPose;
    private long fusedCaptures = 0L;
    private long lastOfferedCaptureNanos = 0L;
    private String message = "";

    @Override
    public void init() {
        light = new LogiLight(hardwareMap);
        light.pipelineSwitch(LogiOpModeTools.goalsPipelineIndex());
        drive = new LogiMecanumDrive(hardwareMap);

        alliance = PoseHandoff.alliance();
        LogiOpModeTools.PinpointHandle handle = LogiOpModeTools.createPinpoint(hardwareMap);
        pinpoint = handle.localizer;
        pinpointFault = handle.fault;
        if (pinpoint != null) {
            try {
                fusion = new LogiLightFusion(pinpoint, LogiOpModeTools.startPose(alliance));
            } catch (Throwable t) {
                fusion = null;
                fusionFault = "NOT AVAILABLE - " + LogiOpModeTools.describe(t);
            }
        }
    }

    @Override
    public void init_loop() {
        boolean xPressed = gamepad1.xWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean backPressed = gamepad1.backWasPressed();
        boolean leftBumper = gamepad1.left_bumper;

        String combo = LogiOpModeTools.settingsCombo(leftBumper, yPressed, xPressed);
        if (combo != null) {
            message = combo;
        }
        if (!leftBumper) {
            GoalScanner.Alliance picked = LogiOpModeTools.allianceFromButtons(alliance,
                    LogiOpModeTools.pressedWithoutStart(xPressed, gamepad1),
                    LogiOpModeTools.pressedWithoutStart(bPressed, gamepad1));
            if (picked != alliance) {
                alliance = picked;
                rezero();
            }
        }
        if (backPressed) {
            rezero();
        }

        LogiResult result = tick();
        telemetry.addLine("LogiLight Localization Test - nothing moves until START");
        telemetry.addData("Alliance", "%s   (gp1 X = BLUE, B = RED, BACK = re-zero)", alliance);
        addPoseTelemetry(result);
    }

    @Override
    public void start() {
        gamepad1.resetEdgeDetection();
        message = "";
    }

    @Override
    public void loop() {
        boolean xPressed = gamepad1.xWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean backPressed = gamepad1.backWasPressed();
        boolean upPressed = gamepad1.dpadUpWasPressed();
        boolean slowPressed = gamepad1.leftStickButtonWasPressed();
        boolean leftBumper = gamepad1.left_bumper;

        String combo = LogiOpModeTools.settingsCombo(leftBumper, yPressed, xPressed);
        if (combo != null) {
            message = combo;
        }
        if (!leftBumper) {
            if (xPressed) {
                toggleFusion();
            }
            if (yPressed) {
                snapToVision();
            }
        }
        if (backPressed) {
            rezero();
        }
        if (upPressed) {
            LogiLocalizationConfig loc = LogiLightSettings.LOCALIZATION;
            if (loc != null) {
                loc.fuseMt1Heading = !loc.fuseMt1Heading;
                message = "MT1 heading fusion " + (loc.fuseMt1Heading ? "ON" : "OFF");
            }
        }
        if (slowPressed) {
            slowMode = !slowMode;
        }

        double scale = slowMode ? SLOW_SCALE : 1.0;
        drive.drive(LogiOpModeTools.deadzone(-gamepad1.left_stick_y),
                -LogiOpModeTools.deadzone(gamepad1.left_stick_x),
                -LogiOpModeTools.deadzone(gamepad1.right_stick_x), scale, 0.0, 0.0);

        LogiResult result = tick();
        telemetry.addLine("LogiLight Localization Test");
        addPoseTelemetry(result);
        telemetry.addLine("gp1 X = fusion on/off   Y = snap pose to vision   dpad up = MT1 heading fusion");
        telemetry.addLine("gp1 sticks drive   L-stick btn = slow   BACK = re-zero at start pose");
        telemetry.addLine(LogiOpModeTools.settingsHint());
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (light != null) {
            light.close();
        }
    }

    private LogiResult tick() {
        if (fusion != null) {
            try {
                fusion.update();
                fusion.publishHeading(light);
            } catch (Throwable t) {
                fusionFault = "!! fusion update failed - " + LogiOpModeTools.describe(t);
            }
        } else if (pinpoint != null) {
            try {
                pinpoint.update();
                Pose raw = pinpoint.pose();
                if (raw != null) {
                    light.updateRobotOrientation(raw.heading());
                }
            } catch (Throwable t) {
                pinpointFault = "!! Pinpoint read failed - " + LogiOpModeTools.describe(t);
            }
        }
        light.update();
        LogiResult result = light.getLatestResult();
        long capture = result.getCaptureNanos();
        if (fusion != null && fusion.isEnabled() && result.hasBotpose() && capture != lastOfferedCaptureNanos) {
            lastOfferedCaptureNanos = capture;
            if (fusion.fuse(result)) {
                fusedCaptures++;
            }
        }
        odometryPose = pinpoint != null ? safePose(pinpoint) : null;
        fusedPose = fusion != null ? fusion.pose() : null;
        LogiLightField.drawResult(odometryPose, result, fusedPose, light.getFieldMap());
        return result;
    }

    private static Pose safePose(Localizer localizer) {
        try {
            return localizer.pose();
        } catch (Throwable t) {
            return null;
        }
    }

    private void toggleFusion() {
        if (fusion == null) {
            message = "!! no fusion - Pinpoint is missing";
            return;
        }
        fusion.setEnabled(!fusion.isEnabled());
        message = "fusion " + (fusion.isEnabled() ? "ON" : "OFF - fused pose now follows odometry only");
    }

    private void snapToVision() {
        if (fusion == null) {
            message = "!! cannot snap - Pinpoint is missing";
            return;
        }
        LogiResult result = light.getLatestResult();
        LogiBotPose mt1 = result.getBotpose();
        LogiBotPose mt2 = result.getBotpose_MT2();
        Pose current = fusion.pose();
        if (mt1 != null && mt1.isFinite() && mt1.hasHeading() && mt1.ageMs() <= MAX_SNAP_AGE_MS) {
            fusion.setPose(mt1.toPedroPose());
            message = "snapped to MT1 " + LogiLightTelemetry.formatPose(mt1.pose);
            return;
        }
        if (mt2 != null && mt2.pose != null && mt2.pose.isPositionFinite() && mt2.ageMs() <= MAX_SNAP_AGE_MS
                && current != null) {
            fusion.setPose(new Pose(mt2.pose.x, mt2.pose.y, current.heading()));
            message = "snapped position to MT2 " + LogiLightTelemetry.formatPose(mt2.pose);
            return;
        }
        message = "!! no fresh vision pose to snap to";
    }

    private void rezero() {
        Pose start = LogiOpModeTools.startPose(alliance);
        if (fusion != null) {
            fusion.setPose(start);
        } else if (pinpoint != null) {
            try {
                pinpoint.setPose(start);
            } catch (Throwable t) {
                pinpointFault = "!! Pinpoint setPose failed - " + LogiOpModeTools.describe(t);
            }
        }
        message = "re-zeroed at the " + alliance + " start pose " + LogiLightTelemetry.formatPose(start);
    }

    private void addPoseTelemetry(LogiResult result) {
        LogiBotPose mt1 = result.getBotpose();
        LogiBotPose mt2 = result.getBotpose_MT2();
        telemetry.addData("Odometry", pinpoint == null ? "NOT AVAILABLE  " + pinpointFault
                : LogiLightTelemetry.formatPose(odometryPose));
        telemetry.addData("Vision MT1", mt1 == null ? "none" : LogiLightTelemetry.botPoseLine(mt1));
        telemetry.addData("Vision MT2", mt2 == null ? (pinpoint == null ? "none (needs heading)" : "none")
                : LogiLightTelemetry.botPoseLine(mt2));
        if (fusion == null) {
            telemetry.addData("Fused", "NOT AVAILABLE  %s", fusionFault.isEmpty() ? "needs Pinpoint" : fusionFault);
        } else {
            telemetry.addData("Fused", LogiLightTelemetry.formatPose(fusedPose));
            telemetry.addData("Fusion", fusion.summary());
            double sinceMs = fusion.msSinceLastAccepted();
            telemetry.addData("Last fix", LogiOpModeTools.isFinite(sinceMs)
                    ? String.format(Locale.US, "%.0f ms ago  (%d fused this run)", sinceMs, fusedCaptures)
                    : "never");
            if (odometryPose != null && fusedPose != null) {
                double dx = fusedPose.x() - odometryPose.x();
                double dy = fusedPose.y() - odometryPose.y();
                telemetry.addData("Correction", "%.1f in  (dx %+.1f  dy %+.1f)  fused minus odometry",
                        Math.hypot(dx, dy), dx, dy);
            }
            LogiBotPose best = result.getBestBotpose();
            if (best != null) {
                telemetry.addData("Residual", "%s  vision minus fused", LogiOpModeTools.poseError(best, fusedPose));
            }
            if (!fusionFault.isEmpty()) {
                telemetry.addLine(fusionFault);
            }
        }
        LogiFieldMap map = light.getFieldMap();
        telemetry.addData("Field map", map == null ? "none" : map.summary());
        LogiLocalizationConfig loc = LogiLightSettings.LOCALIZATION;
        if (map != null && map.surveyedCount() == 0 && (loc == null || !loc.useUnsurveyedTargets)) {
            telemetry.addLine("!! NO SURVEYED TARGETS - run LogiLight Field Survey first");
            telemetry.addLine("   (LOCALIZATION.useUnsurveyedTargets = MT2 only, needs the Pinpoint heading, rough until surveyed)");
        } else if (map != null && map.surveyedCount() == 0) {
            telemetry.addLine("!! unsurveyed targets: MT2 only (Pinpoint heading), rough until LogiLight Field Survey is run");
        }
        if (loc != null) {
            telemetry.addData("MT1 heading fusion", loc.fuseMt1Heading ? "ON" : "OFF  (position only, Pinpoint heading)");
        }
        if (!drive.isAvailable()) {
            telemetry.addLine("!! DRIVE MOTORS " + drive.fault());
        }
        if (!message.isEmpty()) {
            telemetry.addData("Note", message);
        }
        LogiLightTelemetry.addResult(telemetry, result);
        LogiLightTelemetry.addStatus(telemetry, light);
    }
}
