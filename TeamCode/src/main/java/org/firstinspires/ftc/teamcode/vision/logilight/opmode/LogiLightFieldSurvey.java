package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiBotPose;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFieldMap;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightStore;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLocalizationConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;
import org.firstinspires.ftc.teamcode.vision.logilight.math.FieldTargetState;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.SurveyInfo;
import org.firstinspires.ftc.teamcode.vision.logilight.math.TargetGeometry;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "LogiLight Field Survey", group = "LogiLight")
public class LogiLightFieldSurvey extends OpMode {

    public static final int MAX_SAMPLES_PER_TARGET = 60;
    public static final int MIN_SAMPLES_TO_SAVE = 10;
    public static final double MAX_STILL_SPEED_INCHES_PER_SECOND = 0.5;
    public static final double MAX_STILL_TURN_DEGREES_PER_SECOND = 2.0;
    public static final double STILL_SETTLE_SECONDS = 0.3;
    public static final double MAX_VALIDATION_AGE_MS = 300.0;
    public static final String LOG_TAG = "LogiLightSurvey";

    private static final String[] STATE_LABELS = {LogiFieldMap.DEFAULT_STATE_LABEL, "tipped-a", "tipped-b"};

    private LogiLight light;
    private Localizer localizer;
    private String localizerFault = "";
    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private Pose robotPose;
    private int goalsPipeline;

    private final Map<String, Accumulator> accumulators = new LinkedHashMap<String, Accumulator>();
    private final List<String> savedLines = new ArrayList<String>();
    private boolean averaging = false;
    private int labelIndex = 0;
    private long lastSampledCaptureNanos = 0L;
    private int skippedMovingFrames = 0;
    private double stillSince = -1.0;
    private String message = "";

    private static final class Accumulator {
        final String name;
        final List<FieldTargetState> samples = new ArrayList<FieldTargetState>();
        double distanceSum = 0.0;
        int distanceCount = 0;
        int minPercent = 100;
        double poseXSum = 0.0;
        double poseYSum = 0.0;
        double poseSinSum = 0.0;
        double poseCosSum = 0.0;
        int poseCount = 0;
        CameraModel mount;

        Accumulator(String name) {
            this.name = name;
        }

        boolean isFull() {
            return samples.size() >= MAX_SAMPLES_PER_TARGET;
        }

        void add(FieldTargetState sample, LogiFiducial fiducial, PlanarPose robot, CameraModel model) {
            samples.add(sample);
            if (robot != null && robot.isFinite()) {
                poseXSum += robot.x;
                poseYSum += robot.y;
                poseSinSum += Math.sin(robot.heading);
                poseCosSum += Math.cos(robot.heading);
                poseCount++;
            }
            if (model != null) {
                mount = model;
            }
            if (LogiOpModeTools.isFinite(fiducial.robotDistanceInches)) {
                distanceSum += fiducial.robotDistanceInches;
                distanceCount++;
            }
            minPercent = Math.min(minPercent, fiducial.percentClusterFound);
        }

        double averageDistance() {
            return distanceCount == 0 ? Double.NaN : distanceSum / distanceCount;
        }

        FieldTargetState average(String label) {
            return FieldTargetState.average(label, samples);
        }

        SurveyInfo surveyInfo() {
            if (poseCount == 0 || mount == null) {
                return null;
            }
            PlanarPose robot = new PlanarPose(poseXSum / poseCount, poseYSum / poseCount,
                    Math.atan2(poseSinSum, poseCosSum));
            SurveyInfo info = SurveyInfo.of(robot, mount);
            return info != null && info.isFinite() ? info : null;
        }

        double originSpreadInches(FieldTargetState mean) {
            if (mean == null || samples.isEmpty()) {
                return Double.NaN;
            }
            double sum = 0.0;
            for (FieldTargetState sample : samples) {
                double d = sample.originField.distanceTo(mean.originField);
                sum += d * d;
            }
            return Math.sqrt(sum / samples.size());
        }

        double rotationSpreadDegrees(FieldTargetState mean) {
            if (mean == null || samples.isEmpty()) {
                return Double.NaN;
            }
            double worst = 0.0;
            for (FieldTargetState sample : samples) {
                worst = Math.max(worst, sample.rotationField.angleTo(mean.rotationField));
            }
            return Math.toDegrees(worst);
        }
    }

    @Override
    public void init() {
        light = new LogiLight(hardwareMap);
        goalsPipeline = LogiOpModeTools.goalsPipelineIndex();
        light.pipelineSwitch(goalsPipeline);

        LogiOpModeTools.PinpointHandle pinpoint = LogiOpModeTools.createPinpoint(hardwareMap);
        localizer = pinpoint.localizer;
        localizerFault = pinpoint.fault;

        alliance = PoseHandoff.alliance();
        rezero("start pose set");
    }

    @Override
    public void init_loop() {
        survey();
    }

    @Override
    public void loop() {
        survey();
    }

    @Override
    public void stop() {
        if (light != null) {
            light.close();
        }
    }

    private void survey() {
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean aPressed = gamepad1.aWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean xPressed = gamepad1.xWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean backPressed = gamepad1.backWasPressed();
        boolean leftPressed = gamepad1.dpadLeftWasPressed();
        boolean rightPressed = gamepad1.dpadRightWasPressed();

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
                rezero(alliance + " start pose set");
            }
            if (yPressed) {
                if (rightBumper) {
                    if (LogiOpModeTools.confirmTwice(LogiOpModeTools.FIELD_MAP_DELETE_CONFIRM_KEY)) {
                        deleteSavedMap();
                    } else {
                        message = String.format(Locale.US,
                                "!! press RB+Y again within %.0f s to DELETE the saved field map",
                                LogiOpModeTools.CONFIRM_WINDOW_SECONDS);
                    }
                } else {
                    LogiOpModeTools.disarmConfirm();
                    saveSurvey();
                }
            }
        }
        if (backPressed) {
            rezero("re-zeroed at the " + alliance + " start pose");
        }
        if (aPressed) {
            toggleAveraging();
        }
        if (leftPressed) {
            labelIndex = (labelIndex + STATE_LABELS.length - 1) % STATE_LABELS.length;
        }
        if (rightPressed) {
            labelIndex = (labelIndex + 1) % STATE_LABELS.length;
        }

        updatePose();
        updateStillness();
        if (robotPose != null) {
            light.updateRobotOrientation(robotPose.heading());
        }
        light.update();
        LogiResult result = light.getLatestResult();
        collect(result);

        LogiLightField.drawResult(robotPose, result, null, light.getFieldMap());
        addTelemetry(result);
    }

    private void updatePose() {
        if (localizer == null) {
            robotPose = LogiOpModeTools.startPose(alliance);
            return;
        }
        try {
            localizer.update();
            robotPose = localizer.pose();
        } catch (Throwable t) {
            localizerFault = "!! Pinpoint read failed - " + LogiOpModeTools.describe(t);
        }
        if (robotPose == null) {
            robotPose = LogiOpModeTools.startPose(alliance);
        }
    }

    private void updateStillness() {
        if (LogiOpModeTools.isStill(localizer, MAX_STILL_SPEED_INCHES_PER_SECOND, MAX_STILL_TURN_DEGREES_PER_SECOND)) {
            if (stillSince < 0.0) {
                stillSince = getRuntime();
            }
        } else {
            stillSince = -1.0;
        }
    }

    private boolean settledStill() {
        return stillSince >= 0.0 && getRuntime() - stillSince >= STILL_SETTLE_SECONDS;
    }

    private void rezero(String note) {
        Pose start = LogiOpModeTools.startPose(alliance);
        if (localizer != null) {
            try {
                localizer.setPose(start);
            } catch (Throwable t) {
                localizerFault = "!! Pinpoint setPose failed - " + LogiOpModeTools.describe(t);
            }
        }
        robotPose = start;
        stillSince = -1.0;
        boolean hadSamples = !accumulators.isEmpty();
        accumulators.clear();
        averaging = false;
        message = hadSamples ? note + " - samples cleared, press A to average again" : note;
    }

    private void toggleAveraging() {
        if (averaging) {
            averaging = false;
            message = "averaging stopped - Y saves targets with at least " + MIN_SAMPLES_TO_SAVE + " frames";
            return;
        }
        accumulators.clear();
        lastSampledCaptureNanos = 0L;
        skippedMovingFrames = 0;
        averaging = true;
        message = "averaging - keep the robot still";
    }

    private void collect(LogiResult result) {
        if (!averaging || result == null) {
            return;
        }
        long capture = result.getCaptureNanos();
        if (capture == 0L || capture == lastSampledCaptureNanos || result.isExpired()) {
            return;
        }
        lastSampledCaptureNanos = capture;
        if (!LogiOpModeTools.seesTags(result.getPipelineIndex())) {
            return;
        }
        if (!settledStill()) {
            skippedMovingFrames++;
            return;
        }
        PlanarPose planar = LogiOpModeTools.toPlanar(robotPose);
        if (planar == null || !planar.isFinite()) {
            return;
        }
        LogiLocalizationConfig loc = LogiLightSettings.LOCALIZATION != null ? LogiLightSettings.LOCALIZATION
                : new LogiLocalizationConfig();
        CameraModel mount = light.getCameraModel();
        for (LogiFiducial fiducial : result.getFiducialResults()) {
            if (fiducial == null || !fiducial.hasPose() || fiducial.name == null || fiducial.name.trim().isEmpty()) {
                continue;
            }
            if (fiducial.percentClusterFound < loc.minPercentClusterFound) {
                continue;
            }
            if (LogiOpModeTools.isFinite(fiducial.robotDistanceInches)
                    && fiducial.robotDistanceInches > loc.maxTargetDistanceInches) {
                continue;
            }
            String name = fiducial.name.trim();
            Accumulator accumulator = accumulators.get(name);
            if (accumulator == null) {
                accumulator = new Accumulator(name);
                accumulators.put(name, accumulator);
            }
            if (accumulator.isFull()) {
                continue;
            }
            FieldTargetState sample = TargetGeometry.survey(name, planar, fiducial.targetInRobot, fiducial.rotationInRobot);
            if (sample == null || !sample.isFinite()) {
                continue;
            }
            accumulator.add(sample, fiducial, planar, mount);
        }
        if (!accumulators.isEmpty() && allFull()) {
            averaging = false;
            message = "averaging done - every visible target has " + MAX_SAMPLES_PER_TARGET + " frames, Y = save";
        }
    }

    private boolean allFull() {
        for (Accumulator accumulator : accumulators.values()) {
            if (!accumulator.isFull()) {
                return false;
            }
        }
        return true;
    }

    private void saveSurvey() {
        String label = STATE_LABELS[labelIndex];
        LogiFieldMap map = light.getFieldMap();
        if (map == null) {
            map = LogiLightStore.loadFieldMap();
        }
        List<String> lines = new ArrayList<String>();
        int saved = 0;
        for (Accumulator accumulator : accumulators.values()) {
            if (accumulator.samples.size() < MIN_SAMPLES_TO_SAVE) {
                lines.add(String.format(Locale.US, "-- %s skipped: %d frames (need %d)",
                        accumulator.name, accumulator.samples.size(), MIN_SAMPLES_TO_SAVE));
                continue;
            }
            FieldTargetState mean = accumulator.average(label);
            if (mean == null || !mean.isFinite()) {
                lines.add("-- " + accumulator.name + " skipped: average is not finite");
                continue;
            }
            SurveyInfo info = accumulator.surveyInfo();
            if (info != null) {
                mean = mean.withSurvey(info);
            }
            map = map.withSurveyedState(accumulator.name, mean);
            saved++;
            lines.add(String.format(Locale.US,
                    ">> %s [%s]  x %.2f  y %.2f  z %.2f  yaw %.1f  pitch %.1f  roll %.1f  (%d frames, spread %.2f in %.1f deg)",
                    accumulator.name, label, mean.originField.x, mean.originField.y, mean.originField.z,
                    mean.yawDegrees(), mean.pitchDegrees(), mean.rollDegrees(), accumulator.samples.size(),
                    accumulator.originSpreadInches(mean), accumulator.rotationSpreadDegrees(mean)));
        }
        if (saved == 0) {
            message = "!! nothing saved - press A, keep still until each target has at least "
                    + MIN_SAMPLES_TO_SAVE + " frames";
            if (!lines.isEmpty()) {
                replaceSavedLines(lines);
            }
            return;
        }
        LogiLightStore.clearLastError();
        boolean written = LogiLightStore.saveFieldMap(map);
        String note = LogiLightStore.lastError();
        light.setFieldMap(map.withSource(written ? LogiLightStore.FIELD_MAP_FILE_NAME + " (surveyed now)"
                : "!! survey kept for this OpMode only (save failed)"));
        if (written) {
            message = String.format(Locale.US, "saved %d target%s as \"%s\" to %s%s", saved, saved == 1 ? "" : "s",
                    label, LogiLightStore.fieldMapFile().getPath(), note.isEmpty() ? "" : "  !! " + note);
        } else {
            message = "!! FIELD MAP NOT SAVED - " + note;
        }
        lines.add(String.format(Locale.US, ">> robot pose used: %s  alliance %s", LogiLightTelemetry.formatPose(robotPose), alliance));
        replaceSavedLines(lines);
        for (String line : lines) {
            RobotLog.ii(LOG_TAG, "%s", line);
        }
    }

    private void deleteSavedMap() {
        LogiLightStore.clearLastError();
        boolean deleted = LogiLightStore.deleteFieldMap();
        light.setFieldMap(LogiLightStore.loadFieldMap());
        String note = LogiLightStore.lastError();
        if (deleted) {
            message = "saved field map moved to " + LogiLightStore.fieldMapBackupFile().getName() + " - placeholders are back";
        } else {
            message = "!! field map not deleted" + (note.isEmpty() ? "" : " - " + note);
        }
        savedLines.clear();
    }

    private void replaceSavedLines(List<String> lines) {
        savedLines.clear();
        savedLines.addAll(lines);
    }

    private void addTelemetry(LogiResult result) {
        telemetry.addLine("LogiLight Field Survey");
        LogiLightTelemetry.addStatus(telemetry, light);
        telemetry.addData("Alliance", "%s   (gp1 X = BLUE, B = RED)", alliance);
        if (localizer == null) {
            telemetry.addData("Robot pose", "%s  !! NO PINPOINT - fixed start pose, do not move the robot",
                    LogiLightTelemetry.formatPose(robotPose));
            telemetry.addData("Pinpoint", localizerFault);
        } else {
            telemetry.addData("Robot pose", "%s  (%s)", LogiLightTelemetry.formatPose(robotPose),
                    LogiOpModeTools.speedText(localizer));
            if (!localizerFault.isEmpty()) {
                telemetry.addLine(localizerFault);
            }
        }
        boolean pointsConfirmed = alliance == GoalScanner.Alliance.BLUE ? FieldPointsBlue.POINTS_CONFIRMED
                : FieldPoints.POINTS_CONFIRMED;
        if (!pointsConfirmed) {
            telemetry.addLine("!! " + alliance + " START POSE NOT CONFIRMED - the survey is only as good as the start pose");
        }
        telemetry.addData("State label", "%s   (dpad L/R: up / tipped-a / tipped-b)", STATE_LABELS[labelIndex]);
        telemetry.addData("Averaging", averaging ? "ON  (gp1 A = stop)" : "off  (gp1 A = start)");
        if (skippedMovingFrames > 0) {
            telemetry.addData("Skipped", "%d frames while the robot was moving", skippedMovingFrames);
        }
        for (Accumulator accumulator : accumulators.values()) {
            FieldTargetState mean = accumulator.average(STATE_LABELS[labelIndex]);
            telemetry.addData(accumulator.name, "%d/%d%s  d %.0f in  spread %.2f in  %.1f deg  min %d%%",
                    accumulator.samples.size(), MAX_SAMPLES_PER_TARGET, accumulator.isFull() ? " FULL" : "",
                    accumulator.averageDistance(), accumulator.originSpreadInches(mean),
                    accumulator.rotationSpreadDegrees(mean), accumulator.minPercent);
        }
        if (result.getFiducialCount() == 0) {
            telemetry.addData("Visible", "no tags  (pipe %s)", LogiOpModeTools.pipelineLabel(result.getPipelineIndex()));
        }
        for (LogiFiducial fiducial : result.getFiducialResults()) {
            telemetry.addLine("  " + LogiLightTelemetry.fiducialLine(fiducial));
        }
        addValidation(result);
        LogiFieldMap map = light.getFieldMap();
        telemetry.addData("Field map", map == null ? "none" : map.summary());
        if (!message.isEmpty()) {
            telemetry.addData("Note", message);
        }
        for (String line : savedLines) {
            telemetry.addLine(line);
        }
        telemetry.addLine("gp1 A = start/stop averaging   Y = save averaged states   RB+Y twice = delete saved field map");
        telemetry.addLine("gp1 X/B = alliance start pose   BACK = re-zero   dpad L/R = state label");
        telemetry.addLine(LogiOpModeTools.settingsHint());
    }

    private void addValidation(LogiResult result) {
        LogiBotPose mt1 = result.getBotpose();
        LogiBotPose mt2 = result.getBotpose_MT2();
        if (mt1 == null && mt2 == null) {
            telemetry.addData("Check", "no botpose - surveyed targets give one after Y");
            return;
        }
        if (mt1 != null && mt1.ageMs() <= MAX_VALIDATION_AGE_MS) {
            telemetry.addData("Check MT1", "%s  error %s", LogiLightTelemetry.formatPose(mt1.pose),
                    LogiOpModeTools.poseError(mt1, robotPose));
        }
        if (mt2 != null && mt2.ageMs() <= MAX_VALIDATION_AGE_MS) {
            telemetry.addData("Check MT2", "%s  error %s", LogiLightTelemetry.formatPose(mt2.pose),
                    LogiOpModeTools.poseError(mt2, robotPose));
        }
    }
}
