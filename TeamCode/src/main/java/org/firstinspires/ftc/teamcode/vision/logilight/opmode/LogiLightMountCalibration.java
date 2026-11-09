package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorProfile;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiCameraConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFieldMap;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiPipeline;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.PipelineType;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;
import org.firstinspires.ftc.teamcode.vision.logilight.math.FieldTargetState;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Mat3;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.TargetGeometry;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "LogiLight Mount Calibration", group = "LogiLight")
public class LogiLightMountCalibration extends LogiOpModeBase {

    private static final double DEFAULT_DISTANCE_INCHES = 24.0;
    private static final double MIN_DISTANCE_INCHES = 2.0;
    private static final double MAX_DISTANCE_INCHES = 200.0;
    private static final double COARSE_DISTANCE_STEP_INCHES = 1.0;
    private static final double FINE_DISTANCE_STEP_INCHES = 0.1;
    private static final double DEFAULT_BALL_HEIGHT_INCHES = 1.4;
    private static final int MAX_BALL_SAMPLES = 15;
    private static final int MAX_TAG_SAMPLES = 30;
    private static final double MAX_BALL_SAMPLE_AGE_MS = 1500.0;
    private static final double MAX_TAG_SAMPLE_AGE_MS = 3000.0;
    private static final int MAX_TARGET_LINES = 4;
    private static final double MIN_TILT_DEGREES = -89.5;
    private static final double MAX_TILT_DEGREES = 89.5;
    private static final double FLOOR_SEARCH_MAX_TILT_DEGREES = 60.0;
    private static final double COARSE_TILT_STEP_DEGREES = 0.5;
    private static final double FINE_TILT_STEP_DEGREES = 0.01;
    private static final double SOLVE_INTERVAL_SECONDS = 0.2;
    private static final double MATCH_TOLERANCE_INCHES = 1.0;
    private static final double STRAIGHT_AHEAD_DEGREES = 10.0;
    private static final double EDGE_MARGIN_FRACTION = 0.05;
    private static final double POSE_STILL_INCHES = 0.5;
    private static final double POSE_STILL_DEGREES = 0.5;
    private static final double FIT_START_STEP_DEGREES = 4.0;
    private static final double FIT_MIN_STEP_DEGREES = 0.005;
    private static final int FIT_MAX_ITERATIONS = 400;
    private static final double FIT_MAX_YAW_CHANGE_DEGREES = 45.0;
    private static final double FIT_TRUST_RESIDUAL_INCHES = 3.0;
    private static final double MIN_SURVEY_RANGE_DIFFERENCE_INCHES = 20.0;
    private static final String RESURVEY_WARNING = "  !! re-run LogiLight Field Survey with the new mount";
    private static final String NO_SURVEY_MOUNT = "!! field map has no survey mount - re-survey first";
    private static final String RANGE_WARNING = "!! move to a spot whose range differs from the survey by >= 20 in";
    private static final double NANOS_PER_MILLI = 1e6;

    private enum Mode {
        FLOOR,
        GOAL
    }

    private static final class BallSample {
        final double u;
        final double v;
        final double heightInches;
        final long captureNanos;

        BallSample(double u, double v, double heightInches, long captureNanos) {
            this.u = u;
            this.v = v;
            this.heightInches = heightInches;
            this.captureNanos = captureNanos;
        }
    }

    private static final class TagSample {
        final Vec3 targetInOptical;
        final Mat3 rotationInOptical;
        final long captureNanos;

        TagSample(Vec3 targetInOptical, Mat3 rotationInOptical, long captureNanos) {
            this.targetInOptical = targetInOptical;
            this.rotationInOptical = rotationInOptical;
            this.captureNanos = captureNanos;
        }
    }

    private static final class TagTrack {
        final Deque<TagSample> samples = new ArrayDeque<TagSample>();
        PlanarPose startPose;
        long lastCaptureNanos;
    }

    private interface FitCost {
        double at(double yaw, double tilt);
    }

    private static final class TagEvaluation {
        PlanarPose knownPose;
        boolean hasSurveyMount;
        PlanarPose surveyPose;
        Vec3 surveyOptical;
        double surveyRangeInches = Double.NaN;
        String name;
        int sampleCount;
        boolean surveyed;
        boolean hasState;
        String stateLabel = "";
        Vec3 averagedOptical;
        Vec3 expectedInRobot;
        Vec3 observedInRobot;
        PlanarPose mt1;
        PlanarPose mt2;
        double mt1ErrorInches = Double.NaN;
        double mt1HeadingErrorDegrees = Double.NaN;
        double mt2ErrorInches = Double.NaN;
        double heightErrorInches = Double.NaN;
        double rangeErrorInches = Double.NaN;
        double bearingErrorDegrees = Double.NaN;
        double tiltMismatchDegrees = Double.NaN;
        double distanceInches = Double.NaN;
    }

    private final Deque<BallSample> ballSamples = new ArrayDeque<BallSample>();
    private final Map<String, TagTrack> tagTracks = new LinkedHashMap<String, TagTrack>();
    private final ElapsedTime solveTimer = new ElapsedTime();

    private Mode mode = Mode.GOAL;
    private double enteredDistanceInches = DEFAULT_DISTANCE_INCHES;
    private long lastBallCaptureNanos = 0L;
    private String ballProfileName = "";
    private double solvedTiltDegrees = Double.NaN;
    private double solvedErrorInches = Double.NaN;
    private boolean solveRequested = false;
    private double[] fit;
    private double fitCurrentResidualInches = Double.NaN;
    private int fitTargetCount = 0;
    private boolean fitUsesPlaceholders = false;
    private boolean fitTwoView = false;
    private boolean fitRangeWarning = false;
    private String fitRefusal = "";
    private List<TagEvaluation> evaluations = new ArrayList<TagEvaluation>();

    @Override
    protected boolean usesOdometry() {
        return true;
    }

    @Override
    protected void onInit() {
        odometry.resetToStartPose();
        mode = camera().tiltUpDegrees < 0.0 ? Mode.FLOOR : Mode.GOAL;
        enterMode();
    }

    @Override
    protected void onLoop() {
        if (pad1.rightBumper()) {
            mode = mode == Mode.FLOOR ? Mode.GOAL : Mode.FLOOR;
            enterMode();
            showMessage("mode " + mode);
        }
        LogiResult result = ll.getLatestResult();
        if (mode == Mode.FLOOR) {
            runFloor(result);
        } else {
            runGoal(result);
        }
    }

    @Override
    protected void onSettingsReset() {
        clearAll();
        enterMode();
    }

    private void enterMode() {
        clearAll();
        LogiPipeline pipeline = ll.getPipeline();
        if (mode == Mode.FLOOR) {
            if (pipeline == null || pipeline.typeOrDefault() != PipelineType.COLOR) {
                ensurePipelineOfType(PipelineType.COLOR);
            }
        } else if (pipeline == null || !pipeline.typeOrDefault().usesAprilTag()) {
            int goals = LogiLightSettings.findPipeline("goals");
            LogiPipeline named = LogiLightSettings.pipelineAt(goals);
            if (named != null && named.typeOrDefault().usesAprilTag()) {
                ll.pipelineSwitch(goals);
            } else {
                ensurePipelineOfType(PipelineType.APRILTAG);
            }
        }
    }

    private void clearAll() {
        ballSamples.clear();
        tagTracks.clear();
        evaluations = new ArrayList<TagEvaluation>();
        solvedTiltDegrees = Double.NaN;
        solvedErrorInches = Double.NaN;
        fit = null;
        fitCurrentResidualInches = Double.NaN;
        fitTargetCount = 0;
        fitUsesPlaceholders = false;
    }

    private void runFloor(LogiResult result) {
        boolean shift = comboHeld();
        if (pad1.dpadUp()) {
            changeDistance(COARSE_DISTANCE_STEP_INCHES);
        }
        if (pad1.dpadDown()) {
            changeDistance(-COARSE_DISTANCE_STEP_INCHES);
        }
        if (pad1.dpadRight()) {
            changeDistance(FINE_DISTANCE_STEP_INCHES);
        }
        if (pad1.dpadLeft()) {
            changeDistance(-FINE_DISTANCE_STEP_INCHES);
        }
        if (pad1.x() && !shift) {
            cyclePipelineOfType(PipelineType.COLOR);
            ballSamples.clear();
        }
        if (pad1.b() && !shift) {
            ballSamples.clear();
            showMessage("ball samples cleared");
        }
        collectBall(result);
        CameraModel model = ll.getCameraModel();
        BallSample median = medianBall();
        if (median != null && (solveRequested || solveTimer.seconds() >= SOLVE_INTERVAL_SECONDS)) {
            solveTimer.reset();
            solveRequested = false;
            double[] solution = solveTilt(model, median.u, median.v, median.heightInches, enteredDistanceInches);
            solvedTiltDegrees = solution == null ? Double.NaN : solution[0];
            solvedErrorInches = solution == null ? Double.NaN : solution[1];
        } else if (median == null) {
            solvedTiltDegrees = Double.NaN;
            solvedErrorInches = Double.NaN;
        }
        if (pad1.a() && !shift) {
            applyFloorTilt();
        }
        addFloorTelemetry(model, median);
    }

    private void changeDistance(double delta) {
        enteredDistanceInches = Math.max(MIN_DISTANCE_INCHES, Math.min(MAX_DISTANCE_INCHES,
                Math.round((enteredDistanceInches + delta) * 10.0) / 10.0));
        solveRequested = true;
    }

    private void collectBall(LogiResult result) {
        long now = System.nanoTime();
        while (!ballSamples.isEmpty()
                && (now - ballSamples.peekFirst().captureNanos) / NANOS_PER_MILLI > MAX_BALL_SAMPLE_AGE_MS) {
            ballSamples.removeFirst();
        }
        ColorTarget target = result.getPrimaryColor();
        long capture = result.getCaptureNanos();
        if (target == null || result.isExpired() || capture == 0L || capture == lastBallCaptureNanos) {
            return;
        }
        if (!isFinite(target.centerX) || !isFinite(target.centerY)) {
            return;
        }
        lastBallCaptureNanos = capture;
        String name = target.profileName == null ? "" : target.profileName;
        if (!name.equals(ballProfileName)) {
            ballSamples.clear();
            ballProfileName = name;
        }
        ColorProfile profile = LogiLightSettings.findColorProfile(name);
        double height = profile != null && isFinite(profile.targetCenterHeightInches)
                ? profile.targetCenterHeightInches : DEFAULT_BALL_HEIGHT_INCHES;
        ballSamples.addLast(new BallSample(target.centerX, target.centerY, height, capture));
        while (ballSamples.size() > MAX_BALL_SAMPLES) {
            ballSamples.removeFirst();
        }
    }

    private BallSample medianBall() {
        int count = ballSamples.size();
        if (count == 0) {
            return null;
        }
        double[] us = new double[count];
        double[] vs = new double[count];
        int i = 0;
        double height = DEFAULT_BALL_HEIGHT_INCHES;
        long newest = 0L;
        for (BallSample sample : ballSamples) {
            us[i] = sample.u;
            vs[i] = sample.v;
            height = sample.heightInches;
            newest = Math.max(newest, sample.captureNanos);
            i++;
        }
        return new BallSample(median(us), median(vs), height, newest);
    }

    private static double median(double[] values) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        int n = sorted.length;
        if (n == 0) {
            return Double.NaN;
        }
        return n % 2 == 1 ? sorted[n / 2] : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
    }

    private static double[] solveTilt(CameraModel model, double u, double v, double height, double wanted) {
        double bestTilt = Double.NaN;
        double bestError = Double.POSITIVE_INFINITY;
        int coarseSteps = (int) Math.round((FLOOR_SEARCH_MAX_TILT_DEGREES - MIN_TILT_DEGREES) / COARSE_TILT_STEP_DEGREES);
        for (int i = 0; i <= coarseSteps; i++) {
            double tilt = MIN_TILT_DEGREES + i * COARSE_TILT_STEP_DEGREES;
            double error = Math.abs(groundDistance(model, tilt, u, v, height) - wanted);
            if (error < bestError) {
                bestError = error;
                bestTilt = tilt;
            }
        }
        if (Double.isNaN(bestTilt)) {
            return null;
        }
        double low = Math.max(MIN_TILT_DEGREES, bestTilt - COARSE_TILT_STEP_DEGREES);
        double high = Math.min(FLOOR_SEARCH_MAX_TILT_DEGREES, bestTilt + COARSE_TILT_STEP_DEGREES);
        int fineSteps = (int) Math.round((high - low) / FINE_TILT_STEP_DEGREES);
        for (int i = 0; i <= fineSteps; i++) {
            double tilt = low + i * FINE_TILT_STEP_DEGREES;
            double error = Math.abs(groundDistance(model, tilt, u, v, height) - wanted);
            if (error < bestError) {
                bestError = error;
                bestTilt = tilt;
            }
        }
        return new double[]{bestTilt, bestError};
    }

    private static double groundDistance(CameraModel model, double tilt, double u, double v, double height) {
        CameraModel candidate = model.withMount(model.forwardInches, model.leftInches, model.upInches,
                model.yawLeftDegrees, tilt, model.rollClockwiseDegrees);
        Vec3 hit = candidate.groundIntersection(u, v, height);
        return hit == null ? Double.NaN : hit.horizontalNorm();
    }

    private void applyFloorTilt() {
        if (!isFinite(solvedTiltDegrees)) {
            showMessage("!! no tilt solution yet - put a ball in view");
            return;
        }
        LogiCameraConfig cfg = camera();
        double before = cfg.tiltUpDegrees;
        cfg.tiltUpDegrees = Math.round(solvedTiltDegrees * 100.0) / 100.0;
        ll.refreshMount();
        showMessage(String.format(Locale.US, "tiltUpDegrees %.2f -> %.2f (LB + Y to save)", before, cfg.tiltUpDegrees));
    }

    private void addFloorTelemetry(CameraModel model, BallSample median) {
        LogiCameraConfig cfg = camera();
        telemetry.addData("Mode", "FLOOR camera - pollen/nectar ball on the tiles   (RB = GOAL mode)");
        addPipelineLine();
        telemetry.addData("Webcam", "%s  %s", cfg.webcamName, cfg.mountSummary());
        telemetry.addData("Entered distance", "%.1f in  (robot center -> ball center, on the floor)", enteredDistanceInches);
        if (median == null) {
            telemetry.addLine("!! no ball in view - check the pipeline / Color Tuner");
        } else {
            telemetry.addData("Ball", "%s  px (%.0f, %.0f)  center height %.2f in  %d frames", ballProfileName,
                    median.u, median.v, median.heightInches, ballSamples.size());
            Vec3 hit = model.groundIntersection(median.u, median.v, median.heightInches);
            if (hit == null) {
                telemetry.addLine("!! with the current tilt the ray never hits the floor - tilt is far off");
            } else {
                double distance = hit.horizontalNorm();
                double bearing = TargetGeometry.horizontalBearingLeftDegrees(hit);
                telemetry.addData("Vision now", "%.1f in  (error %+.1f in)  bearing %+.1f deg", distance,
                        distance - enteredDistanceInches, bearing);
                if (Math.abs(bearing) > STRAIGHT_AHEAD_DEGREES) {
                    telemetry.addLine("!! ball is not straight ahead of the robot - move it onto the robot's center line");
                }
            }
            if (median.u < model.width * EDGE_MARGIN_FRACTION || median.u > model.width * (1.0 - EDGE_MARGIN_FRACTION)
                    || median.v < model.height * EDGE_MARGIN_FRACTION || median.v > model.height * (1.0 - EDGE_MARGIN_FRACTION)) {
                telemetry.addLine("!! ball is near the image edge - lens distortion makes this less accurate");
            }
        }
        if (isFinite(solvedTiltDegrees)) {
            telemetry.addData(">> tiltUpDegrees", "%.2f  (now %.2f, match %.2f in)", solvedTiltDegrees,
                    cfg.tiltUpDegrees, solvedErrorInches);
            if (solvedErrorInches > MATCH_TOLERANCE_INCHES) {
                telemetry.addLine("!! no tilt matches the distance - check upInches (lens height) and forwardInches");
            }
        } else {
            telemetry.addData(">> tiltUpDegrees", "no solution yet");
        }
        if (cfg.upInches <= DEFAULT_BALL_HEIGHT_INCHES) {
            telemetry.addLine("!! upInches is below the ball - measure the lens height above the tiles");
        }
        telemetry.addLine();
        addHint("1. Measure upInches/forwardInches/leftInches with a ruler and set them (Panels or code).");
        addHint("2. Put a ball straight ahead at a measured distance from the robot CENTER; dpad sets it.");
        addHint("3. A = apply the tilt, then LB + Y to save. Repeat at a 2nd distance to check.");
        addHint("dpad U/D = +-1 in   dpad R/L = +-0.1 in   X = next color pipeline   B = clear samples   RB = GOAL mode");
        addHint(SETTINGS_HINT);
    }

    private void runGoal(LogiResult result) {
        boolean shift = comboHeld();
        if (pad1.x() && !shift) {
            setAlliance(GoalScanner.Alliance.BLUE);
        }
        if (pad1.b() && !shift) {
            setAlliance(GoalScanner.Alliance.RED);
        }
        if (pad1.back()) {
            odometry.resetToStartPose();
            tagTracks.clear();
            showMessage("pose re-zeroed at the " + odometry.alliance() + " start pose");
        }
        if (pad1.dpadDown()) {
            tagTracks.clear();
            showMessage("tag samples cleared");
        }
        PlanarPose known = knownPose();
        collectTags(result, known);
        CameraModel model = ll.getCameraModel();
        LogiFieldMap map = ll.getFieldMap();
        evaluations = evaluate(model, map, known);
        if (solveTimer.seconds() >= SOLVE_INTERVAL_SECONDS) {
            solveTimer.reset();
            runFit(model);
        }
        if (pad1.a() && !shift) {
            applyFit();
        }
        Pose odometryPose = odometry.pose();
        LogiLightField.drawResult(odometryPose != null ? odometryPose : odometry.startPose(), result, null, map);
        addGoalTelemetry(known, map);
    }

    private void setAlliance(GoalScanner.Alliance alliance) {
        odometry.setAlliance(alliance);
        odometry.resetToStartPose();
        tagTracks.clear();
        showMessage("known pose = " + alliance + " start pose  " + LogiLightTelemetry.formatPose(odometry.startPose()));
    }

    private PlanarPose knownPose() {
        PlanarPose fromOdometry = odometry.planarPose();
        if (fromOdometry != null && fromOdometry.isFinite()) {
            return fromOdometry;
        }
        Pose start = odometry.startPose();
        return new PlanarPose(start.x(), start.y(), start.heading());
    }

    private void collectTags(LogiResult result, PlanarPose known) {
        long now = System.nanoTime();
        Iterator<Map.Entry<String, TagTrack>> iterator = tagTracks.entrySet().iterator();
        while (iterator.hasNext()) {
            TagTrack track = iterator.next().getValue();
            while (!track.samples.isEmpty()
                    && (now - track.samples.peekFirst().captureNanos) / NANOS_PER_MILLI > MAX_TAG_SAMPLE_AGE_MS) {
                track.samples.removeFirst();
            }
            if (track.samples.isEmpty()) {
                iterator.remove();
            }
        }
        if (result.isExpired()) {
            return;
        }
        for (LogiFiducial fiducial : result.getFiducialResults()) {
            if (!fiducial.hasPose() || fiducial.captureNanos == 0L) {
                continue;
            }
            TagTrack track = tagTracks.get(fiducial.name);
            if (track == null) {
                track = new TagTrack();
                tagTracks.put(fiducial.name, track);
            }
            if (track.startPose == null || moved(track.startPose, known)) {
                track.samples.clear();
                track.startPose = known;
            }
            if (fiducial.captureNanos == track.lastCaptureNanos) {
                continue;
            }
            track.lastCaptureNanos = fiducial.captureNanos;
            track.samples.addLast(new TagSample(fiducial.targetInOptical, fiducial.rotationInOptical, fiducial.captureNanos));
            while (track.samples.size() > MAX_TAG_SAMPLES) {
                track.samples.removeFirst();
            }
        }
    }

    private static boolean moved(PlanarPose from, PlanarPose to) {
        if (from == null || to == null) {
            return true;
        }
        return from.distanceTo(to) > POSE_STILL_INCHES
                || Math.abs(Math.toDegrees(from.headingErrorTo(to))) > POSE_STILL_DEGREES;
    }

    private List<TagEvaluation> evaluate(CameraModel model, LogiFieldMap map, PlanarPose known) {
        List<TagEvaluation> out = new ArrayList<TagEvaluation>();
        for (Map.Entry<String, TagTrack> entry : tagTracks.entrySet()) {
            TagTrack track = entry.getValue();
            if (track.samples.isEmpty()) {
                continue;
            }
            List<Vec3> positions = new ArrayList<Vec3>();
            List<Mat3> rotations = new ArrayList<Mat3>();
            for (TagSample sample : track.samples) {
                positions.add(sample.targetInOptical);
                rotations.add(sample.rotationInOptical);
            }
            TagEvaluation e = new TagEvaluation();
            e.name = entry.getKey();
            e.sampleCount = track.samples.size();
            e.averagedOptical = Vec3.average(positions);
            Mat3 rotationOptical = Mat3.average(rotations);
            e.observedInRobot = model.opticalPointToRobot(e.averagedOptical);
            Mat3 rotationRobot = model.opticalRotationToRobot(rotationOptical);
            e.distanceInches = TargetGeometry.horizontalDistanceInches(e.observedInRobot);
            LogiFieldMap.Target target = map == null ? null : map.find(e.name);
            FieldTargetState state = bestState(target, rotationRobot);
            if (target != null && state != null) {
                e.surveyed = target.surveyed;
                e.hasState = true;
                e.stateLabel = state.label;
                e.knownPose = known;
                if (state.hasSurvey()) {
                    e.surveyPose = state.survey.robotPose;
                    e.surveyOptical = state.survey.opticalPointOf(state.originField, model);
                    e.hasSurveyMount = e.surveyOptical != null && e.surveyOptical.isFinite();
                    Vec3 surveyRobot = TargetGeometry.fieldPointToRobot(e.surveyPose, state.originField);
                    e.surveyRangeInches = TargetGeometry.horizontalDistanceInches(surveyRobot);
                }
                e.expectedInRobot = TargetGeometry.fieldPointToRobot(known, state.originField);
                e.mt1 = TargetGeometry.solveFull(e.observedInRobot, rotationRobot, state);
                e.mt2 = TargetGeometry.solveKnownHeading(e.observedInRobot, state.originField, known.heading);
                if (e.mt1 != null) {
                    e.mt1ErrorInches = e.mt1.distanceTo(known);
                    e.mt1HeadingErrorDegrees = Math.toDegrees(known.headingErrorTo(e.mt1));
                }
                if (e.mt2 != null) {
                    e.mt2ErrorInches = e.mt2.distanceTo(known);
                }
                e.heightErrorInches = TargetGeometry.heightErrorInches(e.observedInRobot, state.originField);
                e.tiltMismatchDegrees = Math.toDegrees(TargetGeometry.tiltMismatchRadians(rotationRobot, state.rotationField));
                if (e.expectedInRobot != null) {
                    e.rangeErrorInches = e.distanceInches - TargetGeometry.horizontalDistanceInches(e.expectedInRobot);
                    e.bearingErrorDegrees = PlanarPose.normalizeDegrees(
                            TargetGeometry.horizontalBearingLeftDegrees(e.observedInRobot)
                                    - TargetGeometry.horizontalBearingLeftDegrees(e.expectedInRobot));
                }
            }
            out.add(e);
        }
        return out;
    }

    private static FieldTargetState bestState(LogiFieldMap.Target target, Mat3 rotationRobot) {
        if (target == null || target.states == null || target.states.isEmpty()) {
            return null;
        }
        FieldTargetState best = null;
        double bestMismatch = Double.POSITIVE_INFINITY;
        for (FieldTargetState state : target.states) {
            if (state == null) {
                continue;
            }
            double mismatch = TargetGeometry.tiltMismatchRadians(rotationRobot, state.rotationField);
            if (Double.isNaN(mismatch)) {
                mismatch = Math.PI;
            }
            if (best == null || mismatch < bestMismatch) {
                best = state;
                bestMismatch = mismatch;
            }
        }
        return best;
    }

    private void runFit(final CameraModel model) {
        boolean anySurveyed = false;
        boolean anySurveyMount = false;
        for (TagEvaluation e : evaluations) {
            if (e.hasState && e.surveyed) {
                anySurveyed = true;
                if (e.hasSurveyMount && e.averagedOptical != null && e.knownPose != null) {
                    anySurveyMount = true;
                }
            }
        }
        fitRefusal = "";
        fitRangeWarning = false;
        fitTwoView = false;
        if (anySurveyMount) {
            final List<TagEvaluation> views = new ArrayList<TagEvaluation>();
            boolean allClose = true;
            for (TagEvaluation e : evaluations) {
                if (!e.hasState || !e.surveyed || !e.hasSurveyMount || e.averagedOptical == null || e.knownPose == null) {
                    continue;
                }
                views.add(e);
                if (!isFinite(e.surveyRangeInches) || !isFinite(e.distanceInches)
                        || Math.abs(e.distanceInches - e.surveyRangeInches) >= MIN_SURVEY_RANGE_DIFFERENCE_INCHES) {
                    allClose = false;
                }
            }
            fitTargetCount = views.size();
            fitUsesPlaceholders = false;
            fitTwoView = true;
            fitRangeWarning = allClose;
            FitCost cost = new FitCost() {
                @Override
                public double at(double yaw, double tilt) {
                    return twoViewCost(model, yaw, tilt, views);
                }
            };
            fitCurrentResidualInches = Math.sqrt(cost.at(model.yawLeftDegrees, model.tiltUpDegrees) / views.size());
            fit = fitYawTilt(model, cost, views.size());
            return;
        }
        if (anySurveyed) {
            fit = null;
            fitTargetCount = 0;
            fitUsesPlaceholders = false;
            fitCurrentResidualInches = Double.NaN;
            fitRefusal = NO_SURVEY_MOUNT;
            return;
        }
        final List<Vec3> optical = new ArrayList<Vec3>();
        final List<Vec3> expected = new ArrayList<Vec3>();
        for (TagEvaluation e : evaluations) {
            if (!e.hasState || e.expectedInRobot == null || e.averagedOptical == null) {
                continue;
            }
            optical.add(e.averagedOptical);
            expected.add(e.expectedInRobot);
        }
        fitTargetCount = optical.size();
        fitUsesPlaceholders = fitTargetCount > 0;
        if (optical.isEmpty()) {
            fit = null;
            fitCurrentResidualInches = Double.NaN;
            return;
        }
        FitCost cost = new FitCost() {
            @Override
            public double at(double yaw, double tilt) {
                return fitCost(model, yaw, tilt, optical, expected);
            }
        };
        fitCurrentResidualInches = Math.sqrt(cost.at(model.yawLeftDegrees, model.tiltUpDegrees) / optical.size());
        fit = fitYawTilt(model, cost, optical.size());
    }

    private static double[] fitYawTilt(CameraModel model, FitCost cost, int count) {
        double yaw = model.yawLeftDegrees;
        double tilt = model.tiltUpDegrees;
        double best = cost.at(yaw, tilt);
        double step = FIT_START_STEP_DEGREES;
        int iterations = 0;
        while (step > FIT_MIN_STEP_DEGREES && iterations < FIT_MAX_ITERATIONS) {
            iterations++;
            boolean improved = false;
            for (int k = 0; k < 4; k++) {
                double candidateYaw = yaw + (k == 0 ? step : k == 1 ? -step : 0.0);
                double candidateTilt = tilt + (k == 2 ? step : k == 3 ? -step : 0.0);
                if (Math.abs(candidateYaw - model.yawLeftDegrees) > FIT_MAX_YAW_CHANGE_DEGREES
                        || candidateTilt < MIN_TILT_DEGREES || candidateTilt > MAX_TILT_DEGREES) {
                    continue;
                }
                double value = cost.at(candidateYaw, candidateTilt);
                if (value < best) {
                    best = value;
                    yaw = candidateYaw;
                    tilt = candidateTilt;
                    improved = true;
                }
            }
            if (!improved) {
                step *= 0.5;
            }
        }
        return new double[]{yaw, tilt, Math.sqrt(best / Math.max(1, count))};
    }

    private static double fitCost(CameraModel model, double yaw, double tilt, List<Vec3> optical, List<Vec3> expected) {
        CameraModel candidate = model.withMount(model.forwardInches, model.leftInches, model.upInches, yaw, tilt,
                model.rollClockwiseDegrees);
        double sum = 0.0;
        for (int i = 0; i < optical.size(); i++) {
            Vec3 difference = candidate.opticalPointToRobot(optical.get(i)).minus(expected.get(i));
            sum += difference.dot(difference);
        }
        return isFinite(sum) ? sum : Double.POSITIVE_INFINITY;
    }

    private static double twoViewCost(CameraModel model, double yaw, double tilt, List<TagEvaluation> views) {
        CameraModel candidate = model.withMount(model.forwardInches, model.leftInches, model.upInches, yaw, tilt,
                model.rollClockwiseDegrees);
        double sum = 0.0;
        for (TagEvaluation e : views) {
            Vec3 now = TargetGeometry.robotPointToField(e.knownPose, candidate.opticalPointToRobot(e.averagedOptical));
            Vec3 then = TargetGeometry.robotPointToField(e.surveyPose, candidate.opticalPointToRobot(e.surveyOptical));
            if (now == null || then == null) {
                return Double.POSITIVE_INFINITY;
            }
            Vec3 difference = now.minus(then);
            sum += difference.dot(difference);
        }
        return isFinite(sum) ? sum : Double.POSITIVE_INFINITY;
    }

    private void applyFit() {
        if (!fitRefusal.isEmpty()) {
            showMessage(fitRefusal);
            return;
        }
        if (fit == null) {
            showMessage("!! no fit yet - put a goal from the field map in view");
            return;
        }
        LogiCameraConfig cfg = camera();
        double yawBefore = cfg.yawLeftDegrees;
        double tiltBefore = cfg.tiltUpDegrees;
        cfg.yawLeftDegrees = Math.round(fit[0] * 100.0) / 100.0;
        cfg.tiltUpDegrees = Math.round(fit[1] * 100.0) / 100.0;
        ll.refreshMount();
        showMessage(String.format(Locale.US, "yaw %.2f -> %.2f  tilt %.2f -> %.2f (LB + Y to save)%s%s%s",
                yawBefore, cfg.yawLeftDegrees, tiltBefore, cfg.tiltUpDegrees,
                fitUsesPlaceholders ? "  !! fitted to PLACEHOLDER goals" : "",
                fitRangeWarning ? "  " + RANGE_WARNING : "",
                RESURVEY_WARNING));
    }

    private void addGoalTelemetry(PlanarPose known, LogiFieldMap map) {
        LogiCameraConfig cfg = camera();
        telemetry.addData("Mode", "GOAL camera - robot on a known pose   (RB = FLOOR mode)");
        addPipelineLine();
        telemetry.addData("Webcam", "%s  %s", cfg.webcamName, cfg.mountSummary());
        telemetry.addData("Known pose", "%s  (%s)", LogiLightTelemetry.formatPose(known),
                odometry.isAvailable() ? "Pinpoint from " + odometry.alliance() + " start" : odometry.alliance() + " start pose");
        if (!odometry.isAvailable()) {
            telemetry.addLine("!! no Pinpoint - the robot must stay exactly on the start pose");
        }
        if (map == null || map.surveyedCount() == 0) {
            telemetry.addLine("!! field map has only PLACEHOLDER goals - run LogiLight Field Survey from another spot first");
        }
        if (evaluations.isEmpty()) {
            telemetry.addLine("no goal with a pose in view yet");
        }
        int shown = 0;
        for (TagEvaluation e : evaluations) {
            if (shown >= MAX_TARGET_LINES) {
                telemetry.addLine(String.format(Locale.US, "... %d more", evaluations.size() - shown));
                break;
            }
            addEvaluation(e);
            shown++;
        }
        if (fit != null) {
            telemetry.addData(">> tiltUpDegrees", "%.2f  (now %.2f)", fit[1], cfg.tiltUpDegrees);
            telemetry.addData(">> yawLeftDegrees", "%.2f  (now %.2f)", fit[0], cfg.yawLeftDegrees);
            telemetry.addData("Fit", "%d goals  %s  residual %.2f in -> %.2f in%s", fitTargetCount,
                    fitTwoView ? "survey view + this view" : "vs field map",
                    fitCurrentResidualInches, fit[2], fitUsesPlaceholders ? "  !! placeholders" : "");
            if (fit[2] > FIT_TRUST_RESIDUAL_INCHES) {
                telemetry.addLine("!! residual stays large - check upInches / forwardInches / leftInches / roll or the survey");
            }
            if (fitRangeWarning) {
                telemetry.addLine(RANGE_WARNING);
            }
        } else if (!fitRefusal.isEmpty()) {
            telemetry.addData(">> tiltUpDegrees", fitRefusal);
        } else {
            telemetry.addData(">> tiltUpDegrees", "no fit yet");
        }
        telemetry.addLine();
        addHint("1. Put the robot exactly on the start pose (X = BLUE, B = RED), BACK = re-zero there.");
        addHint("2. Keep still: each goal is averaged over " + MAX_TAG_SAMPLES + " frames (dpad down = clear).");
        addHint("3. MT1/MT2 errors near 0 = mount OK. A = apply suggested tilt/yaw, LB + Y = save.");
        addHint("!! Survey the field map from a spot >= 20 in nearer or farther than this one; after A + save, re-survey.");
        addHint(SETTINGS_HINT);
    }

    private void addEvaluation(TagEvaluation e) {
        if (!e.hasState) {
            telemetry.addLine(String.format(Locale.US, "%s  n %d  d %.1f in  !! not in the field map", e.name,
                    e.sampleCount, e.distanceInches));
            return;
        }
        telemetry.addLine(String.format(Locale.US, "%s  %s '%s'  n %d  d %.1f in", e.name,
                e.surveyed ? "surveyed" : "!! placeholder", e.stateLabel, e.sampleCount, e.distanceInches));
        telemetry.addLine(String.format(Locale.US, "   MT1 err %s  h %s   MT2 err %s",
                inches(e.mt1ErrorInches), degrees(e.mt1HeadingErrorDegrees), inches(e.mt2ErrorInches)));
        telemetry.addLine(String.format(Locale.US, "   height %s  range %s  bearing %s  tilt-mismatch %s",
                signedInches(e.heightErrorInches), signedInches(e.rangeErrorInches), degrees(e.bearingErrorDegrees),
                degrees(e.tiltMismatchDegrees)));
    }

    private static String inches(double value) {
        return isFinite(value) ? String.format(Locale.US, "%.1f in", value) : "--";
    }

    private static String signedInches(double value) {
        return isFinite(value) ? String.format(Locale.US, "%+.1f in", value) : "--";
    }

    private static String degrees(double value) {
        return isFinite(value) ? String.format(Locale.US, "%+.1f deg", value) : "--";
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
