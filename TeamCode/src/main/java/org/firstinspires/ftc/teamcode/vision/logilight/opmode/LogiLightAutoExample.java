package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiAimConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiBotPose;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFieldMap;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightFusion;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLocalizationConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.math.HeadingHistory;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;

import java.util.Locale;

@Configurable
@Autonomous(name = "LogiLight Auto Example", group = "LogiLight")
public class LogiLightAutoExample extends LinearOpMode {

    public static double RELOCALIZE_SECONDS = 1.0;
    public static int RELOCALIZE_ENOUGH_FIXES = 6;
    public static double GOAL_SCAN_SECONDS = 0.6;
    public static double FACE_GOAL_SECONDS = 2.5;
    public static double TURN_PASS_SECONDS = 1.0;
    public static int FACE_GOAL_MAX_PASSES = 2;
    public static double FACE_TOLERANCE_DEGREES = 2.0;
    public static double MAX_TURN_DEGREES = 90.0;
    public static boolean APPROACH_ENABLED = true;
    public static double POLLEN_ACQUIRE_SECONDS = 0.8;
    public static double APPROACH_SECONDS = 3.0;
    public static double SEARCH_SECONDS = 1.0;
    public static double ARRIVE_LOSS_CONFIRM_SECONDS = 0.25;
    public static double MAX_RESULT_AGE_MS = LogiOpModeTools.DEFAULT_MAX_RESULT_AGE_MS;

    private static final double AUTONOMOUS_BUDGET_SECONDS = FieldPoints.AUTONOMOUS_BUDGET_SECONDS;

    private LogiLight light;
    private Localizer pinpoint;
    private String pinpointFault = "";
    private LogiLightFusion fusion;
    private Follower follower;
    private String followerFault = "";
    private Drivetrain directDrive;
    private String driveFault = "";
    private final HeadingHistory headingHistory = new HeadingHistory(LogiOpModeTools.HEADING_HISTORY_CAPACITY);
    private final ElapsedTime runtime = new ElapsedTime();

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private int goalsPipeline = 0;
    private long lastOfferedCaptureNanos = 0L;
    private boolean fusionAllowed = false;
    private boolean started = false;
    private LogiResult lastResult = LogiResult.EMPTY;
    private String settingsMessage = "";

    private String phase = "init";
    private String relocalizeNote = "not run";
    private String goalNote = "not run";
    private String pollenNote = "not run";
    private String stopReason = "mission complete";

    private static final class GoalSighting {
        final LogiFiducial fiducial;
        final long captureNanos;

        GoalSighting(LogiFiducial fiducial, long captureNanos) {
            this.fiducial = fiducial;
            this.captureNanos = captureNanos;
        }

        double errorLeftDegrees() {
            return LogiOpModeTools.goalBearingLeftDegrees(fiducial) + Constants.SHOOTER_HEADING_OFFSET_DEGREES;
        }
    }

    @Override
    public void runOpMode() {
        PoseHandoff.clear();
        try {
            setup();
            waitInInit();
            if (isStopRequested()) {
                return;
            }
            runtime.reset();
            started = true;
            try {
                runMission();
            } finally {
                stopMotors();
                if (follower != null) {
                    PoseHandoff.save(follower.pose(), alliance);
                }
            }
            fusionAllowed = true;
            while (opModeIsActive()) {
                tick();
                if (follower != null) {
                    Pose latest = follower.pose();
                    if (latest != null) {
                        PoseHandoff.save(latest, alliance);
                    }
                }
                stopMotorsQuietlyInIdle();
                addSummaryTelemetry();
                telemetry.update();
            }
        } finally {
            stopMotors();
            if (light != null) {
                light.close();
            }
        }
    }

    private void setup() {
        light = new LogiLight(hardwareMap);
        goalsPipeline = LogiOpModeTools.goalsPipelineIndex();
        light.pipelineSwitch(goalsPipeline);

        LogiOpModeTools.PinpointHandle handle = LogiOpModeTools.createPinpoint(hardwareMap);
        pinpoint = handle.localizer;
        pinpointFault = handle.fault;
        if (pinpoint != null) {
            try {
                fusion = new LogiLightFusion(pinpoint, startPose());
            } catch (Throwable t) {
                fusion = null;
                followerFault = "fusion NOT AVAILABLE - " + LogiOpModeTools.describe(t);
            }
        }
        if (fusion != null) {
            try {
                follower = new Follower(fusion.localizer(), Constants.createDrivetrain(hardwareMap),
                        Constants.createAlgorithm());
            } catch (Throwable t) {
                follower = null;
                followerFault = "Follower NOT AVAILABLE - " + LogiOpModeTools.describe(t);
            }
        }
        if (follower == null) {
            try {
                directDrive = Constants.createDrivetrain(hardwareMap);
            } catch (Throwable t) {
                directDrive = null;
                driveFault = "NOT AVAILABLE - " + LogiOpModeTools.describe(t);
            }
        }
    }

    private void waitInInit() {
        fusionAllowed = false;
        while (!isStarted() && !isStopRequested()) {
            boolean yPressed = gamepad1.yWasPressed();
            boolean xPressed = gamepad1.xWasPressed();
            boolean backPressed = gamepad1.backWasPressed();
            boolean leftBumper = gamepad1.left_bumper;
            String combo = LogiOpModeTools.settingsCombo(leftBumper, yPressed, xPressed);
            if (combo != null) {
                settingsMessage = combo;
            }
            if (!leftBumper) {
                GoalScanner.Alliance picked = alliance;
                if ((gamepad1.x && !gamepad1.start) || (gamepad2.x && !gamepad2.start)) {
                    picked = GoalScanner.Alliance.BLUE;
                }
                if ((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start)) {
                    picked = GoalScanner.Alliance.RED;
                }
                if (picked != alliance) {
                    alliance = picked;
                    resetToStartPose();
                }
            }
            if (backPressed) {
                resetToStartPose();
            }

            LogiResult result = tick();
            addInitTelemetry(result);
            telemetry.update();
            idle();
        }
    }

    private void runMission() {
        if (light == null || !light.isAvailable()) {
            stopReason = "camera NOT AVAILABLE - nothing to do, robot kept still";
            return;
        }
        if (follower == null && directDrive == null) {
            stopReason = "no drivetrain - nothing can move";
            return;
        }
        relocalize();
        if (!canRun()) {
            stopReason = budgetReason("after relocalize");
            return;
        }
        faceGoal();
        if (!canRun()) {
            stopReason = budgetReason("after facing the goal");
            return;
        }
        approachPollen();
        if (!opModeIsActive()) {
            stopReason = "stop requested";
        }
    }

    private void relocalize() {
        phase = "relocalize";
        if (fusion == null) {
            relocalizeNote = "skipped - no Pinpoint, no fusion";
            return;
        }
        fusionAllowed = true;
        Pose before = currentPose();
        int startAccepted = fusion.acceptedCount();
        int seenAccepted = startAccepted;
        holdHere();
        ElapsedTime timer = new ElapsedTime();
        while (canRun() && timer.seconds() < RELOCALIZE_SECONDS) {
            tick();
            int accepted = fusion.acceptedCount();
            if (accepted != seenAccepted) {
                seenAccepted = accepted;
                holdHere();
            }
            if (accepted - startAccepted >= RELOCALIZE_ENOUGH_FIXES) {
                break;
            }
            addPhaseTelemetry(String.format(Locale.US, "%.2f / %.2f s  fixes %d", timer.seconds(), RELOCALIZE_SECONDS,
                    accepted - startAccepted));
        }
        Pose after = currentPose();
        double moved = before != null && after != null ? before.distance(after) : 0.0;
        int fixes = fusion.acceptedCount() - startAccepted;
        relocalizeNote = String.format(Locale.US, "%d vision fix%s in %.2f s  pose moved %.1f in  (%s)",
                fixes, fixes == 1 ? "" : "es", timer.seconds(), moved, fusion.lastDecision());
        holdHere();
    }

    private void faceGoal() {
        phase = "face goal";
        light.pipelineSwitch(goalsPipeline);
        ElapsedTime phaseTimer = new ElapsedTime();
        long acceptAfterNanos = System.nanoTime();
        int passes = 0;
        while (canRun() && phaseTimer.seconds() < FACE_GOAL_SECONDS && passes < FACE_GOAL_MAX_PASSES) {
            GoalSighting sighting = scanForGoal(acceptAfterNanos, phaseTimer);
            if (sighting == null) {
                if (passes == 0) {
                    faceGoalByPose(phaseTimer);
                } else {
                    goalNote = goalNote + "  (goal not seen again after pass " + passes + ")";
                }
                return;
            }
            double errorLeft = sighting.errorLeftDegrees();
            if (Math.abs(errorLeft) <= FACE_TOLERANCE_DEGREES) {
                goalNote = String.format(Locale.US, "ALIGNED on %s  err %+.1f deg  d %.1f in  passes %d",
                        sighting.fiducial.name, errorLeft, LogiOpModeTools.goalDistanceInches(sighting.fiducial), passes);
                return;
            }
            passes++;
            double correction = LogiOpModeTools.clip(errorLeft, MAX_TURN_DEGREES);
            if (follower != null) {
                turnByFollower(sighting, correction, phaseTimer, passes);
            } else {
                turnByVision(phaseTimer);
                return;
            }
            acceptAfterNanos = System.nanoTime();
        }
        if (passes >= FACE_GOAL_MAX_PASSES) {
            goalNote = goalNote + "  (pass limit)";
        } else if (phaseTimer.seconds() >= FACE_GOAL_SECONDS) {
            goalNote = goalNote + "  (phase timeout)";
        }
    }

    private GoalSighting scanForGoal(long acceptAfterNanos, ElapsedTime phaseTimer) {
        ElapsedTime scanTimer = new ElapsedTime();
        while (canRun() && scanTimer.seconds() < GOAL_SCAN_SECONDS && phaseTimer.seconds() < FACE_GOAL_SECONDS) {
            LogiResult result = tick();
            if (LogiOpModeTools.isFresh(result, goalsPipeline, MAX_RESULT_AGE_MS) && result.getCaptureNanos() > acceptAfterNanos) {
                LogiFiducial goal = LogiOpModeTools.chooseGoal(result, alliance, true);
                if (goal != null) {
                    return new GoalSighting(goal, result.getCaptureNanos());
                }
            }
            addPhaseTelemetry(String.format(Locale.US, "scanning for %s goals  %.2f / %.2f s", alliance,
                    scanTimer.seconds(), GOAL_SCAN_SECONDS));
        }
        return null;
    }

    private void turnByFollower(GoalSighting sighting, double correctionDegrees, ElapsedTime phaseTimer, int pass) {
        Pose here = currentPose();
        if (here == null) {
            goalNote = "no pose - turn skipped";
            return;
        }
        Double headingAtCapture = headingHistory.headingAt(sighting.captureNanos, LogiOpModeTools.HEADING_LOOKUP_SLACK_NANOS);
        double base = headingAtCapture != null ? headingAtCapture : here.heading();
        double targetHeading = PlanarPose.normalizeRadians(base + Math.toRadians(correctionDegrees));
        follower.hold(here.withHeading(targetHeading));
        int seenAccepted = acceptedFixes();
        ElapsedTime turnTimer = new ElapsedTime();
        double error = headingErrorDegrees(targetHeading);
        while (canRun() && turnTimer.seconds() < TURN_PASS_SECONDS && phaseTimer.seconds() < FACE_GOAL_SECONDS) {
            tick();
            seenAccepted = reholdIfCorrected(seenAccepted, targetHeading);
            error = headingErrorDegrees(targetHeading);
            if (Math.abs(error) <= FACE_TOLERANCE_DEGREES) {
                break;
            }
            addPhaseTelemetry(String.format(Locale.US, "pass %d  turning %+.1f deg to %s  err %+.1f deg", pass,
                    correctionDegrees, sighting.fiducial.name, error));
        }
        goalNote = String.format(Locale.US, "pass %d: %s bL %+.1f deg -> turned %+.1f deg  residual %+.1f deg%s",
                pass, sighting.fiducial.name, LogiOpModeTools.goalBearingLeftDegrees(sighting.fiducial), correctionDegrees,
                error, Math.abs(error) <= FACE_TOLERANCE_DEGREES ? "" : "  TURN TIMEOUT");
    }

    private void turnByVision(ElapsedTime phaseTimer) {
        LogiAimConfig aim = aimConfig();
        String name = "none";
        double lastError = Double.NaN;
        boolean aligned = false;
        while (canRun() && phaseTimer.seconds() < FACE_GOAL_SECONDS) {
            LogiResult result = tick();
            LogiFiducial goal = LogiOpModeTools.isFresh(result, goalsPipeline, MAX_RESULT_AGE_MS)
                    ? LogiOpModeTools.chooseGoal(result, alliance, true) : null;
            if (goal == null) {
                command(0.0, 0.0, 0.0);
                addPhaseTelemetry("vision-only turn - waiting for the goal");
                continue;
            }
            name = goal.name;
            lastError = LogiOpModeTools.goalBearingLeftDegrees(goal) + Constants.SHOOTER_HEADING_OFFSET_DEGREES;
            if (Math.abs(lastError) <= FACE_TOLERANCE_DEGREES) {
                aligned = true;
                break;
            }
            double turn = aim.turnLeftPowerForTx(-LogiOpModeTools.clip(lastError, MAX_TURN_DEGREES));
            command(0.0, 0.0, turn);
            addPhaseTelemetry(String.format(Locale.US, "vision-only turn  %s err %+.1f deg  power %+.2f", name, lastError, turn));
        }
        command(0.0, 0.0, 0.0);
        goalNote = String.format(Locale.US, "vision-only turn on %s  err %+.1f deg  %s", name, lastError,
                aligned ? "ALIGNED" : "TIMEOUT");
    }

    private void faceGoalByPose(ElapsedTime phaseTimer) {
        if (follower == null || !GoalAim.goalsConfirmed(alliance)) {
            goalNote = String.format(Locale.US, "no %s goal seen in %.1f s - heading kept%s", alliance, GOAL_SCAN_SECONDS,
                    GoalAim.goalsConfirmed(alliance) ? "" : " (goal points not confirmed, no pose fallback)");
            return;
        }
        Pose here = currentPose();
        GoalAim.Solution solution = GoalAim.solve(alliance, here);
        follower.hold(here.withHeading(solution.heading));
        int seenAccepted = acceptedFixes();
        ElapsedTime turnTimer = new ElapsedTime();
        double error = headingErrorDegrees(solution.heading);
        while (canRun() && turnTimer.seconds() < TURN_PASS_SECONDS && phaseTimer.seconds() < FACE_GOAL_SECONDS) {
            tick();
            seenAccepted = reholdIfCorrected(seenAccepted, solution.heading);
            error = headingErrorDegrees(solution.heading);
            if (Math.abs(error) <= FACE_TOLERANCE_DEGREES) {
                break;
            }
            addPhaseTelemetry(String.format(Locale.US, "pose fallback turn to %s  err %+.1f deg", solution.goalName, error));
        }
        goalNote = String.format(Locale.US, "no goal seen - pose fallback to %s  residual %+.1f deg", solution.goalName, error);
    }

    private void approachPollen() {
        phase = "approach ball";
        if (!APPROACH_ENABLED) {
            pollenNote = "disabled (APPROACH_ENABLED = false)";
            return;
        }
        String profile = LogiOpModeTools.allianceProfile(alliance);
        int index = LogiOpModeTools.pollenPipelineIndex(profile);
        if (index < 0) {
            profile = LogiOpModeTools.YELLOW_PROFILE;
            index = LogiOpModeTools.pollenPipelineIndex(profile);
        }
        if (index < 0) {
            pollenNote = "skipped - no COLOR pipeline for that ball color";
            return;
        }
        light.pipelineSwitch(index);
        LogiAimConfig aim = aimConfig();
        ElapsedTime timer = new ElapsedTime();
        double lastSeen = -1.0;
        double lastSign = 0.0;
        double lastDistance = Double.NaN;
        double lossDistance = Double.NaN;
        boolean arrived = false;
        String outcome = "TIMEOUT";
        holdHere();
        boolean holdingStill = true;
        int seenAccepted = acceptedFixes();
        while (canRun() && timer.seconds() < APPROACH_SECONDS) {
            LogiResult result = tick();
            int accepted = acceptedFixes();
            if (accepted != seenAccepted) {
                seenAccepted = accepted;
                if (holdingStill) {
                    holdHere();
                }
            }
            ColorTarget target = LogiOpModeTools.isFresh(result, index, MAX_RESULT_AGE_MS) ? result.getPrimaryColor() : null;
            double forward = 0.0;
            double turn = 0.0;
            boolean move = false;
            String detail;
            if (target != null) {
                Pose pose = currentPose();
                double tx = LogiOpModeTools.robotCenteredTx(target, headingHistory, result.getCaptureNanos(),
                        pose != null ? pose.heading() : Double.NaN);
                lastSeen = timer.seconds();
                lastSign = Math.signum(tx);
                lossDistance = target.hasGroundPosition() ? target.groundDistanceInches : Double.NaN;
                if (target.hasGroundPosition()) {
                    lastDistance = target.groundDistanceInches;
                    if (target.groundDistanceInches <= aim.stopDistanceInches) {
                        arrived = true;
                        outcome = "ARRIVED";
                        break;
                    }
                }
                turn = aim.turnLeftPowerForTx(tx);
                forward = target.hasGroundPosition() ? aim.forwardPowerForDistance(target.groundDistanceInches, tx) : 0.0;
                move = true;
                detail = String.format(Locale.US, "%s tx %+.1f  d %s  fwd %+.2f  turn %+.2f", target.profileName, tx,
                        target.hasGroundPosition() ? String.format(Locale.US, "%.1f in", target.groundDistanceInches)
                                : "no ground fix", forward, turn);
            } else if (lastSeen >= 0.0 && LogiOpModeTools.isFinite(lossDistance) && lossDistance <= aim.arriveOnLossInches) {
                if (timer.seconds() - lastSeen >= ARRIVE_LOSS_CONFIRM_SECONDS) {
                    arrived = true;
                    outcome = "ARRIVED";
                    break;
                }
                detail = String.format(Locale.US, "lost at %.1f in - confirming arrival", lossDistance);
            } else if (lastSeen >= 0.0 && timer.seconds() - lastSeen <= SEARCH_SECONDS && lastSign != 0.0) {
                turn = -lastSign * aim.searchTurnPower;
                move = true;
                detail = "target lost - searching " + (lastSign > 0 ? "right" : "left");
            } else if (lastSeen >= 0.0) {
                outcome = "LOST";
                break;
            } else if (timer.seconds() > POLLEN_ACQUIRE_SECONDS) {
                outcome = "NOT SEEN";
                break;
            } else {
                detail = "waiting for " + profile + " ball";
            }
            if (move) {
                command(forward, 0.0, turn);
                holdingStill = false;
            } else if (!holdingStill) {
                holdHere();
                holdingStill = true;
            }
            addPhaseTelemetry(String.format(Locale.US, "%.1f / %.1f s  %s", timer.seconds(), APPROACH_SECONDS, detail));
        }
        holdHere();
        pollenNote = String.format(Locale.US, "%s %s ball in %.1f s  last distance %s%s", outcome, profile, timer.seconds(),
                LogiOpModeTools.isFinite(lastDistance) ? String.format(Locale.US, "%.1f in", lastDistance) : "unknown",
                arrived ? String.format(Locale.US, "  (stop at %.1f in)", aim.stopDistanceInches) : "");
    }

    private LogiResult tick() {
        if (follower != null && started) {
            if (fusion != null) {
                fusion.beforeUpdate();
            }
            follower.update();
        } else if (fusion != null) {
            fusion.update();
        }
        Pose pose = currentPose();
        if (pose != null && LogiOpModeTools.isFinite(pose.heading())) {
            long now = System.nanoTime();
            headingHistory.add(now, pose.heading());
            light.updateRobotOrientation(pose.heading(), now);
        }
        light.update();
        LogiResult result = light.getLatestResult();
        long capture = result.getCaptureNanos();
        if (fusion != null && fusionAllowed && result.hasBotpose() && capture != lastOfferedCaptureNanos) {
            lastOfferedCaptureNanos = capture;
            fusion.fuse(result);
        }
        lastResult = result;
        LogiLightField.drawResult(odometryPose(), result, pose, light.getFieldMap());
        return result;
    }

    private Pose currentPose() {
        if (follower != null) {
            return follower.pose();
        }
        if (fusion != null) {
            return fusion.pose();
        }
        return null;
    }

    private Pose odometryPose() {
        if (pinpoint == null) {
            return null;
        }
        try {
            return pinpoint.pose();
        } catch (Throwable t) {
            return null;
        }
    }

    private Pose startPose() {
        return LogiOpModeTools.startPose(alliance);
    }

    private void resetToStartPose() {
        if (fusion != null) {
            fusion.setPose(startPose());
        }
        headingHistory.clear();
    }

    private double headingErrorDegrees(double targetHeading) {
        Pose pose = currentPose();
        if (pose == null) {
            return Double.NaN;
        }
        return Math.toDegrees(PlanarPose.normalizeRadians(targetHeading - pose.heading()));
    }

    private void command(double forward, double strafeLeft, double turnLeft) {
        if (follower != null) {
            follower.manual(forward, strafeLeft, turnLeft);
        } else if (directDrive != null) {
            directDrive.drive(new DrivePowers(forward, strafeLeft, turnLeft), true);
        }
    }

    private int acceptedFixes() {
        return fusion != null ? fusion.acceptedCount() : 0;
    }

    private int reholdIfCorrected(int seenAccepted, double targetHeading) {
        int accepted = acceptedFixes();
        if (accepted == seenAccepted || follower == null) {
            return seenAccepted;
        }
        Pose here = currentPose();
        if (here != null) {
            follower.hold(here.withHeading(targetHeading));
        }
        return accepted;
    }

    private void holdHere() {
        if (follower != null) {
            Pose pose = follower.pose();
            if (pose != null) {
                follower.hold(pose);
            }
        } else if (directDrive != null) {
            directDrive.stop();
        }
    }

    private void stopMotors() {
        try {
            if (follower != null) {
                follower.stop();
                follower.update();
            }
        } catch (Throwable ignored) {
        }
        try {
            if (directDrive != null) {
                directDrive.stop();
            }
        } catch (Throwable ignored) {
        }
    }

    private void stopMotorsQuietlyInIdle() {
        if (follower == null && directDrive != null) {
            try {
                directDrive.stop();
            } catch (Throwable ignored) {
            }
        }
    }

    private boolean canRun() {
        return opModeIsActive() && runtime.seconds() < AUTONOMOUS_BUDGET_SECONDS;
    }

    private String budgetReason(String where) {
        if (!opModeIsActive()) {
            return "stop requested " + where;
        }
        return String.format(Locale.US, "budget %.1f s reached %s", AUTONOMOUS_BUDGET_SECONDS, where);
    }

    private static LogiAimConfig aimConfig() {
        LogiAimConfig aim = LogiLightSettings.AIM;
        return aim == null ? new LogiAimConfig() : aim;
    }

    private void addInitTelemetry(LogiResult result) {
        telemetry.addLine("LogiLight Auto Example");
        telemetry.addData("ALLIANCE", "%s   (X = BLUE, B = RED, gp1 BACK = re-zero)", alliance);
        telemetry.addData("Start pose", LogiLightTelemetry.formatPose(startPose()));
        telemetry.addData("Measured pose", LogiLightTelemetry.formatPose(currentPose()));
        boolean pointsConfirmed = alliance == GoalScanner.Alliance.BLUE ? FieldPointsBlue.POINTS_CONFIRMED
                : FieldPoints.POINTS_CONFIRMED;
        if (!pointsConfirmed) {
            telemetry.addLine("!! " + alliance + " START POSE NOT CONFIRMED - only vision fixes make the pose absolute");
        }
        if (light == null || !light.isAvailable()) {
            telemetry.addLine("!! CAMERA NOT AVAILABLE - AUTO WILL NOT MOVE");
            telemetry.addData("Camera fault", light == null ? "not created" : light.fault());
        } else {
            telemetry.addData("Camera", light.getStatus().summaryLine());
            telemetry.addLine("Wait for the camera to stream before pressing START.");
        }
        if (pinpoint == null) {
            telemetry.addLine("!! PINPOINT NOT AVAILABLE - vision-only mode: no relocalize, direct drive power");
            telemetry.addData("Pinpoint fault", pinpointFault);
        }
        if (follower != null) {
            telemetry.addData("Drive", "Pedro Follower on LogiLight fusion");
        } else if (directDrive != null) {
            telemetry.addData("Drive", "direct Mecanum power (no Follower)  %s", followerFault);
        } else {
            telemetry.addLine("!! NO DRIVETRAIN - NOTHING WILL MOVE  " + driveFault + "  " + followerFault);
        }
        LogiFieldMap map = light == null ? null : light.getFieldMap();
        telemetry.addData("Field map", map == null ? "none" : map.summary());
        LogiLocalizationConfig loc = LogiLightSettings.LOCALIZATION;
        if (map != null && map.surveyedCount() == 0 && (loc == null || !loc.useUnsurveyedTargets)) {
            telemetry.addLine("!! NO SURVEYED TARGETS - relocalize will find nothing (run LogiLight Field Survey)");
        } else if (map != null && map.surveyedCount() == 0) {
            telemetry.addLine("!! unsurveyed targets: MT2 only (Pinpoint heading), rough until LogiLight Field Survey is run");
        }
        LogiBotPose best = result.getBestBotpose();
        telemetry.addData("Vision vs start", best == null ? "no botpose" : LogiOpModeTools.poseError(best, currentPose()));
        LogiFiducial goal = LogiOpModeTools.chooseGoal(result, alliance, true);
        telemetry.addData("Nearest goal", goal == null ? "none visible" : LogiLightTelemetry.fiducialLine(goal));
        telemetry.addData("Plan", "relocalize <= %.1f s, face goal <= %.1f s, ball <= %.1f s%s", RELOCALIZE_SECONDS,
                FACE_GOAL_SECONDS, APPROACH_SECONDS, APPROACH_ENABLED ? "" : " (approach OFF)");
        if (!settingsMessage.isEmpty()) {
            telemetry.addData("Settings", settingsMessage);
        }
        telemetry.addLine(LogiOpModeTools.settingsHint());
        telemetry.addLine("Nothing is commanded until START");
    }

    private void addPhaseTelemetry(String detail) {
        telemetry.addData("Phase", "%s  %s", phase, detail);
        telemetry.addData("Runtime", "%.1f / %.1f s", runtime.seconds(), AUTONOMOUS_BUDGET_SECONDS);
        telemetry.addData("Pose", LogiLightTelemetry.formatPose(currentPose()));
        if (fusion != null) {
            telemetry.addData("Fusion", fusion.summary());
        }
        LogiLightTelemetry.addCompact(telemetry, lastResult);
        telemetry.addData("Relocalize", relocalizeNote);
        telemetry.addData("Goal", goalNote);
        telemetry.update();
    }

    private void addSummaryTelemetry() {
        telemetry.addData("Status", "Finished - motors stopped");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Reason", stopReason);
        telemetry.addData("Relocalize", relocalizeNote);
        telemetry.addData("Goal", goalNote);
        telemetry.addData("Ball", pollenNote);
        telemetry.addData("Final pose", LogiLightTelemetry.formatPose(currentPose()));
        if (fusion != null) {
            telemetry.addData("Fusion", fusion.summary());
        }
        LogiLightTelemetry.addCompact(telemetry, lastResult);
    }
}
