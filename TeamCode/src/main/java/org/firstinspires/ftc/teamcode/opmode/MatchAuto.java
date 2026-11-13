package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.launcher.Launcher;
import org.firstinspires.ftc.teamcode.opmode.autolib.AutoLibrary;
import org.firstinspires.ftc.teamcode.panels.PanelsBridge;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name = "Match Auto", group = "Auto")
public class MatchAuto extends LinearOpMode {

    /*
     * 原来 MatchAuto 的路线已经全部在 AutoLibrary 里了，对应的序列号:
     *   #1   START          -> SHOOT_1          shoot preload
     *   #72  SHOOT_1 ~ ANGLE_1 -> PARK
     * 红蓝用同一套号码，坐标自动从 FieldPoints / FieldPointsBlue 取。
     * 改顺序只要改下面 path(...) 里的数字，所有号码见 autolib/AutoLibrary.java。
     */
    private List<Step> route() {
        List<Step> steps = new ArrayList<>();

        steps.add(path(1).shoot(FieldPoints.SHOTS_AT_SHOOT_POSE_1));

        steps.add(path(72));

        return steps;
    }

    private static final double MAX_ACTION_SECONDS = 8.0;
    public static double SETTLE_SECONDS_BEFORE_SHOOT = 0.2;
    public static double TURN_TOLERANCE_DEGREES = 2.0;

    private static final double AUTONOMOUS_BUDGET_SECONDS = FieldPoints.AUTONOMOUS_BUDGET_SECONDS;
    private static final double MIN_PATH_LENGTH_INCHES = FieldPoints.MIN_PATH_LENGTH_INCHES;
    private static final String CONFIRM_FLAGS = "POINTS_CONFIRMED (and PARK_ROUTE_CONFIRMED for rack points)";

    private static final boolean VISION_AIMING_ENABLED = FieldPoints.VISION_AIMING_ENABLED;

    private static final double AIM_BEARING_TOLERANCE_DEGREES = FieldPoints.AIM_BEARING_TOLERANCE_DEGREES;
    private static final double AIM_BEARING_MAX_CORRECTION_DEGREES = FieldPoints.AIM_BEARING_MAX_CORRECTION_DEGREES;
    private static final double AIM_HEADING_SETTLE_TOLERANCE_DEGREES = FieldPoints.AIM_HEADING_SETTLE_TOLERANCE_DEGREES;

    private static final double SCAN_TIMEOUT_SECONDS = FieldPoints.SCAN_TIMEOUT_SECONDS;
    private static final double AIM_TURN_TIMEOUT_SECONDS = FieldPoints.AIM_TURN_TIMEOUT_SECONDS;
    private static final double PER_SHOT_TIMEOUT_SECONDS = FieldPoints.PER_SHOT_TIMEOUT_SECONDS;

    private static final double CONFIRM_SCAN_SECONDS = 0.25;
    private static final int AIM_MAX_TURN_PASSES = 2;
    private static final double AIM_MIN_PASS_SECONDS = 0.40;

    private static final double LOCALIZER_CHECK_SECONDS = 3.0;
    private static final double LOCALIZER_BACKWARD_LIMIT_INCHES = 3.0;
    private static final int LOCALIZER_FAULT_TICKS = 5;
    private static final double LOCALIZER_POD_DOMINANCE = 3.0;

    private String localizerFault = null;
    private String localizerFix = null;
    private String localizerCheck = null;

    private Follower follower;
    private GoalScanner scanner;
    private DcMotor intake;
    private Launcher launcher;

    private final ElapsedTime runtime = new ElapsedTime();

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;

    private String lastPathResult = "none";
    private String lastShotResult = "none";
    private String stopReason = "mission complete";

    private String aimCluster = "none";
    private int aimPercent = 0;
    private double aimRange = 0.0;
    private double aimBearingBefore = 0.0;
    private double aimBearingAfter = 0.0;
    private long aimSightingAgeMs = -1L;
    private boolean aimUsedFallback = true;
    private String aimNote = "not aimed yet";
    private String poseAimNote = "not aimed yet";
    private String shootGoal = null;
    private String aimTurnResult = "no turn yet";
    private int aimTurnPasses = 0;

    @Override
    public void runOpMode() {
        PoseHandoff.clear();
        checkRoute(route());

        follower = Constants.createFollower(hardwareMap);
        scanner = new GoalScanner(hardwareMap);


        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE);
        intake.setDirection(Constants.INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher = new Launcher(hardwareMap);
        launcher.setReadyTimeoutSeconds(Launcher.AUTO_READY_TIMEOUT_SECONDS);
        launcher.setStagingTransferPower(Launcher.AUTO_STAGING_TRANSFER_POWER);
        telemetry.addData("Outtake velocity", "%.0f ticks/s", () -> launcher.measuredVelocity());

        follower.setPose(startPose());

        List<Step> mission = route();

        try {
            while (!isStarted() && !isStopRequested()) {
                GoalScanner.Alliance picked = alliance;
                if ((gamepad1.x && !gamepad1.start) || (gamepad2.x && !gamepad2.start)) {
                    picked = GoalScanner.Alliance.BLUE;
                }
                if ((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start)) {
                    picked = GoalScanner.Alliance.RED;
                }
                if (picked != alliance) {
                    alliance = picked;
                    mission = route();
                    follower.setPose(startPose());
                }

                follower.localizer.update();
                PanelsBridge.drawRobot(follower.pose());
                scanner.update();
                telemetry.addLine("Match Auto");
                telemetry.addData("ALLIANCE", "%s   (X = BLUE, B = RED)", alliance);
                telemetry.addData("Points file", pointsFile());
                telemetry.addData("Steps", mission.size());
                telemetry.addData("Paths", pathNumbers(mission));
                telemetry.addData("Budget", "%.1f s", AUTONOMOUS_BUDGET_SECONDS);
                telemetry.addData("Start pose", formatPose(startPose()));
                telemetry.addData("Measured pose", formatPose(follower.pose()));
                telemetry.addData("Localizer", Constants.PINPOINT_NAME);
                launcher.sense();

                telemetry.addLine();
                telemetry.addData("Vision aiming", VISION_AIMING_ENABLED ? "ENABLED" : "DISABLED (static aim only)");
                telemetry.addData("Goal clusters", "%s / %s", alliance.firstCluster, alliance.secondCluster);
                telemetry.addData("Camera", scanner.cameraState());
                if (!scanner.isAvailable()) {
                    telemetry.addLine("!! CAMERA NOT AVAILABLE - AUTO WILL USE THE STATIC AIM");
                    telemetry.addData("Camera fault", scanner.fault());
                } else {
                    telemetry.addData("Candidates", scanner.candidateSummary(alliance));
                    telemetry.addLine("Wait for Camera = STREAMING before pressing START.");
                }

                if (!pointsConfirmed()) {
                    telemetry.addLine("!! " + alliance + " FIELD POINTS NOT CONFIRMED - AUTO WILL NOT MOVE");
                    telemetry.addLine("Measure the real coordinates in " + pointsFile() + ",");
                    telemetry.addLine("then set " + CONFIRM_FLAGS + " = true.");
                }

                telemetry.addLine("Nothing is commanded until START");
                telemetry.update();
            }

            if (!pointsConfirmed()) {
                stopAll();

                while (opModeIsActive()) {
                    telemetry.addLine("!! " + alliance + " FIELD POINTS NOT CONFIRMED - REFUSING TO DRIVE");
                    telemetry.addLine("The poses in " + pointsFile() + " are placeholders.");
                    telemetry.addLine("Measure them, then set " + CONFIRM_FLAGS + " = true.");
                    telemetry.update();
                }

                return;
            }

            if (isStopRequested()) {
                return;
            }

            try {
                follower.setPose(startPose());
                tick();
                runtime.reset();

                for (int index = 0; index < mission.size(); index++) {
                    if (!opModeIsActive()) {
                        stopReason = "stop requested";
                        break;
                    }
                    if (budgetExhausted()) {
                        stopReason = String.format("budget %.1f s reached at step %d",
                                AUTONOMOUS_BUDGET_SECONDS, index + 1);
                        break;
                    }
                    runStep(index, mission.size(), mission.get(index));
                    if (localizerFault != null) {
                        stopReason = localizerFault;
                        break;
                    }
                }
            } finally {
                stopAll();
                PoseHandoff.save(follower.pose(), alliance);
            }

            if (localizerFault != null) {
                while (opModeIsActive()) {
                    addLocalizerFaultTelemetry();
                    telemetry.addData("Last path", lastPathResult);
                    telemetry.update();
                    sleep(50);
                }
                addLocalizerFaultTelemetry();
            }
            telemetry.addData("Status", "Finished");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Reason", stopReason);
            telemetry.addData("Runtime", "%.1f s", runtime.seconds());
            telemetry.addData("Final pose", formatPose(follower.pose()));
            telemetry.addData("Last path", lastPathResult);
            telemetry.addData("Last shot", lastShotResult);
            telemetry.addData("Last pose aim", poseAimNote);
            telemetry.addData("Last aim", aimNote);
            telemetry.addData("Vision frames", "%d updates, %d with a cluster",
                    scanner.updateCount(), scanner.clusterFrameCount());
            telemetry.update();
        } finally {
            scanner.close();
        }
    }

    private boolean isBlue() {
        return alliance == GoalScanner.Alliance.BLUE;
    }

    private Pose startPose() {
        return route().get(0).leg.start(alliance);
    }

    private boolean pointsConfirmed() {
        for (Step step : route()) {
            if (!step.leg.confirmed(alliance)) {
                return false;
            }
        }
        return true;
    }

    private static String pathNumbers(List<Step> steps) {
        StringBuilder text = new StringBuilder();
        for (Step step : steps) {
            if (text.length() > 0) text.append(", ");
            text.append(step.leg.number);
        }
        return text.toString();
    }

    private String pointsFile() {
        return isBlue() ? "FieldPointsBlue.java" : "FieldPoints.java";
    }

    private void checkRoute(List<Step> steps) {
        if (steps.isEmpty()) {
            throw new IllegalStateException("MatchAuto route has no paths");
        }
        for (int i = 1; i < steps.size(); i++) {
            AutoLibrary.Leg before = steps.get(i - 1).leg;
            AutoLibrary.Leg now = steps.get(i).leg;
            if (before.to != now.from) {
                throw new IllegalStateException("MatchAuto step " + (i + 1) + " path #" + now.number
                        + " starts at " + now.from + " but step " + i + " path #" + before.number
                        + " ends at " + before.to);
            }
        }
    }

    private void tick() {
        follower.update();
        PanelsBridge.drawRobot(follower.pose());
        scanner.update();
        launcher.update();
    }

    private boolean budgetExhausted() {
        return runtime.seconds() >= AUTONOMOUS_BUDGET_SECONDS;
    }

    private boolean canKeepRunning() {
        return opModeIsActive() && !budgetExhausted();
    }

    private void runStep(int index, int total, Step step) {
        if (step.shots > 0) {
            launcher.spinUp();
        }
        boolean timedOut = drive(index, total, step);

        if (!opModeIsActive() || localizerFault != null) {
            return;
        }

        follower.hold(timedOut ? follower.pose() : step.to);
        tick();

        addStepTelemetry(index, total, step, timedOut ? "TIMED OUT" : "arrived");
        if (timedOut) {
            telemetry.addLine("WARNING path timed out - holding the measured pose, not the target");
        }
        telemetry.update();

        if (budgetExhausted()) {
            return;
        }

        if (step.shots > 0) {
            holdFor(SETTLE_SECONDS_BEFORE_SHOOT);
            runShoot(index, total, step);
        }
        if (step.intakeSeconds > 0 && canKeepRunning()) {
            runIntake(index, total, step);
        }
        if (step.pauseSeconds > 0 && canKeepRunning()) {
            holdFor(step.pauseSeconds);
        }
    }

    private boolean drive(int index, int total, Step step) {
        if (step.leg.length(alliance) < MIN_PATH_LENGTH_INCHES) {
            return turnInPlace(index, total, step);
        }

        follower.follow(step.leg.build(alliance));

        if (step.intakeWhileDriving) {
            intake.setPower(Constants.INTAKE_POWER);
        }

        ElapsedTime pathTimer = new ElapsedTime();
        boolean timedOut = false;

        Pose[] legPoses = step.leg.poses(alliance);
        Pose legStart = legPoses[0];
        double legX = legPoses[1].x() - legStart.x();
        double legY = legPoses[1].y() - legStart.y();
        double legLength = Math.hypot(legX, legY);
        int backwardTicks = 0;

        while (canKeepRunning() && (follower.following() || follower.isBusy())) {
            if (pathTimer.seconds() >= step.timeoutSeconds) {
                timedOut = true;
                break;
            }
            tick();
            if (index == 0 && legLength > 0.0 && pathTimer.seconds() <= LOCALIZER_CHECK_SECONDS) {
                Pose here = follower.pose();
                double along = ((here.x() - legStart.x()) * legX + (here.y() - legStart.y()) * legY) / legLength;
                backwardTicks = along < -LOCALIZER_BACKWARD_LIMIT_INCHES ? backwardTicks + 1 : 0;
                if (backwardTicks >= LOCALIZER_FAULT_TICKS) {
                    reportLocalizerFault(legStart, legX, legY, along);
                    break;
                }
            }
            addStepTelemetry(index, total, step, "following");
            telemetry.addData("Remaining on path", "%.1f in", follower.remainingDistance());
            telemetry.addData("Path time", "%.1f / %.1f s", pathTimer.seconds(), step.timeoutSeconds);
            telemetry.update();
        }

        intake.setPower(0.0);

        if (localizerFault != null) {
            lastPathResult = String.format("step %d STOPPED after %.1f s - localizer fault", index + 1, pathTimer.seconds());
            return false;
        }

        lastPathResult = timedOut
                ? String.format("step %d TIMEOUT after %.1f s", index + 1, pathTimer.seconds())
                : String.format("step %d arrived in %.1f s", index + 1, pathTimer.seconds());
        return timedOut;
    }

    private void reportLocalizerFault(Pose legStart, double legX, double legY, double along) {
        double heading = legStart.heading();
        double forwardPart = Math.abs(Math.cos(heading) * legX + Math.sin(heading) * legY);
        double strafePart = Math.abs(-Math.sin(heading) * legX + Math.cos(heading) * legY);
        localizerFault = String.format("Pinpoint measured %.1f in BACKWARD while driving the first path", -along);
        if (forwardPart >= LOCALIZER_POD_DOMINANCE * strafePart) {
            localizerFix = "If it moves AWAY: flip PINPOINT_X_POD_DIRECTION in pedroPathing/Constants.java.";
        } else if (strafePart >= LOCALIZER_POD_DOMINANCE * forwardPart) {
            localizerFix = "If it moves AWAY: flip PINPOINT_Y_POD_DIRECTION in pedroPathing/Constants.java.";
        } else {
            localizerFix = "If it moves AWAY: check PINPOINT_X_POD_DIRECTION and PINPOINT_Y_POD_DIRECTION"
                    + " in pedroPathing/Constants.java.";
        }
        localizerCheck = String.format("Measured pose must move from (%.1f, %.1f) toward (%.1f, %.1f).",
                legStart.x(), legStart.y(), legStart.x() + legX, legStart.y() + legY);
    }

    private void addLocalizerFaultTelemetry() {
        telemetry.addLine("!! LOCALIZER READS BACKWARD - AUTO STOPPED");
        telemetry.addLine(localizerFault);
        telemetry.addLine("Change nothing yet. INIT this OpMode and push the robot by hand along the first path.");
        telemetry.addLine(localizerCheck);
        telemetry.addLine(localizerFix);
        telemetry.addLine("If the other coordinate changes instead: the two pod cables are swapped on the Pinpoint.");
        telemetry.addLine("If the robot itself drove backward, fix the drive motor directions instead.");
    }

    private boolean turnInPlace(int index, int total, Step step) {
        follower.hold(step.to);

        if (step.intakeWhileDriving) {
            intake.setPower(Constants.INTAKE_POWER);
        }

        ElapsedTime turnTimer = new ElapsedTime();
        boolean timedOut = false;

        while (canKeepRunning() && headingErrorDegrees(step.to) > TURN_TOLERANCE_DEGREES) {
            if (turnTimer.seconds() >= step.timeoutSeconds) {
                timedOut = true;
                break;
            }
            tick();
            addStepTelemetry(index, total, step, "turning in place");
            telemetry.addData("Heading error", "%.1f deg", headingErrorDegrees(step.to));
            telemetry.addData("Turn time", "%.1f / %.1f s", turnTimer.seconds(), step.timeoutSeconds);
            telemetry.update();
        }

        intake.setPower(0.0);

        lastPathResult = timedOut
                ? String.format("step %d TURN TIMEOUT after %.1f s", index + 1, turnTimer.seconds())
                : String.format("step %d turned in place in %.1f s", index + 1, turnTimer.seconds());
        return timedOut;
    }

    private double headingErrorDegrees(Pose target) {
        return Math.abs(Math.toDegrees(normalizeRadians(target.heading() - follower.pose().heading())));
    }

    private void runShoot(int index, int total, Step step) {
        int shots = step.shots;

        if (shots <= 0) {
            lastShotResult = String.format("step %d requested 0 shots - skipped", index + 1);
            return;
        }

        launcher.spinUp();
        shootGoal = turnToGoal(index, total, step, null);
        if (!waitForOpenGoal(index, total, step)) {
            lastShotResult = canKeepRunning()
                    ? String.format("step %d: goal FACING DOWN for %.1f s - shots skipped",
                            index + 1, Constants.GOAL_OPEN_WAIT_SECONDS)
                    : String.format("step %d: out of time before shooting", index + 1);
            if (!Launcher.AUTO_KEEP_SPINNING) {
                launcher.stop();
            }
            return;
        }
        int detected = 0;
        int fired = 0;
        boolean turnedDown = false;
        for (int shot = 0; shot < shots && canKeepRunning(); shot++) {
            if (goalTurnedDown()) {
                turnedDown = true;
                break;
            }
            scanAimAndCommit(index, total, step, shot, shots);
            if (goalTurnedDown()) {
                turnedDown = true;
                break;
            }
            fired++;
            if (fireOne(index, total, step, String.format("shot %d / %d", shot + 1, shots))) {
                detected++;
            }
        }
        for (int extra = 0; !turnedDown && detected < shots && extra < Launcher.AUTO_EXTRA_ATTEMPTS && canKeepRunning(); extra++) {
            if (goalTurnedDown()) {
                turnedDown = true;
                break;
            }
            fired++;
            if (fireOne(index, total, step, String.format("extra feed %d", extra + 1))) {
                detected++;
            }
        }

        Launcher.Shot last = launcher.lastShot();
        lastShotResult = String.format("step %d: %d / %d shots detected%s%s", index + 1, detected, shots,
                turnedDown ? String.format("  STOPPED after %d feeds - goal turned down", fired) : "",
                last == null ? "" : "  last " + last.summary());

        if (!Launcher.AUTO_KEEP_SPINNING) {
            launcher.stop();
        }
    }

    private boolean goalTurnedDown() {
        tick();
        GoalScanner.GoalView view = shootGoal != null ? scanner.goalView(shootGoal) : scanner.facingView(alliance);
        return view == GoalScanner.GoalView.FACING_DOWN && scanner.blocksShot(view);
    }

    private boolean waitForOpenGoal(int index, int total, Step step) {
        ElapsedTime waitTimer = new ElapsedTime();
        boolean switched = false;
        while (canKeepRunning()) {
            tick();
            GoalScanner.GoalView view = shootGoal != null ? scanner.goalView(shootGoal) : scanner.facingView(alliance);
            if (!scanner.blocksShot(view)) {
                return true;
            }
            if (!switched && shootGoal != null && view == GoalScanner.GoalView.FACING_DOWN) {
                GoalAim.Solution other = GoalAim.solveUp(alliance, follower.pose(), scanner);
                if (!other.goalName.equals(shootGoal)) {
                    switched = true;
                    shootGoal = turnToGoal(index, total, step, other.goalName);
                    continue;
                }
            }
            if (waitTimer.seconds() >= Constants.GOAL_OPEN_WAIT_SECONDS) {
                return false;
            }
            addStepTelemetry(index, total, step, "waiting - goal " + view);
            telemetry.addData("Wait", "%.1f / %.1f s", waitTimer.seconds(), Constants.GOAL_OPEN_WAIT_SECONDS);
            telemetry.addData("Candidates", scanner.candidateSummary(alliance));
            telemetry.update();
        }
        return false;
    }

    private boolean fireOne(int index, int total, Step step, String label) {
        int attemptsBefore = launcher.shotsAttempted();
        int detectedBefore = launcher.shotsDetected();
        launcher.requestShots(1);
        ElapsedTime shotTimer = new ElapsedTime();

        while (canKeepRunning() && (launcher.shotsAttempted() == attemptsBefore || launcher.isBusy())) {
            if (!launcher.isBusy() && (!launcher.canFire() || shotTimer.seconds() >= Launcher.AUTO_SHOT_TIMEOUT_SECONDS)) {
                break;
            }
            tick();
            intake.setPower(launcher.isFeeding() ? Launcher.AUTO_FEED_INTAKE_POWER : 0.0);
            addStepTelemetry(index, total, step, label);
            launcher.addTelemetry(telemetry);
            addAimTelemetry();
            telemetry.update();
        }

        launcher.cancelShots();
        intake.setPower(0.0);
        return launcher.shotsDetected() > detectedBefore;
    }

    private String turnToGoal(int index, int total, Step step, String goalName) {
        if (!GoalAim.goalsConfirmed(alliance)) {
            poseAimNote = alliance + " goal points not confirmed - kept path heading and default velocity";
            return null;
        }
        Pose here = follower.pose();
        GoalAim.Solution aim = goalName == null
                ? GoalAim.solveUp(alliance, here, scanner)
                : GoalAim.solveNamed(alliance, here, goalName);
        launcher.setTargetVelocity(aim.velocity);
        follower.hold(here.withHeading(aim.heading));

        ElapsedTime turnTimer = new ElapsedTime();
        boolean settled = false;
        while (canKeepRunning() && turnTimer.seconds() < AIM_TURN_TIMEOUT_SECONDS) {
            tick();
            double errorDegrees = Math.toDegrees(aim.headingError(follower.pose()));
            if (Math.abs(errorDegrees) <= AIM_HEADING_SETTLE_TOLERANCE_DEGREES) {
                settled = true;
                break;
            }
            addStepTelemetry(index, total, step, "pose aim turn");
            telemetry.addData("Goal", aim.summary(follower.pose()));
            telemetry.addData("Velocity", "%.0f for %.1f in", aim.velocity, aim.distance);
            telemetry.addData("Turn time", "%.2f / %.2f s", turnTimer.seconds(), AIM_TURN_TIMEOUT_SECONDS);
            telemetry.update();
        }

        poseAimNote = String.format("%s  %.1f in -> %.0f ticks/s  %s%s",
                aim.goalName, aim.distance, aim.velocity,
                settled ? "aligned" : String.format("TURN TIMEOUT (%.1f deg off)",
                        Math.toDegrees(aim.headingError(follower.pose()))),
                aim.inEntryAngle ? "" : "  BAD ENTRY ANGLE");
        return aim.goalName;
    }

    private void scanAimAndCommit(int index, int total, Step step, int shot, int shots) {
        ElapsedTime aimTimer = new ElapsedTime();

        aimCluster = "none";
        aimPercent = 0;
        aimRange = 0.0;
        aimBearingBefore = 0.0;
        aimBearingAfter = 0.0;
        aimSightingAgeMs = -1L;
        aimTurnResult = "no turn needed";
        aimTurnPasses = 0;

        if (!VISION_AIMING_ENABLED) {
            commitFallback("vision aiming disabled in FieldPoints");
            return;
        }
        if (!scanner.isAvailable()) {
            commitFallback("no camera: " + scanner.fault());
            return;
        }

        double scanLimit = Math.min(SCAN_TIMEOUT_SECONDS, remainingAimSeconds(aimTimer));
        GoalScanner.Sighting target = scanFor(scanLimit, index, total, step, shot, shots);

        if (target == null) {
            commitFallback(String.format("no usable %s / %s cluster (>= %d%%, age <= %d ms) in %.2f s",
                    alliance.firstCluster, alliance.secondCluster,
                    GoalScanner.MIN_PERCENT_CLUSTER_FOUND, GoalScanner.SIGHTING_FRESHNESS_MS, scanLimit));
            return;
        }

        String lockedCluster = target.shortName;
        aimCluster = lockedCluster;
        aimPercent = target.percentClusterFound;
        aimRange = target.range;
        aimBearingBefore = target.bearing;
        aimBearingAfter = target.bearing;
        aimSightingAgeMs = target.ageMs();

        double workingBearing = target.bearing;
        boolean bearingConfirmedAfterTurn = false;

        while (Math.abs(workingBearing) > AIM_BEARING_TOLERANCE_DEGREES
                && aimTurnPasses < AIM_MAX_TURN_PASSES
                && remainingAimSeconds(aimTimer) >= AIM_MIN_PASS_SECONDS
                && canKeepRunning()) {

            aimTurnPasses++;

            boolean settled = turnToReduceBearing(workingBearing, lockedCluster, aimPercent,
                    aimTimer, index, total, step, shot, shots);
            long turnEndNanos = System.nanoTime();

            GoalScanner.Sighting confirm = confirmFor(
                    Math.min(CONFIRM_SCAN_SECONDS, remainingAimSeconds(aimTimer)),
                    lockedCluster, turnEndNanos, index, total, step, shot, shots);

            if (confirm == null) {
                aimTurnResult = aimTurnResult + " (no confirm sighting, kept pre-turn range)";
                break;
            }

            bearingConfirmedAfterTurn = true;
            aimPercent = confirm.percentClusterFound;
            aimRange = confirm.range;
            aimBearingAfter = confirm.bearing;
            aimSightingAgeMs = confirm.ageMs();
            workingBearing = confirm.bearing;

            if (!settled) {
                break;
            }
        }

        if (aimTurnPasses == 0) {
            aimTurnResult = String.format("no turn, already within %.1f deg",
                    AIM_BEARING_TOLERANCE_DEGREES);
        } else if (bearingConfirmedAfterTurn) {
            aimTurnResult = String.format("%s | %d pass, confirmed residual %.1f deg",
                    aimTurnResult, aimTurnPasses, aimBearingAfter);
        } else {
            aimTurnResult = String.format("%s | %d pass, residual UNCONFIRMED (last seen %.1f deg)",
                    aimTurnResult, aimTurnPasses, aimBearingBefore);
        }

        aimUsedFallback = false;
        aimNote = String.format("%s %d%% range %.1f in (bearing %.1f -> %.1f deg)",
                aimCluster, aimPercent, aimRange, aimBearingBefore, aimBearingAfter);
    }

    private void commitFallback(String reason) {
        aimUsedFallback = true;
        aimNote = "NO VISION TURN - " + reason;
    }

    private double remainingAimSeconds(ElapsedTime aimTimer) {
        return Math.max(0.0, PER_SHOT_TIMEOUT_SECONDS - aimTimer.seconds());
    }

    private GoalScanner.Sighting scanFor(double seconds, int index, int total, Step step,
                                         int shot, int shots) {
        ElapsedTime scanTimer = new ElapsedTime();
        GoalScanner.Sighting found = null;

        while (canKeepRunning() && scanTimer.seconds() < seconds) {
            tick();
            found = shootGoal != null ? scanner.usable(shootGoal) : scanner.bestOpenTargetFor(alliance);
            if (found != null) {
                break;
            }
            addStepTelemetry(index, total, step, "scanning");
            telemetry.addData("Shot", "%d / %d", shot + 1, shots);
            telemetry.addData("Camera", scanner.cameraState());
            telemetry.addData("Looking for", "%s or %s", alliance.firstCluster, alliance.secondCluster);
            telemetry.addData("Candidates", scanner.candidateSummary(alliance));
            telemetry.addData("Scan time", "%.2f / %.2f s", scanTimer.seconds(), seconds);
            telemetry.update();
        }

        if (found == null && canKeepRunning()) {
            tick();
            found = shootGoal != null ? scanner.usable(shootGoal) : scanner.bestOpenTargetFor(alliance);
        }

        return found;
    }

    private GoalScanner.Sighting confirmFor(double seconds, String shortName, long sinceNanoTime,
                                            int index, int total, Step step,
                                            int shot, int shots) {
        ElapsedTime confirmTimer = new ElapsedTime();
        GoalScanner.Sighting found = null;

        while (canKeepRunning() && confirmTimer.seconds() < seconds) {
            tick();
            found = scanner.usableSightingSince(shortName, sinceNanoTime);
            if (found != null) {
                break;
            }
            addStepTelemetry(index, total, step, "confirming");
            telemetry.addData("Shot", "%d / %d", shot + 1, shots);
            telemetry.addData("Confirming", shortName);
            telemetry.addData("Candidates", scanner.candidateSummary(alliance));
            telemetry.addData("Confirm time", "%.2f / %.2f s", confirmTimer.seconds(), seconds);
            telemetry.update();
        }

        return found;
    }

    private boolean turnToReduceBearing(double bearingDegrees, String clusterName, int clusterPercent,
                                        ElapsedTime aimTimer, int index, int total,
                                        Step step, int shot, int shots) {
        double correctionDegrees = Range.clip(bearingDegrees,
                -AIM_BEARING_MAX_CORRECTION_DEGREES, AIM_BEARING_MAX_CORRECTION_DEGREES);

        Pose here = follower.pose();
        double goalHeading = here.heading() + Math.toRadians(correctionDegrees);
        Pose goalPose = here.withHeading(goalHeading);

        follower.hold(goalPose);

        double limit = Math.min(AIM_TURN_TIMEOUT_SECONDS, remainingAimSeconds(aimTimer));
        ElapsedTime turnTimer = new ElapsedTime();
        boolean settled = false;

        while (canKeepRunning() && turnTimer.seconds() < limit) {
            tick();
            double errorDegrees = Math.toDegrees(
                    normalizeRadians(goalHeading - follower.pose().heading()));
            if (Math.abs(errorDegrees) <= AIM_HEADING_SETTLE_TOLERANCE_DEGREES) {
                settled = true;
                break;
            }
            addStepTelemetry(index, total, step, "aiming turn");
            telemetry.addData("Shot", "%d / %d", shot + 1, shots);
            telemetry.addData("Turn pass", "%d / %d", aimTurnPasses, AIM_MAX_TURN_PASSES);
            telemetry.addData("Target cluster", "%s %d%%", clusterName, clusterPercent);
            telemetry.addData("Bearing before", "%.1f deg", bearingDegrees);
            telemetry.addData("Turning", "%.1f deg  (clipped from %.1f)", correctionDegrees, bearingDegrees);
            telemetry.addData("Heading error", "%.1f deg", errorDegrees);
            telemetry.addData("Turn time", "%.2f / %.2f s", turnTimer.seconds(), limit);
            telemetry.update();
        }

        aimTurnResult = settled
                ? String.format("turned %.1f deg in %.2f s", correctionDegrees, turnTimer.seconds())
                : String.format("TURN TIMEOUT after %.2f s (wanted %.1f deg) - shooting anyway",
                        turnTimer.seconds(), correctionDegrees);

        return settled;
    }

    private double normalizeRadians(double radians) {
        double value = radians;
        while (value > Math.PI) {
            value -= 2.0 * Math.PI;
        }
        while (value < -Math.PI) {
            value += 2.0 * Math.PI;
        }
        return value;
    }

    private void runIntake(int index, int total, Step step) {
        double seconds = Math.min(step.intakeSeconds, MAX_ACTION_SECONDS);
        boolean clamped = step.intakeSeconds > MAX_ACTION_SECONDS;

        intake.setPower(Constants.INTAKE_POWER);

        ElapsedTime intakeTimer = new ElapsedTime();
        while (canKeepRunning() && intakeTimer.seconds() < seconds) {
            tick();
            addStepTelemetry(index, total, step, "intaking");
            telemetry.addData("Intake time", "%.1f / %.1f s", intakeTimer.seconds(), seconds);
            if (clamped) {
                telemetry.addData("WARNING", "intake clamped from %.1f to %.1f s",
                        step.intakeSeconds, MAX_ACTION_SECONDS);
            }
            telemetry.update();
        }

        intake.setPower(0.0);
    }

    private void holdFor(double seconds) {
        double limit = Math.min(seconds, MAX_ACTION_SECONDS);
        ElapsedTime timer = new ElapsedTime();
        while (canKeepRunning() && timer.seconds() < limit) {
            tick();
        }
    }

    private void addAimTelemetry() {
        telemetry.addData("Pose aim", poseAimNote);
        telemetry.addData("Aim source", aimUsedFallback ? "NO VISION TURN" : "VISION");
        telemetry.addData("Aim cluster", "%s  %d%%", aimCluster, aimPercent);
        telemetry.addData("Aim range", "%.1f in", aimRange);
        telemetry.addData("Aim bearing", "before %.1f deg -> after %.1f deg",
                aimBearingBefore, aimBearingAfter);
        telemetry.addData("Sighting age", aimSightingAgeMs < 0 ? "n/a" : aimSightingAgeMs + " ms");
        telemetry.addData("Aim turn passes", "%d / %d", aimTurnPasses, AIM_MAX_TURN_PASSES);
        telemetry.addData("Aim turn", aimTurnResult);
        telemetry.addData("Aim note", aimNote);
    }

    private void addStepTelemetry(int index, int total, Step step, String phase) {
        Pose pose = follower.pose();
        telemetry.addData("Step", "%d / %d  %s", index + 1, total, step.name);
        telemetry.addData("Phase", phase);
        telemetry.addData("Plan", step.actions());
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Target", formatPose(step.to));
        telemetry.addData("Current", formatPose(pose));
        telemetry.addData("Distance to target", "%.1f in", step.to.distance(pose));
        telemetry.addData("Runtime", "%.1f / %.1f s", runtime.seconds(), AUTONOMOUS_BUDGET_SECONDS);
    }

    private static String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    private void stopAll() {
        intake.setPower(0.0);
        launcher.shutdown();
        follower.stop();
        follower.update();
    }

    private Step path(int number) {
        return new Step(AutoLibrary.number(number), alliance);
    }

    private static final class Step {
        final AutoLibrary.Leg leg;
        final String name;
        final Pose to;
        int shots = 0;
        double intakeSeconds = 0.0;
        boolean intakeWhileDriving = false;
        double pauseSeconds = 0.0;
        double timeoutSeconds;

        Step(AutoLibrary.Leg leg, GoalScanner.Alliance alliance) {
            this.leg = leg;
            this.name = leg.describe();
            this.to = leg.end(alliance);
            this.timeoutSeconds = leg.timeoutSeconds();
        }

        Step shoot(int count) {
            shots = count;
            return this;
        }

        Step intake(double seconds) {
            intakeSeconds = seconds;
            return this;
        }

        Step intakeWhileDriving() {
            intakeWhileDriving = true;
            return this;
        }

        Step pause(double seconds) {
            pauseSeconds = seconds;
            return this;
        }

        Step timeout(double seconds) {
            timeoutSeconds = seconds;
            return this;
        }

        String actions() {
            StringBuilder text = new StringBuilder();
            if (intakeWhileDriving) text.append("  intake-drive");
            if (shots > 0) text.append(String.format("  shoot %d", shots));
            if (intakeSeconds > 0) text.append(String.format("  intake %.1fs", intakeSeconds));
            if (pauseSeconds > 0) text.append(String.format("  pause %.1fs", pauseSeconds));
            return text.length() == 0 ? "drive" : text.toString().trim();
        }
    }
}
