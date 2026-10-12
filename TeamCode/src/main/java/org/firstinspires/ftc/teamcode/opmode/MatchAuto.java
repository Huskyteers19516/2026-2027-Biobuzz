package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Match Auto", group = "Auto")
public class MatchAuto extends LinearOpMode {

    private static final double OUTTAKE_VELOCITY = RobotTeleOp.OUTTAKE_VELOCITY;
    private static final double OUTTAKE_TOLERANCE = RobotTeleOp.OUTTAKE_TOLERANCE;
    private static final double SPINUP_TIMEOUT_SECONDS = RobotTeleOp.SPINUP_TIMEOUT_SECONDS;
    private static final double FEED_POWER = RobotTeleOp.FEED_POWER;
    private static final double FEED_SECONDS = RobotTeleOp.FEED_SECONDS;
    private static final boolean KEEP_OUTTAKE_SPINNING = RobotTeleOp.KEEP_OUTTAKE_SPINNING;
    private static final double INTAKE_POWER = RobotTeleOp.INTAKE_POWER;

    private static final double SHOT_GAP_SECONDS = 0.4;
    private static final double MAX_ACTION_SECONDS = 8.0;
    private static final double SETTLE_SECONDS_BEFORE_SHOOT = 0.2;

    private static final double AUTONOMOUS_BUDGET_SECONDS = FieldPoints.AUTONOMOUS_BUDGET_SECONDS;
    private static final double MIN_PATH_LENGTH_INCHES = FieldPoints.MIN_PATH_LENGTH_INCHES;

    private static final double AIM_DEFAULT_POSITION = RobotTeleOp.AIM_START_POSITION;

    private static final GoalScanner.Alliance ALLIANCE = FieldPoints.ALLIANCE;
    private static final boolean VISION_AIMING_ENABLED = FieldPoints.VISION_AIMING_ENABLED;

    private static final double RANGE_NEAR = FieldPoints.RANGE_NEAR_INCHES;
    private static final double RANGE_FAR = FieldPoints.RANGE_FAR_INCHES;
    private static final double AIM_AT_NEAR = FieldPoints.AIM_AT_NEAR;
    private static final double AIM_AT_FAR = FieldPoints.AIM_AT_FAR;

    private static final double AIM_BEARING_TOLERANCE_DEGREES = FieldPoints.AIM_BEARING_TOLERANCE_DEGREES;
    private static final double AIM_BEARING_MAX_CORRECTION_DEGREES = FieldPoints.AIM_BEARING_MAX_CORRECTION_DEGREES;
    private static final double AIM_HEADING_SETTLE_TOLERANCE_DEGREES = FieldPoints.AIM_HEADING_SETTLE_TOLERANCE_DEGREES;

    private static final double SCAN_TIMEOUT_SECONDS = FieldPoints.SCAN_TIMEOUT_SECONDS;
    private static final double AIM_TURN_TIMEOUT_SECONDS = FieldPoints.AIM_TURN_TIMEOUT_SECONDS;
    private static final double PER_SHOT_TIMEOUT_SECONDS = FieldPoints.PER_SHOT_TIMEOUT_SECONDS;

    private static final double CONFIRM_SCAN_SECONDS = 0.25;
    private static final int AIM_MAX_TURN_PASSES = 2;
    private static final double AIM_MIN_PASS_SECONDS = 0.40;

    private Follower follower;
    private GoalScanner scanner;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo launcherServo;
    private DcMotor LeftFront;
    private DcMotor Leftback;
    private DcMotor riightfront;
    private DcMotor rightBack;

    private final ElapsedTime runtime = new ElapsedTime();

    private String lastPathResult = "none";
    private String lastShotResult = "none";
    private String stopReason = "mission complete";

    private String aimCluster = "none";
    private int aimPercent = 0;
    private double aimRange = 0.0;
    private double aimBearingBefore = 0.0;
    private double aimBearingAfter = 0.0;
    private long aimSightingAgeMs = -1L;
    private double aimValueUsed = AIM_DEFAULT_POSITION;
    private boolean aimUsedFallback = true;
    private String aimNote = "not aimed yet";
    private String aimTurnResult = "no turn yet";
    private int aimTurnPasses = 0;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        scanner = new GoalScanner(hardwareMap);


        intake = hardwareMap.get(DcMotor.class, RobotTeleOp.INTAKE_NAME);
        transfer = hardwareMap.get(DcMotor.class, RobotTeleOp.TRANSFER_NAME);
        outtake = hardwareMap.get(DcMotorEx.class, RobotTeleOp.OUTTAKE_NAME);
        launcherServo = hardwareMap.get(Servo.class, RobotTeleOp.LAUNCHER_SERVO_NAME);

        intake.setDirection(RobotTeleOp.INTAKE_DIRECTION);
        transfer.setDirection(RobotTeleOp.TRANSFER_DIRECTION);
        outtake.setDirection(RobotTeleOp.OUTTAKE_DIRECTION);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower.setPose(FieldPoints.START_POSE);

        List<FieldPoints.Step> mission = FieldPoints.mission();

        try {
            while (!isStarted() && !isStopRequested()) {
                follower.localizer.update();
                scanner.update();
                telemetry.addLine("Match Auto");
                telemetry.addData("Steps", mission.size());
                telemetry.addData("Budget", "%.1f s", AUTONOMOUS_BUDGET_SECONDS);
                telemetry.addData("Start pose", formatPose(FieldPoints.START_POSE));
                telemetry.addData("Measured pose", formatPose(follower.pose()));
                telemetry.addData("Localizer", Constants.PINPOINT_NAME);
                telemetry.addData("Outtake ticks", outtake.getCurrentPosition());

                telemetry.addLine();
                telemetry.addData("ALLIANCE", ALLIANCE);
                telemetry.addData("Vision aiming", VISION_AIMING_ENABLED ? "ENABLED" : "DISABLED (static aim only)");
                telemetry.addData("Goal clusters", "%s / %s", ALLIANCE.firstCluster, ALLIANCE.secondCluster);
                telemetry.addData("Camera", scanner.cameraState());
                if (!scanner.isAvailable()) {
                    telemetry.addLine("!! CAMERA NOT AVAILABLE - AUTO WILL USE THE STATIC AIM");
                    telemetry.addData("Camera fault", scanner.fault());
                } else {
                    telemetry.addData("Candidates", scanner.candidateSummary(ALLIANCE));
                    telemetry.addLine("Wait for Camera = STREAMING before pressing START.");
                }

                if (!FieldPoints.POINTS_CONFIRMED) {
                    telemetry.addLine("!! FIELD POINTS NOT CONFIRMED - AUTO WILL NOT MOVE");
                    telemetry.addLine("Measure the real coordinates in FieldPoints.java,");
                    telemetry.addLine("then set POINTS_CONFIRMED = true.");
                }

                telemetry.addLine("Nothing is commanded until START");
                telemetry.update();
            }

            if (!FieldPoints.POINTS_CONFIRMED) {
                stopAll();

                while (opModeIsActive()) {
                    telemetry.addLine("!! FIELD POINTS NOT CONFIRMED - REFUSING TO DRIVE");
                    telemetry.addLine("The poses in FieldPoints.java are placeholders.");
                    telemetry.addLine("Measure them, then set POINTS_CONFIRMED = true.");
                    telemetry.update();
                }

                return;
            }

            if (isStopRequested()) {
                return;
            }

            try {
                setAim(AIM_DEFAULT_POSITION);

                follower.setPose(FieldPoints.START_POSE);
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
                }
            } finally {
                stopAll();
            }

            telemetry.addData("Status", "Finished");
            telemetry.addData("Reason", stopReason);
            telemetry.addData("Runtime", "%.1f s", runtime.seconds());
            telemetry.addData("Final pose", formatPose(follower.pose()));
            telemetry.addData("Last path", lastPathResult);
            telemetry.addData("Last shot", lastShotResult);
            telemetry.addData("Last aim", aimNote);
            telemetry.addData("Vision frames", "%d updates, %d with a cluster",
                    scanner.updateCount(), scanner.clusterFrameCount());
            telemetry.update();
        } finally {
            scanner.close();
        }
    }

    private void tick() {
        follower.update();
        scanner.update();
    }

    private boolean budgetExhausted() {
        return runtime.seconds() >= AUTONOMOUS_BUDGET_SECONDS;
    }

    private boolean canKeepRunning() {
        return opModeIsActive() && !budgetExhausted();
    }

    private void setAim(double position) {
        launcherServo.setPosition(Range.clip(position,
                RobotTeleOp.AIM_MIN_POSITION, RobotTeleOp.AIM_MAX_POSITION));
    }

    private void runStep(int index, int total, FieldPoints.Step step) {
        Pose startPose = follower.pose();
        double legLength = startPose.distance(step.target);
        boolean timedOut = false;
        ElapsedTime pathTimer = new ElapsedTime();

        if (legLength < MIN_PATH_LENGTH_INCHES) {
            follower.hold(step.target);
            tick();
            lastPathResult = String.format("step %d already on target (%.2f in)", index + 1, legLength);
        } else {
            Path path = Paths.line(startPose, step.target).linear(startPose, step.target);
            follower.follow(path);

            if (step.intakeWhileDriving) {
                intake.setPower(INTAKE_POWER);
            }

            while (canKeepRunning() && (follower.following() || follower.isBusy())) {
                if (pathTimer.seconds() >= step.pathTimeoutSeconds) {
                    timedOut = true;
                    break;
                }
                tick();
                addStepTelemetry(index, total, step, "following");
                telemetry.addData("Remaining on path", "%.1f in", follower.remainingDistance());
                telemetry.addData("Path time", "%.1f / %.1f s", pathTimer.seconds(), step.pathTimeoutSeconds);
                telemetry.update();
            }

            intake.setPower(0.0);

            lastPathResult = timedOut
                    ? String.format("step %d TIMEOUT after %.1f s", index + 1, pathTimer.seconds())
                    : String.format("step %d arrived in %.1f s", index + 1, pathTimer.seconds());
        }

        if (!opModeIsActive()) {
            return;
        }

        follower.hold(timedOut ? follower.pose() : step.target);
        tick();

        addStepTelemetry(index, total, step, timedOut ? "TIMED OUT" : "arrived");
        if (timedOut) {
            telemetry.addLine("WARNING path timed out - holding the measured pose, not the target");
        }
        telemetry.update();

        if (budgetExhausted()) {
            return;
        }

        switch (step.action) {
            case SHOOT:
                holdFor(SETTLE_SECONDS_BEFORE_SHOOT);
                runShoot(index, total, step);
                break;
            case INTAKE:
                runIntake(index, total, step);
                break;
            case NONE:
            default:
                break;
        }
    }

    private void runShoot(int index, int total, FieldPoints.Step step) {
        int shots = step.shots;

        if (shots <= 0) {
            lastShotResult = String.format("step %d requested 0 shots - skipped", index + 1);
            return;
        }

        for (int shot = 0; shot < shots; shot++) {
            if (!canKeepRunning()) {
                break;
            }

            outtake.setVelocity(OUTTAKE_VELOCITY);

            scanAimAndCommit(index, total, step, shot, shots);

            ElapsedTime spinUpTimer = new ElapsedTime();
            boolean atSpeed = false;

            while (canKeepRunning() && spinUpTimer.seconds() < SPINUP_TIMEOUT_SECONDS) {
                tick();
                double velocity = outtake.getVelocity();
                if (Math.abs(velocity - OUTTAKE_VELOCITY) <= OUTTAKE_TOLERANCE) {
                    atSpeed = true;
                    break;
                }
                addStepTelemetry(index, total, step, "spinning up");
                telemetry.addData("Shot", "%d / %d", shot + 1, shots);
                addAimTelemetry();
                telemetry.addData("Outtake vel", "measured %.0f / target %.0f  tol %.0f",
                        velocity, OUTTAKE_VELOCITY, OUTTAKE_TOLERANCE);
                telemetry.addData("Spin-up time", "%.2f / %.2f s",
                        spinUpTimer.seconds(), SPINUP_TIMEOUT_SECONDS);
                telemetry.update();
            }

            lastShotResult = atSpeed
                    ? String.format("shot %d at speed after %.2f s", shot + 1, spinUpTimer.seconds())
                    : String.format("shot %d SPIN-UP TIMEOUT at %.0f tps", shot + 1, outtake.getVelocity());

            if (!opModeIsActive()) {
                break;
            }

            if (!atSpeed && budgetExhausted()) {
                lastShotResult = String.format("shot %d abandoned - budget %.1f s reached",
                        shot + 1, AUTONOMOUS_BUDGET_SECONDS);
                break;
            }

            transfer.setPower(FEED_POWER);

            ElapsedTime feedTimer = new ElapsedTime();
            while (opModeIsActive() && feedTimer.seconds() < FEED_SECONDS) {
                tick();
                addStepTelemetry(index, total, step, "feeding");
                telemetry.addData("Shot", "%d / %d", shot + 1, shots);
                addAimTelemetry();
                telemetry.addData("Feed time", "%.2f / %.2f s", feedTimer.seconds(), FEED_SECONDS);
                telemetry.addData("Outtake vel", "%.0f", outtake.getVelocity());
                telemetry.update();
            }

            transfer.setPower(0.0);

            if (shot < shots - 1 && canKeepRunning()) {
                holdFor(SHOT_GAP_SECONDS);
            }
        }

        transfer.setPower(0.0);

        if (!KEEP_OUTTAKE_SPINNING) {
            outtake.setVelocity(0.0);
            outtake.setPower(0.0);
        }
    }

    private void scanAimAndCommit(int index, int total, FieldPoints.Step step, int shot, int shots) {
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
            commitFallback(step, "vision aiming disabled in FieldPoints");
            return;
        }
        if (!scanner.isAvailable()) {
            commitFallback(step, "no camera: " + scanner.fault());
            return;
        }

        double scanLimit = Math.min(SCAN_TIMEOUT_SECONDS, remainingAimSeconds(aimTimer));
        GoalScanner.Sighting target = scanFor(scanLimit, index, total, step, shot, shots);

        if (target == null) {
            commitFallback(step, String.format("no usable %s / %s cluster (>= %d%%, age <= %d ms) in %.2f s",
                    ALLIANCE.firstCluster, ALLIANCE.secondCluster,
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
        aimValueUsed = aimForRange(aimRange);
        aimNote = String.format("%s %d%% range %.1f in -> aim %.3f (bearing %.1f -> %.1f deg)",
                aimCluster, aimPercent, aimRange, aimValueUsed, aimBearingBefore, aimBearingAfter);
        setAim(aimValueUsed);
    }

    private void commitFallback(FieldPoints.Step step, String reason) {
        aimUsedFallback = true;
        aimValueUsed = step.aim;
        aimNote = "FALLBACK static aim " + String.format("%.3f", step.aim) + " - " + reason;
        setAim(aimValueUsed);
    }

    private double remainingAimSeconds(ElapsedTime aimTimer) {
        return Math.max(0.0, PER_SHOT_TIMEOUT_SECONDS - aimTimer.seconds());
    }

    private GoalScanner.Sighting scanFor(double seconds, int index, int total, FieldPoints.Step step,
                                         int shot, int shots) {
        ElapsedTime scanTimer = new ElapsedTime();
        GoalScanner.Sighting found = null;

        while (canKeepRunning() && scanTimer.seconds() < seconds) {
            tick();
            found = scanner.bestTargetFor(ALLIANCE);
            if (found != null) {
                break;
            }
            addStepTelemetry(index, total, step, "scanning");
            telemetry.addData("Shot", "%d / %d", shot + 1, shots);
            telemetry.addData("Camera", scanner.cameraState());
            telemetry.addData("Looking for", "%s or %s", ALLIANCE.firstCluster, ALLIANCE.secondCluster);
            telemetry.addData("Candidates", scanner.candidateSummary(ALLIANCE));
            telemetry.addData("Scan time", "%.2f / %.2f s", scanTimer.seconds(), seconds);
            telemetry.update();
        }

        if (found == null && canKeepRunning()) {
            tick();
            found = scanner.bestTargetFor(ALLIANCE);
        }

        return found;
    }

    private GoalScanner.Sighting confirmFor(double seconds, String shortName, long sinceNanoTime,
                                            int index, int total, FieldPoints.Step step,
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
            telemetry.addData("Candidates", scanner.candidateSummary(ALLIANCE));
            telemetry.addData("Confirm time", "%.2f / %.2f s", confirmTimer.seconds(), seconds);
            telemetry.update();
        }

        return found;
    }

    private boolean turnToReduceBearing(double bearingDegrees, String clusterName, int clusterPercent,
                                        ElapsedTime aimTimer, int index, int total,
                                        FieldPoints.Step step, int shot, int shots) {
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

    private double aimForRange(double range) {
        double span = RANGE_FAR - RANGE_NEAR;
        double aim;
        if (Math.abs(span) < 1e-6) {
            aim = AIM_AT_NEAR;
        } else {
            double t = Range.clip((range - RANGE_NEAR) / span, 0.0, 1.0);
            aim = AIM_AT_NEAR + t * (AIM_AT_FAR - AIM_AT_NEAR);
        }
        return Range.clip(aim, RobotTeleOp.AIM_MIN_POSITION, RobotTeleOp.AIM_MAX_POSITION);
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

    private void runIntake(int index, int total, FieldPoints.Step step) {
        double seconds = Math.min(step.seconds, MAX_ACTION_SECONDS);
        boolean clamped = step.seconds > MAX_ACTION_SECONDS;

        intake.setPower(INTAKE_POWER);

        ElapsedTime intakeTimer = new ElapsedTime();
        while (canKeepRunning() && intakeTimer.seconds() < seconds) {
            tick();
            addStepTelemetry(index, total, step, "intaking");
            telemetry.addData("Intake time", "%.1f / %.1f s", intakeTimer.seconds(), seconds);
            if (clamped) {
                telemetry.addData("WARNING", "intake clamped from %.1f to %.1f s",
                        step.seconds, MAX_ACTION_SECONDS);
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
        telemetry.addData("Aim source", aimUsedFallback ? "STATIC FALLBACK" : "VISION");
        telemetry.addData("Aim value", "%.3f", aimValueUsed);
        telemetry.addData("Aim cluster", "%s  %d%%", aimCluster, aimPercent);
        telemetry.addData("Aim range", "%.1f in", aimRange);
        telemetry.addData("Aim bearing", "before %.1f deg -> after %.1f deg",
                aimBearingBefore, aimBearingAfter);
        telemetry.addData("Sighting age", aimSightingAgeMs < 0 ? "n/a" : aimSightingAgeMs + " ms");
        telemetry.addData("Aim turn passes", "%d / %d", aimTurnPasses, AIM_MAX_TURN_PASSES);
        telemetry.addData("Aim turn", aimTurnResult);
        telemetry.addData("Aim note", aimNote);
    }

    private void addStepTelemetry(int index, int total, FieldPoints.Step step, String phase) {
        Pose pose = follower.pose();
        telemetry.addData("Step", "%d / %d  %s", index + 1, total, step.name);
        telemetry.addData("Phase", phase);
        telemetry.addData("Action", step.action);
        telemetry.addData("Alliance", ALLIANCE);
        telemetry.addData("Target", formatPose(step.target));
        telemetry.addData("Current", formatPose(pose));
        telemetry.addData("Distance to target", "%.1f in", step.target.distance(pose));
        telemetry.addData("Runtime", "%.1f / %.1f s", runtime.seconds(), AUTONOMOUS_BUDGET_SECONDS);
    }

    private String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    private void stopAll() {
        intake.setPower(0.0);
        transfer.setPower(0.0);
        outtake.setVelocity(0.0);
        outtake.setPower(0.0);
        follower.stop();
        follower.update();
    }
}
