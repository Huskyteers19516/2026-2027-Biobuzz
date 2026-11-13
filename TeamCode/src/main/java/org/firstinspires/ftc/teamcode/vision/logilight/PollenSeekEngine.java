package org.firstinspires.ftc.teamcode.vision.logilight;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PoseHistory;
import org.firstinspires.ftc.teamcode.vision.logilight.math.TargetGeometry;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class PollenSeekEngine {

    public enum State { IDLE, SCANNING, TURNING, FOLLOWING, FINAL_APPROACH, INTAKING, DONE, ABORTED }

    public enum LocalizerHealth { READY, TRANSIENT, FAULT }

    public interface Clock {
        long nanoTime();
    }

    public interface Intake {
        void run(double power);

        void stop();
    }

    public static final class Detection {
        public final double forward;
        public final double left;
        public final double distance;
        public final double bearingLeftDegrees;

        public Detection(double forward, double left, double distance, double bearingLeftDegrees) {
            this.forward = forward;
            this.left = left;
            this.distance = distance;
            this.bearingLeftDegrees = bearingLeftDegrees;
        }
    }

    public static final class FieldBall {
        public final double x;
        public final double y;
        public final double distanceFromRobot;
        public final double bearingLeftDegrees;
        public final int framesSeen;
        public final double ageMs;
        public final boolean locked;

        FieldBall(double x, double y, double distanceFromRobot, double bearingLeftDegrees, int framesSeen, double ageMs,
                  boolean locked) {
            this.x = x;
            this.y = y;
            this.distanceFromRobot = distanceFromRobot;
            this.bearingLeftDegrees = bearingLeftDegrees;
            this.framesSeen = framesSeen;
            this.ageMs = ageMs;
            this.locked = locked;
        }

        public String describe() {
            return String.format(Locale.US, "%s at (%.1f, %.1f)  %.1f in @ %+.0f deg  %d frames  seen %.0f ms ago",
                    locked ? "LOCKED" : "candidate", x, y, distanceFromRobot, bearingLeftDegrees, framesSeen, ageMs);
        }
    }

    public static final Clock SYSTEM_CLOCK = new Clock() {
        @Override
        public long nanoTime() {
            return System.nanoTime();
        }
    };

    private static final double NANOS_PER_SECOND = 1e9;
    private static final double NANOS_PER_MILLI = 1e6;
    private static final long CONFIG_REFRESH_NANOS = 250_000_000L;
    private static final int MAX_TURN_PASSES = 3;
    private static final double TURN_WRONG_WAY_RADIANS = Math.toRadians(20.0);
    private static final double MIN_SIDEWAYS_ABORT_INCHES = 3.0;
    private static final double TURN_COAST_SECONDS = 0.3;
    private static final double TURN_COAST_MAX_INCHES = 4.0;
    private static final int LOCALIZER_TRANSIENT_LOOPS = 3;
    private static final double FROZEN_MIN_WHEEL_POWER = 0.2;
    private static final long FROZEN_WINDOW_NANOS = 300_000_000L;
    private static final long FROZEN_GRACE_NANOS = 500_000_000L;
    private static final double FROZEN_MIN_MOVE_INCHES = 0.3;
    private static final String RESTART_ADVICE = "restart the OpMode and redo the push test";
    private static final int REZERO_GRACE_SAMPLES = 5;

    private final Follower follower;
    private final PollenSeekConfig liveConfig;
    private final Clock clock;
    private final PoseHistory history = new PoseHistory(PoseHistory.DEFAULT_CAPACITY);
    private final PollenTracker tracker = new PollenTracker();
    private final PollenSeekGuards.ProgressMonitor progress = new PollenSeekGuards.ProgressMonitor();
    private final PollenSeekGuards.CrossCheck crossCheck = new PollenSeekGuards.CrossCheck();
    private final PollenSeekGuards.PoseSanity poseSanity = new PollenSeekGuards.PoseSanity();
    private final boolean holdEndDisabled;

    private PollenSeekConfig cfg;
    private PollenRoutePlanner planner;
    private long configNanos;
    private Intake intake;
    private boolean intakeRunning;
    private double defaultIntakePower = 1.0;
    private boolean intakeOnArrive;
    private boolean permitted;
    private String permitReason = "push test not run";
    private String cameraRefusal;

    private State state = State.IDLE;
    private String status = "idle";
    private String abortReason = "";
    private String lastEvent = "";
    private String planNote = "no plan yet";
    private String speedNote = "";

    private PollenRoutePlanner.Plan plan;
    private double planBallX = Double.NaN;
    private double planBallY = Double.NaN;
    private long planNanos;
    private long attemptNanos;
    private long stateNanos;
    private long finalLegNanos;
    private boolean onFinalLeg;
    private int legHint;
    private double attemptStartX;
    private double attemptStartY;
    private double travelLimit;
    private double turnHeading;
    private double turnStartAbs;
    private double turnStartX;
    private double turnStartY;
    private double turnDriftLimit;
    private boolean driven;
    private long drivenSinceNanos;
    private int turnPasses;
    private boolean arriveAfterTurn;
    private boolean cameFromTurn;
    private double creepX;
    private double creepY;
    private double creepHeading;
    private boolean ownsMotion;
    private String poseFault;
    private String localizerLatch;
    private String poseLatch;
    private boolean localizerMonitored;
    private LocalizerHealth localizerHealth = LocalizerHealth.READY;
    private String localizerStatusName = "not monitored";
    private int localizerTransientLoops;
    private int rezeroGraceSamples;
    private int attempts;
    private int lostCount;
    private int abortCount;
    private int replans;
    private long framesUsed;
    private long framesWithoutPose;

    public PollenSeekEngine(Follower follower, PollenSeekConfig config, Clock clock) {
        this.follower = follower;
        this.liveConfig = config == null ? new PollenSeekConfig() : config;
        this.clock = clock == null ? SYSTEM_CLOCK : clock;
        this.cfg = liveConfig.sanitized();
        this.planner = new PollenRoutePlanner(cfg);
        this.configNanos = this.clock.nanoTime();
        this.intakeOnArrive = cfg.runIntakeOnArrive;
        boolean disabled = false;
        if (follower != null) {
            try {
                follower.holdEnd.set(Boolean.FALSE);
                disabled = !Boolean.TRUE.equals(follower.holdEnd.get());
            } catch (Throwable ignored) {
                disabled = false;
            }
        }
        this.holdEndDisabled = disabled;
        applyCap();
    }

    public void setIntake(Intake intake) {
        stopIntake();
        this.intake = intake;
    }

    public boolean hasIntake() {
        return intake != null;
    }

    public void setIntakeOnArrive(boolean enabled) {
        intakeOnArrive = enabled;
    }

    public boolean intakeOnArrive() {
        return intakeOnArrive;
    }

    public void setDefaultIntakePower(double power) {
        if (!Double.isNaN(power) && !Double.isInfinite(power)) {
            defaultIntakePower = Math.max(-1.0, Math.min(1.0, power));
        }
    }

    public void setPermit(boolean permitted, String reason) {
        this.permitted = permitted;
        this.permitReason = reason == null || reason.isEmpty() ? "driving not permitted" : reason;
    }

    public void setCameraRefusal(String reason) {
        cameraRefusal = reason == null || reason.isEmpty() ? null : reason;
    }

    public PollenSeekConfig config() {
        return cfg;
    }

    public void recordPose() {
        long now = clock.nanoTime();
        if (follower == null) {
            history.add(now, 0.0, 0.0, 0.0);
            return;
        }
        Pose pose = safePose();
        if (pose == null) {
            latchPose("!! pose unavailable - Pinpoint fault");
            notePoseFault("!! pose unavailable - Pinpoint fault");
            return;
        }
        if (rezeroGraceSamples > 0) {
            rezeroGraceSamples--;
            poseSanity.reset();
        }
        String fault = poseSanity.sample(pose.x(), pose.y(), pose.heading(), cfg);
        if (fault != null) {
            latchPose(fault);
            notePoseFault(fault);
        }
        history.add(now, pose.x(), pose.y(), pose.heading());
    }

    public void resetPoseHistory() {
        history.clear();
        tracker.clear();
        poseSanity.reset();
        poseFault = null;
        poseLatch = null;
        rezeroGraceSamples = REZERO_GRACE_SAMPLES;
    }

    public void reportLocalizerStatus(LocalizerHealth health, String name) {
        LocalizerHealth h = health == null ? LocalizerHealth.TRANSIENT : health;
        localizerMonitored = true;
        localizerHealth = h;
        localizerStatusName = name == null || name.isEmpty() ? h.name() : name;
        if (h == LocalizerHealth.READY) {
            localizerTransientLoops = 0;
            return;
        }
        if (h == LocalizerHealth.FAULT) {
            String fault = "!! Pinpoint status " + localizerStatusName + " - odometry not trusted, check the pod cables";
            latchLocalizer(fault);
            notePoseFault(fault);
            return;
        }
        localizerTransientLoops++;
        if (localizerTransientLoops >= LOCALIZER_TRANSIENT_LOOPS && isMotionState()) {
            String fault = String.format(Locale.US, "!! Pinpoint status %s for %d loops - odometry frozen? stopped",
                    localizerStatusName, localizerTransientLoops);
            latchLocalizer(fault);
            notePoseFault(fault);
        }
    }

    public void reportLocalizerFault(String reason) {
        String fault = reason == null || reason.isEmpty() ? "!! localizer fault" : reason;
        latchLocalizer(fault);
        notePoseFault(fault);
    }

    public boolean localizerMonitored() {
        return localizerMonitored;
    }

    public LocalizerHealth localizerHealth() {
        return localizerHealth;
    }

    public String localizerStatusName() {
        return localizerStatusName;
    }

    public boolean localizerSuspect() {
        return localizerLatch != null || poseLatch != null;
    }

    public String localizerSuspectReason() {
        if (localizerLatch != null) {
            return localizerLatch;
        }
        return poseLatch == null ? "" : poseLatch;
    }

    private void latchLocalizer(String reason) {
        if (localizerLatch == null) {
            localizerLatch = reason == null || reason.isEmpty() ? "localizer fault" : reason;
        }
    }

    private void latchPose(String reason) {
        if (poseLatch == null) {
            poseLatch = reason == null || reason.isEmpty() ? "pose fault" : reason;
        }
    }

    private void abortLocalizer(String reason) {
        latchLocalizer(reason);
        abort(reason);
    }

    public void observeFrame(long captureNanos, List<Detection> detections) {
        maybeRefreshConfig();
        PlanarPose at = history.poseAt(captureNanos, (long) (cfg.poseLookupSlackMs * NANOS_PER_MILLI));
        if (at == null || !at.isFinite()) {
            framesWithoutPose++;
            return;
        }
        List<PollenTracker.Ball> balls = new ArrayList<PollenTracker.Ball>();
        if (detections != null) {
            for (Detection d : detections) {
                if (d == null || !finite(d.forward) || !finite(d.left)) {
                    continue;
                }
                Vec3 field = TargetGeometry.robotPointToField(at, new Vec3(d.forward, d.left, 0.0));
                if (field == null || !field.isFinite()) {
                    continue;
                }
                balls.add(new PollenTracker.Ball(field.x, field.y, d.forward, d.left, d.distance, d.bearingLeftDegrees,
                        captureNanos));
            }
        }
        if (!tracker.addFrame(captureNanos, balls, cfg)) {
            return;
        }
        framesUsed++;
        if (crossCheck.isArmed() && isMotionState()) {
            PollenTracker.Ball near = tracker.newestNearestTo(crossCheck.anchorX(), crossCheck.anchorY(), crossCheck.gate(cfg));
            if (near != null && crossCheck.check(at, near.forward, near.left)) {
                abortLocalizer(crossCheck.tripMessage());
            }
        }
    }

    public State step(boolean driveAllowed) {
        long now = clock.nanoTime();
        if (!driveAllowed) {
            release();
            return state;
        }
        switch (state) {
            case IDLE:
                beginAttempt(now);
                break;
            case SCANNING:
                scanStep(now);
                break;
            case TURNING:
                turnStep(now);
                break;
            case FOLLOWING:
            case FINAL_APPROACH:
                followStep(now);
                break;
            case INTAKING:
                intakeStep(now);
                break;
            case DONE:
            case ABORTED:
            default:
                break;
        }
        return state;
    }

    public void cancel() {
        boolean wasMoving = isMotionState() || ownsMotion;
        haltMotion();
        clearAttempt();
        if (wasMoving || state == State.SCANNING) {
            lastEvent = "cancelled - stopped";
        }
        state = State.IDLE;
        status = "idle (cancelled)";
    }

    public void shutdown() {
        haltMotion();
        clearAttempt();
        state = State.IDLE;
        status = "stopped";
    }

    private void release() {
        if (state == State.IDLE) {
            return;
        }
        boolean wasMoving = isMotionState() || ownsMotion;
        if (wasMoving) {
            haltMotion();
            lastEvent = "released while " + state + " - stopped";
        } else {
            stopIntake();
        }
        clearAttempt();
        state = State.IDLE;
        status = "idle";
    }

    private void clearAttempt() {
        tracker.unlock();
        crossCheck.disarm();
        progress.stop();
        plan = null;
        arriveAfterTurn = false;
        cameFromTurn = false;
        turnPasses = 0;
    }

    private void beginAttempt(long now) {
        refreshConfig(now);
        haltMotion();
        clearAttempt();
        poseFault = null;
        poseSanity.reset();
        String refusal = refusal();
        if (refusal != null) {
            enterAborted(now, "REFUSED - " + refusal);
            return;
        }
        state = State.SCANNING;
        stateNanos = now;
        status = "SCANNING for yellow POLLEN";
        planNote = "waiting for a confirmed POLLEN";
    }

    public String refusalReason() {
        return refusal();
    }

    private String refusal() {
        if (follower == null) {
            return "no Pinpoint / Follower - driving disabled (detections still shown)";
        }
        if (!(follower.drivetrain instanceof CappedDrivetrain)) {
            return "drivetrain has no power cap (build the Follower with CappedDrivetrain)";
        }
        if (!(((CappedDrivetrain) follower.drivetrain).cap() > 0.0)) {
            return "power cap is 0";
        }
        if (!holdEndDisabled) {
            return "could not turn off the Follower end hold";
        }
        if (cameraRefusal != null) {
            return cameraRefusal;
        }
        if (!permitted) {
            return permitReason;
        }
        if (localizerSuspect()) {
            return "localizer fault earlier this run - " + RESTART_ADVICE + " (" + localizerSuspectReason() + ")";
        }
        if (localizerMonitored && (localizerHealth == LocalizerHealth.FAULT
                || localizerTransientLoops >= LOCALIZER_TRANSIENT_LOOPS)) {
            return "Pinpoint status " + localizerStatusName + " - odometry not trusted";
        }
        Pose here = safePose();
        if (here == null || !finite(here.x()) || !finite(here.y()) || !finite(here.heading())) {
            return "pose unavailable";
        }
        if (PollenSeekGuards.PoseSanity.outsideField(here.x(), here.y())) {
            return String.format(Locale.US, "pose (%.1f, %.1f) is outside the field - re-zero at the start pose",
                    here.x(), here.y());
        }
        return null;
    }

    private void scanStep(long now) {
        Pose here = safePose();
        if (here == null) {
            status = "SCANNING  !! no pose";
            return;
        }
        List<PollenTracker.Cluster> clusters = tracker.confirmedClusters(cfg, now, here.x(), here.y());
        if (clusters.isEmpty()) {
            status = "SCANNING - no confirmed POLLEN yet";
            return;
        }
        PollenTracker.Cluster chosen = null;
        PollenRoutePlanner.Plan chosenPlan = null;
        String firstReason = null;
        for (PollenTracker.Cluster cluster : clusters) {
            PollenRoutePlanner.Plan candidate = planner.plan(here.x(), here.y(), here.heading(), cluster.x, cluster.y);
            if (candidate.reachable()) {
                chosen = cluster;
                chosenPlan = candidate;
                break;
            }
            if (firstReason == null) {
                firstReason = candidate.reason;
            }
        }
        if (chosen == null) {
            planNote = "UNREACHABLE - " + firstReason;
            status = "SCANNING - nearest POLLEN is UNREACHABLE";
            return;
        }
        tracker.lock(chosen, cfg, now);
        attempts++;
        attemptNanos = now;
        attemptStartX = here.x();
        attemptStartY = here.y();
        double distance = chosen.distanceTo(here.x(), here.y());
        crossCheck.arm(chosen.x, chosen.y, distance, cfg);
        travelLimit = chosenPlan.length + cfg.extraTravelInches + crossCheck.tolerance();
        lastEvent = String.format(Locale.US, "locked POLLEN at (%.1f, %.1f), %.1f in away, %d frames", chosen.x, chosen.y,
                distance, chosen.frames);
        startPlan(now, chosenPlan, false);
    }

    private void startPlan(long now, PollenRoutePlanner.Plan next, boolean replanning) {
        plan = next;
        planNote = next.describe();
        planBallX = next.ballX;
        planBallY = next.ballY;
        planNanos = now;
        Pose here = safePose();
        if (here == null) {
            abort("!! pose unavailable");
            return;
        }
        if (next.status == PollenRoutePlanner.Status.ARRIVED) {
            double error = Angle.normalizeSigned(next.approachHeading - here.heading());
            if (Math.abs(error) > Math.toRadians(cfg.turnFirstDegrees)) {
                arriveAfterTurn = true;
                enterTurn(now, next.approachHeading);
                return;
            }
            arrive(now, "already within reach");
            return;
        }
        double error = Angle.normalizeSigned(next.firstHeading() - here.heading());
        if (Math.abs(error) > Math.toRadians(cfg.turnFirstDegrees)) {
            enterTurn(now, next.firstHeading());
            return;
        }
        followPlan(now, replanning);
    }

    private void enterTurn(long now, double heading) {
        turnPasses++;
        if (turnPasses > MAX_TURN_PASSES) {
            abort("!! could not face the route after " + MAX_TURN_PASSES + " turns");
            return;
        }
        Pose here = safePose();
        if (here == null) {
            abort("!! pose unavailable");
            return;
        }
        state = State.TURNING;
        stateNanos = now;
        turnHeading = heading;
        turnStartAbs = Math.abs(Angle.normalizeSigned(heading - here.heading()));
        turnStartX = here.x();
        turnStartY = here.y();
        turnDriftLimit = cfg.turnDriftAbortInches + Math.min(TURN_COAST_MAX_INCHES, measuredSpeed() * TURN_COAST_SECONDS);
        cameFromTurn = true;
        turnStep(now);
    }

    private double measuredSpeed() {
        if (follower == null) {
            return 0.0;
        }
        try {
            Velocity v = follower.velocity();
            if (v == null) {
                return 0.0;
            }
            double speed = Math.hypot(v.vx, v.vy);
            return finite(speed) ? speed : 0.0;
        } catch (Throwable t) {
            return 0.0;
        }
    }

    private void turnStep(long now) {
        if (!motionGuards(now)) {
            return;
        }
        Pose here = safePose();
        double drift = Math.hypot(here.x() - turnStartX, here.y() - turnStartY);
        if (drift > turnDriftLimit) {
            abortLocalizer(String.format(Locale.US,
                    "!! odometry moved %.1f in during an in-place turn - Y pod reversed or pod offsets wrong - push-test LEFT",
                    drift));
            return;
        }
        double error = Angle.normalizeSigned(turnHeading - here.heading());
        double abs = Math.abs(error);
        if (abs > turnStartAbs + TURN_WRONG_WAY_RADIANS) {
            abortLocalizer(String.format(Locale.US,
                    "!! turning the wrong way (heading error %.0f deg grew from %.0f deg) - check the IMU heading",
                    Math.toDegrees(abs), Math.toDegrees(turnStartAbs)));
            return;
        }
        double elapsed = (now - stateNanos) / NANOS_PER_SECOND;
        boolean aligned = abs <= Math.toRadians(cfg.turnToleranceDegrees);
        if (aligned || elapsed >= cfg.turnTimeoutSeconds) {
            if (abs > Math.toRadians(cfg.turnFirstDegrees)) {
                abort(String.format(Locale.US, "!! turn timed out after %.1f s with %.0f deg still to go", elapsed,
                        Math.toDegrees(abs)));
                return;
            }
            haltWheels();
            if (arriveAfterTurn) {
                arriveAfterTurn = false;
                arrive(now, "turned to face the ball");
                return;
            }
            replanAfterTurn(now);
            return;
        }
        follower.manual(0.0, 0.0, PollenSeekGuards.turnPower(error, cfg));
        ownsMotion = true;
        status = String.format(Locale.US, "TURNING  error %+.0f deg  %.1f / %.1f s", Math.toDegrees(error), elapsed,
                cfg.turnTimeoutSeconds);
    }

    private void replanAfterTurn(long now) {
        Pose here = safePose();
        double bx = tracker.estimateX();
        double by = tracker.estimateY();
        PollenRoutePlanner.Plan next = planner.plan(here.x(), here.y(), here.heading(), bx, by);
        if (!next.reachable()) {
            abort("!! target became UNREACHABLE after the turn: " + next.reason);
            return;
        }
        startPlan(now, next, progress.isActive());
    }

    private void followPlan(long now, boolean replanning) {
        Path path = buildPath(plan);
        if (path == null) {
            arrive(now, "path shorter than the minimum");
            return;
        }
        try {
            follower.follow(path);
        } catch (RuntimeException e) {
            abort("!! could not follow the path: " + e.getClass().getSimpleName() + " " + e.getMessage());
            return;
        }
        ownsMotion = true;
        driven = false;
        Pose here = safePose();
        legHint = 0;
        double remaining = PollenRoutePlanner.progress(plan, here.x(), here.y(), legHint).remaining;
        if (replanning && progress.isActive()) {
            progress.replanned(remaining);
        } else {
            progress.start(remaining, now);
        }
        if (cameFromTurn) {
            progress.resume(now);
            cameFromTurn = false;
        }
        onFinalLeg = plan.legCount() <= 1;
        finalLegNanos = onFinalLeg ? now : 0L;
        double toBall = Math.hypot(tracker.estimateX() - here.x(), tracker.estimateY() - here.y());
        state = toBall <= cfg.finalApproachInches ? State.FINAL_APPROACH : State.FOLLOWING;
        stateNanos = now;
        status = state + " " + planNote;
    }

    private Path buildPath(PollenRoutePlanner.Plan route) {
        if (route == null || route.pointCount() < 2 || route.length < cfg.minPathInches) {
            return null;
        }
        List<Path> legs = new ArrayList<Path>();
        for (int i = 0; i + 1 < route.pointCount(); i++) {
            double length = Math.hypot(route.xs[i + 1] - route.xs[i], route.ys[i + 1] - route.ys[i]);
            if (length < 1e-3) {
                continue;
            }
            legs.add(Paths.line(new Pose(route.xs[i], route.ys[i], 0.0), new Pose(route.xs[i + 1], route.ys[i + 1], 0.0)));
        }
        if (legs.isEmpty()) {
            return null;
        }
        Path path = legs.size() == 1 ? legs.get(0).tangent() : Paths.path(legs.toArray(new Path[0])).tangent();
        Modifier speed = speedModifier();
        return speed == null ? path : path.with(speed);
    }

    private Modifier speedModifier() {
        if (!(cfg.maxVelocityInchesPerSecond > 0.0) || follower == null) {
            speedNote = "";
            return null;
        }
        Algorithm algorithm = follower.algorithm();
        if (!(algorithm instanceof Foresight)) {
            speedNote = "no Foresight - speed limit not applied";
            return null;
        }
        try {
            Modifier modifier = ((Foresight) algorithm).config.maxVelocityConstraint.at(cfg.maxVelocityInchesPerSecond);
            speedNote = String.format(Locale.US, "%.0f in/s", cfg.maxVelocityInchesPerSecond);
            return modifier;
        } catch (Throwable t) {
            speedNote = "speed limit rejected: " + t.getClass().getSimpleName();
            return null;
        }
    }

    private void followStep(long now) {
        if (!motionGuards(now)) {
            return;
        }
        Pose here = safePose();
        PollenRoutePlanner.Progress along = PollenRoutePlanner.progress(plan, here.x(), here.y(), legHint);
        legHint = along.legIndex;
        String verdict = progress.update(along.remaining, now, cfg);
        if (verdict != null) {
            if (progress.diverged()) {
                abortLocalizer(verdict);
            } else {
                abort(verdict);
            }
            return;
        }
        String frozen = frozenOdometry(now, here);
        if (frozen != null) {
            abort(frozen);
            return;
        }
        double estX = tracker.estimateX();
        double estY = tracker.estimateY();
        if (crossCheck.estimateDrifted(estX, estY)) {
            abort(String.format(Locale.US,
                    "!! target estimate moved %.1f in from the lock point (> %.1f in) - odometry disagrees with camera",
                    crossCheck.estimateDrift(estX, estY), crossCheck.tolerance()));
            return;
        }
        if (!onFinalLeg && along.legIndex >= plan.legCount() - 1) {
            onFinalLeg = true;
            finalLegNanos = now;
        }
        double toBall = Math.hypot(estX - here.x(), estY - here.y());
        if (state == State.FOLLOWING) {
            if (toBall <= cfg.finalApproachInches) {
                state = State.FINAL_APPROACH;
                stateNanos = now;
                lastEvent = String.format(Locale.US, "final approach at %.1f in - committed to the last estimate", toBall);
            } else {
                if (onFinalLeg) {
                    long since = Math.max(tracker.lastSeenNanos(), finalLegNanos);
                    if (now - since > (long) (cfg.lostSeconds * NANOS_PER_SECOND)) {
                        lose(now);
                        return;
                    }
                }
                boolean moved = Math.hypot(estX - planBallX, estY - planBallY) > cfg.replanInches;
                boolean waited = now - planNanos >= (long) (cfg.replanMinSeconds * NANOS_PER_SECOND);
                if (moved && waited) {
                    replan(now, here, estX, estY);
                    return;
                }
            }
        }
        double toStop = Math.hypot(plan.approachX - here.x(), plan.approachY - here.y());
        boolean pathEnded = !follower.following();
        if (toStop <= cfg.arriveToleranceInches || pathEnded) {
            arrive(now, pathEnded
                    ? String.format(Locale.US, "path ended %.1f in from the stop point", toStop)
                    : String.format(Locale.US, "%.1f in from the stop point", toStop));
            return;
        }
        status = String.format(Locale.US, "%s  %.1f in to go  ball %.1f in", state, along.remaining, toBall);
    }

    private String frozenOdometry(long now, Pose here) {
        double peak = follower.drivetrain instanceof CappedDrivetrain
                ? ((CappedDrivetrain) follower.drivetrain).lastPeakWheelPower() : 0.0;
        if (!(peak >= FROZEN_MIN_WHEEL_POWER)) {
            driven = false;
            return null;
        }
        if (!driven) {
            driven = true;
            drivenSinceNanos = now;
            return null;
        }
        if (now - drivenSinceNanos < FROZEN_WINDOW_NANOS || now - planNanos < FROZEN_GRACE_NANOS) {
            return null;
        }
        PlanarPose then = history.poseAt(now - FROZEN_WINDOW_NANOS, 0L);
        if (then == null || !then.isFinite()) {
            return null;
        }
        double moved = Math.hypot(here.x() - then.x, here.y() - then.y);
        if (moved >= FROZEN_MIN_MOVE_INCHES) {
            return null;
        }
        return String.format(Locale.US,
                "!! wheels driven (peak %.2f) for %.1f s but odometry moved only %.1f in - Pinpoint frozen or robot blocked - stopped",
                peak, (now - drivenSinceNanos) / NANOS_PER_SECOND, moved);
    }

    private void replan(long now, Pose here, double estX, double estY) {
        PollenRoutePlanner.Plan next = planner.plan(here.x(), here.y(), here.heading(), estX, estY);
        if (!next.reachable()) {
            abort(String.format(Locale.US,
                    "!! target estimate moved %.1f in to an UNREACHABLE spot (%s) - odometry vs camera? push-test the Pinpoint",
                    crossCheck.estimateDrift(estX, estY), next.reason));
            return;
        }
        replans++;
        lastEvent = String.format(Locale.US, "re-planned (#%d) to (%.1f, %.1f)", replans, estX, estY);
        startPlan(now, next, true);
    }

    private void arrive(long now, String note) {
        haltWheels();
        progress.stop();
        lastEvent = "ARRIVED - " + note;
        if (intakeOnArrive && intake != null && cfg.intakeSeconds > 0.0) {
            Pose here = safePose();
            if (here == null) {
                abort("!! pose unavailable");
                return;
            }
            state = State.INTAKING;
            stateNanos = now;
            creepX = here.x();
            creepY = here.y();
            creepHeading = here.heading();
            runIntake();
            status = "INTAKING";
            return;
        }
        state = State.DONE;
        stateNanos = now;
        String skipped = intakeOnArrive && intake == null ? "  (no intake motor - intake skipped)" : "";
        status = "DONE - " + note + skipped;
    }

    private void intakeStep(long now) {
        if (!motionGuards(now)) {
            return;
        }
        Pose here = safePose();
        double dx = here.x() - creepX;
        double dy = here.y() - creepY;
        double c = Math.cos(creepHeading);
        double s = Math.sin(creepHeading);
        double along = dx * c + dy * s;
        double side = -dx * s + dy * c;
        if (along < -cfg.intakeBackwardAbortInches) {
            abortLocalizer(String.format(Locale.US,
                    "!! creeping forward but odometry reads %.1f in BACKWARD - X pod reversed? push-test the Pinpoint", along));
            return;
        }
        if (Math.abs(side) > Math.max(MIN_SIDEWAYS_ABORT_INCHES, cfg.intakeCreepInches)) {
            abortLocalizer(String.format(Locale.US, "!! creeping forward but odometry reads %.1f in SIDEWAYS - Y pod?", side));
            return;
        }
        double elapsed = (now - stateNanos) / NANOS_PER_SECOND;
        if (elapsed >= cfg.intakeSeconds) {
            stopIntake();
            haltWheels();
            state = State.DONE;
            stateNanos = now;
            status = String.format(Locale.US, "DONE - intake ran %.1f s, crept %.1f in", elapsed, along);
            lastEvent = status;
            return;
        }
        runIntake();
        if (cfg.intakeCreepInches > 0.0 && cfg.creepPower > 0.0 && along < cfg.intakeCreepInches) {
            follower.manual(cfg.creepPower, 0.0, 0.0);
            ownsMotion = true;
        } else if (ownsMotion) {
            haltWheels();
        }
        status = String.format(Locale.US, "INTAKING  %.1f / %.1f s  crept %.1f / %.1f in", elapsed, cfg.intakeSeconds,
                along, cfg.intakeCreepInches);
    }

    private boolean motionGuards(long now) {
        if (follower == null) {
            abort("!! no Follower");
            return false;
        }
        if (poseFault != null) {
            abort(poseFault);
            return false;
        }
        if (state != State.INTAKING && now - attemptNanos > (long) (cfg.seekTimeoutSeconds * NANOS_PER_SECOND)) {
            abort(String.format(Locale.US, "!! seek timeout %.1f s - stopped", cfg.seekTimeoutSeconds));
            return false;
        }
        Pose here = safePose();
        if (here == null || !finite(here.x()) || !finite(here.y()) || !finite(here.heading())) {
            latchPose("!! pose unavailable");
            abort("!! pose unavailable");
            return false;
        }
        double traveled = Math.hypot(here.x() - attemptStartX, here.y() - attemptStartY);
        if (traveled > travelLimit) {
            abort(String.format(Locale.US, "!! traveled %.1f in from the start, more than the plan allows (%.1f in)",
                    traveled, travelLimit));
            return false;
        }
        return true;
    }

    private void lose(long now) {
        haltMotion();
        lostCount++;
        clearAttempt();
        state = State.SCANNING;
        stateNanos = now;
        lastEvent = String.format(Locale.US, "!! lost the ball for %.1f s - stopped, scanning again", cfg.lostSeconds);
        status = lastEvent;
        planNote = "lost the ball";
    }

    private void abort(String reason) {
        enterAborted(clock.nanoTime(), reason);
    }

    private void enterAborted(long now, String reason) {
        haltMotion();
        crossCheck.disarm();
        progress.stop();
        arriveAfterTurn = false;
        state = State.ABORTED;
        stateNanos = now;
        abortReason = reason == null ? "aborted" : reason;
        status = abortReason;
        lastEvent = abortReason;
        abortCount++;
    }

    private void notePoseFault(String fault) {
        poseFault = fault;
        if (isMotionState()) {
            abort(fault);
        }
    }

    private void haltMotion() {
        stopIntake();
        haltWheels();
    }

    private void haltWheels() {
        if (follower != null) {
            try {
                follower.stop();
            } catch (Throwable ignored) {
            }
            try {
                follower.drivetrain.stop();
            } catch (Throwable ignored) {
            }
        }
        ownsMotion = false;
    }

    private void runIntake() {
        if (intake == null) {
            return;
        }
        try {
            intake.run(intakePower());
            intakeRunning = true;
        } catch (Throwable t) {
            intakeRunning = false;
            abort("!! intake failed: " + t.getClass().getSimpleName());
        }
    }

    private void stopIntake() {
        if (intake == null) {
            intakeRunning = false;
            return;
        }
        try {
            intake.stop();
        } catch (Throwable ignored) {
        }
        intakeRunning = false;
    }

    public double intakePower() {
        return cfg.useConstantsIntakePower ? defaultIntakePower : cfg.intakePower;
    }

    private void maybeRefreshConfig() {
        long now = clock.nanoTime();
        if (!isMotionState() && state != State.SCANNING && now - configNanos >= CONFIG_REFRESH_NANOS) {
            refreshConfig(now);
        }
    }

    private void refreshConfig(long now) {
        cfg = liveConfig.sanitized();
        planner = new PollenRoutePlanner(cfg);
        configNanos = now;
        applyCap();
    }

    private void applyCap() {
        if (follower != null && follower.drivetrain instanceof CappedDrivetrain) {
            ((CappedDrivetrain) follower.drivetrain).setCap(cfg.maxPower);
        }
    }

    private Pose safePose() {
        if (follower == null) {
            return null;
        }
        try {
            return follower.pose();
        } catch (Throwable t) {
            return null;
        }
    }

    private PlanarPose referencePose() {
        if (follower == null) {
            return new PlanarPose(0.0, 0.0, 0.0);
        }
        Pose pose = safePose();
        if (pose == null || !finite(pose.x()) || !finite(pose.y()) || !finite(pose.heading())) {
            return null;
        }
        return new PlanarPose(pose.x(), pose.y(), pose.heading());
    }

    public boolean isMotionState() {
        return state == State.TURNING || state == State.FOLLOWING || state == State.FINAL_APPROACH
                || state == State.INTAKING;
    }

    public boolean canSeePollen() {
        long now = clock.nanoTime();
        PlanarPose robot = referencePose();
        if (robot == null) {
            return false;
        }
        if (tracker.isLocked() && now - tracker.lastSeenNanos() <= (long) (cfg.confirmWindowSeconds * NANOS_PER_SECOND)) {
            return true;
        }
        return !tracker.confirmedClusters(cfg, now, robot.x, robot.y).isEmpty();
    }

    public List<PollenTracker.Ball> visibleBalls() {
        long now = clock.nanoTime();
        if (now - tracker.newestFrameNanos() > (long) (cfg.maxFrameAgeMs * NANOS_PER_MILLI)) {
            return Collections.emptyList();
        }
        return tracker.newestBalls();
    }

    public PollenTracker.Ball nearestVisible() {
        PollenTracker.Ball best = null;
        for (PollenTracker.Ball ball : visibleBalls()) {
            if (best == null || ball.distance < best.distance) {
                best = ball;
            }
        }
        return best;
    }

    public FieldBall currentTarget() {
        long now = clock.nanoTime();
        PlanarPose robot = referencePose();
        if (robot == null) {
            return null;
        }
        if (tracker.isLocked()) {
            return fieldBall(robot, tracker.estimateX(), tracker.estimateY(), tracker.framesSeen(),
                    (now - tracker.lastSeenNanos()) / NANOS_PER_MILLI, true);
        }
        List<PollenTracker.Cluster> clusters = tracker.confirmedClusters(cfg, now, robot.x, robot.y);
        if (clusters.isEmpty()) {
            return null;
        }
        PollenTracker.Cluster nearest = clusters.get(0);
        return fieldBall(robot, nearest.x, nearest.y, nearest.frames, (now - nearest.newestNanos) / NANOS_PER_MILLI, false);
    }

    private static FieldBall fieldBall(PlanarPose robot, double x, double y, int frames, double ageMs, boolean locked) {
        Vec3 inRobot = TargetGeometry.fieldPointToRobot(robot, new Vec3(x, y, 0.0));
        double distance = TargetGeometry.horizontalDistanceInches(inRobot);
        double bearing = TargetGeometry.horizontalBearingLeftDegrees(inRobot);
        return new FieldBall(x, y, distance, bearing, frames, ageMs, locked);
    }

    public String visibilityLine() {
        List<PollenTracker.Ball> balls = visibleBalls();
        boolean confirmed = canSeePollen();
        PollenTracker.Ball nearest = nearestVisible();
        if (balls.isEmpty()) {
            return confirmed ? "YES (not in the newest frame)" : "NO";
        }
        String count = balls.size() == 1 ? "1 ball" : balls.size() + " balls";
        return String.format(Locale.US, "%s %s, nearest %.1f in @ %+.0f deg  field (%.1f, %.1f)%s",
                confirmed ? "YES" : "seen, confirming", count, nearest.distance, nearest.bearingLeftDegrees,
                nearest.x, nearest.y, confirmed ? "" : String.format(Locale.US, "  (needs %d frames)", cfg.confirmFrames));
    }

    public String guardLine() {
        long now = clock.nanoTime();
        StringBuilder line = new StringBuilder();
        if (progress.isActive()) {
            line.append(String.format(Locale.US, "away +%.1f / %.1f in  no-progress %.1f / %.1f s", progress.excess(),
                    cfg.divergeInches, progress.secondsSinceProgress(now), cfg.stallSeconds));
        } else {
            line.append("progress monitor idle");
        }
        if (crossCheck.isArmed()) {
            double last = crossCheck.lastError();
            line.append(String.format(Locale.US, "  cam-vs-odo %s / %.1f in (%d/%d)",
                    Double.isNaN(last) ? "--" : String.format(Locale.US, "%.1f", last), crossCheck.tolerance(),
                    crossCheck.consecutive(), cfg.crossCheckFrames));
        }
        if (isMotionState() && state != State.INTAKING) {
            line.append(String.format(Locale.US, "  time %.1f / %.1f s", (now - attemptNanos) / NANOS_PER_SECOND,
                    cfg.seekTimeoutSeconds));
        }
        return line.toString();
    }

    public String capLine() {
        double cap = follower != null && follower.drivetrain instanceof CappedDrivetrain
                ? ((CappedDrivetrain) follower.drivetrain).cap() : Double.NaN;
        return String.format(Locale.US, "max wheel power %s  path speed %s  turn <= %.2f  creep %.2f",
                Double.isNaN(cap) ? "NO CAP" : String.format(Locale.US, "%.2f", cap),
                cfg.maxVelocityInchesPerSecond > 0.0 ? String.format(Locale.US, "<= %.0f in/s", cfg.maxVelocityInchesPerSecond)
                        : "unlimited", cfg.maxTurnPower, cfg.creepPower);
    }

    public Pose plannedTarget() {
        PollenRoutePlanner.Plan current = plan;
        if (current == null || current.status == PollenRoutePlanner.Status.UNREACHABLE) {
            return null;
        }
        return new Pose(current.approachX, current.approachY, current.approachHeading);
    }

    public List<Pose> plannedVia() {
        PollenRoutePlanner.Plan current = plan;
        List<Pose> via = new ArrayList<Pose>();
        if (current == null) {
            return via;
        }
        for (int i = 1; i + 1 < current.pointCount(); i++) {
            via.add(new Pose(current.xs[i], current.ys[i], 0.0));
        }
        return via;
    }

    public PollenRoutePlanner.Plan plan() {
        return plan;
    }

    public PollenRoutePlanner planner() {
        return planner;
    }

    public State state() {
        return state;
    }

    public String status() {
        return status;
    }

    public String abortReason() {
        return abortReason;
    }

    public String lastEvent() {
        return lastEvent;
    }

    public String planNote() {
        return planNote;
    }

    public String speedNote() {
        return speedNote;
    }

    public String poseFault() {
        return poseFault;
    }

    public int attempts() {
        return attempts;
    }

    public int lostCount() {
        return lostCount;
    }

    public int abortCount() {
        return abortCount;
    }

    public int replans() {
        return replans;
    }

    public long framesUsed() {
        return framesUsed;
    }

    public long framesWithoutPose() {
        return framesWithoutPose;
    }

    public boolean intakeRunning() {
        return intakeRunning;
    }

    public boolean holdEndDisabled() {
        return holdEndDisabled;
    }

    public PollenSeekGuards.CrossCheck crossCheck() {
        return crossCheck;
    }

    public PollenSeekGuards.ProgressMonitor progressMonitor() {
        return progress;
    }

    public PollenTracker tracker() {
        return tracker;
    }

    public PoseHistory poseHistory() {
        return history;
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
