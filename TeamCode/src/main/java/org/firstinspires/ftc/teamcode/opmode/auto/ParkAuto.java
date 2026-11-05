package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class ParkAuto extends LinearOpMode {

    private static final double MIN_SEGMENT_INCHES = 0.5;
    private static final double TURN_TOLERANCE_DEGREES = 3.0;

    private Follower follower;
    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private final ElapsedTime runtime = new ElapsedTime();
    private String lastResult = "none";
    private String stopReason = "parked";

    protected abstract String routeName();

    protected abstract Pose[] route(GoalScanner.Alliance alliance);

    protected abstract double[] segmentTimeouts(GoalScanner.Alliance alliance);

    protected abstract boolean routeConfirmed(GoalScanner.Alliance alliance);

    protected abstract String pointsFile(GoalScanner.Alliance alliance);

    @Override
    public void runOpMode() {
        PoseHandoff.clear();
        checkRoute(GoalScanner.Alliance.RED);
        checkRoute(GoalScanner.Alliance.BLUE);

        follower = Constants.createFollower(hardwareMap);
        alliance = PoseHandoff.alliance();
        follower.setPose(GoalAim.startPose(alliance));

        while (opModeInInit()) {
            GoalScanner.Alliance picked = alliance;
            if ((gamepad1.x && !gamepad1.start) || (gamepad2.x && !gamepad2.start)) {
                picked = GoalScanner.Alliance.BLUE;
            }
            if ((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start)) {
                picked = GoalScanner.Alliance.RED;
            }
            if (picked != alliance) {
                alliance = picked;
                follower.setPose(GoalAim.startPose(alliance));
            }
            follower.localizer.update();

            telemetry.addLine(routeName());
            telemetry.addData("ALLIANCE", "%s   (X = BLUE, B = RED)", alliance);
            telemetry.addData("Points file", pointsFile(alliance));
            telemetry.addData("Start pose", GoalAim.formatPose(GoalAim.startPose(alliance)));
            telemetry.addData("Measured pose", GoalAim.formatPose(follower.pose()));
            Pose[] points = route(alliance);
            for (int i = 0; i < points.length; i++) {
                telemetry.addData(i == points.length - 1 ? "Park" : "Point " + (i + 1), GoalAim.formatPose(points[i]));
            }
            if (!routeConfirmed(alliance)) {
                telemetry.addLine("!! " + alliance + " PARK ROUTE NOT CONFIRMED - AUTO WILL NOT MOVE");
                telemetry.addLine("Measure the points in " + pointsFile(alliance) + ", then set PARK_ROUTE_CONFIRMED = true.");
            }
            telemetry.addLine("Put the robot exactly on the start pose. Nothing moves until START.");
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        if (!routeConfirmed(alliance)) {
            follower.stop();
            follower.update();
            while (opModeIsActive()) {
                telemetry.addLine("!! " + alliance + " PARK ROUTE NOT CONFIRMED - REFUSING TO DRIVE");
                telemetry.addLine("The points in " + pointsFile(alliance) + " are placeholders.");
                telemetry.update();
            }
            return;
        }

        Pose[] points = route(alliance);
        double[] timeouts = segmentTimeouts(alliance);
        try {
            follower.setPose(GoalAim.startPose(alliance));
            follower.update();
            runtime.reset();
            boolean reachedAll = true;
            for (int i = 0; i < points.length && reachedAll; i++) {
                int tries = 0;
                boolean reached = false;
                while (canRun() && !reached) {
                    reached = driveTo(i, points.length, points[i], timeouts[i]);
                    tries++;
                }
                if (!reached) {
                    reachedAll = false;
                    stopReason = opModeIsActive()
                            ? String.format("out of time going to point %d (%d tries) - stopped where it was", i + 1, tries)
                            : "stop requested";
                }
            }
            Pose park = points[points.length - 1];
            follower.hold(reachedAll ? park : follower.pose());
            while (opModeIsActive()) {
                follower.update();
                telemetry.addData("Status", "%s - holding", stopReason);
                telemetry.addData("Alliance", alliance);
                telemetry.addData("Last result", lastResult);
                telemetry.addData("Pose", GoalAim.formatPose(follower.pose()));
                telemetry.addData("Distance to park", "%.1f in", park.distance(follower.pose()));
                telemetry.addData("Runtime", "%.1f s", runtime.seconds());
                telemetry.update();
            }
        } finally {
            follower.stop();
            follower.update();
            PoseHandoff.save(follower.pose(), alliance);
        }
    }

    private boolean driveTo(int index, int total, Pose target, double timeoutSeconds) {
        Pose start = follower.pose();
        String label = String.format("%d / %d", index + 1, total);
        ElapsedTime timer = new ElapsedTime();
        boolean timedOut = false;

        if (start.distance(target) < MIN_SEGMENT_INCHES) {
            follower.hold(target);
            while (canRun() && headingErrorDegrees(target) > TURN_TOLERANCE_DEGREES) {
                if (timer.seconds() >= timeoutSeconds) {
                    timedOut = true;
                    break;
                }
                follower.update();
                addDriveTelemetry(label, target, timer, timeoutSeconds, "turning in place");
            }
        } else {
            follower.follow(Paths.line(start, target).linear(start, target));
            while (canRun() && (follower.following() || follower.isBusy())) {
                if (timer.seconds() >= timeoutSeconds) {
                    timedOut = true;
                    break;
                }
                follower.update();
                addDriveTelemetry(label, target, timer, timeoutSeconds, "following");
            }
        }

        boolean reached = !timedOut && canRun();
        lastResult = String.format("point %s %s after %.1f s", label,
                reached ? "reached" : timedOut ? "TIMEOUT - retrying the same point" : "stopped", timer.seconds());
        return reached;
    }

    private void addDriveTelemetry(String label, Pose target, ElapsedTime timer, double timeoutSeconds, String phase) {
        telemetry.addLine(routeName());
        telemetry.addData("Point", "%s  %s", label, phase);
        telemetry.addData("Target", GoalAim.formatPose(target));
        telemetry.addData("Pose", GoalAim.formatPose(follower.pose()));
        telemetry.addData("Distance", "%.1f in", target.distance(follower.pose()));
        telemetry.addData("Segment time", "%.1f / %.1f s", timer.seconds(), timeoutSeconds);
        telemetry.addData("Runtime", "%.1f / %.1f s", runtime.seconds(), FieldPoints.AUTONOMOUS_BUDGET_SECONDS);
        telemetry.update();
    }

    private double headingErrorDegrees(Pose target) {
        return Math.abs(Math.toDegrees(GoalAim.normalizeRadians(target.heading() - follower.pose().heading())));
    }

    private boolean canRun() {
        return opModeIsActive() && runtime.seconds() < FieldPoints.AUTONOMOUS_BUDGET_SECONDS;
    }

    private void checkRoute(GoalScanner.Alliance which) {
        Pose[] points = route(which);
        double[] timeouts = segmentTimeouts(which);
        if (points.length == 0 || points.length != timeouts.length) {
            throw new IllegalStateException(routeName() + " " + which
                    + " route needs at least one point and one timeout per point");
        }
    }
}
