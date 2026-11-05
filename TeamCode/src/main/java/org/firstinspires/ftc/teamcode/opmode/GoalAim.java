package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.launcher.ShotTable;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public final class GoalAim {

    public static final class Solution {
        public final String goalName;
        public final Pose goal;
        public final double heading;
        public final double distance;
        public final double entryAngleDegrees;
        public final boolean inEntryAngle;
        public final double velocity;

        Solution(String goalName, Pose goal, Pose robot) {
            this.goalName = goalName;
            this.goal = goal;
            double dx = goal.x() - robot.x();
            double dy = goal.y() - robot.y();
            heading = normalizeRadians(Math.atan2(dy, dx) + Math.toRadians(Constants.SHOOTER_HEADING_OFFSET_DEGREES));
            distance = Math.hypot(dx, dy);
            entryAngleDegrees = Math.abs(Math.toDegrees(normalizeRadians(Math.atan2(-dy, -dx) - goal.heading())));
            inEntryAngle = entryAngleDegrees <= Constants.MAX_GOAL_ENTRY_ANGLE_DEGREES;
            velocity = velocityForDistance(distance);
        }

        public double headingError(Pose robot) {
            return normalizeRadians(heading - robot.heading());
        }

        public String summary(Pose robot) {
            return String.format("%s  dist %.1f in  turn %+.1f deg  entry %.0f deg%s",
                    goalName, distance, Math.toDegrees(headingError(robot)), entryAngleDegrees,
                    inEntryAngle ? "" : "  BAD ANGLE");
        }
    }

    private GoalAim() {
    }

    public static boolean isBlue(GoalScanner.Alliance alliance) {
        return alliance == GoalScanner.Alliance.BLUE;
    }

    public static boolean goalsConfirmed(GoalScanner.Alliance alliance) {
        return isBlue(alliance) ? FieldPointsBlue.GOALS_CONFIRMED : FieldPoints.GOALS_CONFIRMED;
    }

    public static Pose startPose(GoalScanner.Alliance alliance) {
        return isBlue(alliance) ? FieldPointsBlue.START_POSE : FieldPoints.START_POSE;
    }

    public static Solution solve(GoalScanner.Alliance alliance, Pose robot) {
        Solution scoring = isBlue(alliance)
                ? new Solution(GoalScanner.CLUSTER_BLUE_SCORING, FieldPointsBlue.GOAL_SCORING, robot)
                : new Solution(GoalScanner.CLUSTER_RED_SCORING, FieldPoints.GOAL_SCORING, robot);
        Solution audience = isBlue(alliance)
                ? new Solution(GoalScanner.CLUSTER_BLUE_AUDIENCE, FieldPointsBlue.GOAL_AUDIENCE, robot)
                : new Solution(GoalScanner.CLUSTER_RED_AUDIENCE, FieldPoints.GOAL_AUDIENCE, robot);
        if (scoring.inEntryAngle != audience.inEntryAngle) {
            return scoring.inEntryAngle ? scoring : audience;
        }
        return scoring.distance <= audience.distance ? scoring : audience;
    }

    public static Solution solveUp(GoalScanner.Alliance alliance, Pose robot, GoalScanner scanner) {
        Solution best = solve(alliance, robot);
        if (scanner == null || !scanner.classificationTrusted()
                || scanner.goalView(best.goalName) != GoalScanner.GoalView.FACING_DOWN) {
            return best;
        }
        String partner = GoalScanner.partnerOf(best.goalName);
        if (partner == null || scanner.goalView(partner) == GoalScanner.GoalView.FACING_DOWN) {
            return best;
        }
        Solution other = solveNamed(alliance, robot, partner);
        return other.inEntryAngle ? other : best;
    }

    public static Solution solveNamed(GoalScanner.Alliance alliance, Pose robot, String goalName) {
        boolean blue = isBlue(alliance);
        String audience = blue ? GoalScanner.CLUSTER_BLUE_AUDIENCE : GoalScanner.CLUSTER_RED_AUDIENCE;
        if (audience.equals(goalName)) {
            return new Solution(audience, blue ? FieldPointsBlue.GOAL_AUDIENCE : FieldPoints.GOAL_AUDIENCE, robot);
        }
        return new Solution(blue ? GoalScanner.CLUSTER_BLUE_SCORING : GoalScanner.CLUSTER_RED_SCORING,
                blue ? FieldPointsBlue.GOAL_SCORING : FieldPoints.GOAL_SCORING, robot);
    }

    public static double velocityForDistance(double inches) {
        return ShotTable.velocityForDistance(inches);
    }

    public static String shotTableProblem() {
        return ShotTable.problem();
    }

    public static double normalizeRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    public static String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
