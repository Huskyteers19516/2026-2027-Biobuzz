package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;

public final class PoseHandoff {

    public static final double MAX_AGE_SECONDS = 90.0;

    private static Pose pose = null;
    private static long savedAtNanos = 0L;
    private static GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;

    private PoseHandoff() {
    }

    public static void save(Pose finalPose, GoalScanner.Alliance finalAlliance) {
        pose = finalPose;
        alliance = finalAlliance;
        savedAtNanos = System.nanoTime();
    }

    public static Pose pose() {
        if (pose == null || (System.nanoTime() - savedAtNanos) / 1e9 > MAX_AGE_SECONDS) {
            return null;
        }
        return pose;
    }

    public static GoalScanner.Alliance alliance() {
        return alliance;
    }

    public static void clear() {
        pose = null;
    }
}
