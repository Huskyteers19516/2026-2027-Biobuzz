package org.firstinspires.ftc.teamtestcode.OpModeForTest;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;

public class Paths {

    public static final Pose START_POSE = new Pose(9.0, 84.0, Math.toRadians(0.0));
    public static final Pose SHOOT_POSE_1 = new Pose(36.0, 84.0, Math.toRadians(-30.0));
    public static final Pose PICKUP_CONTROL = new Pose(36.0, 104.0, Math.toRadians(0.0));
    public static final Pose PICKUP_POSE_1 = new Pose(20.0, 110.0, Math.toRadians(90.0));
    public static final Pose SHOOT_POSE_2 = new Pose(44.0, 88.0, Math.toRadians(-30.0));
    public static final Pose PARK_POSE = new Pose(60.0, 60.0, Math.toRadians(0.0));

    public static final int SHOTS_AT_STOP_1 = 3;
    public static final int SHOTS_AT_STOP_2 = 3;
    public static final double INTAKE_SECONDS_AT_PICKUP = 1.5;

    public static final int USE_DEFAULT_SHOTS = 0;

    public enum Action {
        NONE,
        SHOOT,
        INTAKE,
        WAIT
    }

    public static List<Step> mission() {
        List<Step> steps = new ArrayList<Step>();

        steps.add(shoot("Drive to shooting spot 1",
                straight(START_POSE, SHOOT_POSE_1),
                SHOOT_POSE_1,
                SHOTS_AT_STOP_1));

        steps.add(collect("Curve to pickup 1 while intaking",
                curved(SHOOT_POSE_1, PICKUP_CONTROL, PICKUP_POSE_1),
                PICKUP_POSE_1,
                INTAKE_SECONDS_AT_PICKUP));

        steps.add(shoot("Drive to shooting spot 2",
                straight(PICKUP_POSE_1, SHOOT_POSE_2),
                SHOOT_POSE_2,
                SHOTS_AT_STOP_2));

        steps.add(drive("Park",
                straight(SHOOT_POSE_2, PARK_POSE),
                PARK_POSE));

        return steps;
    }

    public static Path straight(Pose start, Pose end) {
        return com.pedropathing.api.Paths.line(start, end).linear(start, end);
    }

    public static Path curved(Pose start, Pose control, Pose end) {
        return com.pedropathing.api.Paths.curve(start, control, end).linear(start, end);
    }

    public static Path curved(Pose start, Pose firstControl, Pose secondControl, Pose end) {
        return com.pedropathing.api.Paths.curve(start, firstControl, secondControl, end).linear(start, end);
    }

    public static Step drive(String name, Path path, Pose target) {
        return new Step(name, path, target, Action.NONE, USE_DEFAULT_SHOTS, 0.0, false);
    }

    public static Step shoot(String name, Path path, Pose target) {
        return shoot(name, path, target, USE_DEFAULT_SHOTS);
    }

    public static Step shoot(String name, Path path, Pose target, int shots) {
        return new Step(name, path, target, Action.SHOOT, shots, 0.0, false);
    }

    public static Step collect(String name, Path path, Pose target, double intakeSeconds) {
        return new Step(name, path, target, Action.INTAKE, USE_DEFAULT_SHOTS, intakeSeconds, true);
    }

    public static Step pause(String name, Path path, Pose target, double waitSeconds) {
        return new Step(name, path, target, Action.WAIT, USE_DEFAULT_SHOTS, waitSeconds, false);
    }

    public static class Step {

        public final String name;
        public final Path path;
        public final Pose target;
        public final Action action;
        public final int shots;
        public final double seconds;
        public final boolean intakeDuringPath;

        public Step(String name, Path path, Pose target, Action action, int shots, double seconds,
                    boolean intakeDuringPath) {
            this.name = name;
            this.path = path;
            this.target = target;
            this.action = action;
            this.shots = shots;
            this.seconds = seconds;
            this.intakeDuringPath = intakeDuringPath;
        }
    }
}
