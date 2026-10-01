package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

public class FieldPoint {

    public static final boolean POINTS_CONFIRMED = false;

    public static final Pose START_POSE = new Pose(9.0, 84.0, Math.toRadians(0.0));

    public static final Pose SHOOT_POSE_1 = new Pose(59.508, 35.382, Math.toRadians(-30.0));
    public static final Pose SHOOT_POSE_2 = new Pose(59.426, 104.948, Math.toRadians(-45.0));

    public static final Pose FLOWER_1_START = new Pose(17.267, 46.868, Math.toRadians(90.0));
    public static final Pose FLOWER_1_END = new Pose(17.267,47.092,Math.toRadians(180.0));
    public static final Pose FLOWER_2_START = new Pose(48.0, 108.0, Math.toRadians(90.0));
    public static final Pose FLOWER_2_END = new Pose(8.297,47.092, Math.toRadians(180));
    public static final Pose FLOWER_3_START = new Pose(72.0, 108.0, Math.toRadians(90.0));
    public static final Pose FLOWER_3_END= new Pose(0.0,0.0,Math.toRadians(180.0));
    public static final Pose FLOWER_4_START = new Pose(96.0, 108.0, Math.toRadians(90.0));
    public static final Pose FLOWER_4_END = new Pose(0.0,0.0,Math.toRadians(90.0));

    public static final Pose PARK_POSE = new Pose(60.0, 84.0, Math.toRadians(0.0));

    public static final double AIM_AT_SHOOT_POSE_1 = 0.50;
    public static final double AIM_AT_SHOOT_POSE_2 = 0.55;

    public static final int SHOTS_AT_SHOOT_POSE_1 = 3;
    public static final int SHOTS_AT_SHOOT_POSE_2 = 3;
    public static final int SHOTS_AFTER_FLOWER = 1;

    public static final double INTAKE_SECONDS_AT_FLOWER = 1.5;

    public static final boolean INTAKE_WHILE_DRIVING_TO_FLOWER = true;

    public static final double DEFAULT_PATH_TIMEOUT_SECONDS = 6.0;

    public static final double AUTONOMOUS_BUDGET_SECONDS = 28.0;

    public static final double MIN_PATH_LENGTH_INCHES = 0.5;

    public enum Action {
        NONE,
        SHOOT,
        INTAKE
    }

    public static List<Step> mission() {
        List<Step> steps = new ArrayList<Step>();

        steps.add(shoot("Shoot preload at shooting point 1", SHOOT_POSE_1, SHOTS_AT_SHOOT_POSE_1, AIM_AT_SHOOT_POSE_1));

        steps.add(move("Move to flower 1 start", FLOWER_1_START));
        steps.add(collect("Sweep flower 1", FLOWER_1_END, INTAKE_SECONDS_AT_FLOWER));
        steps.add(shoot("Shoot flower 1 at shooting point 1", SHOOT_POSE_1, SHOTS_AFTER_FLOWER, AIM_AT_SHOOT_POSE_1));

        steps.add(move("Move to flower 2 start", FLOWER_2_START));
        steps.add(collect("Sweep flower 2", FLOWER_2_END, INTAKE_SECONDS_AT_FLOWER));
        steps.add(shoot("Shoot flower 2 at shooting point 2", SHOOT_POSE_2, SHOTS_AFTER_FLOWER, AIM_AT_SHOOT_POSE_2));

        steps.add(move("Move to flower 3 start", FLOWER_3_START));
        steps.add(collect("Sweep flower 3", FLOWER_3_END, INTAKE_SECONDS_AT_FLOWER));
        steps.add(shoot("Shoot flower 3 at shooting point 2", SHOOT_POSE_2, SHOTS_AFTER_FLOWER, AIM_AT_SHOOT_POSE_2));

        steps.add(move("Move to flower 4 start", FLOWER_4_START));
        steps.add(collect("Sweep flower 4", FLOWER_4_END, INTAKE_SECONDS_AT_FLOWER));
        steps.add(shoot("Shoot flower 4 at shooting point 2", SHOOT_POSE_2, SHOTS_AT_SHOOT_POSE_2, AIM_AT_SHOOT_POSE_2));

        steps.add(park("Park", PARK_POSE));

        return steps;
    }

    public static Step move(String name, Pose target) {
        return new Step(name, target, Action.NONE, 0, 0.0, false,
                DEFAULT_PATH_TIMEOUT_SECONDS, AIM_AT_SHOOT_POSE_1);
    }

    public static Step move(String name, Pose target, double timeoutSeconds) {
        return new Step(name, target, Action.NONE, 0, 0.0, false,
                timeoutSeconds, AIM_AT_SHOOT_POSE_1);
    }

    public static Step moveWhileIntaking(String name, Pose target) {
        return new Step(name, target, Action.NONE, 0, 0.0, true,
                DEFAULT_PATH_TIMEOUT_SECONDS, AIM_AT_SHOOT_POSE_1);
    }

    public static Step park(String name, Pose target) {
        return move(name, target);
    }

    public static Step shoot(String name, Pose target, int shots, double aim) {
        return new Step(name, target, Action.SHOOT, shots, 0.0, false,
                DEFAULT_PATH_TIMEOUT_SECONDS, aim);
    }

    public static Step collect(String name, Pose target, double intakeSeconds) {
        return new Step(name, target, Action.INTAKE, 0, intakeSeconds,
                INTAKE_WHILE_DRIVING_TO_FLOWER, DEFAULT_PATH_TIMEOUT_SECONDS, AIM_AT_SHOOT_POSE_1);
    }

    public static class Step {

        public final String name;
        public final Pose target;
        public final Action action;
        public final int shots;
        public final double seconds;
        public final boolean intakeWhileDriving;
        public final double pathTimeoutSeconds;
        public final double aim;

        public Step(String name, Pose target, Action action, int shots, double seconds,
                    boolean intakeWhileDriving, double pathTimeoutSeconds, double aim) {
            this.name = name;
            this.target = target;
            this.action = action;
            this.shots = shots;
            this.seconds = seconds;
            this.intakeWhileDriving = intakeWhileDriving;
            this.pathTimeoutSeconds = pathTimeoutSeconds;
            this.aim = aim;
        }
    }
}
