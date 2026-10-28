package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;

public class FieldPoints {

    public static final boolean POINTS_CONFIRMED = false;

    public static final Pose START_POSE = new Pose(60.0, 11.0, Math.toRadians(90.0));

    public static final Pose SHOOT_POSE_1 = new Pose(59.508, 35.382, Math.toRadians(-30.0));
    public static final Pose SHOOT_POSE_2 = new Pose(59.426, 104.948, Math.toRadians(-45.0));
    public static final Pose FLOWER_1_START = new Pose(17, 55, Math.toRadians(200.0));
    public static final Pose FLOWER_1_END = new Pose(19,40,Math.toRadians(150.0));
    public static final Pose FLOWER_2_START = new Pose(40.5, 127.5, Math.toRadians(60.0));
    public static final Pose FLOWER_2_END = new Pose(56.7,126.3, Math.toRadians(120));
    public static final Pose FLOWER_3_START = new Pose(126.0, 99.9, Math.toRadians(330.0));
    public static final Pose FLOWER_3_END= new Pose(126.0,84.5,Math.toRadians(30.0));
    public static final Pose FLOWER_4_START = new Pose(102.7, 14.1, Math.toRadians(240.0));
    public static final Pose FLOWER_4_END = new Pose(85.0,13.3,Math.toRadians(300.0));
    public static final Pose ANGLE_1 = new Pose(36.0,36.0,Math.toRadians(90.0));
    public static final Pose ANGLE_2 = new Pose(36.0,108.0,Math.toRadians(0.0));
    public static final Pose ANGLE_3 = new Pose(108.0,108.0,Math.toRadians(270.0));
    public static final Pose ANGLE_4 = new Pose(108.0,36.0,Math.toRadians(180.0));


    public static final Pose PARK_POSE = new Pose(9.5, 105.0, Math.toRadians(90.0));

    public static final double AIM_AT_SHOOT_POSE_1 = 0.50;
    public static final double AIM_AT_SHOOT_POSE_2 = 0.55;

    public static final GoalScanner.Alliance DEFAULT_ALLIANCE = GoalScanner.Alliance.RED;

    public static final boolean VISION_AIMING_ENABLED = true;

    public static final double RANGE_NEAR_INCHES = 40.0;
    public static final double RANGE_FAR_INCHES = 100.0;
    public static final double AIM_AT_NEAR = 0.45;
    public static final double AIM_AT_FAR = 0.65;

    public static final double AIM_BEARING_TOLERANCE_DEGREES = 2.5;
    public static final double AIM_BEARING_MAX_CORRECTION_DEGREES = 30.0;
    public static final double AIM_HEADING_SETTLE_TOLERANCE_DEGREES = 1.5;

    public static final double SCAN_TIMEOUT_SECONDS = 0.7;
    public static final double AIM_TURN_TIMEOUT_SECONDS = 1.0;
    public static final double PER_SHOT_TIMEOUT_SECONDS = 4.0;

    public static final int SHOTS_AT_SHOOT_POSE_1 = 3;
    public static final int SHOTS_AT_SHOOT_POSE_2 = 3;
    public static final int SHOTS_AFTER_FLOWER = 1;

    public static final double INTAKE_SECONDS_AT_FLOWER = 1.5;

    public static final double DEFAULT_PATH_TIMEOUT_SECONDS = 6.0;

    public static final double AUTONOMOUS_BUDGET_SECONDS = 28.0;

    public static final double MIN_PATH_LENGTH_INCHES = 0.5;

    public static boolean GOAL_ONE = false;

    public static double sweepTime = 5;
}
