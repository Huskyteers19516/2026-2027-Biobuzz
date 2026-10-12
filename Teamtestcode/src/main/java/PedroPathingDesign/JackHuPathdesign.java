package PedroPathingDesign;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers.FieldPoint;

import java.util.ArrayList;
import java.util.List;

public class JackHuPathdesign {

    // red alliance, points read off the Pedro visualizer field, not measured yet
    public static final boolean POINTS_CONFIRMED = false;

    public static final Pose START_POSE = new Pose(61.75, 8.0, Math.toRadians(90.0));

    public static final Pose SHOOT_POSE_SOUTH = new Pose(61.75, 30.0, Math.toRadians(90.0));
    public static final Pose SHOOT_POSE_NORTH = new Pose(61.75, 114.0, Math.toRadians(-90.0));

    public static final Pose WALL_FLOWER_START = new Pose(30.0, 34.0, Math.toRadians(180.0));
    public static final Pose WALL_FLOWER_END = new Pose(13.0, 46.0, Math.toRadians(180.0));
    public static final Pose TOP_FLOWER_START = new Pose(48.0, 118.0, Math.toRadians(90.0));
    public static final Pose TOP_FLOWER_END = new Pose(48.0, 131.0, Math.toRadians(90.0));

    // side lane between the red wall and the HIVE, used to get past the HIVE
    public static final Pose LANE_SOUTH = new Pose(24.0, 34.0, Math.toRadians(90.0));
    public static final Pose LANE_NORTH = new Pose(24.0, 110.0, Math.toRadians(90.0));

    public static final Pose PARK_POSE = new Pose(11.0, 108.0, Math.toRadians(180.0));

    public static final double AIM_AT_SHOOT_POSE_SOUTH = 0.0;
    public static final double AIM_AT_SHOOT_POSE_NORTH = 0.0;

    public static final int SHOTS_PRELOAD = 4;
    public static final int SHOTS_AFTER_FLOWER = 1;

    public static final double INTAKE_SECONDS_AT_FLOWER = 0.0;

    public static List<FieldPoint.Step> mission() {
        List<FieldPoint.Step> steps = new ArrayList<FieldPoint.Step>();

        steps.add(FieldPoint.shoot("Shoot preload at south shooting point", SHOOT_POSE_SOUTH, SHOTS_PRELOAD, AIM_AT_SHOOT_POSE_SOUTH));

        steps.add(FieldPoint.move("Move to wall flower start", WALL_FLOWER_START));
        steps.add(FieldPoint.collect("Sweep wall flower", WALL_FLOWER_END, INTAKE_SECONDS_AT_FLOWER));
        steps.add(FieldPoint.shoot("Shoot wall flower at south shooting point", SHOOT_POSE_SOUTH, SHOTS_AFTER_FLOWER, AIM_AT_SHOOT_POSE_SOUTH));

        steps.add(FieldPoint.move("Move to side lane", LANE_SOUTH));
        steps.add(FieldPoint.move("Drive past the HIVE", LANE_NORTH));

        steps.add(FieldPoint.move("Move to top flower start", TOP_FLOWER_START));
        steps.add(FieldPoint.collect("Sweep top flower", TOP_FLOWER_END, INTAKE_SECONDS_AT_FLOWER));
        steps.add(FieldPoint.shoot("Shoot top flower at north shooting point", SHOOT_POSE_NORTH, SHOTS_AFTER_FLOWER, AIM_AT_SHOOT_POSE_NORTH));

        steps.add(FieldPoint.park("Park", PARK_POSE));

        return steps;
    }
}
