package org.firstinspires.ftc.teamcode.opmode.autolib;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;

public enum Spot {

    START(FieldPoints.START_POSE, FieldPointsBlue.START_POSE, false),

    SHOOT_1(FieldPoints.SHOOT_POSE_1, FieldPointsBlue.SHOOT_POSE_1, false),
    SHOOT_2(FieldPoints.SHOOT_POSE_2, FieldPointsBlue.SHOOT_POSE_2, false),

    FLOWER_1_START(FieldPoints.FLOWER_1_START, FieldPointsBlue.FLOWER_1_START, false),
    FLOWER_1_END(FieldPoints.FLOWER_1_END, FieldPointsBlue.FLOWER_1_END, false),
    FLOWER_2_START(FieldPoints.FLOWER_2_START, FieldPointsBlue.FLOWER_2_START, false),
    FLOWER_2_END(FieldPoints.FLOWER_2_END, FieldPointsBlue.FLOWER_2_END, false),
    FLOWER_3_START(FieldPoints.FLOWER_3_START, FieldPointsBlue.FLOWER_3_START, false),
    FLOWER_3_END(FieldPoints.FLOWER_3_END, FieldPointsBlue.FLOWER_3_END, false),
    FLOWER_4_START(FieldPoints.FLOWER_4_START, FieldPointsBlue.FLOWER_4_START, false),
    FLOWER_4_END(FieldPoints.FLOWER_4_END, FieldPointsBlue.FLOWER_4_END, false),

    PARK(FieldPoints.PARK_POSE, FieldPointsBlue.PARK_POSE, false),

    ANGLE_1(FieldPoints.ANGLE_1, FieldPointsBlue.ANGLE_1, false),
    ANGLE_2(FieldPoints.ANGLE_2, FieldPointsBlue.ANGLE_2, false),
    ANGLE_3(FieldPoints.ANGLE_3, FieldPointsBlue.ANGLE_3, false),
    ANGLE_4(FieldPoints.ANGLE_4, FieldPointsBlue.ANGLE_4, false),

    RACK_AROUND_1(FieldPoints.RACK_AROUND_1, FieldPointsBlue.RACK_AROUND_1, true),
    RACK_AROUND_2(FieldPoints.RACK_AROUND_2, FieldPointsBlue.RACK_AROUND_2, true),

    UNDER_RACK_ENTRY(FieldPoints.UNDER_RACK_CONTROL_POINT_2, FieldPointsBlue.UNDER_RACK_CONTROL_POINT_1, true),
    UNDER_RACK_EXIT(FieldPoints.UNDER_RACK_CONTROL_POINT_1, FieldPointsBlue.UNDER_RACK_CONTROL_POINT_2, true);

    private final Pose red;
    private final Pose blue;
    private final boolean parkRoutePoint;

    Spot(Pose red, Pose blue, boolean parkRoutePoint) {
        this.red = red;
        this.blue = blue;
        this.parkRoutePoint = parkRoutePoint;
    }

    public Pose pose(GoalScanner.Alliance alliance) {
        return GoalAim.isBlue(alliance) ? blue : red;
    }

    public boolean confirmed(GoalScanner.Alliance alliance) {
        boolean isBlue = GoalAim.isBlue(alliance);
        if (parkRoutePoint) {
            return isBlue ? FieldPointsBlue.PARK_ROUTE_CONFIRMED : FieldPoints.PARK_ROUTE_CONFIRMED;
        }
        return isBlue ? FieldPointsBlue.POINTS_CONFIRMED : FieldPoints.POINTS_CONFIRMED;
    }

    public String confirmFlag() {
        return parkRoutePoint ? "PARK_ROUTE_CONFIRMED" : "POINTS_CONFIRMED";
    }

    public static String pointsFile(GoalScanner.Alliance alliance) {
        return GoalAim.isBlue(alliance) ? "FieldPointsBlue.java" : "FieldPoints.java";
    }
}
