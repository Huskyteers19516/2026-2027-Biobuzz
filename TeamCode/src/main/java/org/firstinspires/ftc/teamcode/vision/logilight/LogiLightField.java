package org.firstinspires.ftc.teamcode.vision.logilight;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vision.logilight.math.FieldTargetState;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;

import java.util.List;

public final class LogiLightField {

    public static final double ROBOT_RADIUS_INCHES = 9.0;
    public static final double TARGET_RADIUS_INCHES = 2.5;
    public static final double TARGET_NORMAL_LENGTH_INCHES = 6.0;
    public static final double OUTLINE_WIDTH = 0.75;
    public static final double MIN_DRAWN_NORMAL = 0.17;

    public static final String ODOMETRY_OUTLINE = "#2196f3";
    public static final String ODOMETRY_FILL = "rgba(33, 150, 243, 0.25)";
    public static final String VISION_OUTLINE = "#ff9800";
    public static final String VISION_FILL = "rgba(255, 152, 0, 0.25)";
    public static final String FUSED_OUTLINE = "#4caf50";
    public static final String FUSED_FILL = "rgba(76, 175, 80, 0.30)";
    public static final String SURVEYED_TARGET_OUTLINE = "#e91e63";
    public static final String SURVEYED_TARGET_FILL = "rgba(233, 30, 99, 0.45)";
    public static final String PLACEHOLDER_TARGET_OUTLINE = "#9e9e9e";
    public static final String PLACEHOLDER_TARGET_FILL = "rgba(158, 158, 158, 0.35)";
    public static final String SIGHT_LINE = "rgba(255, 152, 0, 0.6)";
    public static final double POLLEN_RADIUS_INCHES = 1.4;
    public static final String POLLEN_OUTLINE = "#fdd835";
    public static final String POLLEN_FILL = "rgba(253, 216, 53, 0.75)";
    public static final String SEEK_APPROACH_OUTLINE = "#4caf50";
    public static final String SEEK_APPROACH_FILL = "rgba(76, 175, 80, 0.15)";
    public static final String SEEK_PATH_LINE = "rgba(76, 175, 80, 0.8)";

    private LogiLightField() {
    }

    public static boolean hasViewers() {
        if (LogiLightSettings.EVENT_MODE) {
            return false;
        }
        try {
            return Panels.INSTANCE.getClientsCount() > 0;
        } catch (Exception | LinkageError ignored) {
            return false;
        }
    }

    public static void drawPoses(Pose odometry, Pose vision, Pose fused) {
        drawPoses(odometry, vision, fused, null);
    }

    public static void drawPoses(Pose odometry, Pose vision, Pose fused, LogiFieldMap map) {
        if (!hasViewers()) {
            return;
        }
        try {
            FieldManager field = PanelsField.INSTANCE.getField();
            if (!field.getShouldUpdateCanvas()) {
                return;
            }
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            drawTargetsOn(field, map);
            drawRobotOn(field, odometry, ODOMETRY_FILL, ODOMETRY_OUTLINE);
            drawRobotOn(field, vision, VISION_FILL, VISION_OUTLINE);
            drawRobotOn(field, fused, FUSED_FILL, FUSED_OUTLINE);
            field.update();
        } catch (Exception | LinkageError ignored) {
        }
    }

    public static void drawResult(Pose odometry, LogiResult result, Pose fused, LogiFieldMap map) {
        if (!hasViewers()) {
            return;
        }
        try {
            FieldManager field = PanelsField.INSTANCE.getField();
            if (!field.getShouldUpdateCanvas()) {
                return;
            }
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            drawTargetsOn(field, map);
            drawRobotOn(field, odometry, ODOMETRY_FILL, ODOMETRY_OUTLINE);
            LogiBotPose best = result == null ? null : result.getBestBotpose();
            if (best != null && best.pose != null && best.pose.isPositionFinite()) {
                PlanarPose vision = best.pose;
                double heading = vision.hasHeading() ? vision.heading : Double.NaN;
                if (map != null) {
                    for (String name : best.targetNames) {
                        LogiFieldMap.Target target = map.find(name);
                        FieldTargetState state = target == null ? null : target.firstState();
                        if (state != null && state.originField.isFinite()) {
                            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), SIGHT_LINE, OUTLINE_WIDTH);
                            field.moveCursor(vision.x, vision.y);
                            field.line(state.originField.x, state.originField.y);
                        }
                    }
                }
                drawRobotOn(field, vision.x, vision.y, heading, VISION_FILL, VISION_OUTLINE);
            }
            drawRobotOn(field, fused, FUSED_FILL, FUSED_OUTLINE);
            field.update();
        } catch (Exception | LinkageError ignored) {
        }
    }

    public static void drawTargets(LogiFieldMap map) {
        if (map == null || !hasViewers()) {
            return;
        }
        try {
            FieldManager field = PanelsField.INSTANCE.getField();
            if (!field.getShouldUpdateCanvas()) {
                return;
            }
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            drawTargetsOn(field, map);
            field.update();
        } catch (Exception | LinkageError ignored) {
        }
    }

    public static void drawSeek(Pose robot, double ballX, double ballY, Pose approach, List<Pose> via) {
        if (!hasViewers()) {
            return;
        }
        try {
            FieldManager field = PanelsField.INSTANCE.getField();
            if (!field.getShouldUpdateCanvas()) {
                return;
            }
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            if (robot != null && approach != null) {
                field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), SEEK_PATH_LINE, OUTLINE_WIDTH);
                double fromX = robot.x();
                double fromY = robot.y();
                if (via != null) {
                    for (Pose point : via) {
                        if (point == null) {
                            continue;
                        }
                        field.moveCursor(fromX, fromY);
                        field.line(point.x(), point.y());
                        fromX = point.x();
                        fromY = point.y();
                    }
                }
                field.moveCursor(fromX, fromY);
                field.line(approach.x(), approach.y());
            }
            if (approach != null) {
                drawRobotOn(field, approach, SEEK_APPROACH_FILL, SEEK_APPROACH_OUTLINE);
            }
            if (!Double.isNaN(ballX) && !Double.isNaN(ballY) && !Double.isInfinite(ballX) && !Double.isInfinite(ballY)) {
                field.setStyle(POLLEN_FILL, POLLEN_OUTLINE, OUTLINE_WIDTH);
                field.moveCursor(ballX, ballY);
                field.circle(POLLEN_RADIUS_INCHES);
            }
            drawRobotOn(field, robot, ODOMETRY_FILL, ODOMETRY_OUTLINE);
            field.update();
        } catch (Exception | LinkageError ignored) {
        }
    }

    private static void drawTargetsOn(FieldManager field, LogiFieldMap map) {
        if (map == null) {
            return;
        }
        for (LogiFieldMap.Target target : map.targets()) {
            String fill = target.surveyed ? SURVEYED_TARGET_FILL : PLACEHOLDER_TARGET_FILL;
            String outline = target.surveyed ? SURVEYED_TARGET_OUTLINE : PLACEHOLDER_TARGET_OUTLINE;
            for (FieldTargetState state : target.states) {
                Vec3 origin = state.originField;
                if (origin == null || !origin.isFinite()) {
                    continue;
                }
                field.setStyle(fill, outline, OUTLINE_WIDTH);
                field.moveCursor(origin.x, origin.y);
                field.circle(TARGET_RADIUS_INCHES);
                Vec3 normal = state.normalInField();
                double horizontal = Math.hypot(normal.x, normal.y);
                if (horizontal > MIN_DRAWN_NORMAL) {
                    field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), outline, OUTLINE_WIDTH);
                    field.moveCursor(origin.x, origin.y);
                    field.line(origin.x - TARGET_NORMAL_LENGTH_INCHES * normal.x / horizontal,
                            origin.y - TARGET_NORMAL_LENGTH_INCHES * normal.y / horizontal);
                }
            }
        }
    }

    private static void drawRobotOn(FieldManager field, Pose pose, String fill, String outline) {
        if (pose == null) {
            return;
        }
        drawRobotOn(field, pose.x(), pose.y(), pose.heading(), fill, outline);
    }

    private static void drawRobotOn(FieldManager field, double x, double y, double heading, String fill, String outline) {
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isInfinite(x) || Double.isInfinite(y)) {
            return;
        }
        field.setStyle(fill, outline, OUTLINE_WIDTH);
        field.moveCursor(x, y);
        field.circle(ROBOT_RADIUS_INCHES);
        if (!Double.isNaN(heading) && !Double.isInfinite(heading)) {
            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), outline, OUTLINE_WIDTH);
            field.moveCursor(x, y);
            field.line(x + ROBOT_RADIUS_INCHES * Math.cos(heading), y + ROBOT_RADIUS_INCHES * Math.sin(heading));
        }
    }
}
