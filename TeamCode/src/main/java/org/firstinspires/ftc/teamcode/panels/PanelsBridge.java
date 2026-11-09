package org.firstinspires.ftc.teamcode.panels;

import android.content.Context;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.pedropathing.math.Pose;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.util.List;

public final class PanelsBridge {

    public static final double ROBOT_RADIUS_INCHES = 9.0;
    private static final String ROBOT_COLOR = "#3f51b5";
    private static final String ROBOT_FILL = "rgba(63, 81, 181, 0.35)";
    private static final String PATH_COLOR = "#9e9e9e";
    private static final String ACTIVE_PATH_COLOR = "#ff9800";

    private static final OpModeManagerNotifier.Notifications LISTENER = new OpModeManagerNotifier.Notifications() {
        @Override
        public void onOpModePreInit(OpMode opMode) {
            mirrorTelemetry(opMode);
        }

        @Override
        public void onOpModePreStart(OpMode opMode) {
        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
            if (opMode != null && opMode.telemetry instanceof MirroredTelemetry) {
                ((MirroredTelemetry) opMode.telemetry).flushUnsent();
            }
        }
    };

    private PanelsBridge() {
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        OpModeManagerImpl manager = eventLoop.getOpModeManager();
        if (manager == null) {
            return;
        }
        manager.unregisterListener(LISTENER);
        manager.registerListener(LISTENER);
    }

    private static void mirrorTelemetry(OpMode opMode) {
        if (opMode == null || opMode instanceof OpModeManagerImpl.DefaultOpMode) {
            return;
        }
        if (opMode.telemetry != null && !(opMode.telemetry instanceof MirroredTelemetry)) {
            opMode.telemetry = new MirroredTelemetry(opMode.telemetry);
        }
    }

    public static boolean hasViewers() {
        try {
            return Panels.INSTANCE.getClientsCount() > 0;
        } catch (Exception | LinkageError ignored) {
            return false;
        }
    }

    public static void drawRobot(Pose pose) {
        if (pose == null || !hasViewers()) {
            return;
        }
        drawRobot(pose.x(), pose.y(), pose.heading());
    }

    public static void drawRobot(double x, double y, double headingRadians) {
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(headingRadians)) {
            return;
        }
        try {
            if (!hasViewers()) {
                return;
            }
            FieldManager field = PanelsField.INSTANCE.getField();
            if (!field.getShouldUpdateCanvas()) {
                return;
            }
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            drawRobotShape(field, x, y, headingRadians);
            field.update();
        } catch (Exception | LinkageError ignored) {
        }
    }

    public static void drawPaths(List<Pose[]> paths, int highlighted, Pose robot) {
        if (paths == null) {
            return;
        }
        try {
            if (!hasViewers()) {
                return;
            }
            FieldManager field = PanelsField.INSTANCE.getField();
            if (!field.getShouldUpdateCanvas()) {
                return;
            }
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            for (int i = 0; i < paths.size(); i++) {
                if (i != highlighted) {
                    drawPolyline(field, paths.get(i), PATH_COLOR, 0.5);
                }
            }
            if (highlighted >= 0 && highlighted < paths.size()) {
                drawPolyline(field, paths.get(highlighted), ACTIVE_PATH_COLOR, 1.0);
            }
            if (robot != null && !Double.isNaN(robot.x()) && !Double.isNaN(robot.y()) && !Double.isNaN(robot.heading())) {
                drawRobotShape(field, robot.x(), robot.y(), robot.heading());
            }
            field.update();
        } catch (Exception | LinkageError ignored) {
        }
    }

    private static void drawPolyline(FieldManager field, Pose[] points, String color, double width) {
        if (points == null || points.length < 2) {
            return;
        }
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), color, width);
        for (int p = 1; p < points.length; p++) {
            field.moveCursor(points[p - 1].x(), points[p - 1].y());
            field.line(points[p].x(), points[p].y());
        }
        Pose end = points[points.length - 1];
        field.moveCursor(end.x(), end.y());
        field.circle(1.5);
    }

    private static void drawRobotShape(FieldManager field, double x, double y, double headingRadians) {
        double r = ROBOT_RADIUS_INCHES;
        field.setStyle(ROBOT_FILL, ROBOT_COLOR, 0.75);
        field.moveCursor(x, y);
        field.circle(r);
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), ROBOT_COLOR, 0.75);
        field.moveCursor(x, y);
        field.line(x + r * Math.cos(headingRadians), y + r * Math.sin(headingRadians));
    }
}
