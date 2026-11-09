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

public final class PanelsBridge {

    public static final double ROBOT_RADIUS_INCHES = 9.0;
    private static final String ROBOT_COLOR = "#3f51b5";
    private static final String ROBOT_FILL = "rgba(63, 81, 181, 0.35)";

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
            double r = ROBOT_RADIUS_INCHES;
            field.setStyle(ROBOT_FILL, ROBOT_COLOR, 0.75);
            field.moveCursor(x, y);
            field.circle(r);
            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), ROBOT_COLOR, 0.75);
            field.moveCursor(x, y);
            field.line(x + r * Math.cos(headingRadians), y + r * Math.sin(headingRadians));
            field.update();
        } catch (Exception | LinkageError ignored) {
        }
    }
}
