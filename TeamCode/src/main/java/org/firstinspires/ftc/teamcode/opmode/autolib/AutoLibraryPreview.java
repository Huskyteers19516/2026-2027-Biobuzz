package org.firstinspires.ftc.teamcode.opmode.autolib;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.panels.PanelsBridge;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "Auto Library Preview", group = "Library")
public class AutoLibraryPreview extends LinearOpMode {

    private boolean lastLeft;
    private boolean lastRight;
    private boolean lastUp;
    private boolean lastDown;
    private boolean lastA;

    @Override
    public void runOpMode() {
        List<AutoLibrary.Leg> legs = AutoLibrary.get().all();
        GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
        int index = 0;
        boolean showAll = false;

        while (opModeInInit() || opModeIsActive()) {
            if (gamepad1.x && !gamepad1.start) {
                alliance = GoalScanner.Alliance.BLUE;
            }
            if (gamepad1.b && !gamepad1.start) {
                alliance = GoalScanner.Alliance.RED;
            }
            if (pressed(gamepad1.dpad_right, lastRight)) {
                index = (index + 1) % legs.size();
            }
            if (pressed(gamepad1.dpad_left, lastLeft)) {
                index = (index - 1 + legs.size()) % legs.size();
            }
            if (pressed(gamepad1.dpad_up, lastUp)) {
                index = nextSection(legs, index, 1);
            }
            if (pressed(gamepad1.dpad_down, lastDown)) {
                index = nextSection(legs, index, -1);
            }
            if (pressed(gamepad1.a, lastA)) {
                showAll = !showAll;
            }
            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastA = gamepad1.a;

            AutoLibrary.Leg leg = legs.get(index);
            Pose[] points = leg.poses(alliance);
            if (showAll) {
                List<Pose[]> everything = new ArrayList<>();
                for (AutoLibrary.Leg other : legs) {
                    everything.add(other.poses(alliance));
                }
                PanelsBridge.drawPaths(everything, index, points[0]);
            } else {
                PanelsBridge.drawPaths(Collections.singletonList(points), 0, points[0]);
            }

            telemetry.addLine("Auto Library Preview  (robot does not move)");
            telemetry.addData("ALLIANCE", "%s   (X = BLUE, B = RED)", alliance);
            telemetry.addData("PATH NUMBER", "#%d   (path(%d) in MatchAuto)", leg.number, leg.number);
            telemetry.addData("Section", leg.section);
            telemetry.addData("Path", leg.describe());
            telemetry.addData("Entry", "%d / %d", index + 1, legs.size());
            telemetry.addData("Length", "%.1f in", leg.length(alliance));
            telemetry.addData("Timeout", "%.1f s", leg.timeoutSeconds());
            List<Spot> spots = leg.spots();
            for (int i = 0; i < spots.size(); i++) {
                Spot spot = spots.get(i);
                telemetry.addData(spot.name(), "%s%s", GoalAim.formatPose(points[i]),
                        spot.confirmed(alliance) ? "" : "  (" + spot.confirmFlag() + " = false)");
            }
            telemetry.addLine();
            telemetry.addLine("Dpad left/right = previous/next leg");
            telemetry.addLine("Dpad up/down = next/previous section");
            telemetry.addData("A = show all legs", showAll ? "ON" : "OFF");
            telemetry.addLine("Open Panels > Field to see the path");
            telemetry.update();
        }
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private static int nextSection(List<AutoLibrary.Leg> legs, int index, int direction) {
        String current = legs.get(index).section;
        int i = index;
        for (int step = 0; step < legs.size(); step++) {
            i = (i + direction + legs.size()) % legs.size();
            if (!legs.get(i).section.equals(current)) {
                String target = legs.get(i).section;
                while (direction < 0) {
                    int before = (i - 1 + legs.size()) % legs.size();
                    if (!legs.get(before).section.equals(target)) {
                        break;
                    }
                    i = before;
                }
                return i;
            }
        }
        return index;
    }
}
