package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Goal Tilt Test", group = "Test")
public class GoalTiltTest extends OpMode {

    private static final String[] CLUSTERS = {
            GoalScanner.CLUSTER_RED_SCORING,
            GoalScanner.CLUSTER_RED_AUDIENCE,
            GoalScanner.CLUSTER_BLUE_SCORING,
            GoalScanner.CLUSTER_BLUE_AUDIENCE
    };

    private GoalScanner scanner;
    private int selected = 0;
    private double upRoll = Double.NaN;
    private double downRoll = Double.NaN;

    @Override
    public void init() {
        scanner = new GoalScanner(hardwareMap);
    }

    @Override
    public void init_loop() {
        run();
    }

    @Override
    public void loop() {
        run();
    }

    @Override
    public void stop() {
        scanner.close();
    }

    private static double wrap(double degrees) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)), Math.cos(Math.toRadians(degrees))));
    }

    private static double middleOffset(double up, double down) {
        double a = Math.toRadians(up), b = Math.toRadians(down + 180.0);
        return Math.toDegrees(Math.atan2(Math.sin(a) + Math.sin(b), Math.cos(a) + Math.cos(b)));
    }

    private void run() {
        scanner.update();
        if (gamepad1.dpadRightWasPressed()) {
            selected = (selected + 1) % CLUSTERS.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            selected = (selected + CLUSTERS.length - 1) % CLUSTERS.length;
        }

        GoalScanner.Sighting chosen = scanner.latest(CLUSTERS[selected]);
        boolean chosenUsable = chosen != null && chosen.isUsable();
        if (gamepad1.aWasPressed() && chosenUsable) {
            upRoll = chosen.roll;
        }
        if (gamepad1.bWasPressed() && chosenUsable) {
            downRoll = chosen.roll;
        }

        telemetry.addData("Camera", scanner.cameraState());
        if (!scanner.isAvailable()) {
            telemetry.addData("Camera fault", scanner.fault());
        }
        telemetry.addData("Calibrated", Constants.GOAL_CHECK_CALIBRATED ? "YES" : "NO (shots are never blocked)");
        telemetry.addData("Rule", "UP when |roll - %.0f| < %.0f deg",
                Constants.CAMERA_ROLL_OFFSET_DEGREES, Constants.GOAL_UP_MAX_ABS_ROLL_DEGREES);
        telemetry.addLine();

        for (int i = 0; i < CLUSTERS.length; i++) {
            GoalScanner.Sighting s = scanner.latest(CLUSTERS[i]);
            String mark = i == selected ? ">> " : "   ";
            if (s == null) {
                telemetry.addData(mark + CLUSTERS[i], "never seen   (says %s)", scanner.goalView(CLUSTERS[i]));
                continue;
            }
            telemetry.addData(mark + CLUSTERS[i], "%s  roll %.0f  %d%%  range %.0f in  bearing %.0f  %dms",
                    scanner.goalView(CLUSTERS[i]), s.roll, s.percentClusterFound, s.range, s.bearing, s.ageMs());
        }

        telemetry.addLine();
        telemetry.addData("A  UP cell roll", Double.isNaN(upRoll) ? "not saved" : String.format("%.0f", upRoll));
        telemetry.addData("B  DOWN cell roll", Double.isNaN(downRoll) ? "not saved" : String.format("%.0f", downRoll));
        if (!Double.isNaN(upRoll)) {
            double offset = Double.isNaN(downRoll) ? Math.round(upRoll / 90.0) * 90.0 : middleOffset(upRoll, downRoll);
            boolean same = Math.abs(wrap(offset - Constants.CAMERA_ROLL_OFFSET_DEGREES)) < 1.0;
            telemetry.addData(">> CAMERA_ROLL_OFFSET_DEGREES", "%.0f%s", offset, same ? "  (already set)" : "  <- change it");
            if (!Double.isNaN(downRoll)) {
                double upOff = Math.abs(wrap(upRoll - offset));
                double downOff = Math.abs(wrap(downRoll - offset));
                double limit = Constants.GOAL_UP_MAX_ABS_ROLL_DEGREES;
                boolean ok = upOff < limit && downOff >= limit;
                telemetry.addData("With that offset", "UP reads %.0f, DOWN reads %.0f (limit %.0f)  %s",
                        upOff, downOff, limit, ok ? "ok" : "!! wrong - the camera cannot tell from here");
            }
        }
        telemetry.addLine();
        telemetry.addLine("Point the robot (launcher side) at a HIVE from where you shoot.");
        telemetry.addLine("dpad left/right = pick a cell (>>). A = save while it is UP, B = save while it is DOWN.");
        telemetry.addLine("Check from 2-3 shooting spots. When UP/DOWN are always right, set GOAL_CHECK_CALIBRATED = true.");
        telemetry.addLine("Nothing moves in this OpMode.");
    }
}
