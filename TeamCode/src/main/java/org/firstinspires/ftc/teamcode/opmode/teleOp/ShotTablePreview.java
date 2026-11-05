package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.launcher.ShotLog;
import org.firstinspires.ftc.teamcode.launcher.ShotTable;

import java.util.List;

@TeleOp(name = "Shot Table Preview", group = "Test")
public class ShotTablePreview extends OpMode {

    private static final double GRID_START_INCHES = 24.0;
    private static final double GRID_END_INCHES = 144.0;
    private static final double GRID_STEP_INCHES = 12.0;
    private static final double PROBE_SMALL_STEP = 1.0;
    private static final double PROBE_BIG_STEP = 12.0;
    private static final int LOG_LINES_SHOWN = 12;

    private double probeInches = 60.0;

    @Override
    public void init() {
        telemetry.addLine("Nothing moves in this OpMode.");
    }

    @Override
    public void init_loop() {
        show();
    }

    @Override
    public void loop() {
        show();
    }

    private void show() {
        if (gamepad1.dpadUpWasPressed()) {
            probeInches += PROBE_SMALL_STEP;
        }
        if (gamepad1.dpadDownWasPressed()) {
            probeInches -= PROBE_SMALL_STEP;
        }
        if (gamepad1.dpadRightWasPressed()) {
            probeInches += PROBE_BIG_STEP;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            probeInches -= PROBE_BIG_STEP;
        }
        probeInches = Math.max(0.0, probeInches);

        ShotTable.Fit fit = ShotTable.fit();
        telemetry.addData("Table", ShotTable.summary());
        if (fit == null) {
            telemetry.addLine("Fix ShotTable.SHOT_DATA. Until then every distance uses the default velocity.");
        } else {
            telemetry.addData("Formula", fit.formula());
            telemetry.addData("Probe (dpad)", "%.0f in -> %.0f", probeInches, ShotTable.velocityForDistance(probeInches));
            telemetry.addLine();
            telemetry.addLine("Your data:  distance  measured  ->  table  (error)");
            for (double[] row : ShotTable.SHOT_DATA) {
                double predicted = ShotTable.velocityForDistance(row[0]);
                telemetry.addLine(String.format("  %.1f in  %.0f  ->  %.0f  (%+.0f)",
                        row[0], row[1], predicted, predicted - row[1]));
            }
            telemetry.addLine();
            telemetry.addLine("Table every 12 in:");
            for (double d = GRID_START_INCHES; d <= GRID_END_INCHES; d += GRID_STEP_INCHES) {
                telemetry.addLine(String.format("  %.0f in -> %.0f", d, ShotTable.velocityForDistance(d)));
            }
        }

        telemetry.addLine();
        List<String> saved = ShotLog.last(LOG_LINES_SHOWN);
        telemetry.addData("Saved shots", "%d in %s (newest last)", ShotLog.count(), ShotLog.FILE);
        if (saved.isEmpty()) {
            telemetry.addLine("  none yet - in Robot TeleOp press gp2 BACK after a shot goes in");
        } else {
            telemetry.addLine("  copy these lines into ShotTable.SHOT_DATA:");
            for (String line : saved) {
                telemetry.addLine("  " + line);
            }
        }
        if (!ShotLog.lastError().isEmpty()) {
            telemetry.addData("File error", ShotLog.lastError());
        }
    }
}
