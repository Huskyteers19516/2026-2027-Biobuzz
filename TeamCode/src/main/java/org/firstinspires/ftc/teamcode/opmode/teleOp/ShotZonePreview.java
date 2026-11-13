package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.launcher.Launcher;
import org.firstinspires.ftc.teamcode.launcher.ShotLog;
import org.firstinspires.ftc.teamcode.launcher.ShotZones;

import java.util.List;

@TeleOp(name = "Shot Zone Preview", group = "Test")
public class ShotZonePreview extends OpMode {

    private static final double GRID_START_INCHES = 34.0;
    private static final double GRID_END_INCHES = 66.0;
    private static final double GRID_STEP_INCHES = 2.0;
    private static final double PROBE_SMALL_STEP = 0.5;
    private static final double PROBE_BIG_STEP = 4.0;
    private static final int LOG_LINES_SHOWN = 12;

    private double probeInches = 45.0;

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

        String problem = ShotZones.problem();
        if (problem != null) {
            telemetry.addLine("!! " + problem);
            telemetry.addLine(String.format("Fix ShotZones.ZONES. Until then every distance uses %.0f.",
                    Launcher.DEFAULT_TARGET_VELOCITY));
        } else {
            telemetry.addLine("Zones (distance = robot center to goal point):");
            for (ShotZones.Zone zone : ShotZones.zones()) {
                telemetry.addLine("  " + zone.describe());
            }
            telemetry.addData("Burst", "+%.0f on top while firing 3", Launcher.DEFAULT_BURST_BOOST);
            telemetry.addData("Probe (dpad)", ShotZones.lookup(probeInches).describe());
            telemetry.addLine();
            telemetry.addLine(String.format("Every %.0f in:", GRID_STEP_INCHES));
            for (double d = GRID_START_INCHES; d <= GRID_END_INCHES; d += GRID_STEP_INCHES) {
                telemetry.addLine("  " + ShotZones.lookup(d).describe());
            }
        }

        telemetry.addLine();
        List<String> saved = ShotLog.last(LOG_LINES_SHOWN);
        telemetry.addData("Saved shots", "%d in %s (newest last)", ShotLog.count(), ShotLog.FILE);
        if (saved.isEmpty()) {
            telemetry.addLine("  none yet - in Robot TeleOp press gp2 BACK after a shot goes in");
        } else {
            telemetry.addLine("  {distance, velocity} of shots that went in - use them to add or move zones:");
            for (String line : saved) {
                telemetry.addLine("  " + line);
            }
        }
        if (!ShotLog.lastError().isEmpty()) {
            telemetry.addData("File error", ShotLog.lastError());
        }
    }
}
