package org.firstinspires.ftc.teamcode.launcher;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class ShotLog {

    public static final File FILE = new File(AppUtil.FIRST_FOLDER, "shot_data.txt");
    private static final String TAG = "ShotLog";

    private static final List<String> lines = new ArrayList<>();
    private static boolean loaded = false;
    private static String lastError = "";

    private ShotLog() {
    }

    public static String format(double distance, double velocity) {
        return String.format(Locale.US, "{%.1f, %.0f},", distance, velocity);
    }

    public static synchronized String record(double distance, double velocity) {
        load();
        String line = format(distance, velocity);
        lines.add(line);
        RobotLog.ii(TAG, "recorded %s", line);
        save();
        return line;
    }

    public static synchronized String undo() {
        load();
        if (lines.isEmpty()) {
            return null;
        }
        String removed = lines.remove(lines.size() - 1);
        RobotLog.ii(TAG, "removed %s", removed);
        save();
        return removed;
    }

    public static synchronized List<String> all() {
        load();
        return Collections.unmodifiableList(new ArrayList<>(lines));
    }

    public static synchronized List<String> last(int count) {
        load();
        int from = Math.max(0, lines.size() - count);
        return new ArrayList<>(lines.subList(from, lines.size()));
    }

    public static synchronized int count() {
        load();
        return lines.size();
    }

    public static synchronized String lastError() {
        return lastError;
    }

    private static void load() {
        if (loaded) {
            return;
        }
        lines.clear();
        if (!FILE.exists()) {
            loaded = true;
            return;
        }
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(new FileInputStream(FILE), StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String trimmed = line.trim();
                if (!trimmed.isEmpty()) {
                    lines.add(trimmed);
                }
            }
        } catch (IOException | RuntimeException e) {
            lines.clear();
            lastError = "read " + FILE + ": " + e.getMessage();
            RobotLog.ee(TAG, lastError);
            return;
        }
        loaded = true;
    }

    private static void save() {
        if (!loaded) {
            lastError = "not saving: could not read " + FILE + " (" + lastError + ")";
            return;
        }
        try (Writer writer = new OutputStreamWriter(new FileOutputStream(FILE, false), StandardCharsets.UTF_8)) {
            for (String line : lines) {
                writer.write(line);
                writer.write("\n");
            }
            lastError = "";
        } catch (IOException | RuntimeException e) {
            lastError = "write " + FILE + ": " + e.getMessage();
            RobotLog.ee(TAG, lastError);
        }
    }
}
