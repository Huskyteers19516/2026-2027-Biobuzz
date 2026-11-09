package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class LogiPipeline {

    public String name = "goals";
    public PipelineType type = PipelineType.APRILTAG;
    public String colorProfiles = "";
    public double exposureMs = -1.0;
    public int gain = -1;
    public double decimation = -1.0;
    public String priorityTargets = "";
    public TargetSort fiducialSort = TargetSort.CLOSEST_TO_CROSSHAIR;
    public boolean localization = true;
    public double aimOffsetX = 0.0;
    public double aimOffsetY = 0.0;
    public double aimOffsetZ = 0.0;
    public String scriptName = "";

    public LogiPipeline() {
    }

    public LogiPipeline copy() {
        LogiPipeline copy = new LogiPipeline();
        copy.name = name;
        copy.type = type;
        copy.colorProfiles = colorProfiles;
        copy.exposureMs = exposureMs;
        copy.gain = gain;
        copy.decimation = decimation;
        copy.priorityTargets = priorityTargets;
        copy.fiducialSort = fiducialSort;
        copy.localization = localization;
        copy.aimOffsetX = aimOffsetX;
        copy.aimOffsetY = aimOffsetY;
        copy.aimOffsetZ = aimOffsetZ;
        copy.scriptName = scriptName;
        return copy;
    }

    public static LogiPipeline goals() {
        LogiPipeline pipeline = new LogiPipeline();
        pipeline.name = "goals";
        pipeline.type = PipelineType.APRILTAG;
        pipeline.localization = true;
        return pipeline;
    }

    public static LogiPipeline color(String name, String profiles) {
        LogiPipeline pipeline = new LogiPipeline();
        pipeline.name = safe(name);
        pipeline.type = PipelineType.COLOR;
        pipeline.colorProfiles = safe(profiles);
        pipeline.localization = false;
        return pipeline;
    }

    public static LogiPipeline combo(String name, String profiles) {
        LogiPipeline pipeline = new LogiPipeline();
        pipeline.name = safe(name);
        pipeline.type = PipelineType.APRILTAG_AND_COLOR;
        pipeline.colorProfiles = safe(profiles);
        pipeline.localization = true;
        return pipeline;
    }

    public static LogiPipeline script(String name, String scriptName) {
        LogiPipeline pipeline = new LogiPipeline();
        pipeline.name = safe(name);
        pipeline.type = PipelineType.SCRIPT;
        pipeline.scriptName = safe(scriptName);
        pipeline.localization = false;
        return pipeline;
    }

    public static LogiPipeline driver() {
        LogiPipeline pipeline = new LogiPipeline();
        pipeline.name = "driver";
        pipeline.type = PipelineType.DRIVER;
        pipeline.localization = false;
        return pipeline;
    }

    public static LogiPipeline[] defaults() {
        return new LogiPipeline[]{
                goals(),
                color("pollen-yellow", "yellow"),
                color("nectar-red", "red"),
                color("nectar-blue", "blue"),
                combo("goals+balls", "yellow,red,blue"),
                script("color-coverage", "color-coverage"),
                driver()
        };
    }

    public List<String> colorProfileNames() {
        return splitNames(colorProfiles);
    }

    public List<String> priorityTargetNames() {
        return splitNames(priorityTargets);
    }

    public int priorityRank(String targetName) {
        if (targetName == null) {
            return Integer.MAX_VALUE;
        }
        String wanted = targetName.trim();
        List<String> names = priorityTargetNames();
        for (int i = 0; i < names.size(); i++) {
            if (names.get(i).equalsIgnoreCase(wanted)) {
                return i;
            }
        }
        return Integer.MAX_VALUE;
    }

    public boolean usesColorProfile(String profileName) {
        if (profileName == null) {
            return false;
        }
        String wanted = profileName.trim();
        for (String candidate : colorProfileNames()) {
            if (candidate.equalsIgnoreCase(wanted)) {
                return true;
            }
        }
        return false;
    }

    public boolean overridesExposure() {
        return exposureMs > 0.0 && !Double.isInfinite(exposureMs);
    }

    public boolean overridesGain() {
        return gain >= 0;
    }

    public boolean overridesDecimation() {
        return decimation > 0.0 && !Double.isInfinite(decimation);
    }

    public double effectiveExposureMs(LogiCameraConfig camera) {
        if (overridesExposure() || camera == null) {
            return exposureMs;
        }
        return camera.exposureMs;
    }

    public int effectiveGain(LogiCameraConfig camera) {
        if (overridesGain() || camera == null) {
            return gain;
        }
        return camera.gain;
    }

    public double effectiveDecimation(LogiCameraConfig camera) {
        if (overridesDecimation() || camera == null) {
            return decimation;
        }
        return camera.decimation;
    }

    public boolean hasAimOffset() {
        return aimOffsetX != 0.0 || aimOffsetY != 0.0 || aimOffsetZ != 0.0;
    }

    public PipelineType typeOrDefault() {
        return type == null ? PipelineType.APRILTAG : type;
    }

    public static List<String> splitNames(String commaSeparated) {
        if (commaSeparated == null || commaSeparated.trim().isEmpty()) {
            return Collections.emptyList();
        }
        List<String> names = new ArrayList<String>();
        for (String part : commaSeparated.split(",")) {
            String trimmed = part.trim();
            if (trimmed.isEmpty()) {
                continue;
            }
            boolean duplicate = false;
            for (String existing : names) {
                if (existing.equalsIgnoreCase(trimmed)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                names.add(trimmed);
            }
        }
        return Collections.unmodifiableList(names);
    }

    private static String safe(String text) {
        return text == null ? "" : text.trim();
    }
}
