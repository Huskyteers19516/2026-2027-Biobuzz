package org.firstinspires.ftc.teamcode.vision.logilight;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

@Configurable
public final class LogiLightSettings {

    public static LogiCameraConfig CAMERA = new LogiCameraConfig();
    public static LogiPipeline[] PIPELINES = LogiPipeline.defaults();
    public static ColorProfile[] COLOR_PROFILES = {ColorProfile.pollenYellow(), ColorProfile.nectarRed(), ColorProfile.nectarBlue()};
    public static LogiLocalizationConfig LOCALIZATION = new LogiLocalizationConfig();
    public static LogiAimConfig AIM = new LogiAimConfig();
    public static int DEFAULT_PIPELINE = 0;
    public static StreamView STREAM_VIEW = StreamView.ANNOTATED;
    public static boolean PANELS_STREAM = true;
    public static boolean DRIVER_STATION_STREAM = true;

    public static final boolean EVENT_MODE = false;
    public static final boolean LOAD_SAVED_SETTINGS = true;
    public static final int SETTINGS_VERSION = 1;
    public static final String STORAGE_DIRECTORY = "/sdcard/FIRST/logilight";

    private static final AtomicLong REVISION = new AtomicLong(0L);

    private LogiLightSettings() {
    }

    public static ColorProfile[] defaultColorProfiles() {
        return new ColorProfile[]{ColorProfile.pollenYellow(), ColorProfile.nectarRed(), ColorProfile.nectarBlue()};
    }

    public static ColorProfile findColorProfile(String name) {
        String wanted = trimmed(name);
        if (wanted.isEmpty()) {
            return null;
        }
        ColorProfile[] profiles = COLOR_PROFILES;
        if (profiles == null) {
            return null;
        }
        for (ColorProfile profile : profiles) {
            if (profile != null && wanted.equalsIgnoreCase(trimmed(profile.name))) {
                return profile;
            }
        }
        return null;
    }

    public static int findPipeline(String name) {
        String wanted = trimmed(name);
        if (wanted.isEmpty()) {
            return -1;
        }
        LogiPipeline[] pipelines = PIPELINES;
        if (pipelines == null) {
            return -1;
        }
        for (int i = 0; i < pipelines.length; i++) {
            LogiPipeline pipeline = pipelines[i];
            if (pipeline != null && wanted.equalsIgnoreCase(trimmed(pipeline.name))) {
                return i;
            }
        }
        return -1;
    }

    public static void resetToCodeDefaults() {
        boolean panelsNeedsRefresh;
        synchronized (LogiLightSettings.class) {
            int previousPipelineCount = lengthOf(PIPELINES);
            int previousProfileCount = lengthOf(COLOR_PROFILES);
            CAMERA = new LogiCameraConfig();
            PIPELINES = LogiPipeline.defaults();
            COLOR_PROFILES = defaultColorProfiles();
            LOCALIZATION = new LogiLocalizationConfig();
            AIM = new LogiAimConfig();
            DEFAULT_PIPELINE = 0;
            STREAM_VIEW = StreamView.ANNOTATED;
            PANELS_STREAM = true;
            DRIVER_STATION_STREAM = true;
            LogiLightStore.noteSettingsSource(LogiLightStore.CODE_DEFAULTS_SOURCE + " (reset)");
            panelsNeedsRefresh = settingsReplaced(previousPipelineCount, previousProfileCount);
        }
        if (panelsNeedsRefresh) {
            refreshPanels();
        }
    }

    public static int pipelineCount() {
        return lengthOf(PIPELINES);
    }

    public static LogiPipeline pipelineAt(int index) {
        LogiPipeline[] pipelines = PIPELINES;
        if (pipelines == null || index < 0 || index >= pipelines.length) {
            return null;
        }
        return pipelines[index];
    }

    public static int defaultPipelineIndex() {
        int index = DEFAULT_PIPELINE;
        int count = pipelineCount();
        if (count == 0) {
            return 0;
        }
        return index >= 0 && index < count ? index : 0;
    }

    public static ColorProfile[] resolveColorProfiles(LogiPipeline pipeline) {
        if (pipeline == null) {
            return new ColorProfile[0];
        }
        List<ColorProfile> resolved = new ArrayList<ColorProfile>();
        for (String profileName : pipeline.colorProfileNames()) {
            ColorProfile profile = findColorProfile(profileName);
            if (profile != null && !resolved.contains(profile)) {
                resolved.add(profile);
            }
        }
        return resolved.toArray(new ColorProfile[0]);
    }

    public static List<String> missingColorProfiles(LogiPipeline pipeline) {
        if (pipeline == null) {
            return Collections.emptyList();
        }
        List<String> missing = new ArrayList<String>();
        for (String profileName : pipeline.colorProfileNames()) {
            if (findColorProfile(profileName) == null) {
                missing.add(profileName);
            }
        }
        return missing;
    }

    public static boolean panelsStreamEnabled() {
        return PANELS_STREAM && !EVENT_MODE;
    }

    public static long revision() {
        return REVISION.get();
    }

    public static void refreshPanels() {
        try {
            PanelsConfigurables.refreshClass(LogiLightSettings.class);
        } catch (Throwable ignored) {
        }
    }

    static boolean settingsReplaced(int previousPipelineCount, int previousProfileCount) {
        REVISION.incrementAndGet();
        return previousPipelineCount != lengthOf(PIPELINES) || previousProfileCount != lengthOf(COLOR_PROFILES);
    }

    private static int lengthOf(Object[] array) {
        return array == null ? 0 : array.length;
    }

    private static String trimmed(String text) {
        return text == null ? "" : text.trim();
    }
}
