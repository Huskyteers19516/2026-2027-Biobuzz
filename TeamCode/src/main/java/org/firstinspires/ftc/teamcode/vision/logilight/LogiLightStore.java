package org.firstinspires.ftc.teamcode.vision.logilight;

import org.firstinspires.ftc.teamcode.vision.logilight.math.FieldTargetState;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Mat3;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.SurveyInfo;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Array;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class LogiLightStore {

    public static final String SETTINGS_FILE_NAME = "settings.json";
    public static final String FIELD_MAP_FILE_NAME = "fieldmap.json";
    public static final String SNAPSHOT_DIRECTORY_NAME = "snapshots";
    public static final int FIELD_MAP_VERSION = 1;
    public static final String TIMESTAMP_PATTERN = "yyyy-MM-dd HH:mm:ss";
    public static final String CODE_DEFAULTS_SOURCE = "code defaults";
    public static final String BACKUP_SUFFIX = ".bak";
    public static final String REJECTED_SUFFIX = ".bad-";
    public static final String REJECTED_TIME_PATTERN = "yyyyMMdd_HHmmss";

    private static final long MAX_FILE_BYTES = 4L * 1024L * 1024L;
    private static final int MAX_NESTING_DEPTH = 8;
    private static final int JSON_INDENT_SPACES = 2;
    private static final int READ_CHUNK_BYTES = 8192;
    private static final double FIELD_MAP_ROUNDING_SCALE = 10000.0;
    private static final double GIMBAL_LOCK_SINE = 0.999999;
    private static final String TEMP_SUFFIX = ".tmp";
    private static final String BYTE_ORDER_MARK = "\uFEFF";
    private static final Charset UTF_8 = Charset.forName("UTF-8");
    private static final Object NO_VALUE = new Object();
    private static final Object FILE_LOCK = new Object();
    private static final Object LOAD_ONCE_LOCK = new Object();

    private static volatile String lastError = "";
    private static volatile String settingsSource = CODE_DEFAULTS_SOURCE;
    private static volatile boolean settingsFileRejected = false;
    private static volatile boolean fieldMapFileRejected = false;
    private static boolean settingsLoadAttempted = false;
    private static String firstLoadMessage = "";

    private LogiLightStore() {
    }

    public static File storageDirectory() {
        return new File(LogiLightSettings.STORAGE_DIRECTORY);
    }

    public static File settingsFile() {
        return new File(storageDirectory(), SETTINGS_FILE_NAME);
    }

    public static File fieldMapFile() {
        return new File(storageDirectory(), FIELD_MAP_FILE_NAME);
    }

    public static boolean hasSavedSettings() {
        try {
            return settingsFile().isFile();
        } catch (Throwable t) {
            return false;
        }
    }

    public static boolean hasSavedFieldMap() {
        try {
            return fieldMapFile().isFile();
        } catch (Throwable t) {
            return false;
        }
    }

    public static boolean saveSettings() {
        synchronized (FILE_LOCK) {
            try {
                List<String> problems = new ArrayList<String>();
                String savedAt = timestamp();
                JSONObject root = buildSettingsJson(savedAt, problems);
                File file = settingsFile();
                if (settingsFileRejected) {
                    backupRejected(file);
                }
                writeTextAtomically(file, root.toString(JSON_INDENT_SPACES));
                settingsFileRejected = false;
                settingsSource = "live values saved to " + SETTINGS_FILE_NAME + " at " + savedAt;
                if (!problems.isEmpty()) {
                    lastError = "saved " + SETTINGS_FILE_NAME + " but skipped " + describeProblems(problems);
                }
                return true;
            } catch (Throwable t) {
                lastError = "save " + SETTINGS_FILE_NAME + " failed: " + describe(t);
                return false;
            }
        }
    }

    public static String loadSettingsOnce() {
        try {
            synchronized (LOAD_ONCE_LOCK) {
                if (!LogiLightSettings.LOAD_SAVED_SETTINGS) {
                    return "using code values (LOAD_SAVED_SETTINGS = false)";
                }
                if (settingsLoadAttempted) {
                    return "keeping live settings: " + settingsSource;
                }
                settingsLoadAttempted = true;
                firstLoadMessage = loadSettings();
                return firstLoadMessage;
            }
        } catch (Throwable t) {
            lastError = "load settings once failed: " + describe(t);
            return "!! settings not loaded (" + describe(t) + ") - using " + settingsSource;
        }
    }

    public static String firstLoadMessage() {
        synchronized (LOAD_ONCE_LOCK) {
            return firstLoadMessage;
        }
    }

    public static String loadSettings() {
        synchronized (FILE_LOCK) {
            try {
                File file = settingsFile();
                if (!file.isFile()) {
                    return "no saved " + SETTINGS_FILE_NAME + " - using " + settingsSource;
                }
                JSONObject root = new JSONObject(readText(file));
                int version = root.optInt("version", -1);
                if (version != LogiLightSettings.SETTINGS_VERSION) {
                    String message = String.format(Locale.US, "!! %s version %d ignored (code expects %d) - using %s",
                            SETTINGS_FILE_NAME, version, LogiLightSettings.SETTINGS_VERSION, settingsSource);
                    lastError = message;
                    settingsFileRejected = true;
                    settingsSource = withWarning(settingsSource, "!! " + SETTINGS_FILE_NAME + " version " + version + " ignored");
                    return message;
                }
                String savedAt = stringValue(root.opt("savedAt"), "unknown time");
                List<String> problems = new ArrayList<String>();
                ParsedSettings parsed = parseSettings(root, problems);
                parsed.commit(problems);
                settingsSource = SETTINGS_FILE_NAME + " (saved " + savedAt + ")";
                settingsFileRejected = !problems.isEmpty();
                if (!problems.isEmpty()) {
                    settingsSource = withWarning(settingsSource, "!! " + problems.size() + " values skipped");
                }
                StringBuilder message = new StringBuilder();
                message.append("loaded ").append(SETTINGS_FILE_NAME).append(" saved ").append(savedAt)
                        .append("  pipelines ").append(LogiLightSettings.pipelineCount())
                        .append("  color profiles ").append(lengthOf(LogiLightSettings.COLOR_PROFILES));
                if (!problems.isEmpty()) {
                    lastError = SETTINGS_FILE_NAME + ": skipped " + describeProblems(problems);
                    message.append("  !! ").append(problems.size()).append(" values skipped");
                }
                return message.toString();
            } catch (Throwable t) {
                String message = "!! " + SETTINGS_FILE_NAME + " unreadable (" + describe(t) + ") - using " + settingsSource;
                lastError = message;
                settingsFileRejected = true;
                settingsSource = withWarning(settingsSource, "!! " + SETTINGS_FILE_NAME + " unreadable");
                return message;
            }
        }
    }

    public static boolean deleteSavedSettings() {
        synchronized (FILE_LOCK) {
            try {
                File file = settingsFile();
                boolean deleted = moveToBackup(file);
                deleteIfPresent(tempFileFor(file));
                if (!deleted) {
                    lastError = "could not move " + file.getPath() + " to " + BACKUP_SUFFIX;
                }
                return deleted;
            } catch (Throwable t) {
                lastError = "delete " + SETTINGS_FILE_NAME + " failed: " + describe(t);
                return false;
            }
        }
    }

    public static String settingsSource() {
        return settingsSource;
    }

    public static JSONObject settingsToJson() {
        try {
            List<String> problems = new ArrayList<String>();
            JSONObject root = buildSettingsJson(timestamp(), problems);
            if (!problems.isEmpty()) {
                lastError = "settings to JSON skipped " + describeProblems(problems);
            }
            return root;
        } catch (Throwable t) {
            lastError = "settings to JSON failed: " + describe(t);
            return emptyJson();
        }
    }

    public static LogiFieldMap loadFieldMap() {
        LogiFieldMap defaults = defaultFieldMap();
        try {
            synchronized (FILE_LOCK) {
                File file = fieldMapFile();
                if (!file.isFile()) {
                    fieldMapFileRejected = false;
                    return defaults.withSource(CODE_DEFAULTS_SOURCE + " (no " + FIELD_MAP_FILE_NAME + ")");
                }
                fieldMapFileRejected = true;
                JSONObject root = new JSONObject(readText(file));
                int version = root.optInt("version", -1);
                if (version != FIELD_MAP_VERSION) {
                    lastError = String.format(Locale.US, "%s version %d ignored (code expects %d)",
                            FIELD_MAP_FILE_NAME, version, FIELD_MAP_VERSION);
                    return defaults.withSource(CODE_DEFAULTS_SOURCE + " (!! " + FIELD_MAP_FILE_NAME
                            + " version " + version + " ignored)");
                }
                List<String> problems = new ArrayList<String>();
                LogiFieldMap merged = mergeFieldMap(root, defaults, problems);
                String savedAt = stringValue(root.opt("savedAt"), "");
                StringBuilder source = new StringBuilder(FIELD_MAP_FILE_NAME);
                if (!savedAt.isEmpty()) {
                    source.append(" (saved ").append(savedAt).append(")");
                }
                source.append("  ").append(merged.surveyedCount()).append(" surveyed");
                if (!problems.isEmpty()) {
                    lastError = FIELD_MAP_FILE_NAME + ": skipped " + describeProblems(problems);
                    source.append("  !! ").append(problems.size()).append(" entries skipped");
                }
                fieldMapFileRejected = !problems.isEmpty();
                return merged.withSource(source.toString());
            }
        } catch (Throwable t) {
            lastError = "load " + FIELD_MAP_FILE_NAME + " failed: " + describe(t);
            if (defaults == null) {
                return null;
            }
            return defaults.withSource(CODE_DEFAULTS_SOURCE + " (!! " + FIELD_MAP_FILE_NAME + " unreadable)");
        }
    }

    public static boolean saveFieldMap(LogiFieldMap map) {
        if (map == null) {
            lastError = "save " + FIELD_MAP_FILE_NAME + " failed: map is null";
            return false;
        }
        synchronized (FILE_LOCK) {
            try {
                List<String> problems = new ArrayList<String>();
                JSONObject root = buildFieldMapJson(map, timestamp(), problems);
                File file = fieldMapFile();
                if (fieldMapFileRejected) {
                    backupRejected(file);
                }
                writeTextAtomically(file, root.toString(JSON_INDENT_SPACES));
                fieldMapFileRejected = false;
                if (!problems.isEmpty()) {
                    lastError = "saved " + FIELD_MAP_FILE_NAME + " but skipped " + describeProblems(problems);
                }
                return true;
            } catch (Throwable t) {
                lastError = "save " + FIELD_MAP_FILE_NAME + " failed: " + describe(t);
                return false;
            }
        }
    }

    public static boolean deleteFieldMap() {
        synchronized (FILE_LOCK) {
            try {
                File file = fieldMapFile();
                boolean deleted = moveToBackup(file);
                deleteIfPresent(tempFileFor(file));
                if (!deleted) {
                    lastError = "could not move " + file.getPath() + " to " + BACKUP_SUFFIX;
                }
                return deleted;
            } catch (Throwable t) {
                lastError = "delete " + FIELD_MAP_FILE_NAME + " failed: " + describe(t);
                return false;
            }
        }
    }

    public static JSONObject fieldMapToJson(LogiFieldMap map) {
        try {
            List<String> problems = new ArrayList<String>();
            JSONObject root = buildFieldMapJson(map, timestamp(), problems);
            if (!problems.isEmpty()) {
                lastError = "field map to JSON skipped " + describeProblems(problems);
            }
            return root;
        } catch (Throwable t) {
            lastError = "field map to JSON failed: " + describe(t);
            return emptyJson();
        }
    }

    public static LogiFieldMap fieldMapFromJson(JSONObject root, String source) {
        LogiFieldMap defaults = defaultFieldMap();
        try {
            List<String> problems = new ArrayList<String>();
            LogiFieldMap merged = root == null ? defaults : mergeFieldMap(root, defaults, problems);
            if (!problems.isEmpty()) {
                lastError = "field map JSON: skipped " + describeProblems(problems);
            }
            return merged.withSource(source);
        } catch (Throwable t) {
            lastError = "field map from JSON failed: " + describe(t);
            return defaults == null ? null : defaults.withSource(source);
        }
    }

    public static File snapshotDirectory() {
        File directory = new File(storageDirectory(), SNAPSHOT_DIRECTORY_NAME);
        try {
            if (!ensureDirectory(directory)) {
                lastError = "cannot create " + directory.getPath();
            }
        } catch (Throwable t) {
            lastError = "snapshot directory failed: " + describe(t);
        }
        return directory;
    }

    public static String lastError() {
        return lastError;
    }

    public static void clearLastError() {
        lastError = "";
    }

    public static JSONObject toJson(Object configObject) {
        try {
            List<String> problems = new ArrayList<String>();
            JSONObject json = encodeObject(configObject, 0, "", problems);
            if (!problems.isEmpty()) {
                lastError = "toJson skipped " + describeProblems(problems);
            }
            return json;
        } catch (Throwable t) {
            lastError = "toJson failed: " + describe(t);
            return emptyJson();
        }
    }

    public static void fromJson(JSONObject json, Object configObject) {
        try {
            List<String> problems = new ArrayList<String>();
            applyJson(json, configObject, 0, "", problems);
            if (!problems.isEmpty()) {
                lastError = "fromJson skipped " + describeProblems(problems);
            }
        } catch (Throwable t) {
            lastError = "fromJson failed: " + describe(t);
        }
    }

    public static int fromJsonCounting(JSONObject json, Object configObject) {
        int[] applied = new int[1];
        try {
            List<String> problems = new ArrayList<String>();
            applyJson(json, configObject, 0, "", problems, applied);
            if (!problems.isEmpty()) {
                lastError = "fromJson skipped " + describeProblems(problems);
            }
        } catch (Throwable t) {
            lastError = "fromJson failed: " + describe(t);
        }
        return applied[0];
    }

    public static File settingsBackupFile() {
        return backupFileFor(settingsFile());
    }

    public static File fieldMapBackupFile() {
        return backupFileFor(fieldMapFile());
    }

    static void noteSettingsSource(String source) {
        settingsSource = source == null ? CODE_DEFAULTS_SOURCE : source;
    }

    private static final class ParsedSettings {
        LogiCameraConfig camera;
        LogiPipeline[] pipelines;
        ColorProfile[] colorProfiles;
        LogiLocalizationConfig localization;
        LogiAimConfig aim;
        Integer defaultPipeline;
        StreamView streamView;
        Boolean panelsStream;
        Boolean driverStationStream;

        void commit(List<String> problems) {
            boolean panelsNeedsRefresh;
            synchronized (LogiLightSettings.class) {
                int previousPipelineCount = lengthOf(LogiLightSettings.PIPELINES);
                int previousProfileCount = lengthOf(LogiLightSettings.COLOR_PROFILES);
                if (camera != null) {
                    LogiLightSettings.CAMERA = camera;
                }
                if (pipelines != null) {
                    LogiLightSettings.PIPELINES = pipelines;
                }
                if (colorProfiles != null) {
                    LogiLightSettings.COLOR_PROFILES = colorProfiles;
                }
                if (localization != null) {
                    LogiLightSettings.LOCALIZATION = localization;
                }
                if (aim != null) {
                    LogiLightSettings.AIM = aim;
                }
                if (defaultPipeline != null) {
                    LogiLightSettings.DEFAULT_PIPELINE = defaultPipeline;
                }
                if (streamView != null) {
                    LogiLightSettings.STREAM_VIEW = streamView;
                }
                if (panelsStream != null) {
                    LogiLightSettings.PANELS_STREAM = panelsStream;
                }
                if (driverStationStream != null) {
                    LogiLightSettings.DRIVER_STATION_STREAM = driverStationStream;
                }
                int pipelineCount = lengthOf(LogiLightSettings.PIPELINES);
                if (LogiLightSettings.DEFAULT_PIPELINE < 0 || LogiLightSettings.DEFAULT_PIPELINE >= pipelineCount) {
                    problems.add("defaultPipeline: " + LogiLightSettings.DEFAULT_PIPELINE + " out of range, using 0");
                    LogiLightSettings.DEFAULT_PIPELINE = 0;
                }
                panelsNeedsRefresh = LogiLightSettings.settingsReplaced(previousPipelineCount, previousProfileCount);
            }
            if (panelsNeedsRefresh) {
                LogiLightSettings.refreshPanels();
            }
        }
    }

    private static ParsedSettings parseSettings(JSONObject root, List<String> problems) {
        ParsedSettings parsed = new ParsedSettings();
        parsed.camera = parseConfigObject(root, "camera", LogiCameraConfig.class, problems);
        parsed.pipelines = parseConfigArray(root, "pipelines", LogiPipeline.class, problems);
        parsed.colorProfiles = parseConfigArray(root, "colorProfiles", ColorProfile.class, problems);
        parsed.localization = parseConfigObject(root, "localization", LogiLocalizationConfig.class, problems);
        parsed.aim = parseConfigObject(root, "aim", LogiAimConfig.class, problems);
        Object defaultPipeline = decodeOptional(root, "defaultPipeline", int.class, problems);
        if (defaultPipeline instanceof Integer) {
            parsed.defaultPipeline = (Integer) defaultPipeline;
        }
        Object streamView = decodeOptional(root, "streamView", StreamView.class, problems);
        if (streamView instanceof StreamView) {
            parsed.streamView = (StreamView) streamView;
        }
        Object panelsStream = decodeOptional(root, "panelsStream", boolean.class, problems);
        if (panelsStream instanceof Boolean) {
            parsed.panelsStream = (Boolean) panelsStream;
        }
        Object driverStationStream = decodeOptional(root, "driverStationStream", boolean.class, problems);
        if (driverStationStream instanceof Boolean) {
            parsed.driverStationStream = (Boolean) driverStationStream;
        }
        return parsed;
    }

    private static Object decodeOptional(JSONObject root, String key, Class<?> type, List<String> problems) {
        if (!root.has(key)) {
            return NO_VALUE;
        }
        return decodeValue(root.opt(key), type, 0, key, problems);
    }

    private static <T> T parseConfigObject(JSONObject root, String key, Class<T> type, List<String> problems) {
        if (!root.has(key)) {
            return null;
        }
        Object raw = root.opt(key);
        if (!(raw instanceof JSONObject)) {
            problems.add(key + ": expected an object");
            return null;
        }
        T instance = instantiate(type);
        if (instance == null) {
            problems.add(key + ": cannot create " + type.getSimpleName());
            return null;
        }
        applyJson((JSONObject) raw, instance, 0, key + ".", problems);
        return instance;
    }

    private static <T> T[] parseConfigArray(JSONObject root, String key, Class<T> type, List<String> problems) {
        if (!root.has(key)) {
            return null;
        }
        Object raw = root.opt(key);
        if (!(raw instanceof JSONArray)) {
            problems.add(key + ": expected an array");
            return null;
        }
        JSONArray array = (JSONArray) raw;
        List<T> items = new ArrayList<T>();
        for (int i = 0; i < array.length(); i++) {
            String path = key + "[" + i + "]";
            Object element = array.opt(i);
            if (!(element instanceof JSONObject)) {
                problems.add(path + ": expected an object");
                continue;
            }
            T instance = instantiate(type);
            if (instance == null) {
                problems.add(path + ": cannot create " + type.getSimpleName());
                continue;
            }
            applyJson((JSONObject) element, instance, 0, path + ".", problems);
            items.add(instance);
        }
        if (items.isEmpty()) {
            problems.add(key + ": no usable entries, kept current " + key);
            return null;
        }
        @SuppressWarnings("unchecked")
        T[] result = (T[]) Array.newInstance(type, items.size());
        return items.toArray(result);
    }

    private static JSONObject buildSettingsJson(String savedAt, List<String> problems) throws JSONException {
        LogiCameraConfig camera;
        LogiPipeline[] pipelines;
        ColorProfile[] colorProfiles;
        LogiLocalizationConfig localization;
        LogiAimConfig aim;
        int defaultPipeline;
        StreamView streamView;
        boolean panelsStream;
        boolean driverStationStream;
        synchronized (LogiLightSettings.class) {
            camera = LogiLightSettings.CAMERA;
            pipelines = LogiLightSettings.PIPELINES;
            colorProfiles = LogiLightSettings.COLOR_PROFILES;
            localization = LogiLightSettings.LOCALIZATION;
            aim = LogiLightSettings.AIM;
            defaultPipeline = LogiLightSettings.DEFAULT_PIPELINE;
            streamView = LogiLightSettings.STREAM_VIEW;
            panelsStream = LogiLightSettings.PANELS_STREAM;
            driverStationStream = LogiLightSettings.DRIVER_STATION_STREAM;
        }
        JSONObject root = new JSONObject();
        root.put("version", LogiLightSettings.SETTINGS_VERSION);
        root.put("savedAt", savedAt);
        root.put("camera", encodeObject(camera, 0, "camera.", problems));
        root.put("pipelines", encodeConfigArray(pipelines, "pipelines", problems));
        root.put("colorProfiles", encodeConfigArray(colorProfiles, "colorProfiles", problems));
        root.put("localization", encodeObject(localization, 0, "localization.", problems));
        root.put("aim", encodeObject(aim, 0, "aim.", problems));
        root.put("defaultPipeline", defaultPipeline);
        root.put("streamView", (streamView == null ? StreamView.ANNOTATED : streamView).name());
        root.put("panelsStream", panelsStream);
        root.put("driverStationStream", driverStationStream);
        return root;
    }

    private static JSONArray encodeConfigArray(Object[] items, String key, List<String> problems) throws JSONException {
        JSONArray array = new JSONArray();
        if (items == null) {
            return array;
        }
        for (int i = 0; i < items.length; i++) {
            if (items[i] == null) {
                problems.add(key + "[" + i + "]: null entry");
                continue;
            }
            array.put(encodeObject(items[i], 0, key + "[" + i + "].", problems));
        }
        return array;
    }

    private static JSONObject encodeObject(Object object, int depth, String path, List<String> problems) throws JSONException {
        JSONObject json = new JSONObject();
        if (object == null) {
            return json;
        }
        if (depth > MAX_NESTING_DEPTH) {
            problems.add(path + ": nested too deep");
            return json;
        }
        for (Field field : configFields(object.getClass())) {
            String key = field.getName();
            try {
                Object encoded = encodeValue(field.get(object), depth, path + key, problems);
                if (encoded != null) {
                    json.put(key, encoded);
                }
            } catch (Throwable t) {
                problems.add(path + key + ": " + describe(t));
            }
        }
        return json;
    }

    private static Object encodeValue(Object value, int depth, String path, List<String> problems) throws JSONException {
        if (value == null) {
            return null;
        }
        if (value instanceof Double || value instanceof Float) {
            double number = ((Number) value).doubleValue();
            if (isFinite(number)) {
                return value;
            }
            problems.add(path + ": not a finite number");
            return null;
        }
        if (value instanceof Number || value instanceof Boolean || value instanceof String) {
            return value;
        }
        if (value instanceof Character) {
            return String.valueOf(value);
        }
        if (value instanceof Enum) {
            return ((Enum<?>) value).name();
        }
        if (depth >= MAX_NESTING_DEPTH) {
            problems.add(path + ": nested too deep");
            return null;
        }
        if (value.getClass().isArray()) {
            JSONArray array = new JSONArray();
            int length = Array.getLength(value);
            for (int i = 0; i < length; i++) {
                Object encoded = encodeValue(Array.get(value, i), depth + 1, path + "[" + i + "]", problems);
                array.put(encoded == null ? JSONObject.NULL : encoded);
            }
            return array;
        }
        if (value instanceof Collection) {
            JSONArray array = new JSONArray();
            int index = 0;
            for (Object element : (Collection<?>) value) {
                Object encoded = encodeValue(element, depth + 1, path + "[" + index + "]", problems);
                array.put(encoded == null ? JSONObject.NULL : encoded);
                index++;
            }
            return array;
        }
        if (value instanceof Map) {
            JSONObject json = new JSONObject();
            for (Map.Entry<?, ?> entry : ((Map<?, ?>) value).entrySet()) {
                String key = String.valueOf(entry.getKey());
                Object encoded = encodeValue(entry.getValue(), depth + 1, path + "." + key, problems);
                if (encoded != null) {
                    json.put(key, encoded);
                }
            }
            return json;
        }
        if (isNestedConfigType(value.getClass())) {
            return encodeObject(value, depth + 1, path + ".", problems);
        }
        problems.add(path + ": unsupported type " + value.getClass().getSimpleName());
        return null;
    }

    private static void applyJson(JSONObject json, Object target, int depth, String path, List<String> problems) {
        applyJson(json, target, depth, path, problems, null);
    }

    private static void applyJson(JSONObject json, Object target, int depth, String path, List<String> problems,
                                  int[] applied) {
        if (json == null || target == null) {
            return;
        }
        if (depth > MAX_NESTING_DEPTH) {
            problems.add(path + ": nested too deep");
            return;
        }
        for (Field field : configFields(target.getClass())) {
            if (Modifier.isFinal(field.getModifiers())) {
                continue;
            }
            String key = field.getName();
            if (!json.has(key)) {
                continue;
            }
            String fieldPath = path + key;
            try {
                Object raw = json.opt(key);
                if (raw == null || raw == JSONObject.NULL) {
                    continue;
                }
                Class<?> type = field.getType();
                if (isNestedConfigType(type)) {
                    if (!(raw instanceof JSONObject)) {
                        problems.add(fieldPath + ": expected an object");
                        continue;
                    }
                    Object nested = field.get(target);
                    if (nested == null) {
                        nested = instantiate(type);
                    }
                    if (nested == null) {
                        problems.add(fieldPath + ": cannot create " + type.getSimpleName());
                        continue;
                    }
                    applyJson((JSONObject) raw, nested, depth + 1, fieldPath + ".", problems, applied);
                    field.set(target, nested);
                    continue;
                }
                Object decoded = decodeValue(raw, type, depth, fieldPath, problems);
                if (decoded != NO_VALUE) {
                    field.set(target, decoded);
                    if (applied != null) {
                        applied[0]++;
                    }
                }
            } catch (Throwable t) {
                problems.add(fieldPath + ": " + describe(t));
            }
        }
    }

    private static Object decodeValue(Object raw, Class<?> type, int depth, String path, List<String> problems) {
        if (raw == null || raw == JSONObject.NULL) {
            return NO_VALUE;
        }
        if (type == int.class || type == Integer.class) {
            Double number = finiteNumber(raw);
            if (number == null || Math.abs(number) > Integer.MAX_VALUE) {
                return reject(path, raw, problems);
            }
            return (int) Math.round(number);
        }
        if (type == long.class || type == Long.class) {
            if (raw instanceof Integer || raw instanceof Long || raw instanceof Short || raw instanceof Byte) {
                return ((Number) raw).longValue();
            }
            Double number = finiteNumber(raw);
            if (number == null || Math.abs(number) > Long.MAX_VALUE) {
                return reject(path, raw, problems);
            }
            return Math.round(number);
        }
        if (type == double.class || type == Double.class) {
            Double number = finiteNumber(raw);
            return number == null ? reject(path, raw, problems) : number;
        }
        if (type == float.class || type == Float.class) {
            Double number = finiteNumber(raw);
            if (number == null || Math.abs(number) > Float.MAX_VALUE) {
                return reject(path, raw, problems);
            }
            return (float) number.doubleValue();
        }
        if (type == short.class || type == Short.class) {
            Double number = finiteNumber(raw);
            if (number == null || Math.abs(number) > Short.MAX_VALUE) {
                return reject(path, raw, problems);
            }
            return (short) Math.round(number);
        }
        if (type == byte.class || type == Byte.class) {
            Double number = finiteNumber(raw);
            if (number == null || Math.abs(number) > Byte.MAX_VALUE) {
                return reject(path, raw, problems);
            }
            return (byte) Math.round(number);
        }
        if (type == boolean.class || type == Boolean.class) {
            Boolean flag = booleanValue(raw);
            return flag == null ? reject(path, raw, problems) : flag;
        }
        if (type == String.class) {
            if (raw instanceof JSONObject || raw instanceof JSONArray) {
                return reject(path, raw, problems);
            }
            return raw.toString();
        }
        if (type == char.class || type == Character.class) {
            String text = raw.toString();
            return text.isEmpty() ? reject(path, raw, problems) : Character.valueOf(text.charAt(0));
        }
        if (type.isEnum()) {
            Object constant = enumConstant(type, raw.toString());
            return constant == null ? reject(path, raw, problems) : constant;
        }
        if (type.isArray()) {
            return decodeArray(raw, type.getComponentType(), depth, path, problems);
        }
        if (isNestedConfigType(type)) {
            if (!(raw instanceof JSONObject)) {
                return reject(path, raw, problems);
            }
            Object instance = instantiate(type);
            if (instance == null) {
                problems.add(path + ": cannot create " + type.getSimpleName());
                return NO_VALUE;
            }
            applyJson((JSONObject) raw, instance, depth + 1, path + ".", problems);
            return instance;
        }
        problems.add(path + ": unsupported type " + type.getSimpleName());
        return NO_VALUE;
    }

    private static Object decodeArray(Object raw, Class<?> componentType, int depth, String path, List<String> problems) {
        if (!(raw instanceof JSONArray)) {
            return reject(path, raw, problems);
        }
        if (depth >= MAX_NESTING_DEPTH) {
            problems.add(path + ": nested too deep");
            return NO_VALUE;
        }
        JSONArray array = (JSONArray) raw;
        int length = array.length();
        Object result = Array.newInstance(componentType, length);
        for (int i = 0; i < length; i++) {
            String elementPath = path + "[" + i + "]";
            Object element = array.opt(i);
            if (element == null || element == JSONObject.NULL) {
                if (componentType.isPrimitive()) {
                    problems.add(elementPath + ": missing value, kept previous array");
                    return NO_VALUE;
                }
                continue;
            }
            Object decoded = decodeValue(element, componentType, depth + 1, elementPath, problems);
            if (decoded == NO_VALUE) {
                problems.add(path + ": invalid element " + i + ", kept previous array");
                return NO_VALUE;
            }
            Array.set(result, i, decoded);
        }
        return result;
    }

    private static Object reject(String path, Object raw, List<String> problems) {
        String shown = String.valueOf(raw);
        if (shown.length() > 40) {
            shown = shown.substring(0, 40) + "...";
        }
        problems.add(path + ": bad value '" + shown + "'");
        return NO_VALUE;
    }

    private static Object enumConstant(Class<?> type, String name) {
        Object[] constants = type.getEnumConstants();
        if (constants == null || name == null) {
            return null;
        }
        String wanted = name.trim();
        for (Object constant : constants) {
            if (((Enum<?>) constant).name().equals(wanted)) {
                return constant;
            }
        }
        for (Object constant : constants) {
            if (((Enum<?>) constant).name().equalsIgnoreCase(wanted)) {
                return constant;
            }
        }
        return null;
    }

    private static Double finiteNumber(Object raw) {
        double value;
        if (raw instanceof Number) {
            value = ((Number) raw).doubleValue();
        } else if (raw instanceof String) {
            try {
                value = Double.parseDouble(((String) raw).trim());
            } catch (NumberFormatException e) {
                return null;
            }
        } else {
            return null;
        }
        return isFinite(value) ? value : null;
    }

    private static Boolean booleanValue(Object raw) {
        if (raw instanceof Boolean) {
            return (Boolean) raw;
        }
        if (raw instanceof String) {
            String text = ((String) raw).trim();
            if (text.equalsIgnoreCase("true")) {
                return Boolean.TRUE;
            }
            if (text.equalsIgnoreCase("false")) {
                return Boolean.FALSE;
            }
            return null;
        }
        if (raw instanceof Number) {
            double value = ((Number) raw).doubleValue();
            return isFinite(value) ? Boolean.valueOf(value != 0.0) : null;
        }
        return null;
    }

    private static String stringValue(Object raw, String fallback) {
        if (raw == null || raw == JSONObject.NULL || raw instanceof JSONObject || raw instanceof JSONArray) {
            return fallback;
        }
        String text = raw.toString().trim();
        return text.isEmpty() ? fallback : text;
    }

    private static List<Field> configFields(Class<?> type) {
        List<Field> fields = new ArrayList<Field>();
        for (Field field : type.getFields()) {
            if (Modifier.isStatic(field.getModifiers()) || field.isSynthetic()) {
                continue;
            }
            fields.add(field);
        }
        Field[] sorted = fields.toArray(new Field[0]);
        Arrays.sort(sorted, new Comparator<Field>() {
            @Override
            public int compare(Field a, Field b) {
                return a.getName().compareTo(b.getName());
            }
        });
        return Arrays.asList(sorted);
    }

    private static boolean isNestedConfigType(Class<?> type) {
        if (type == null || type.isPrimitive() || type.isArray() || type.isEnum()
                || type.isInterface() || type.isAnnotation() || Modifier.isAbstract(type.getModifiers())) {
            return false;
        }
        if (Number.class.isAssignableFrom(type) || type == String.class || type == Boolean.class
                || type == Character.class || type == Object.class) {
            return false;
        }
        String name = type.getName();
        return !(name.startsWith("java.") || name.startsWith("javax.") || name.startsWith("android.")
                || name.startsWith("kotlin.") || name.startsWith("org.json."));
    }

    private static <T> T instantiate(Class<T> type) {
        try {
            Constructor<T> constructor = type.getDeclaredConstructor();
            constructor.setAccessible(true);
            return constructor.newInstance();
        } catch (Throwable t) {
            return null;
        }
    }

    private static JSONObject buildFieldMapJson(LogiFieldMap map, String savedAt, List<String> problems) throws JSONException {
        JSONObject root = new JSONObject();
        root.put("version", FIELD_MAP_VERSION);
        root.put("savedAt", savedAt);
        JSONArray targets = new JSONArray();
        if (map != null) {
            LogiFieldMap defaults = defaultFieldMap();
            for (LogiFieldMap.Target target : map.targets()) {
                if (target == null) {
                    continue;
                }
                if (!target.surveyed && defaults != null && defaults.find(target.name) != null) {
                    continue;
                }
                String targetPath = "targets." + target.name;
                JSONArray states = new JSONArray();
                for (FieldTargetState state : target.states) {
                    JSONObject stateJson = encodeState(state, targetPath, problems);
                    if (stateJson != null) {
                        states.put(stateJson);
                    }
                }
                if (states.length() == 0) {
                    problems.add(targetPath + ": no finite states");
                    continue;
                }
                JSONObject targetJson = new JSONObject();
                targetJson.put("name", target.name);
                targetJson.put("surveyed", target.surveyed);
                targetJson.put("states", states);
                targets.put(targetJson);
            }
        }
        root.put("targets", targets);
        return root;
    }

    private static JSONObject encodeState(FieldTargetState state, String targetPath, List<String> problems) throws JSONException {
        if (state == null) {
            return null;
        }
        Vec3 origin = state.originField;
        Mat3 rotation = state.rotationField;
        if (origin == null || rotation == null || !origin.isFinite() || !rotation.isFinite()) {
            problems.add(targetPath + "." + state.label + ": not finite");
            return null;
        }
        double[] eulerDegrees = eulerDegrees(rotation);
        if (!isFinite(eulerDegrees[0]) || !isFinite(eulerDegrees[1]) || !isFinite(eulerDegrees[2])) {
            problems.add(targetPath + "." + state.label + ": rotation not finite");
            return null;
        }
        JSONObject json = new JSONObject();
        json.put("label", state.label.trim().isEmpty() ? LogiFieldMap.DEFAULT_STATE_LABEL : state.label.trim());
        json.put("x", rounded(origin.x));
        json.put("y", rounded(origin.y));
        json.put("z", rounded(origin.z));
        json.put("yawDeg", rounded(eulerDegrees[0]));
        json.put("pitchDeg", rounded(eulerDegrees[1]));
        json.put("rollDeg", rounded(eulerDegrees[2]));
        SurveyInfo survey = state.survey;
        if (survey != null && survey.isFinite()) {
            JSONObject surveyJson = new JSONObject();
            surveyJson.put("robotX", rounded(survey.robotPose.x));
            surveyJson.put("robotY", rounded(survey.robotPose.y));
            surveyJson.put("robotHeadingDeg", rounded(Math.toDegrees(survey.robotPose.heading)));
            surveyJson.put("forwardInches", rounded(survey.forwardInches));
            surveyJson.put("leftInches", rounded(survey.leftInches));
            surveyJson.put("upInches", rounded(survey.upInches));
            surveyJson.put("yawLeftDegrees", rounded(survey.yawLeftDegrees));
            surveyJson.put("tiltUpDegrees", rounded(survey.tiltUpDegrees));
            surveyJson.put("rollClockwiseDegrees", rounded(survey.rollClockwiseDegrees));
            json.put("survey", surveyJson);
        }
        return json;
    }

    private static SurveyInfo decodeSurvey(Object raw, String path, List<String> problems) {
        if (raw == null || raw == JSONObject.NULL) {
            return null;
        }
        if (!(raw instanceof JSONObject)) {
            problems.add(path + ".survey: expected an object");
            return null;
        }
        JSONObject json = (JSONObject) raw;
        String[] keys = {"robotX", "robotY", "robotHeadingDeg", "forwardInches", "leftInches", "upInches",
                "yawLeftDegrees", "tiltUpDegrees", "rollClockwiseDegrees"};
        double[] values = new double[keys.length];
        for (int i = 0; i < keys.length; i++) {
            Double value = finiteNumber(json.opt(keys[i]));
            if (value == null) {
                problems.add(path + ".survey." + keys[i] + ": bad value, survey mount ignored");
                return null;
            }
            values[i] = value;
        }
        SurveyInfo info = new SurveyInfo(new PlanarPose(values[0], values[1], Math.toRadians(values[2])),
                values[3], values[4], values[5], values[6], values[7], values[8]);
        return info.isFinite() ? info : null;
    }

    private static LogiFieldMap mergeFieldMap(JSONObject root, LogiFieldMap defaults, List<String> problems) {
        LogiFieldMap merged = defaults;
        Object rawTargets = root.opt("targets");
        if (!(rawTargets instanceof JSONArray)) {
            if (rawTargets != null && rawTargets != JSONObject.NULL) {
                problems.add("targets: expected an array");
            }
            return merged;
        }
        JSONArray targets = (JSONArray) rawTargets;
        for (int i = 0; i < targets.length(); i++) {
            String path = "targets[" + i + "]";
            Object rawTarget = targets.opt(i);
            if (!(rawTarget instanceof JSONObject)) {
                problems.add(path + ": expected an object");
                continue;
            }
            try {
                LogiFieldMap.Target target = decodeTarget((JSONObject) rawTarget, path, problems);
                if (target != null) {
                    merged = merged.withTarget(target);
                }
            } catch (Throwable t) {
                problems.add(path + ": " + describe(t));
            }
        }
        return merged;
    }

    private static LogiFieldMap.Target decodeTarget(JSONObject json, String path, List<String> problems) {
        String name = stringValue(json.opt("name"), "");
        if (name.isEmpty()) {
            problems.add(path + ": missing name");
            return null;
        }
        String targetPath = path + " " + name;
        boolean surveyed = true;
        if (json.has("surveyed")) {
            Boolean flag = booleanValue(json.opt("surveyed"));
            if (flag == null) {
                problems.add(targetPath + ".surveyed: bad value, assuming true");
            } else {
                surveyed = flag;
            }
        }
        Object rawStates = json.opt("states");
        if (!(rawStates instanceof JSONArray)) {
            problems.add(targetPath + ": missing states, using default");
            return null;
        }
        JSONArray statesJson = (JSONArray) rawStates;
        List<FieldTargetState> states = new ArrayList<FieldTargetState>();
        for (int j = 0; j < statesJson.length(); j++) {
            Object rawState = statesJson.opt(j);
            String statePath = targetPath + ".states[" + j + "]";
            if (!(rawState instanceof JSONObject)) {
                problems.add(statePath + ": expected an object");
                continue;
            }
            FieldTargetState state = decodeState((JSONObject) rawState, statePath, problems);
            if (state != null) {
                states.add(state);
            }
        }
        if (states.isEmpty()) {
            problems.add(targetPath + ": no valid states, using default");
            return null;
        }
        return new LogiFieldMap.Target(name, surveyed, states);
    }

    private static FieldTargetState decodeState(JSONObject json, String path, List<String> problems) {
        String label = stringValue(json.opt("label"), LogiFieldMap.DEFAULT_STATE_LABEL);
        Double x = finiteNumber(json.opt("x"));
        Double y = finiteNumber(json.opt("y"));
        Double z = finiteNumber(json.opt("z"));
        if (x == null || y == null || z == null) {
            problems.add(path + ": x, y and z must be finite numbers");
            return null;
        }
        double yaw = optionalAngle(json, "yawDeg", path, problems);
        double pitch = optionalAngle(json, "pitchDeg", path, problems);
        double roll = optionalAngle(json, "rollDeg", path, problems);
        if (Double.isNaN(yaw) || Double.isNaN(pitch) || Double.isNaN(roll)) {
            return null;
        }
        FieldTargetState state = FieldTargetState.fromDegrees(label, x, y, z, yaw, pitch, roll);
        if (!state.isFinite()) {
            problems.add(path + ": not finite");
            return null;
        }
        SurveyInfo survey = decodeSurvey(json.opt("survey"), path, problems);
        return survey == null ? state : state.withSurvey(survey);
    }

    private static double optionalAngle(JSONObject json, String key, String path, List<String> problems) {
        if (!json.has(key) || json.opt(key) == JSONObject.NULL) {
            return 0.0;
        }
        Double value = finiteNumber(json.opt(key));
        if (value == null) {
            problems.add(path + "." + key + ": bad value");
            return Double.NaN;
        }
        return value;
    }

    private static double[] eulerDegrees(Mat3 rotation) {
        double sinePitch = clamp(-rotation.get(2, 0), -1.0, 1.0);
        double pitch = Math.asin(sinePitch);
        double yaw;
        double roll;
        if (Math.abs(sinePitch) > GIMBAL_LOCK_SINE) {
            roll = 0.0;
            yaw = Math.atan2(-rotation.get(0, 1), rotation.get(1, 1));
        } else {
            yaw = Math.atan2(rotation.get(1, 0), rotation.get(0, 0));
            roll = Math.atan2(rotation.get(2, 1), rotation.get(2, 2));
        }
        return new double[]{Math.toDegrees(yaw), Math.toDegrees(pitch), Math.toDegrees(roll)};
    }

    private static LogiFieldMap defaultFieldMap() {
        try {
            return LogiFieldMap.bioBuzzDefaults();
        } catch (Throwable t) {
            lastError = "field map defaults failed: " + describe(t);
            try {
                return new LogiFieldMap(new ArrayList<LogiFieldMap.Target>(), "empty (!! defaults failed)");
            } catch (Throwable ignored) {
                return null;
            }
        }
    }

    private static void writeTextAtomically(File file, String text) throws IOException {
        File directory = file.getParentFile();
        if (directory != null && !ensureDirectory(directory)) {
            throw new IOException("cannot create directory " + directory.getPath());
        }
        byte[] bytes = text.getBytes(UTF_8);
        File temp = tempFileFor(file);
        writeBytes(temp, bytes);
        if (temp.renameTo(file)) {
            return;
        }
        if (file.exists() && !file.delete()) {
            deleteIfPresent(temp);
            throw new IOException("cannot replace " + file.getPath());
        }
        if (!temp.renameTo(file)) {
            writeBytes(file, bytes);
            deleteIfPresent(temp);
        }
        if (!file.isFile() || file.length() != bytes.length) {
            throw new IOException("write verification failed for " + file.getPath());
        }
    }

    private static void writeBytes(File file, byte[] bytes) throws IOException {
        FileOutputStream output = new FileOutputStream(file);
        try {
            output.write(bytes);
            output.flush();
            try {
                output.getFD().sync();
            } catch (IOException ignored) {
            }
        } finally {
            output.close();
        }
    }

    private static String readText(File file) throws IOException {
        long length = file.length();
        if (length > MAX_FILE_BYTES) {
            throw new IOException(file.getName() + " is too large (" + length + " bytes)");
        }
        InputStream input = new FileInputStream(file);
        try {
            ByteArrayOutputStream buffer = new ByteArrayOutputStream((int) Math.max(256L, length));
            byte[] chunk = new byte[READ_CHUNK_BYTES];
            long total = 0L;
            int count;
            while ((count = input.read(chunk)) != -1) {
                total += count;
                if (total > MAX_FILE_BYTES) {
                    throw new IOException(file.getName() + " is too large");
                }
                buffer.write(chunk, 0, count);
            }
            String text = new String(buffer.toByteArray(), UTF_8);
            if (text.startsWith(BYTE_ORDER_MARK)) {
                text = text.substring(BYTE_ORDER_MARK.length());
            }
            if (text.trim().isEmpty()) {
                throw new IOException(file.getName() + " is empty");
            }
            return text;
        } finally {
            input.close();
        }
    }

    private static boolean ensureDirectory(File directory) {
        if (directory.isDirectory()) {
            return true;
        }
        directory.mkdirs();
        return directory.isDirectory();
    }

    private static File tempFileFor(File file) {
        return new File(file.getPath() + TEMP_SUFFIX);
    }

    private static boolean deleteIfPresent(File file) {
        return !file.exists() || file.delete() || !file.exists();
    }

    private static File backupFileFor(File file) {
        return new File(file.getPath() + BACKUP_SUFFIX);
    }

    private static boolean moveToBackup(File file) {
        if (!file.exists()) {
            return true;
        }
        File backup = backupFileFor(file);
        deleteIfPresent(backup);
        return file.renameTo(backup) || deleteIfPresent(file);
    }

    private static void backupRejected(File file) {
        try {
            if (file == null || !file.isFile()) {
                return;
            }
            String stamp = new SimpleDateFormat(REJECTED_TIME_PATTERN, Locale.US).format(new Date());
            File target = new File(file.getParentFile(), file.getName() + REJECTED_SUFFIX + stamp);
            if (!file.renameTo(target)) {
                lastError = "could not back up rejected " + file.getName();
            }
        } catch (Throwable t) {
            lastError = "back up rejected " + (file == null ? "file" : file.getName()) + " failed: " + describe(t);
        }
    }

    private static String withWarning(String source, String warning) {
        String base = source == null || source.isEmpty() ? CODE_DEFAULTS_SOURCE : source;
        int cut = base.indexOf(" (!! ");
        if (cut >= 0) {
            base = base.substring(0, cut);
        }
        return base + " (" + warning + ")";
    }

    private static JSONObject emptyJson() {
        try {
            return new JSONObject();
        } catch (Throwable t) {
            return null;
        }
    }

    private static String timestamp() {
        return new SimpleDateFormat(TIMESTAMP_PATTERN, Locale.US).format(new Date());
    }

    private static String describe(Throwable t) {
        if (t == null) {
            return "unknown error";
        }
        String message = t.getMessage();
        return message == null || message.isEmpty()
                ? t.getClass().getSimpleName()
                : t.getClass().getSimpleName() + ": " + message;
    }

    private static String describeProblems(List<String> problems) {
        if (problems.isEmpty()) {
            return "";
        }
        if (problems.size() == 1) {
            return problems.get(0);
        }
        return problems.get(0) + " (+" + (problems.size() - 1) + " more)";
    }

    private static double rounded(double value) {
        return Math.round(value * FIELD_MAP_ROUNDING_SCALE) / FIELD_MAP_ROUNDING_SCALE;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static int lengthOf(Object[] array) {
        return array == null ? 0 : array.length;
    }
}
