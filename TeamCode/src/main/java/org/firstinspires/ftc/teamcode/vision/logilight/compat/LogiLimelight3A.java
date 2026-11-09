package org.firstinspires.ftc.teamcode.vision.logilight.compat;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.logilight.ColorProfile;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiBotPose;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightStore;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiPipeline;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiStatus;
import org.firstinspires.ftc.teamcode.vision.logilight.PipelineType;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;
import org.firstinspires.ftc.teamcode.vision.logilight.math.ClusterLayout;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Mat3;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.net.InetAddress;
import java.util.List;
import java.util.Locale;

public class LogiLimelight3A extends Limelight3A implements OpModeManagerNotifier.Notifications {

    public static final String DEFAULT_DEVICE_NAME = "limelight";
    public static final String FIDUCIAL_FAMILY = "36h11";
    public static final double FIELD_CENTER_INCHES = 72.0;
    public static final double METERS_PER_INCH = 0.0254;
    public static final int MAX_PYTHON_INPUTS = 32;
    public static final String SNAPSHOT_EXTENSION = ".png";

    private final LogiLight light;
    private final String webcamName;
    private final Object convertLock = new Object();

    private volatile HardwareMap installedMap;
    private volatile String installedName;
    private volatile String installWarning = "";
    private volatile int pollRateHz = 100;
    private LogiResult convertedSource;
    private LLResult converted;

    public LogiLimelight3A(HardwareMap hardwareMap, String webcamName) {
        super(null, "logilight", InetAddress.getLoopbackAddress());
        this.webcamName = webcamName == null ? "" : webcamName.trim();
        this.light = LogiLight.forWebcam(hardwareMap, this.webcamName);
    }

    public static LogiLimelight3A install(HardwareMap hardwareMap, String deviceName, String webcamName) {
        String name = deviceName == null || deviceName.trim().isEmpty() ? DEFAULT_DEVICE_NAME : deviceName.trim();
        removeStale(hardwareMap, name);
        LogiLimelight3A device = new LogiLimelight3A(hardwareMap, webcamName);
        if (hardwareMap != null) {
            try {
                Limelight3A configured = hardwareMap.tryGet(Limelight3A.class, name);
                if (configured != null && !(configured instanceof LogiLimelight3A)) {
                    device.installWarning = "!! a real Limelight is configured as \"" + name
                            + "\"; hardwareMap.get returns it instead of LogiLight";
                }
                hardwareMap.put(name, device);
                device.installedMap = hardwareMap;
                device.installedName = name;
            } catch (Throwable t) {
                device.installWarning = "!! install failed: " + describe(t);
            }
        }
        return device;
    }

    private static void removeStale(HardwareMap hardwareMap, String name) {
        if (hardwareMap == null) {
            return;
        }
        try {
            List<LogiLimelight3A> existing = hardwareMap.getAll(LogiLimelight3A.class);
            for (LogiLimelight3A device : existing) {
                if (hardwareMap.getNamesOf(device).contains(name)) {
                    hardwareMap.remove(name, device);
                    device.light.close();
                }
            }
        } catch (Throwable ignored) {
        }
    }

    public void uninstall() {
        HardwareMap map = installedMap;
        String name = installedName;
        installedMap = null;
        installedName = null;
        if (map != null && name != null) {
            try {
                map.remove(name, this);
            } catch (Throwable ignored) {
            }
        }
        closeQuietly();
    }

    public LogiLight logiLight() {
        return light;
    }

    public String installWarning() {
        return installWarning;
    }

    @Override
    public synchronized void start() {
        light.start();
    }

    @Override
    public synchronized void pause() {
        light.pause();
    }

    @Override
    public synchronized void stop() {
        light.stop();
    }

    @Override
    public boolean isRunning() {
        return light.isRunning();
    }

    @Override
    public synchronized void setPollRateHz(int rateHz) {
        pollRateHz = Math.max(1, Math.min(250, rateHz));
    }

    public int getPollRateHz() {
        return pollRateHz;
    }

    @Override
    public long getTimeSinceLastUpdate() {
        return light.getTimeSinceLastUpdate();
    }

    @Override
    public boolean isConnected() {
        return light.isConnected();
    }

    @Override
    public LLResult getLatestResult() {
        try {
            light.update();
            LogiResult result = light.getLatestResult();
            synchronized (convertLock) {
                if (converted == null || result != convertedSource) {
                    converted = new LogiLLResult(toLimelightJson(result), result);
                    convertedSource = result;
                }
                return converted;
            }
        } catch (Throwable t) {
            return fallbackResult();
        }
    }

    @Override
    public LLStatus getStatus() {
        try {
            LogiStatus status = light.getStatus();
            double heading = light.getLastHeadingRadians();
            return LogiLLStatus.from(status, Double.isNaN(heading) ? 0.0 : Math.toDegrees(heading), snapshotCount());
        } catch (Throwable t) {
            return new LLStatus();
        }
    }

    @Override
    public boolean reloadPipeline() {
        try {
            int index = light.getPipelineIndex();
            light.update();
            light.pipelineSwitch(index);
            light.refreshMount();
            light.applyCameraControls();
            return light.isAvailable();
        } catch (Throwable t) {
            return false;
        }
    }

    @Override
    public boolean pipelineSwitch(int index) {
        return light.pipelineSwitch(index);
    }

    @Override
    public boolean captureSnapshot(String snapname) {
        return light.captureSnapshot(snapname);
    }

    @Override
    public boolean deleteSnapshots() {
        try {
            File[] files = LogiLightStore.snapshotDirectory().listFiles();
            if (files == null) {
                return false;
            }
            boolean all = true;
            for (File file : files) {
                if (file.isFile() && file.getName().toLowerCase(Locale.US).endsWith(SNAPSHOT_EXTENSION)) {
                    all &= file.delete();
                }
            }
            return all;
        } catch (Throwable t) {
            return false;
        }
    }

    @Override
    public boolean deleteSnapshot(String snapname) {
        if (snapname == null || snapname.trim().isEmpty()) {
            return false;
        }
        try {
            String base = snapname.trim().replaceAll("[^A-Za-z0-9_-]+", "_");
            File[] files = LogiLightStore.snapshotDirectory().listFiles();
            if (files == null) {
                return false;
            }
            boolean deleted = false;
            for (File file : files) {
                String fileName = file.getName();
                boolean exact = fileName.equals(base + SNAPSHOT_EXTENSION) || fileName.equals(snapname.trim());
                boolean stamped = fileName.startsWith(base + "_") && fileName.toLowerCase(Locale.US).endsWith(SNAPSHOT_EXTENSION);
                if (file.isFile() && (exact || stamped)) {
                    deleted |= file.delete();
                }
            }
            return deleted;
        } catch (Throwable t) {
            return false;
        }
    }

    @Override
    public boolean updatePythonInputs(double input1, double input2, double input3, double input4,
                                      double input5, double input6, double input7, double input8) {
        light.updateScriptInputs(input1, input2, input3, input4, input5, input6, input7, input8);
        return true;
    }

    @Override
    public boolean updatePythonInputs(double[] inputs) {
        if (inputs == null || inputs.length == 0 || inputs.length > MAX_PYTHON_INPUTS) {
            return false;
        }
        light.updateScriptInputs(inputs);
        return true;
    }

    @Override
    public boolean updateRobotOrientation(double yaw) {
        if (Double.isNaN(yaw) || Double.isInfinite(yaw)) {
            return false;
        }
        light.updateRobotOrientation(Math.toRadians(yaw));
        return true;
    }

    @Override
    public boolean uploadPipeline(String jsonString, Integer index) {
        if (jsonString == null || jsonString.trim().isEmpty()) {
            return false;
        }
        try {
            JSONObject json = new JSONObject(jsonString);
            int target = index == null ? light.getPipelineIndex() : index.intValue();
            LogiPipeline[] pipelines = LogiLightSettings.PIPELINES;
            if (pipelines == null || target < 0 || target >= pipelines.length) {
                return false;
            }
            LogiPipeline updated = pipelines[target] == null ? new LogiPipeline() : pipelines[target].copy();
            LogiLightStore.clearLastError();
            int applied = LogiLightStore.fromJsonCounting(json, updated);
            if (applied == 0) {
                return false;
            }
            pipelines[target] = updated;
            if (target == light.getPipelineIndex()) {
                light.pipelineSwitch(target);
            }
            return LogiLightStore.lastError().isEmpty();
        } catch (Throwable t) {
            return false;
        }
    }

    @Override
    public boolean uploadFieldmap(LLFieldMap fieldmap, Integer index) {
        return false;
    }

    @Override
    public boolean uploadPython(String pythonString, Integer index) {
        return false;
    }

    @Override
    public LLResultTypes.CalibrationResult getCalDefault() {
        return calibration();
    }

    @Override
    public LLResultTypes.CalibrationResult getCalFile() {
        return calibration();
    }

    @Override
    public LLResultTypes.CalibrationResult getCalEEPROM() {
        return calibration();
    }

    @Override
    public LLResultTypes.CalibrationResult getCalLatest() {
        return calibration();
    }

    @Override
    public void shutdown() {
        closeQuietly();
    }

    @Override
    public void close() {
        closeQuietly();
    }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "LogiLight (" + light.getWebcamName() + ")";
    }

    @Override
    public String getConnectionInfo() {
        return "LogiLight webcam \"" + light.getWebcamName() + "\"";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        try {
            uninstall();
        } catch (Throwable ignored) {
        }
    }

    private void closeQuietly() {
        try {
            light.close();
        } catch (Throwable ignored) {
        }
        try {
            super.stop();
        } catch (Throwable ignored) {
        }
    }

    private LLResult fallbackResult() {
        try {
            JSONObject json = new JSONObject();
            json.put("v", 0);
            return new LogiLLResult(json, LogiResult.EMPTY);
        } catch (Throwable t) {
            return super.getLatestResult();
        }
    }

    private LLResultTypes.CalibrationResult calibration() {
        try {
            CameraModel model = light.getCameraModel();
            double[] matrix = {model.fx, 0.0, model.cx, 0.0, model.fy, model.cy, 0.0, 0.0, 1.0};
            String source = light.getStatus().intrinsicsSource;
            return new LLResultTypes.CalibrationResult("LogiLight " + source, model.width, model.height, 0.0,
                    matrix, new double[5]);
        } catch (Throwable t) {
            return new LLResultTypes.CalibrationResult();
        }
    }

    private static int snapshotCount() {
        try {
            File[] files = LogiLightStore.snapshotDirectory().listFiles();
            if (files == null) {
                return 0;
            }
            int count = 0;
            for (File file : files) {
                if (file.isFile() && file.getName().toLowerCase(Locale.US).endsWith(SNAPSHOT_EXTENSION)) {
                    count++;
                }
            }
            return count;
        } catch (Throwable t) {
            return 0;
        }
    }

    public static String pipelineTypeName(PipelineType type) {
        if (type == null) {
            return "pipe_fiducial";
        }
        switch (type) {
            case COLOR:
                return "pipe_color";
            case SCRIPT:
                return "pipe_python";
            case DRIVER:
                return "pipe_driver";
            case APRILTAG:
            case APRILTAG_AND_COLOR:
            default:
                return "pipe_fiducial";
        }
    }

    public static JSONObject toLimelightJson(LogiResult result) throws JSONException {
        LogiResult r = result == null ? LogiResult.EMPTY : result;
        JSONObject json = new JSONObject();
        json.put("v", r.isValid() ? 1 : 0);
        json.put("pID", r.getPipelineIndex());
        json.put("pipelineType", pipelineTypeName(r.getPipelineType()));
        json.put("tx", num(r.getTx()));
        json.put("ty", num(r.getTy()));
        json.put("txnc", num(r.getTxNC()));
        json.put("tync", num(r.getTyNC()));
        json.put("ta", num(r.getTa()));
        json.put("tl", num(r.getTargetingLatency()));
        json.put("cl", 0.0);
        json.put("ts", num(r.getCaptureNanos() / 1e6));

        LogiBotPose mt1 = r.getBotpose();
        LogiBotPose mt2 = r.getBotpose_MT2();
        if (mt1 != null && mt1.pose != null) {
            json.put("botpose", poseArray(fieldPose(mt1.pose)));
            json.put("stdev_mt1", stddevArray(mt1));
        }
        if (mt2 != null && mt2.pose != null) {
            json.put("botpose_orb", poseArray(fieldPose(mt2.pose)));
            json.put("stdev_mt2", stddevArray(mt2));
        }
        LogiBotPose summary = mt1 != null ? mt1 : mt2;
        if (summary != null) {
            json.put("botpose_tagcount", summary.targetCount);
            json.put("botpose_span", num(summary.spanInches * METERS_PER_INCH));
            json.put("botpose_avgdist", num(summary.avgDistanceInches * METERS_PER_INCH));
            json.put("botpose_avgarea", num(summary.avgAreaPercent));
        }
        json.put("PythonOut", doubleArray(r.getScriptOutput()));

        JSONArray fiducials = new JSONArray();
        for (LogiFiducial f : r.getFiducialResults()) {
            fiducials.put(fiducialJson(f));
        }
        json.put("Fiducial", fiducials);

        JSONArray retro = new JSONArray();
        JSONArray detector = new JSONArray();
        for (ColorTarget t : r.getColorResults()) {
            JSONObject color = colorJson(t);
            retro.put(color);
            JSONObject detection = colorJson(t);
            detection.put("class", t.profileName == null ? "" : t.profileName);
            detection.put("classID", profileIndex(t.profileName));
            detection.put("conf", num(Math.max(0.0, Math.min(1.0, t.fullness)) * 100.0));
            detector.put(detection);
        }
        json.put("Retro", retro);
        json.put("Detector", detector);
        return json;
    }

    private static JSONObject fiducialJson(LogiFiducial f) throws JSONException {
        JSONObject json = new JSONObject();
        int id = f.isCluster ? ClusterLayout.firstTagId(f.name) : f.id;
        json.put("fID", id);
        json.put("fam", FIDUCIAL_FAMILY);
        json.put("skew", 0.0);
        json.put("tx", num(f.tx));
        json.put("ty", num(f.ty));
        json.put("tx_nocross", num(f.txNC));
        json.put("ty_nocross", num(f.tyNC));
        json.put("txp", num(f.centerX));
        json.put("typ", num(f.centerY));
        json.put("ta", num(f.ta));
        JSONArray points = new JSONArray();
        for (double[] quad : f.corners) {
            for (int k = 0; k + 1 < quad.length; k += 2) {
                JSONArray point = new JSONArray();
                point.put(num(quad[k]));
                point.put(num(quad[k + 1]));
                points.put(point);
            }
        }
        json.put("pts", points);
        if (f.targetInOptical != null && f.rotationInOptical != null) {
            json.put("t6t_cs", poseArray(f.targetInOptical, f.rotationInOptical));
            Mat3 inverse = f.rotationInOptical.transpose();
            json.put("t6c_ts", poseArray(inverse.times(f.targetInOptical).negated(), inverse));
        }
        if (f.targetInRobot != null && f.rotationInRobot != null) {
            json.put("t6t_rs", poseArray(f.targetInRobot, f.rotationInRobot));
            Mat3 inverse = f.rotationInRobot.transpose();
            json.put("t6r_ts", poseArray(inverse.times(f.targetInRobot).negated(), inverse));
        }
        PlanarPose robot = f.mt1 != null ? f.mt1 : f.mt2;
        if (robot != null) {
            json.put("t6r_fs", poseArray(fieldPose(robot)));
        }
        json.put("name", f.name);
        json.put("percentClusterFound", f.percentClusterFound);
        return json;
    }

    private static JSONObject colorJson(ColorTarget t) throws JSONException {
        JSONObject json = new JSONObject();
        json.put("tx", num(t.tx));
        json.put("ty", num(t.ty));
        json.put("tx_nocross", num(t.txNC));
        json.put("ty_nocross", num(t.tyNC));
        json.put("txp", num(t.centerX));
        json.put("typ", num(t.centerY));
        json.put("ta", num(t.ta));
        JSONArray points = new JSONArray();
        double[] corners = t.corners;
        if (corners != null) {
            for (int k = 0; k + 1 < corners.length; k += 2) {
                JSONArray point = new JSONArray();
                point.put(num(corners[k]));
                point.put(num(corners[k + 1]));
                points.put(point);
            }
        }
        json.put("pts", points);
        if (t.hasGroundPosition()) {
            double[] ground = {
                    t.groundForwardInches * METERS_PER_INCH,
                    t.groundLeftInches * METERS_PER_INCH,
                    0.0, 0.0, 0.0,
                    t.groundBearingLeftDegrees};
            json.put("t6t_rs", doubleArray(ground));
        }
        return json;
    }

    private static int profileIndex(String profileName) {
        ColorProfile[] profiles = LogiLightSettings.COLOR_PROFILES;
        if (profileName == null || profiles == null) {
            return -1;
        }
        for (int i = 0; i < profiles.length; i++) {
            ColorProfile profile = profiles[i];
            if (profile != null && profile.name != null && profile.name.trim().equalsIgnoreCase(profileName.trim())) {
                return i;
            }
        }
        return -1;
    }

    private static double[] fieldPose(PlanarPose pose) {
        double headingDegrees = pose.hasHeading() ? Math.toDegrees(pose.heading) : 0.0;
        return new double[]{
                (pose.x - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                (pose.y - FIELD_CENTER_INCHES) * METERS_PER_INCH,
                0.0, 0.0, 0.0,
                headingDegrees};
    }

    private static JSONArray poseArray(double[] pose) throws JSONException {
        return doubleArray(pose);
    }

    private static JSONArray poseArray(Vec3 positionInches, Mat3 rotation) throws JSONException {
        return doubleArray(new double[]{
                positionInches.x * METERS_PER_INCH,
                positionInches.y * METERS_PER_INCH,
                positionInches.z * METERS_PER_INCH,
                Math.toDegrees(rotation.roll()),
                Math.toDegrees(rotation.pitch()),
                Math.toDegrees(rotation.yaw())});
    }

    private static JSONArray stddevArray(LogiBotPose pose) throws JSONException {
        double sigma = pose.stddevXYInches * METERS_PER_INCH;
        double headingDegrees = Math.toDegrees(pose.stddevHeadingRadians);
        return doubleArray(new double[]{sigma, sigma, 0.0, 0.0, 0.0, headingDegrees});
    }

    private static JSONArray doubleArray(double[] values) throws JSONException {
        JSONArray array = new JSONArray();
        if (values != null) {
            for (double value : values) {
                array.put(num(value));
            }
        }
        return array;
    }

    private static double num(double value) {
        return Double.isNaN(value) || Double.isInfinite(value) ? 0.0 : value;
    }

    private static String describe(Throwable t) {
        String message = t.getMessage();
        String type = t.getClass().getSimpleName();
        return message == null || message.isEmpty() ? type : type + ": " + message;
    }
}
