package org.firstinspires.ftc.teamcode.vision.logilight.compat;

import com.qualcomm.hardware.limelightvision.LLStatus;

import org.firstinspires.ftc.teamcode.vision.logilight.LogiStatus;
import org.json.JSONObject;

public final class LogiLLStatus extends LLStatus {

    public static final String DEVICE_NAME = "LogiLight";

    private final LogiStatus source;

    public LogiLLStatus(JSONObject json) {
        this(json, null);
    }

    public LogiLLStatus(JSONObject json, LogiStatus source) {
        super(json);
        this.source = source == null ? LogiStatus.UNKNOWN : source;
    }

    public static LogiLLStatus from(LogiStatus status, double headingDegrees, int snapshotCount) {
        LogiStatus s = status == null ? LogiStatus.UNKNOWN : status;
        JSONObject json = new JSONObject();
        try {
            json.put("fps", finite(s.fps));
            json.put("pipelineIndex", s.pipelineIndex);
            json.put("pipelineType", LogiLimelight3A.pipelineTypeName(s.pipelineType));
            json.put("name", DEVICE_NAME);
            json.put("cpu", 0.0);
            json.put("temp", 0.0);
            json.put("ram", 0.0);
            json.put("finalYaw", finite(headingDegrees));
            json.put("pipeImgCount", Math.max(0, snapshotCount));
            json.put("hwType", 0);
            json.put("snapshotMode", 0);
        } catch (Exception ignored) {
        }
        return new LogiLLStatus(json, s);
    }

    public LogiStatus getLogiStatus() {
        return source;
    }

    private static double finite(double value) {
        return Double.isNaN(value) || Double.isInfinite(value) ? 0.0 : value;
    }
}
