package org.firstinspires.ftc.teamcode.vision.logilight.compat;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.json.JSONException;
import org.json.JSONObject;

public final class LogiLLResult extends LLResult {

    private static final double NANOS_PER_MILLI = 1e6;
    private static final long NANOS_PER_MILLI_LONG = 1000000L;

    private final LogiResult source;
    private final long resultWallMillis;
    private final long captureNanos;

    public LogiLLResult(JSONObject json) throws JSONException {
        this(json, null);
    }

    public LogiLLResult(JSONObject json, LogiResult source) throws JSONException {
        super(json == null ? new JSONObject() : json);
        this.source = source == null ? LogiResult.EMPTY : source;
        long nowMillis = System.currentTimeMillis();
        long capture = this.source.getCaptureNanos();
        if (capture > 0L) {
            double ageMs = Math.max(0.0, (System.nanoTime() - capture) / NANOS_PER_MILLI);
            double tl = this.source.getTargetingLatency();
            double readyAgeMs = tl > 0.0 && !Double.isInfinite(tl) ? Math.max(0.0, ageMs - tl) : ageMs;
            this.resultWallMillis = nowMillis - Math.round(readyAgeMs);
            this.captureNanos = capture;
        } else {
            this.resultWallMillis = nowMillis;
            this.captureNanos = System.nanoTime();
        }
    }

    @Override
    public long getControlHubTimeStamp() {
        return resultWallMillis;
    }

    @Override
    public long getControlHubTimeStampNanos() {
        return resultWallMillis * NANOS_PER_MILLI_LONG;
    }

    @Override
    public long getStaleness() {
        return System.currentTimeMillis() - resultWallMillis;
    }

    public long getCaptureNanos() {
        return captureNanos;
    }

    public LogiResult getLogiResult() {
        return source;
    }
}
