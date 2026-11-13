package org.firstinspires.ftc.teamcode.vision.logilight;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;
import org.firstinspires.ftc.teamcode.vision.logilight.opmode.LogiOpModeTools;
import org.firstinspires.ftc.teamcode.vision.logilight.opmode.PushTestGate;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class PollenSeeker {

    public static final String YELLOW_PROFILE = "yellow";
    public static final String POLLEN_PIPELINE_NAME = "pollen-yellow";

    public enum State { IDLE, SCANNING, TURNING, FOLLOWING, FINAL_APPROACH, INTAKING, DONE, ABORTED }

    public interface IntakeControl extends PollenSeekEngine.Intake {
    }

    private final LogiLight light;
    private final Follower follower;
    private final PollenSeekConfig config;
    private final PollenSeekEngine engine;
    private PushTestGate pushTest;

    private int pipelineIndex = -1;
    private String pipelineNote = "";
    private String profileNote = "";
    private String cameraNote = "";
    private String frameNote = "no frames yet";
    private boolean mountIsCodeDefault;
    private boolean cameraLooksUp;
    private long lastCaptureNanos;
    private int lastAccepted;
    private int lastNoGround;
    private int lastRange;
    private int lastNotRound;
    private int lastEdge;
    private int lastSize;
    private int lastOtherColor;
    private long noGroundFrames;
    private long acceptedTotal;

    public PollenSeeker(LogiLight light, Follower follower, PollenSeekConfig config) {
        this.light = light;
        this.follower = follower;
        this.config = config == null ? PollenSeekSettings.current() : config;
        this.engine = new PollenSeekEngine(follower, this.config, PollenSeekEngine.SYSTEM_CLOCK);
        updateCameraChecks();
    }

    public void setIntake(IntakeControl intake) {
        engine.setIntake(intake);
    }

    public void setPushTestGate(PushTestGate gate) {
        pushTest = gate;
    }

    public void setIntakeOnArrive(boolean enabled) {
        engine.setIntakeOnArrive(enabled);
    }

    public boolean intakeOnArrive() {
        return engine.intakeOnArrive();
    }

    public void recordPose() {
        engine.recordPose();
    }

    public void resetPoseHistory() {
        engine.resetPoseHistory();
        lastCaptureNanos = 0L;
    }

    public void observe() {
        if (light == null || !light.isAvailable()) {
            updateCameraChecks();
            frameNote = "camera not available";
            return;
        }
        ColorProfile yellow = lockYellow();
        updateCameraChecks();
        PollenSeekConfig cfg = engine.config();
        LogiResult result = light.getLatestResult();
        if (!LogiOpModeTools.isFresh(result, pipelineIndex, cfg.maxFrameAgeMs)) {
            frameNote = pipelineIndex < 0 ? "no COLOR pipeline" : "waiting for a fresh frame on " + POLLEN_PIPELINE_NAME;
            return;
        }
        if (result.getPipelineType() != PipelineType.COLOR) {
            frameNote = "result is not from a COLOR pipeline";
            return;
        }
        long capture = result.getCaptureNanos();
        if (capture == lastCaptureNanos) {
            return;
        }
        lastCaptureNanos = capture;
        CameraModel model = light.getCameraModel();
        double centerHeight = yellow != null && finite(yellow.targetCenterHeightInches) && yellow.targetCenterHeightInches > 0.0
                ? yellow.targetCenterHeightInches : cfg.ballDiameterInches / 2.0;
        List<PollenSeekEngine.Detection> accepted = new ArrayList<PollenSeekEngine.Detection>();
        int noGround = 0;
        int range = 0;
        int notRound = 0;
        int edge = 0;
        int size = 0;
        int otherColor = 0;
        for (ColorTarget target : result.getColorResults()) {
            if (target == null) {
                continue;
            }
            String reason = rejectReason(target, model, cfg, centerHeight);
            if (reason == null) {
                accepted.add(new PollenSeekEngine.Detection(target.groundForwardInches, target.groundLeftInches,
                        target.groundDistanceInches, target.groundBearingLeftDegrees));
            } else if (REJECT_COLOR.equals(reason)) {
                otherColor++;
            } else if (REJECT_GROUND.equals(reason)) {
                noGround++;
            } else if (REJECT_RANGE.equals(reason)) {
                range++;
            } else if (REJECT_ROUND.equals(reason)) {
                notRound++;
            } else if (REJECT_EDGE.equals(reason)) {
                edge++;
            } else {
                size++;
            }
        }
        lastAccepted = accepted.size();
        lastNoGround = noGround;
        lastRange = range;
        lastNotRound = notRound;
        lastEdge = edge;
        lastSize = size;
        lastOtherColor = otherColor;
        acceptedTotal += accepted.size();
        if (noGround > 0 && accepted.isEmpty()) {
            noGroundFrames++;
        }
        frameNote = String.format(Locale.US, "frame age %.0f ms", result.getStaleness());
        engine.observeFrame(capture, accepted);
    }

    private static final String REJECT_COLOR = "not yellow";
    private static final String REJECT_GROUND = "no floor position";
    private static final String REJECT_RANGE = "out of range";
    private static final String REJECT_ROUND = "not round";
    private static final String REJECT_EDGE = "touches the image edge";
    private static final String REJECT_SIZE = "wrong size for its distance";

    static String rejectReason(ColorTarget target, CameraModel model, PollenSeekConfig cfg, double centerHeight) {
        String profile = target.profileName == null ? "" : target.profileName.trim();
        if (!YELLOW_PROFILE.equalsIgnoreCase(profile)) {
            return REJECT_COLOR;
        }
        if (!target.hasGroundPosition()) {
            return REJECT_GROUND;
        }
        double distance = target.groundDistanceInches;
        if (distance < cfg.minRangeInches || distance > cfg.maxRangeInches) {
            return REJECT_RANGE;
        }
        if (!(target.circularity >= cfg.minCircularity)) {
            return REJECT_ROUND;
        }
        if (model != null) {
            double margin = cfg.borderMarginPixels;
            if (target.boxLeft <= margin || target.boxTop <= margin
                    || target.boxLeft + target.boxWidth >= model.width - margin
                    || target.boxTop + target.boxHeight >= model.height - margin) {
                return REJECT_EDGE;
            }
            double dx = target.groundForwardInches - model.forwardInches;
            double dy = target.groundLeftInches - model.leftInches;
            double dz = centerHeight - model.upInches;
            double slant = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (slant > 1e-6 && model.fy > 0.0) {
                double expected = model.fy * cfg.ballDiameterInches / slant;
                double measured = Math.max(target.boxWidth, target.boxHeight);
                double ratio = measured / expected;
                if (!(ratio >= cfg.minSizeRatio && ratio <= cfg.maxSizeRatio)) {
                    return REJECT_SIZE;
                }
            }
        }
        return null;
    }

    private ColorProfile lockYellow() {
        int index = LogiLightSettings.findPipeline(POLLEN_PIPELINE_NAME);
        LogiPipeline pipeline = LogiLightSettings.pipelineAt(index);
        if (pipeline == null || pipeline.typeOrDefault() != PipelineType.COLOR) {
            index = LogiOpModeTools.firstPipelineOfType(PipelineType.COLOR);
            pipelineNote = index >= 0
                    ? "no COLOR pipeline named " + POLLEN_PIPELINE_NAME + " - using " + LogiOpModeTools.pipelineLabel(index)
                    : "!! no COLOR pipeline in LogiLightSettings.PIPELINES - POLLEN cannot be seen";
        } else {
            pipelineNote = "";
        }
        pipelineIndex = index;
        if (index >= 0 && light.getPipelineIndex() != index) {
            light.pipelineSwitch(index);
        }
        ColorProfile yellow = LogiLightSettings.findColorProfile(YELLOW_PROFILE);
        LogiFrameContext context = light.getFrameContext();
        if (context != null) {
            context.setColorProfileOverride(yellow == null ? new ColorProfile[0] : new ColorProfile[]{yellow});
        }
        if (yellow == null) {
            profileNote = "!! no color profile named yellow in LogiLightSettings.COLOR_PROFILES - nothing is detected";
        } else if (!yellow.enabled) {
            profileNote = "!! yellow color profile is disabled (enabled = NO) - nothing is detected";
        } else {
            profileNote = "";
        }
        return yellow;
    }

    private void updateCameraChecks() {
        String refusal = null;
        if (light == null) {
            refusal = "camera not created";
            cameraNote = "!! camera not created";
        } else if (!light.isAvailable()) {
            refusal = "camera NOT AVAILABLE - " + light.fault();
            cameraNote = "!! camera NOT AVAILABLE - " + light.fault();
        } else {
            LogiCameraConfig camera = light.getCameraConfig();
            if (camera == null) {
                camera = LogiLightSettings.CAMERA;
            }
            LogiCameraConfig defaults = new LogiCameraConfig();
            if (camera == null) {
                cameraLooksUp = true;
                mountIsCodeDefault = true;
            } else {
                cameraLooksUp = !(camera.tiltUpDegrees < 0.0);
                mountIsCodeDefault = camera.upInches == defaults.upInches && camera.tiltUpDegrees == defaults.tiltUpDegrees
                        && camera.forwardInches == defaults.forwardInches;
            }
            if (cameraLooksUp) {
                refusal = "camera is not looking down (tiltUpDegrees >= 0) - run LogiLight Mount Calibration (FLOOR)";
            } else if (pipelineIndex < 0) {
                refusal = "no COLOR pipeline - POLLEN cannot be seen";
            }
            cameraNote = "";
        }
        engine.setCameraRefusal(refusal);
    }

    public State step(boolean driveAllowed) {
        updatePermit();
        engine.setDefaultIntakePower(Constants.INTAKE_POWER);
        return map(engine.step(driveAllowed));
    }

    public String refusalReason() {
        updatePermit();
        return engine.refusalReason();
    }

    private void updatePermit() {
        boolean permit;
        String reason;
        if (!PollenSeekSettings.REQUIRE_PUSH_TEST) {
            permit = true;
            reason = "";
        } else if (pushTest == null) {
            permit = false;
            reason = "push test not run (REQUIRE_PUSH_TEST)";
        } else {
            permit = pushTest.driveEnabled();
            reason = permit ? "" : "Pinpoint push test " + pushTest.shortStatus();
        }
        engine.setPermit(permit, reason);
    }

    public void cancel() {
        engine.cancel();
    }

    public void shutdown() {
        engine.shutdown();
        releaseCamera();
    }

    public void releaseCamera() {
        if (light != null) {
            try {
                LogiFrameContext context = light.getFrameContext();
                if (context != null) {
                    context.clearColorProfileOverride();
                }
            } catch (Throwable ignored) {
            }
        }
    }

    public boolean canSeePollen() {
        return engine.canSeePollen();
    }

    public PollenSeekEngine.FieldBall currentTarget() {
        return engine.currentTarget();
    }

    public State state() {
        return map(engine.state());
    }

    public boolean isMoving() {
        return engine.isMotionState();
    }

    public String status() {
        return engine.status();
    }

    public String abortReason() {
        return engine.abortReason();
    }

    public String lastEvent() {
        return engine.lastEvent();
    }

    public Pose plannedTarget() {
        return engine.plannedTarget();
    }

    public List<Pose> plannedVia() {
        return engine.plannedVia();
    }

    public String planNote() {
        return engine.planNote();
    }

    public String guardLine() {
        return engine.guardLine();
    }

    public String capLine() {
        return engine.capLine();
    }

    public String visibilityLine() {
        return engine.visibilityLine();
    }

    public int lostCount() {
        return engine.lostCount();
    }

    public int attempts() {
        return engine.attempts();
    }

    public boolean hasIntake() {
        return engine.hasIntake();
    }

    public double intakePower() {
        return engine.intakePower();
    }

    public PollenSeekEngine engine() {
        return engine;
    }

    public String rejectLine() {
        return String.format(Locale.US, "accepted %d  rejected: no floor %d  range %d  not round %d  edge %d  size %d  other color %d",
                lastAccepted, lastNoGround, lastRange, lastNotRound, lastEdge, lastSize, lastOtherColor);
    }

    public String frameNote() {
        return frameNote;
    }

    public List<String> warnings() {
        List<String> lines = new ArrayList<String>();
        if (!cameraNote.isEmpty()) {
            lines.add(cameraNote);
        }
        if (!pipelineNote.isEmpty()) {
            lines.add(pipelineNote);
        }
        if (!profileNote.isEmpty()) {
            lines.add(profileNote);
        }
        if (light != null && light.isAvailable()) {
            if (mountIsCodeDefault) {
                lines.add("!! camera mount not set - run LogiLight Mount Calibration (FLOOR) first");
            }
            if (cameraLooksUp) {
                lines.add("!! camera tilt is not negative (not looking down) - seeking is refused");
            }
            if (lastNoGround > 0 && lastAccepted == 0) {
                lines.add("!! camera sees pollen but has no floor position: camera must look down and mount values must be set");
            }
        }
        if (follower == null) {
            lines.add("!! no Pinpoint / Follower - detection only, nothing will drive");
        } else if (!engine.holdEndDisabled()) {
            lines.add("!! could not turn off the Follower end hold - driving refused");
        }
        String latched = engine.localizerSuspect() ? engine.localizerSuspectReason() : null;
        if (latched != null) {
            lines.add("!! seeking OFF for this run - restart the OpMode and redo the push test: " + latched);
        }
        String fault = engine.poseFault();
        if (fault != null && !fault.equals(latched)) {
            lines.add(fault);
        }
        return lines;
    }

    public void reportLocalizerFault(String reason) {
        engine.reportLocalizerFault(reason);
    }

    public String pipelineLine() {
        return light == null ? "none" : LogiOpModeTools.pipelineLabel(light.getPipelineIndex()) + "  colors: yellow only";
    }

    private static State map(PollenSeekEngine.State state) {
        return state == null ? State.IDLE : State.values()[state.ordinal()];
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
