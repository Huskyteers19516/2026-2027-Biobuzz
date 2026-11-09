package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.logilight.ColorProfile;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorSample;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorSpaceMode;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFrameContext;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiPipeline;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiStatus;
import org.firstinspires.ftc.teamcode.vision.logilight.PipelineType;
import org.firstinspires.ftc.teamcode.vision.logilight.StreamView;
import org.firstinspires.ftc.teamcode.vision.logilight.TargetSort;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "LogiLight Color Tuner", group = "LogiLight")
public class LogiLightColorTuner extends LogiOpModeBase {

    private static final double STICK_DEADZONE = 0.1;
    private static final double WAND_TIMEOUT_SECONDS = 1.5;
    private static final int DEFAULT_WAND_RADIUS_PX = 6;
    private static final double MAX_WAND_MARGIN = 128.0;
    private static final int KNOB_WINDOW = 3;
    private static final int MAX_BLOB_LINES = 3;
    private static final double HUE_RANGE = ColorSpaceMode.HUE_MAX;
    private static final double[] HSV_WAND_MARGINS = {8.0, 60.0, 60.0};
    private static final double[] YCRCB_WAND_MARGINS = {60.0, 14.0, 14.0};
    private static final double[] RGB_WAND_MARGINS = {40.0, 40.0, 40.0};

    private interface ProfileReader {
        double read(ColorProfile profile);
    }

    private interface ProfileWriter {
        void write(ColorProfile profile, double value);
    }

    private interface ProfileText {
        String text(ColorProfile profile);
    }

    private static final class Knob {
        final ProfileText label;
        final String help;
        final ProfileReader reader;
        final ProfileWriter writer;
        final ProfileReader upperLimit;
        final double lowerLimit;
        final double step;
        final double ratePerSecond;
        final int decimals;
        final int choices;
        final ProfileText choiceText;

        Knob(ProfileText label, String help, ProfileReader reader, ProfileWriter writer, ProfileReader upperLimit,
             double lowerLimit, double step, double ratePerSecond, int decimals, int choices, ProfileText choiceText) {
            this.label = label;
            this.help = help;
            this.reader = reader;
            this.writer = writer;
            this.upperLimit = upperLimit;
            this.lowerLimit = lowerLimit;
            this.step = step;
            this.ratePerSecond = ratePerSecond;
            this.decimals = decimals;
            this.choices = choices;
            this.choiceText = choiceText;
        }

        boolean isChoice() {
            return choices > 0;
        }

        boolean isWhole() {
            return decimals == 0 && !isChoice();
        }

        double get(ColorProfile profile) {
            return reader.read(profile);
        }

        void set(ColorProfile profile, double value) {
            if (Double.isNaN(value) || Double.isInfinite(value)) {
                return;
            }
            if (isChoice()) {
                int index = (int) Math.round(value);
                index = ((index % choices) + choices) % choices;
                writer.write(profile, index);
                return;
            }
            double upper = upperLimit.read(profile);
            double scale = Math.pow(10.0, decimals);
            double rounded = Math.round(value * scale) / scale;
            writer.write(profile, Math.max(lowerLimit, Math.min(upper, rounded)));
        }

        void stepBy(ColorProfile profile, int direction) {
            set(profile, get(profile) + direction * (isChoice() ? 1.0 : step));
        }

        String valueText(ColorProfile profile) {
            if (isChoice()) {
                return choiceText.text(profile);
            }
            return String.format(Locale.US, "%." + decimals + "f", get(profile));
        }
    }

    private final double[][] wandMargins = {
            HSV_WAND_MARGINS.clone(), YCRCB_WAND_MARGINS.clone(), RGB_WAND_MARGINS.clone()};
    private final ElapsedTime wandTimer = new ElapsedTime();
    private final List<Knob> knobs = new ArrayList<Knob>();

    private int wandRadiusPx = DEFAULT_WAND_RADIUS_PX;
    private int profileIndex = 0;
    private int knobIndex = 0;
    private boolean solo = true;
    private boolean thresholdHeld = false;
    private StreamView viewBeforeThreshold = StreamView.ANNOTATED;
    private boolean wandPending = false;
    private boolean wandExpand = false;
    private ColorSpaceMode wandSpace = ColorSpaceMode.HSV;
    private ColorProfile wandProfile;
    private ColorSample lastSample;
    private double stickAccumulator = 0.0;
    private String pipelineNote = "";

    @Override
    protected void onInit() {
        buildKnobs();
        ColorProfile profile = currentProfile();
        if (profile != null) {
            selectPipelineFor(profile);
        }
    }

    @Override
    protected void onLoop() {
        ColorProfile profile = currentProfile();
        if (profile == null) {
            telemetry.addLine("!! LogiLightSettings.COLOR_PROFILES is empty - nothing to tune");
            addHint(SETTINGS_HINT);
            return;
        }
        handleProfileButtons();
        profile = currentProfile();
        if (profile == null) {
            return;
        }
        LogiPipeline pipeline = ll.getPipeline();
        if (pipeline == null || !pipeline.typeOrDefault().usesColor()) {
            selectPipelineFor(profile);
        }
        applyOverride(profile);
        handleKnobButtons(profile);
        handleStick(profile);
        handleWandButtons(profile);
        handleStreamButtons();
        serviceWand();
        addTelemetry(profile);
    }

    @Override
    protected void onStop() {
        try {
            ll.getFrameContext().clearColorProfileOverride();
        } catch (Throwable ignored) {
        }
    }

    @Override
    protected void onSettingsReset() {
        wandPending = false;
        wandProfile = null;
        profileIndex = 0;
        ColorProfile profile = currentProfile();
        if (profile != null) {
            selectPipelineFor(profile);
            applyOverride(profile);
        }
    }

    private ColorProfile currentProfile() {
        ColorProfile[] profiles = LogiLightSettings.COLOR_PROFILES;
        if (profiles == null || profiles.length == 0) {
            return null;
        }
        if (profileIndex < 0 || profileIndex >= profiles.length) {
            profileIndex = 0;
        }
        ColorProfile profile = profiles[profileIndex];
        if (profile == null) {
            profile = new ColorProfile();
            profiles[profileIndex] = profile;
        }
        if (profile.colorSpace == null) {
            profile.colorSpace = ColorSpaceMode.HSV;
        }
        if (profile.sort == null) {
            profile.sort = TargetSort.LARGEST;
        }
        return profile;
    }

    private void handleProfileButtons() {
        ColorProfile[] profiles = LogiLightSettings.COLOR_PROFILES;
        int count = profiles == null ? 0 : profiles.length;
        if (count == 0) {
            return;
        }
        int delta = 0;
        if (pad1.dpadRight()) {
            delta = 1;
        }
        if (pad1.dpadLeft()) {
            delta = -1;
        }
        if (delta == 0) {
            return;
        }
        profileIndex = ((profileIndex + delta) % count + count) % count;
        ColorProfile profile = currentProfile();
        if (profile != null) {
            selectPipelineFor(profile);
            applyOverride(profile);
            showMessage("tuning profile " + profile.name);
        }
        stickAccumulator = 0.0;
    }

    private void selectPipelineFor(ColorProfile profile) {
        int count = LogiLightSettings.pipelineCount();
        int best = -1;
        for (int pass = 0; pass < 4 && best < 0; pass++) {
            for (int i = 0; i < count && best < 0; i++) {
                LogiPipeline candidate = LogiLightSettings.pipelineAt(i);
                if (candidate == null) {
                    continue;
                }
                PipelineType type = candidate.typeOrDefault();
                boolean listsProfile = candidate.usesColorProfile(profile.name);
                boolean matches;
                if (pass == 0) {
                    matches = type == PipelineType.COLOR && listsProfile;
                } else if (pass == 1) {
                    matches = type.usesColor() && listsProfile;
                } else if (pass == 2) {
                    matches = type == PipelineType.COLOR;
                } else {
                    matches = type.usesColor();
                }
                if (matches) {
                    best = i;
                }
            }
        }
        if (best < 0) {
            pipelineNote = "!! no COLOR pipeline in LogiLightSettings.PIPELINES - add one to see blobs";
            return;
        }
        if (best != ll.getPipelineIndex() || ll.getPipeline() == null || !ll.getPipeline().typeOrDefault().usesColor()) {
            ll.pipelineSwitch(best);
        }
        LogiPipeline chosen = LogiLightSettings.pipelineAt(best);
        pipelineNote = chosen != null && !chosen.usesColorProfile(profile.name)
                ? "pipeline " + chosen.name + " does not list \"" + profile.name + "\" - only solo mode shows it"
                : "";
    }

    private void applyOverride(ColorProfile profile) {
        LogiFrameContext context = ll.getFrameContext();
        if (solo) {
            context.setColorProfileOverride(profile);
        } else {
            context.clearColorProfileOverride();
        }
    }

    private void handleKnobButtons(ColorProfile profile) {
        boolean shift = comboHeld();
        if (!shift && pad1.a()) {
            knobIndex = (knobIndex + 1) % knobs.size();
            stickAccumulator = 0.0;
        }
        if (!shift && pad1.y()) {
            knobIndex = (knobIndex + knobs.size() - 1) % knobs.size();
            stickAccumulator = 0.0;
        }
        Knob knob = knobs.get(knobIndex);
        if (pad1.dpadUp()) {
            knob.stepBy(profile, 1);
        }
        if (pad1.dpadDown()) {
            knob.stepBy(profile, -1);
        }
        if (shift && pad1.a()) {
            restoreFactory(profile);
            showMessage("profile " + profile.name + " restored to code defaults: " + profile.rangeSummary());
        }
        if (pad1.rightBumper()) {
            solo = !solo;
            applyOverride(profile);
            showMessage(solo ? "solo: only " + profile.name + " is detected" : "pipeline profiles: " + pipelineProfiles());
        }
    }

    private void handleStick(ColorProfile profile) {
        Knob knob = knobs.get(knobIndex);
        double stick = gamepad1 == null ? 0.0 : -gamepad1.right_stick_y;
        if (Math.abs(stick) < STICK_DEADZONE || knob.isChoice()) {
            stickAccumulator = 0.0;
            return;
        }
        double shaped = Math.signum(stick) * stick * stick;
        double amount = shaped * knob.ratePerSecond * loopSeconds;
        if (knob.isWhole()) {
            stickAccumulator += amount;
            int whole = (int) stickAccumulator;
            if (whole != 0) {
                knob.set(profile, knob.get(profile) + whole);
                stickAccumulator -= whole;
            }
            return;
        }
        double minimum = Math.pow(10.0, -knob.decimals);
        stickAccumulator += amount;
        if (Math.abs(stickAccumulator) >= minimum) {
            knob.set(profile, knob.get(profile) + stickAccumulator);
            stickAccumulator = 0.0;
        }
    }

    private void handleWandButtons(ColorProfile profile) {
        if (!pad1.b()) {
            return;
        }
        startWand(profile, comboHeld());
    }

    private void handleStreamButtons() {
        boolean wantThreshold = gamepad1 != null && gamepad1.x && !comboHeld();
        if (wantThreshold && !thresholdHeld) {
            viewBeforeThreshold = ll.getStreamView();
            ll.setStreamView(StreamView.THRESHOLD);
            thresholdHeld = true;
        } else if (!wantThreshold && thresholdHeld) {
            ll.setStreamView(viewBeforeThreshold == StreamView.THRESHOLD ? StreamView.ANNOTATED : viewBeforeThreshold);
            thresholdHeld = false;
        }
        if (pad1.back()) {
            StreamView next = ll.getStreamView().next();
            ll.setStreamView(next);
            if (thresholdHeld) {
                viewBeforeThreshold = next;
            }
            showMessage("stream view " + next);
        }
    }

    private void startWand(ColorProfile profile, boolean expand) {
        if (!ll.isAvailable()) {
            showMessage("!! wand needs the camera");
            return;
        }
        LogiFrameContext context = ll.getFrameContext();
        wandSpace = profile.colorSpace == null ? ColorSpaceMode.HSV : profile.colorSpace;
        wandProfile = profile;
        wandExpand = expand;
        context.clearColorSample();
        context.requestColorSample(wandSpace, wandRadiusPx);
        wandPending = true;
        wandTimer.reset();
        showMessage("wand: sampling at the crosshair ...");
    }

    private void serviceWand() {
        if (!wandPending) {
            return;
        }
        ColorSample sample = ll.getFrameContext().latestColorSample();
        if (sample != null && sample.colorSpace == wandSpace) {
            wandPending = false;
            lastSample = sample;
            ColorProfile target = wandProfile;
            wandProfile = null;
            if (target == null) {
                return;
            }
            if (!sample.isValid()) {
                showMessage("!! wand: no pixels in the sample patch");
                return;
            }
            double[] margins = marginsFor(wandSpace);
            if (wandExpand && target.colorSpace == sample.colorSpace) {
                expandRange(target, sample, margins);
                showMessage("wand EXPANDED range: " + target.rangeSummary());
            } else {
                sample.applyRange(target, margins[0], margins[1], margins[2]);
                showMessage("wand SET range: " + target.rangeSummary());
            }
            return;
        }
        if (wandTimer.seconds() > WAND_TIMEOUT_SECONDS) {
            wandPending = false;
            wandProfile = null;
            showMessage("!! wand timed out - is the camera streaming?");
        }
    }

    private double[] marginsFor(ColorSpaceMode space) {
        ColorSpaceMode mode = space == null ? ColorSpaceMode.HSV : space;
        return wandMargins[mode.ordinal()];
    }

    private static void expandRange(ColorProfile profile, ColorSample sample, double[] margins) {
        ColorProfile probe = profile.copy();
        sample.applyRange(probe, margins[0], margins[1], margins[2]);
        if (profile.colorSpace == ColorSpaceMode.HSV) {
            unionHue(profile, probe);
        } else {
            profile.min0 = Math.min(profile.min0, probe.min0);
            profile.max0 = Math.max(profile.max0, probe.max0);
        }
        profile.min1 = Math.min(profile.min1, probe.min1);
        profile.max1 = Math.max(profile.max1, probe.max1);
        profile.min2 = Math.min(profile.min2, probe.min2);
        profile.max2 = Math.max(profile.max2, probe.max2);
    }

    private static void unionHue(ColorProfile profile, ColorProfile probe) {
        double startA = wrapHue(profile.min0);
        double lengthA = arcLength(profile.min0, profile.max0);
        double startB = wrapHue(probe.min0);
        double lengthB = arcLength(probe.min0, probe.max0);
        double coverFromA = Math.max(lengthA, forwardDistance(startA, startB) + lengthB);
        double coverFromB = Math.max(lengthB, forwardDistance(startB, startA) + lengthA);
        double start = coverFromA <= coverFromB ? startA : startB;
        double length = Math.min(coverFromA, coverFromB);
        if (length >= HUE_RANGE) {
            profile.min0 = 0.0;
            profile.max0 = HUE_RANGE;
            return;
        }
        double end = start + length;
        profile.min0 = start;
        profile.max0 = end > HUE_RANGE ? end - HUE_RANGE : end;
    }

    private static double arcLength(double min, double max) {
        return min <= max ? max - min : max + HUE_RANGE - min;
    }

    private static double forwardDistance(double from, double to) {
        double distance = (to - from) % HUE_RANGE;
        return distance < 0.0 ? distance + HUE_RANGE : distance;
    }

    private static double wrapHue(double hue) {
        double wrapped = hue % HUE_RANGE;
        return wrapped < 0.0 ? wrapped + HUE_RANGE : wrapped;
    }

    private static void restoreFactory(ColorProfile profile) {
        String name = profile.name == null ? "" : profile.name.trim();
        ColorProfile factory;
        if (name.equalsIgnoreCase("red")) {
            factory = ColorProfile.nectarRed();
        } else if (name.equalsIgnoreCase("blue")) {
            factory = ColorProfile.nectarBlue();
        } else if (name.equalsIgnoreCase("yellow")) {
            factory = ColorProfile.pollenYellow();
        } else {
            factory = new ColorProfile();
        }
        profile.enabled = factory.enabled;
        profile.colorSpace = factory.colorSpace;
        profile.setRange(factory.min0, factory.min1, factory.min2, factory.max0, factory.max1, factory.max2);
        profile.blurSize = factory.blurSize;
        profile.erodeSize = factory.erodeSize;
        profile.dilateSize = factory.dilateSize;
        profile.minAreaPercent = factory.minAreaPercent;
        profile.maxAreaPercent = factory.maxAreaPercent;
        profile.minFullness = factory.minFullness;
        profile.minAspectRatio = factory.minAspectRatio;
        profile.maxAspectRatio = factory.maxAspectRatio;
        profile.minCircularity = factory.minCircularity;
        profile.speckleRejectPercent = factory.speckleRejectPercent;
        profile.sort = factory.sort;
        profile.targetCenterHeightInches = factory.targetCenterHeightInches;
        profile.maxTargets = factory.maxTargets;
    }

    private String pipelineProfiles() {
        LogiPipeline pipeline = ll.getPipeline();
        return pipeline == null || pipeline.colorProfiles == null || pipeline.colorProfiles.isEmpty()
                ? "(none)" : pipeline.colorProfiles;
    }

    private void addTelemetry(ColorProfile profile) {
        ColorProfile[] profiles = LogiLightSettings.COLOR_PROFILES;
        int count = profiles == null ? 0 : profiles.length;
        telemetry.addData("Profile", "%d/%d %s  %s  (dpad L/R)", profileIndex + 1, count, profile.name,
                solo ? "[solo]" : "[pipeline: " + pipelineProfiles() + "]");
        addPipelineLine();
        if (!pipelineNote.isEmpty()) {
            telemetry.addLine(pipelineNote.startsWith("!!") ? pipelineNote : "   " + pipelineNote);
        }
        telemetry.addData("Range", profile.rangeSummary());
        if (!profile.enabled) {
            telemetry.addLine("!! profile is DISABLED - set \"enabled\" to YES to detect it");
        }
        addKnobWindow(profile);
        addBlobs(profile);
        double[] margins = marginsFor(profile.colorSpace);
        ColorSpaceMode space = profile.colorSpace == null ? ColorSpaceMode.HSV : profile.colorSpace;
        telemetry.addData("Wand", "%s  radius %d px  margins %s %.0f  %s %.0f  %s %.0f", wandPending ? "sampling ..." : "ready",
                wandRadiusPx, space.channelName(0), margins[0], space.channelName(1), margins[1],
                space.channelName(2), margins[2]);
        if (lastSample != null) {
            telemetry.addData("Last sample", lastSample.summary());
        }
        LogiStatus status = ll.getStatus();
        telemetry.addData("Stream", "%s%s  %.0f fps", ll.getStreamView(), thresholdHeld ? " (X held)" : "", status.fps);
        telemetry.addLine();
        addHint("gp1 A / Y = next / previous setting   dpad U/D = step   right stick = fast");
        addHint("gp1 B = wand: set range from the crosshair patch   LB + B = expand range to include it");
        addHint("hold X = threshold view   BACK = cycle stream view   RB = solo / pipeline profiles");
        addHint("LB + A = restore this profile to code defaults");
        addHint(SETTINGS_HINT);
        addHint("Aim the crosshair at a lit part of the ball, press B, then hold X and check only the ball is white.");
    }

    private void addKnobWindow(ColorProfile profile) {
        int first = Math.max(0, Math.min(knobIndex - KNOB_WINDOW, knobs.size() - (2 * KNOB_WINDOW + 1)));
        int last = Math.min(knobs.size() - 1, first + 2 * KNOB_WINDOW);
        for (int i = first; i <= last; i++) {
            Knob knob = knobs.get(i);
            String marker = i == knobIndex ? ">> " : "   ";
            telemetry.addLine(String.format(Locale.US, "%s%2d %s = %s", marker, i + 1, knob.label.text(profile),
                    knob.valueText(profile)));
        }
        telemetry.addData("Setting help", knobs.get(knobIndex).help);
    }

    private void addBlobs(ColorProfile profile) {
        LogiResult result = ll.getLatestResult();
        List<ColorTarget> targets = solo ? result.getColorResults(profile.name) : result.getColorResults();
        telemetry.addData("Blobs", "%d pass the filters%s", targets.size(), result.isExpired() ? "  !! stale" : "");
        int shown = 0;
        for (ColorTarget target : targets) {
            if (shown >= MAX_BLOB_LINES) {
                break;
            }
            String ground = target.hasGroundPosition()
                    ? String.format(Locale.US, "  ground %.1f in", target.groundDistanceInches) : "";
            telemetry.addLine(String.format(Locale.US, "   %s ta %.2f%%  full %.2f  aspect %.2f  circ %.2f  tx %.1f%s",
                    target.profileName, target.ta, target.fullness, target.aspectRatio, target.circularity, target.tx, ground));
            shown++;
        }
    }

    private void buildKnobs() {
        knobs.clear();
        for (int channel = 0; channel < 3; channel++) {
            knobs.add(channelKnob(channel, false));
            knobs.add(channelKnob(channel, true));
        }
        knobs.add(wholeKnob("blur", "Gaussian blur kernel (odd px); smooths noise before thresholding",
                p -> p.blurSize, (p, v) -> p.blurSize = (int) Math.round(v), 0.0, 31.0, 4.0));
        knobs.add(wholeKnob("erode", "erode px; removes thin noise and specks",
                p -> p.erodeSize, (p, v) -> p.erodeSize = (int) Math.round(v), 0.0, 31.0, 4.0));
        knobs.add(wholeKnob("dilate", "dilate px; fills holes and reconnects a ball",
                p -> p.dilateSize, (p, v) -> p.dilateSize = (int) Math.round(v), 0.0, 31.0, 4.0));
        knobs.add(numberKnob("min area %", "smallest blob, percent of the image",
                p -> p.minAreaPercent, (p, v) -> p.minAreaPercent = v, 0.0, 100.0, 0.01, 0.5, 3));
        knobs.add(numberKnob("max area %", "largest blob, percent of the image",
                p -> p.maxAreaPercent, (p, v) -> p.maxAreaPercent = v, 0.0, 100.0, 0.5, 10.0, 1));
        knobs.add(numberKnob("min fullness", "blob area / convex hull area (1 = solid, low = ragged)",
                p -> p.minFullness, (p, v) -> p.minFullness = v, 0.0, 1.0, 0.05, 0.5, 2));
        knobs.add(numberKnob("min aspect", "box width / height lower limit (ball = 1)",
                p -> p.minAspectRatio, (p, v) -> p.minAspectRatio = v, 0.0, 20.0, 0.05, 0.5, 2));
        knobs.add(numberKnob("max aspect", "box width / height upper limit (ball = 1)",
                p -> p.maxAspectRatio, (p, v) -> p.maxAspectRatio = v, 0.0, 20.0, 0.05, 0.5, 2));
        knobs.add(numberKnob("min circularity", "4 pi area / perimeter^2 (circle = 1)",
                p -> p.minCircularity, (p, v) -> p.minCircularity = v, 0.0, 1.0, 0.05, 0.5, 2));
        knobs.add(numberKnob("speckle reject %", "drop blobs smaller than this percent of the biggest blob",
                p -> p.speckleRejectPercent, (p, v) -> p.speckleRejectPercent = v, 0.0, 100.0, 1.0, 20.0, 0));
        knobs.add(wholeKnob("max targets", "how many blobs to report",
                p -> p.maxTargets, (p, v) -> p.maxTargets = (int) Math.round(v), 1.0, 64.0, 8.0));
        knobs.add(numberKnob("center height in", "height of the object center above the tiles (ground distance)",
                p -> p.targetCenterHeightInches, (p, v) -> p.targetCenterHeightInches = v, -12.0, 72.0, 0.25, 4.0, 2));
        knobs.add(new Knob(p -> "sort", "which blob is the primary target",
                p -> p.sort == null ? 0 : p.sort.ordinal(),
                (p, v) -> p.sort = TargetSort.values()[(int) v],
                p -> 0.0, 0.0, 1.0, 0.0, 0, TargetSort.values().length,
                p -> p.sort == null ? TargetSort.LARGEST.name() : p.sort.name()));
        knobs.add(new Knob(p -> "color space", "HSV is easiest; after a change press B (wand) to resample",
                p -> p.colorSpace == null ? 0 : p.colorSpace.ordinal(),
                (p, v) -> p.colorSpace = ColorSpaceMode.values()[(int) v],
                p -> 0.0, 0.0, 1.0, 0.0, 0, ColorSpaceMode.values().length,
                p -> p.colorSpace == null ? ColorSpaceMode.HSV.name() : p.colorSpace.name()));
        knobs.add(new Knob(p -> "enabled", "a disabled profile is never detected",
                p -> p.enabled ? 1.0 : 0.0,
                (p, v) -> p.enabled = v >= 0.5,
                p -> 1.0, 0.0, 1.0, 0.0, 0, 2,
                p -> p.enabled ? "YES" : "!! NO"));
        for (int channel = 0; channel < 3; channel++) {
            knobs.add(wandMarginKnob(channel));
        }
        knobs.add(new Knob(p -> "wand radius px", "half size of the sampled square at the crosshair",
                p -> wandRadiusPx, (p, v) -> wandRadiusPx = (int) Math.round(v),
                p -> ColorSample.MAX_RADIUS_PX, 1.0, 1.0, 6.0, 0, 0, null));
    }

    private Knob channelKnob(final int channel, final boolean isMax) {
        return new Knob(
                p -> spaceOf(p).channelName(channel) + (isMax ? " max" : " min"),
                channel == 0 ? "HSV: hue 0-180 (min > max wraps through red); YCrCb: Y; RGB: R"
                        : channel == 1 ? "HSV: saturation; YCrCb: Cr (red-ness); RGB: G"
                        : "HSV: value (brightness); YCrCb: Cb (blue-ness); RGB: B",
                p -> isMax ? p.channelMax(channel) : p.channelMin(channel),
                (p, v) -> {
                    if (isMax) {
                        p.setChannelMax(channel, v);
                    } else {
                        p.setChannelMin(channel, v);
                    }
                },
                p -> spaceOf(p).channelMax(channel),
                0.0, 1.0, 60.0, 0, 0, null);
    }

    private Knob wandMarginKnob(final int channel) {
        return new Knob(
                p -> "wand margin " + spaceOf(p).channelName(channel),
                "B sets the range to the sampled median +- this margin",
                p -> marginsFor(p.colorSpace)[channel],
                (p, v) -> marginsFor(p.colorSpace)[channel] = v,
                p -> MAX_WAND_MARGIN, 0.0, 1.0, 20.0, 0, 0, null);
    }

    private static Knob wholeKnob(final String name, String help, ProfileReader reader, ProfileWriter writer,
                                  double min, final double max, double ratePerSecond) {
        return new Knob(p -> name, help, reader, writer, p -> max, min, 1.0, ratePerSecond, 0, 0, null);
    }

    private static Knob numberKnob(final String name, String help, ProfileReader reader, ProfileWriter writer,
                                   double min, final double max, double step, double ratePerSecond, int decimals) {
        return new Knob(p -> name, help, reader, writer, p -> max, min, step, ratePerSecond, decimals, 0, null);
    }

    private static ColorSpaceMode spaceOf(ColorProfile profile) {
        return profile == null || profile.colorSpace == null ? ColorSpaceMode.HSV : profile.colorSpace;
    }
}
