package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class ColorFrame {

    public static final ColorFrame EMPTY = new ColorFrame(0L, 0.0, Collections.<ColorTarget>emptyList(),
            new String[0], new int[0], -1, 0, 0);

    public final long captureNanos;
    public final double processingMs;
    public final List<ColorTarget> targets;
    public final String[] profileNames;
    public final int[] profileCounts;
    public final int pipelineIndex;
    public final int frameWidth;
    public final int frameHeight;

    public ColorFrame(long captureNanos, double processingMs, List<ColorTarget> targets,
                      String[] profileNames, int[] profileCounts) {
        this(captureNanos, processingMs, targets, profileNames, profileCounts, -1, 0, 0);
    }

    public ColorFrame(long captureNanos, double processingMs, List<ColorTarget> targets,
                      String[] profileNames, int[] profileCounts,
                      int pipelineIndex, int frameWidth, int frameHeight) {
        this.captureNanos = captureNanos;
        this.processingMs = processingMs;
        this.targets = targets == null || targets.isEmpty()
                ? Collections.<ColorTarget>emptyList()
                : Collections.unmodifiableList(new ArrayList<ColorTarget>(targets));
        this.profileNames = profileNames == null ? new String[0] : profileNames.clone();
        int[] counts = new int[this.profileNames.length];
        if (profileCounts != null) {
            System.arraycopy(profileCounts, 0, counts, 0, Math.min(counts.length, profileCounts.length));
        }
        this.profileCounts = counts;
        this.pipelineIndex = pipelineIndex;
        this.frameWidth = frameWidth;
        this.frameHeight = frameHeight;
    }

    public boolean hasTargets() {
        return !targets.isEmpty();
    }

    public int count() {
        return targets.size();
    }

    public ColorTarget primary() {
        return targets.isEmpty() ? null : targets.get(0);
    }

    public int countFor(String profileName) {
        if (profileName == null) {
            return 0;
        }
        String wanted = profileName.trim();
        for (int i = 0; i < profileNames.length; i++) {
            if (profileNames[i] != null && profileNames[i].trim().equalsIgnoreCase(wanted)) {
                return profileCounts[i];
            }
        }
        return 0;
    }

    public List<ColorTarget> targetsFor(String profileName) {
        if (profileName == null || targets.isEmpty()) {
            return Collections.emptyList();
        }
        String wanted = profileName.trim();
        List<ColorTarget> matching = new ArrayList<ColorTarget>();
        for (ColorTarget target : targets) {
            if (target.profileName.trim().equalsIgnoreCase(wanted)) {
                matching.add(target);
            }
        }
        return Collections.unmodifiableList(matching);
    }

    public ColorTarget primaryFor(String profileName) {
        List<ColorTarget> matching = targetsFor(profileName);
        return matching.isEmpty() ? null : matching.get(0);
    }

    public double ageMs() {
        if (captureNanos == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - captureNanos) / 1.0e6;
    }

    public String countsSummary() {
        if (profileNames.length == 0) {
            return "no profiles";
        }
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < profileNames.length; i++) {
            if (i > 0) {
                builder.append("  ");
            }
            builder.append(profileNames[i]).append(' ').append(profileCounts[i]);
        }
        return builder.toString();
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "%d targets  %s  %.1f ms", targets.size(), countsSummary(), processingMs);
    }
}
