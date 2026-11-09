package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class ClusterLayout {
    public static final String RED_SCORING = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING = "BLUE SCORING";
    public static final String SAMPLE_GOAL_NAME = "Center Goal";
    public static final String SAMPLE_GOAL_SHORT_NAME = "GOAL";

    public static final double BIO_BUZZ_TAG_SIZE_INCHES = 3.25;
    public static final double BIO_BUZZ_MEMBER_Y_INCHES = 7.1874;
    public static final double BIO_BUZZ_MEMBER_Z_INCHES = -5.622;
    public static final double SAMPLE_GOAL_TAG_SIZE_INCHES = 3.0;

    private static final double[] BIO_BUZZ_MEMBER_X_INCHES = {-6.50, -2.75, 2.75, 6.50};

    public final String name;
    public final double tagSizeInches;
    public final double[][] memberCenters;

    public ClusterLayout(String name, double tagSizeInches, double[][] memberCenters) {
        this.name = name == null ? "" : name;
        this.tagSizeInches = tagSizeInches;
        this.memberCenters = copyCenters(memberCenters);
    }

    public static ClusterLayout bioBuzz(String name) {
        double[][] centers = new double[BIO_BUZZ_MEMBER_X_INCHES.length][];
        for (int i = 0; i < centers.length; i++) {
            centers[i] = new double[]{BIO_BUZZ_MEMBER_X_INCHES[i], BIO_BUZZ_MEMBER_Y_INCHES, BIO_BUZZ_MEMBER_Z_INCHES};
        }
        return new ClusterLayout(name, BIO_BUZZ_TAG_SIZE_INCHES, centers);
    }

    public static ClusterLayout sampleGoal() {
        return new ClusterLayout(SAMPLE_GOAL_NAME, SAMPLE_GOAL_TAG_SIZE_INCHES,
                new double[][]{{-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});
    }

    public static ClusterLayout forName(String clusterName) {
        String key = normalizeName(clusterName);
        if (key == null) {
            return null;
        }
        if (key.equals(RED_SCORING) || key.equals(RED_AUDIENCE) || key.equals(BLUE_AUDIENCE) || key.equals(BLUE_SCORING)) {
            return bioBuzz(key);
        }
        if (key.equals(SAMPLE_GOAL_SHORT_NAME) || key.equals(SAMPLE_GOAL_NAME.toUpperCase(Locale.US))) {
            return sampleGoal();
        }
        return null;
    }

    public static int firstTagId(String clusterName) {
        String key = normalizeName(clusterName);
        if (key == null) {
            return -1;
        }
        if (key.equals(RED_SCORING)) {
            return 30;
        }
        if (key.equals(RED_AUDIENCE)) {
            return 34;
        }
        if (key.equals(BLUE_AUDIENCE)) {
            return 38;
        }
        if (key.equals(BLUE_SCORING)) {
            return 42;
        }
        if (key.equals(SAMPLE_GOAL_SHORT_NAME) || key.equals(SAMPLE_GOAL_NAME.toUpperCase(Locale.US))) {
            return 581;
        }
        return -1;
    }

    public int memberCount() {
        return memberCenters.length;
    }

    public Vec3 memberCenter(int index) {
        if (index < 0 || index >= memberCenters.length) {
            return null;
        }
        double[] c = memberCenters[index];
        return new Vec3(c[0], c[1], c[2]);
    }

    public List<Vec3[]> memberCorners() {
        List<Vec3[]> result = new ArrayList<Vec3[]>(memberCenters.length);
        Vec3[] template = singleTagCorners(tagSizeInches);
        for (double[] center : memberCenters) {
            Vec3 offset = new Vec3(center[0], center[1], center[2]);
            Vec3[] corners = new Vec3[template.length];
            for (int k = 0; k < template.length; k++) {
                corners[k] = template[k].plus(offset);
            }
            result.add(corners);
        }
        return Collections.unmodifiableList(result);
    }

    public static Vec3[] singleTagCorners(double tagSizeInches) {
        double half = tagSizeInches / 2.0;
        return new Vec3[]{
                new Vec3(-half, half, 0.0),
                new Vec3(half, half, 0.0),
                new Vec3(half, -half, 0.0),
                new Vec3(-half, -half, 0.0)};
    }

    private static String normalizeName(String clusterName) {
        if (clusterName == null) {
            return null;
        }
        String trimmed = clusterName.trim().replaceAll("\\s+", " ");
        if (trimmed.isEmpty()) {
            return null;
        }
        return trimmed.toUpperCase(Locale.US);
    }

    private static double[][] copyCenters(double[][] centers) {
        if (centers == null) {
            return new double[0][];
        }
        List<double[]> valid = new ArrayList<double[]>();
        for (double[] center : centers) {
            if (center == null) {
                continue;
            }
            double[] copy = new double[3];
            for (int i = 0; i < 3 && i < center.length; i++) {
                copy[i] = center[i];
            }
            valid.add(copy);
        }
        return valid.toArray(new double[valid.size()][]);
    }
}
