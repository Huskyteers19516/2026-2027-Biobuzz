package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.Comparator;
import java.util.Locale;

public final class ColorTarget {

    public static final int CORNER_VALUES = 8;

    public final String profileName;
    public final double centerX;
    public final double centerY;
    public final double tx;
    public final double ty;
    public final double txNC;
    public final double tyNC;
    public final double ta;
    public final double areaPixels;
    public final double boxWidth;
    public final double boxHeight;
    public final double fullness;
    public final double aspectRatio;
    public final double circularity;
    public final double[] corners;
    public final double groundForwardInches;
    public final double groundLeftInches;
    public final double groundDistanceInches;
    public final double groundBearingLeftDegrees;
    public final double boxLeft;
    public final double boxTop;
    public final double rectWidth;
    public final double rectHeight;
    public final double rectAngleDegrees;

    public ColorTarget(String profileName,
                       double centerX, double centerY,
                       double tx, double ty, double txNC, double tyNC,
                       double ta, double areaPixels,
                       double boxWidth, double boxHeight,
                       double fullness, double aspectRatio, double circularity,
                       double[] corners,
                       double groundForwardInches, double groundLeftInches,
                       double groundDistanceInches, double groundBearingLeftDegrees) {
        this(profileName, centerX, centerY, tx, ty, txNC, tyNC, ta, areaPixels, boxWidth, boxHeight,
                fullness, aspectRatio, circularity, corners,
                groundForwardInches, groundLeftInches, groundDistanceInches, groundBearingLeftDegrees,
                centerX - boxWidth / 2.0, centerY - boxHeight / 2.0, boxWidth, boxHeight, 0.0);
    }

    public ColorTarget(String profileName,
                       double centerX, double centerY,
                       double tx, double ty, double txNC, double tyNC,
                       double ta, double areaPixels,
                       double boxWidth, double boxHeight,
                       double fullness, double aspectRatio, double circularity,
                       double[] corners,
                       double groundForwardInches, double groundLeftInches,
                       double groundDistanceInches, double groundBearingLeftDegrees,
                       double boxLeft, double boxTop,
                       double rectWidth, double rectHeight, double rectAngleDegrees) {
        this.profileName = profileName == null ? "" : profileName;
        this.centerX = centerX;
        this.centerY = centerY;
        this.tx = tx;
        this.ty = ty;
        this.txNC = txNC;
        this.tyNC = tyNC;
        this.ta = ta;
        this.areaPixels = areaPixels;
        this.boxWidth = boxWidth;
        this.boxHeight = boxHeight;
        this.fullness = fullness;
        this.aspectRatio = aspectRatio;
        this.circularity = circularity;
        this.corners = new double[CORNER_VALUES];
        if (corners != null) {
            System.arraycopy(corners, 0, this.corners, 0, Math.min(CORNER_VALUES, corners.length));
        }
        this.groundForwardInches = groundForwardInches;
        this.groundLeftInches = groundLeftInches;
        this.groundDistanceInches = groundDistanceInches;
        this.groundBearingLeftDegrees = groundBearingLeftDegrees;
        this.boxLeft = boxLeft;
        this.boxTop = boxTop;
        this.rectWidth = rectWidth;
        this.rectHeight = rectHeight;
        this.rectAngleDegrees = rectAngleDegrees;
    }

    public boolean hasGroundPosition() {
        return isFinite(groundForwardInches) && isFinite(groundLeftInches) && isFinite(groundDistanceInches);
    }

    public double crosshairDistanceDegrees() {
        return Math.hypot(tx, ty);
    }

    public double[] cornersCopy() {
        return corners.clone();
    }

    public double cornerX(int index) {
        return corners[2 * Math.max(0, Math.min(3, index))];
    }

    public double cornerY(int index) {
        return corners[2 * Math.max(0, Math.min(3, index)) + 1];
    }

    public static Comparator<ColorTarget> comparator(TargetSort sort) {
        if (sort == null) {
            return LARGEST_FIRST;
        }
        switch (sort) {
            case SMALLEST:
                return SMALLEST_FIRST;
            case CLOSEST_TO_CROSSHAIR:
                return CLOSEST_TO_CROSSHAIR_FIRST;
            case LEFTMOST:
                return LEFTMOST_FIRST;
            case RIGHTMOST:
                return RIGHTMOST_FIRST;
            case HIGHEST:
                return HIGHEST_FIRST;
            case LOWEST:
                return LOWEST_FIRST;
            case NEAREST_GROUND_DISTANCE:
                return NEAREST_GROUND_FIRST;
            case LARGEST:
            default:
                return LARGEST_FIRST;
        }
    }

    public static final Comparator<ColorTarget> LARGEST_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            return compareNanLast(a.areaPixels, b.areaPixels, false);
        }
    };

    public static final Comparator<ColorTarget> SMALLEST_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            return compareNanLast(a.areaPixels, b.areaPixels, true);
        }
    };

    public static final Comparator<ColorTarget> CLOSEST_TO_CROSSHAIR_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            return compareNanLast(a.crosshairDistanceDegrees(), b.crosshairDistanceDegrees(), true);
        }
    };

    public static final Comparator<ColorTarget> LEFTMOST_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            return compareNanLast(a.tx, b.tx, true);
        }
    };

    public static final Comparator<ColorTarget> RIGHTMOST_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            return compareNanLast(a.tx, b.tx, false);
        }
    };

    public static final Comparator<ColorTarget> HIGHEST_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            return compareNanLast(a.ty, b.ty, false);
        }
    };

    public static final Comparator<ColorTarget> LOWEST_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            return compareNanLast(a.ty, b.ty, true);
        }
    };

    public static final Comparator<ColorTarget> NEAREST_GROUND_FIRST = new Comparator<ColorTarget>() {
        @Override
        public int compare(ColorTarget a, ColorTarget b) {
            int byDistance = compareNanLast(a.groundDistanceInches, b.groundDistanceInches, true);
            if (byDistance != 0) {
                return byDistance;
            }
            return compareNanLast(a.areaPixels, b.areaPixels, false);
        }
    };

    static int compareNanLast(double a, double b, boolean ascending) {
        boolean aBad = Double.isNaN(a);
        boolean bBad = Double.isNaN(b);
        if (aBad || bBad) {
            if (aBad && bBad) {
                return 0;
            }
            return aBad ? 1 : -1;
        }
        return ascending ? Double.compare(a, b) : Double.compare(b, a);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    public String summary() {
        String ground = hasGroundPosition()
                ? String.format(Locale.US, "  d %.1f in  bL %+.1f deg", groundDistanceInches, groundBearingLeftDegrees)
                : "  no ground";
        return String.format(Locale.US, "%s  tx %.1f  ty %.1f  ta %.2f%s", profileName, tx, ty, ta, ground);
    }

    @Override
    public String toString() {
        return summary();
    }
}
