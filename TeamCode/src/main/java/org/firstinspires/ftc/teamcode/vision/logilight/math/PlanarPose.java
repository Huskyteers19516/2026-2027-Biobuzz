package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.Locale;

public final class PlanarPose {
    private static final double TWO_PI = 2.0 * Math.PI;

    public final double x;
    public final double y;
    public final double heading;

    public PlanarPose(double x, double y, double headingRadians) {
        this.x = x;
        this.y = y;
        this.heading = normalizeRadians(headingRadians);
    }

    public double distanceTo(PlanarPose other) {
        if (other == null) {
            return Double.NaN;
        }
        return Math.hypot(x - other.x, y - other.y);
    }

    public double headingDegrees() {
        return Math.toDegrees(heading);
    }

    public PlanarPose withHeading(double headingRadians) {
        return new PlanarPose(x, y, headingRadians);
    }

    public PlanarPose withPosition(double newX, double newY) {
        return new PlanarPose(newX, newY, heading);
    }

    public double headingErrorTo(PlanarPose other) {
        if (other == null) {
            return Double.NaN;
        }
        return normalizeRadians(other.heading - heading);
    }

    public boolean hasHeading() {
        return Vec3.isFiniteValue(heading);
    }

    public boolean isFinite() {
        return Vec3.isFiniteValue(x) && Vec3.isFiniteValue(y) && Vec3.isFiniteValue(heading);
    }

    public boolean isPositionFinite() {
        return Vec3.isFiniteValue(x) && Vec3.isFiniteValue(y);
    }

    public static double normalizeRadians(double radians) {
        if (Double.isNaN(radians) || Double.isInfinite(radians)) {
            return Double.NaN;
        }
        double result = Math.IEEEremainder(radians, TWO_PI);
        if (result <= -Math.PI) {
            result += TWO_PI;
        }
        if (result > Math.PI) {
            result -= TWO_PI;
        }
        return result;
    }

    public static double normalizeDegrees(double degrees) {
        if (Double.isNaN(degrees) || Double.isInfinite(degrees)) {
            return Double.NaN;
        }
        double result = Math.IEEEremainder(degrees, 360.0);
        if (result <= -180.0) {
            result += 360.0;
        }
        if (result > 180.0) {
            result -= 360.0;
        }
        return result;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg", x, y, headingDegrees());
    }
}
