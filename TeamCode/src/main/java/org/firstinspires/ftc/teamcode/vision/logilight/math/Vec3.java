package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.Locale;

public final class Vec3 {
    public static final Vec3 ZERO = new Vec3(0.0, 0.0, 0.0);
    public static final Vec3 UNIT_X = new Vec3(1.0, 0.0, 0.0);
    public static final Vec3 UNIT_Y = new Vec3(0.0, 1.0, 0.0);
    public static final Vec3 UNIT_Z = new Vec3(0.0, 0.0, 1.0);

    private static final double NORMALIZE_EPSILON = 1e-12;

    public final double x;
    public final double y;
    public final double z;

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3 plus(Vec3 other) {
        if (other == null) {
            return this;
        }
        return new Vec3(x + other.x, y + other.y, z + other.z);
    }

    public Vec3 minus(Vec3 other) {
        if (other == null) {
            return this;
        }
        return new Vec3(x - other.x, y - other.y, z - other.z);
    }

    public Vec3 times(double scale) {
        return new Vec3(x * scale, y * scale, z * scale);
    }

    public Vec3 negated() {
        return new Vec3(-x, -y, -z);
    }

    public double dot(Vec3 other) {
        if (other == null) {
            return 0.0;
        }
        return x * other.x + y * other.y + z * other.z;
    }

    public Vec3 cross(Vec3 other) {
        if (other == null) {
            return ZERO;
        }
        return new Vec3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x);
    }

    public double norm() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vec3 normalized() {
        double length = norm();
        if (!(length >= NORMALIZE_EPSILON) || Double.isInfinite(length)) {
            return ZERO;
        }
        return new Vec3(x / length, y / length, z / length);
    }

    public double horizontalNorm() {
        return Math.hypot(x, y);
    }

    public double distanceTo(Vec3 other) {
        if (other == null) {
            return Double.NaN;
        }
        return minus(other).norm();
    }

    public double angleTo(Vec3 other) {
        if (other == null) {
            return Double.NaN;
        }
        return Math.atan2(cross(other).norm(), dot(other));
    }

    public Vec3 withZ(double newZ) {
        return new Vec3(x, y, newZ);
    }

    public boolean isFinite() {
        return isFiniteValue(x) && isFiniteValue(y) && isFiniteValue(z);
    }

    public double[] toArray() {
        return new double[]{x, y, z};
    }

    public static Vec3 lerp(Vec3 from, Vec3 to, double fraction) {
        Vec3 start = from == null ? ZERO : from;
        Vec3 end = to == null ? ZERO : to;
        return new Vec3(
                start.x + (end.x - start.x) * fraction,
                start.y + (end.y - start.y) * fraction,
                start.z + (end.z - start.z) * fraction);
    }

    public static Vec3 average(java.util.List<Vec3> list) {
        if (list == null || list.isEmpty()) {
            return ZERO;
        }
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        int count = 0;
        for (Vec3 v : list) {
            if (v == null) {
                continue;
            }
            sumX += v.x;
            sumY += v.y;
            sumZ += v.z;
            count++;
        }
        if (count == 0) {
            return ZERO;
        }
        return new Vec3(sumX / count, sumY / count, sumZ / count);
    }

    static boolean isFiniteValue(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof Vec3)) {
            return false;
        }
        Vec3 v = (Vec3) other;
        return Double.compare(x, v.x) == 0 && Double.compare(y, v.y) == 0 && Double.compare(z, v.z) == 0;
    }

    @Override
    public int hashCode() {
        long bits = Double.doubleToLongBits(x);
        bits = 31L * bits + Double.doubleToLongBits(y);
        bits = 31L * bits + Double.doubleToLongBits(z);
        return (int) (bits ^ (bits >>> 32));
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "(%.2f, %.2f, %.2f)", x, y, z);
    }
}
