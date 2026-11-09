package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.List;
import java.util.Locale;

public final class Mat3 {
    public static final Mat3 IDENTITY = new Mat3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    private static final double DEGENERATE_EPSILON = 1e-9;

    public final double m00;
    public final double m01;
    public final double m02;
    public final double m10;
    public final double m11;
    public final double m12;
    public final double m20;
    public final double m21;
    public final double m22;

    public Mat3(double m00, double m01, double m02,
                double m10, double m11, double m12,
                double m20, double m21, double m22) {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
    }

    public static Mat3 fromRowMajor(double[] nine) {
        if (nine == null || nine.length < 9) {
            return IDENTITY;
        }
        return new Mat3(nine[0], nine[1], nine[2], nine[3], nine[4], nine[5], nine[6], nine[7], nine[8]);
    }

    public static Mat3 fromColumns(Vec3 column0, Vec3 column1, Vec3 column2) {
        Vec3 a = column0 == null ? Vec3.ZERO : column0;
        Vec3 b = column1 == null ? Vec3.ZERO : column1;
        Vec3 c = column2 == null ? Vec3.ZERO : column2;
        return new Mat3(a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z);
    }

    public static Mat3 fromRows(Vec3 row0, Vec3 row1, Vec3 row2) {
        Vec3 a = row0 == null ? Vec3.ZERO : row0;
        Vec3 b = row1 == null ? Vec3.ZERO : row1;
        Vec3 c = row2 == null ? Vec3.ZERO : row2;
        return new Mat3(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
    }

    public static Mat3 rotX(double rad) {
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        return new Mat3(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c);
    }

    public static Mat3 rotY(double rad) {
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        return new Mat3(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
    }

    public static Mat3 rotZ(double rad) {
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        return new Mat3(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
    }

    public static Mat3 axisAngle(Vec3 axis, double rad) {
        Vec3 unit = axis == null ? Vec3.ZERO : axis.normalized();
        if (unit.norm() == 0.0) {
            return IDENTITY;
        }
        double c = Math.cos(rad);
        double s = Math.sin(rad);
        double t = 1.0 - c;
        double x = unit.x;
        double y = unit.y;
        double z = unit.z;
        return new Mat3(
                t * x * x + c, t * x * y - s * z, t * x * z + s * y,
                t * x * y + s * z, t * y * y + c, t * y * z - s * x,
                t * x * z - s * y, t * y * z + s * x, t * z * z + c);
    }

    public static Mat3 fromYawPitchRoll(double yawRad, double pitchRad, double rollRad) {
        return rotZ(yawRad).times(rotY(pitchRad)).times(rotX(rollRad));
    }

    public double yaw() {
        return Math.atan2(m10, m00);
    }

    public double pitch() {
        return Math.asin(-clamp(m20, -1.0, 1.0));
    }

    public double roll() {
        return Math.atan2(m21, m22);
    }

    public double get(int r, int c) {
        if (r < 0 || r > 2 || c < 0 || c > 2) {
            throw new IndexOutOfBoundsException("Mat3 index (" + r + ", " + c + ")");
        }
        switch (r * 3 + c) {
            case 0:
                return m00;
            case 1:
                return m01;
            case 2:
                return m02;
            case 3:
                return m10;
            case 4:
                return m11;
            case 5:
                return m12;
            case 6:
                return m20;
            case 7:
                return m21;
            case 8:
                return m22;
            default:
                throw new IndexOutOfBoundsException("Mat3 index (" + r + ", " + c + ")");
        }
    }

    public Mat3 times(Mat3 o) {
        if (o == null) {
            return this;
        }
        return new Mat3(
                m00 * o.m00 + m01 * o.m10 + m02 * o.m20,
                m00 * o.m01 + m01 * o.m11 + m02 * o.m21,
                m00 * o.m02 + m01 * o.m12 + m02 * o.m22,
                m10 * o.m00 + m11 * o.m10 + m12 * o.m20,
                m10 * o.m01 + m11 * o.m11 + m12 * o.m21,
                m10 * o.m02 + m11 * o.m12 + m12 * o.m22,
                m20 * o.m00 + m21 * o.m10 + m22 * o.m20,
                m20 * o.m01 + m21 * o.m11 + m22 * o.m21,
                m20 * o.m02 + m21 * o.m12 + m22 * o.m22);
    }

    public Vec3 times(Vec3 v) {
        if (v == null) {
            return Vec3.ZERO;
        }
        return new Vec3(
                m00 * v.x + m01 * v.y + m02 * v.z,
                m10 * v.x + m11 * v.y + m12 * v.z,
                m20 * v.x + m21 * v.y + m22 * v.z);
    }

    public Vec3 transposeTimes(Vec3 v) {
        if (v == null) {
            return Vec3.ZERO;
        }
        return new Vec3(
                m00 * v.x + m10 * v.y + m20 * v.z,
                m01 * v.x + m11 * v.y + m21 * v.z,
                m02 * v.x + m12 * v.y + m22 * v.z);
    }

    public Mat3 times(double scale) {
        return new Mat3(
                m00 * scale, m01 * scale, m02 * scale,
                m10 * scale, m11 * scale, m12 * scale,
                m20 * scale, m21 * scale, m22 * scale);
    }

    public Mat3 plus(Mat3 o) {
        if (o == null) {
            return this;
        }
        return new Mat3(
                m00 + o.m00, m01 + o.m01, m02 + o.m02,
                m10 + o.m10, m11 + o.m11, m12 + o.m12,
                m20 + o.m20, m21 + o.m21, m22 + o.m22);
    }

    public Mat3 transpose() {
        return new Mat3(m00, m10, m20, m01, m11, m21, m02, m12, m22);
    }

    public Vec3 column(int c) {
        switch (c) {
            case 0:
                return new Vec3(m00, m10, m20);
            case 1:
                return new Vec3(m01, m11, m21);
            case 2:
                return new Vec3(m02, m12, m22);
            default:
                throw new IndexOutOfBoundsException("Mat3 column " + c);
        }
    }

    public Vec3 row(int r) {
        switch (r) {
            case 0:
                return new Vec3(m00, m01, m02);
            case 1:
                return new Vec3(m10, m11, m12);
            case 2:
                return new Vec3(m20, m21, m22);
            default:
                throw new IndexOutOfBoundsException("Mat3 row " + r);
        }
    }

    public double trace() {
        return m00 + m11 + m22;
    }

    public double determinant() {
        return m00 * (m11 * m22 - m12 * m21)
                - m01 * (m10 * m22 - m12 * m20)
                + m02 * (m10 * m21 - m11 * m20);
    }

    public Mat3 orthonormalized() {
        if (!isFinite()) {
            return IDENTITY;
        }
        Vec3 first = column(0).normalized();
        if (first.norm() == 0.0) {
            return IDENTITY;
        }
        Vec3 secondRaw = column(1);
        Vec3 second = secondRaw.minus(first.times(secondRaw.dot(first)));
        if (second.norm() < DEGENERATE_EPSILON) {
            Vec3 fromThird = column(2).cross(first);
            if (fromThird.norm() >= DEGENERATE_EPSILON) {
                second = fromThird;
            } else {
                second = anyPerpendicular(first);
            }
        }
        second = second.normalized();
        Vec3 third = first.cross(second).normalized();
        return fromColumns(first, second, third);
    }

    private static Vec3 anyPerpendicular(Vec3 unit) {
        Vec3 helper = Math.abs(unit.x) < 0.9 ? Vec3.UNIT_X : Vec3.UNIT_Y;
        return helper.minus(unit.times(helper.dot(unit)));
    }

    public double angleTo(Mat3 other) {
        if (other == null) {
            return Double.NaN;
        }
        Mat3 relative = transpose().times(other);
        double cosine = clamp((relative.trace() - 1.0) / 2.0, -1.0, 1.0);
        double sine = 0.5 * Math.sqrt(
                square(relative.m21 - relative.m12)
                        + square(relative.m02 - relative.m20)
                        + square(relative.m10 - relative.m01));
        if (!Vec3.isFiniteValue(sine) || !Vec3.isFiniteValue(cosine)) {
            return Double.NaN;
        }
        return Math.atan2(sine, cosine);
    }

    public static Mat3 average(List<Mat3> list) {
        if (list == null || list.isEmpty()) {
            return IDENTITY;
        }
        double[] sum = new double[9];
        int count = 0;
        for (Mat3 m : list) {
            if (m == null || !m.isFinite()) {
                continue;
            }
            double[] values = m.toRowMajor();
            for (int i = 0; i < 9; i++) {
                sum[i] += values[i];
            }
            count++;
        }
        if (count == 0) {
            return IDENTITY;
        }
        for (int i = 0; i < 9; i++) {
            sum[i] /= count;
        }
        return fromRowMajor(sum).orthonormalized();
    }

    public boolean isFinite() {
        return Vec3.isFiniteValue(m00) && Vec3.isFiniteValue(m01) && Vec3.isFiniteValue(m02)
                && Vec3.isFiniteValue(m10) && Vec3.isFiniteValue(m11) && Vec3.isFiniteValue(m12)
                && Vec3.isFiniteValue(m20) && Vec3.isFiniteValue(m21) && Vec3.isFiniteValue(m22);
    }

    public boolean isRotation(double tolerance) {
        if (!isFinite()) {
            return false;
        }
        Mat3 product = transpose().times(this);
        double[] p = product.toRowMajor();
        double[] identity = IDENTITY.toRowMajor();
        for (int i = 0; i < 9; i++) {
            if (Math.abs(p[i] - identity[i]) > tolerance) {
                return false;
            }
        }
        return Math.abs(determinant() - 1.0) <= tolerance;
    }

    public double[] toRowMajor() {
        return new double[]{m00, m01, m02, m10, m11, m12, m20, m21, m22};
    }

    static double clamp(double value, double min, double max) {
        if (Double.isNaN(value)) {
            return value;
        }
        return value < min ? min : (value > max ? max : value);
    }

    private static double square(double value) {
        return value * value;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "[[%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f], [%.4f, %.4f, %.4f]]",
                m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }
}
