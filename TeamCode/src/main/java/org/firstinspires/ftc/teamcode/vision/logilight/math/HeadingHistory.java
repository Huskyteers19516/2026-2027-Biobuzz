package org.firstinspires.ftc.teamcode.vision.logilight.math;

public final class HeadingHistory {
    private static final int MIN_CAPACITY = 2;
    private static final double NANOS_PER_SECOND = 1e9;
    private static final long BACKWARD_RESET_NANOS = 1_000_000_000L;

    private final long[] times;
    private final double[] unwrapped;
    private int start;
    private int count;

    public HeadingHistory(int capacity) {
        int size = Math.max(MIN_CAPACITY, capacity);
        times = new long[size];
        unwrapped = new double[size];
    }

    public synchronized void add(long timeNanos, double headingRad) {
        if (!Vec3.isFiniteValue(headingRad)) {
            return;
        }
        if (count == 0) {
            append(timeNanos, PlanarPose.normalizeRadians(headingRad));
            return;
        }
        long newest = timeAt(count - 1);
        if (timeNanos < newest) {
            if (newest - timeNanos > BACKWARD_RESET_NANOS) {
                start = 0;
                count = 0;
                append(timeNanos, PlanarPose.normalizeRadians(headingRad));
            }
            return;
        }
        if (timeNanos == newest) {
            if (count == 1) {
                unwrapped[physical(0)] = PlanarPose.normalizeRadians(headingRad);
            } else {
                double reference = valueAt(count - 2);
                unwrapped[physical(count - 1)] = reference + PlanarPose.normalizeRadians(headingRad - reference);
            }
            return;
        }
        double reference = valueAt(count - 1);
        append(timeNanos, reference + PlanarPose.normalizeRadians(headingRad - reference));
    }

    public synchronized Double headingAt(long timeNanos, long maxExtrapolateNanos) {
        if (count == 0) {
            return null;
        }
        long oldest = timeAt(0);
        long newest = timeAt(count - 1);
        if (timeNanos > newest) {
            if (timeNanos - newest <= Math.max(0L, maxExtrapolateNanos)) {
                return PlanarPose.normalizeRadians(valueAt(count - 1));
            }
            return null;
        }
        if (timeNanos < oldest) {
            return null;
        }
        if (timeNanos == newest) {
            return PlanarPose.normalizeRadians(valueAt(count - 1));
        }
        int lower = lowerBracket(timeNanos);
        long t0 = timeAt(lower);
        long t1 = timeAt(lower + 1);
        double h0 = valueAt(lower);
        double h1 = valueAt(lower + 1);
        if (t1 == t0) {
            return PlanarPose.normalizeRadians(h1);
        }
        double fraction = (double) (timeNanos - t0) / (double) (t1 - t0);
        return PlanarPose.normalizeRadians(h0 + (h1 - h0) * fraction);
    }

    public synchronized double angularVelocityAt(long timeNanos) {
        if (count < 2) {
            return 0.0;
        }
        long oldest = timeAt(0);
        long newest = timeAt(count - 1);
        if (timeNanos < oldest) {
            return 0.0;
        }
        int lower = timeNanos >= newest ? count - 2 : lowerBracket(timeNanos);
        long t0 = timeAt(lower);
        long t1 = timeAt(lower + 1);
        if (t1 <= t0) {
            return 0.0;
        }
        double velocity = (valueAt(lower + 1) - valueAt(lower)) * NANOS_PER_SECOND / (double) (t1 - t0);
        return Vec3.isFiniteValue(velocity) ? velocity : 0.0;
    }

    public synchronized Double newestHeading() {
        if (count == 0) {
            return null;
        }
        return PlanarPose.normalizeRadians(valueAt(count - 1));
    }

    public synchronized long newestTimeNanos() {
        return count == 0 ? 0L : timeAt(count - 1);
    }

    public synchronized long oldestTimeNanos() {
        return count == 0 ? 0L : timeAt(0);
    }

    public synchronized int size() {
        return count;
    }

    public int capacity() {
        return times.length;
    }

    public synchronized void clear() {
        start = 0;
        count = 0;
    }

    private void append(long timeNanos, double value) {
        if (count < times.length) {
            int index = physical(count);
            times[index] = timeNanos;
            unwrapped[index] = value;
            count++;
        } else {
            times[start] = timeNanos;
            unwrapped[start] = value;
            start = (start + 1) % times.length;
        }
    }

    private int lowerBracket(long timeNanos) {
        int low = 0;
        int high = count - 1;
        while (high - low > 1) {
            int mid = (low + high) >>> 1;
            if (timeAt(mid) <= timeNanos) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return low;
    }

    private int physical(int logical) {
        return (start + logical) % times.length;
    }

    private long timeAt(int logical) {
        return times[physical(logical)];
    }

    private double valueAt(int logical) {
        return unwrapped[physical(logical)];
    }
}
