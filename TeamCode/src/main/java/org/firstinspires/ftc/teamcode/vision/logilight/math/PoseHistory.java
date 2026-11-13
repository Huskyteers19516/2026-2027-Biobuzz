package org.firstinspires.ftc.teamcode.vision.logilight.math;

public final class PoseHistory {
    public static final int DEFAULT_CAPACITY = 400;

    private static final int MIN_CAPACITY = 2;
    private static final long BACKWARD_RESET_NANOS = 1_000_000_000L;

    private final long[] times;
    private final double[] xs;
    private final double[] ys;
    private final double[] headings;
    private int start;
    private int count;

    public PoseHistory() {
        this(DEFAULT_CAPACITY);
    }

    public PoseHistory(int capacity) {
        int size = Math.max(MIN_CAPACITY, capacity);
        times = new long[size];
        xs = new double[size];
        ys = new double[size];
        headings = new double[size];
    }

    public synchronized void add(long timeNanos, double x, double y, double headingRad) {
        if (!Vec3.isFiniteValue(x) || !Vec3.isFiniteValue(y) || !Vec3.isFiniteValue(headingRad)) {
            return;
        }
        double heading = PlanarPose.normalizeRadians(headingRad);
        if (count == 0) {
            append(timeNanos, x, y, heading);
            return;
        }
        long newest = timeAt(count - 1);
        if (timeNanos < newest) {
            if (newest - timeNanos > BACKWARD_RESET_NANOS) {
                start = 0;
                count = 0;
                append(timeNanos, x, y, heading);
            }
            return;
        }
        if (timeNanos == newest) {
            int index = physical(count - 1);
            xs[index] = x;
            ys[index] = y;
            if (count == 1) {
                headings[index] = heading;
            } else {
                double reference = headings[physical(count - 2)];
                headings[index] = reference + PlanarPose.normalizeRadians(heading - reference);
            }
            return;
        }
        double reference = headings[physical(count - 1)];
        append(timeNanos, x, y, reference + PlanarPose.normalizeRadians(heading - reference));
    }

    public synchronized PlanarPose poseAt(long timeNanos, long maxExtrapolateNanos) {
        if (count == 0) {
            return null;
        }
        long oldest = timeAt(0);
        long newest = timeAt(count - 1);
        if (timeNanos > newest) {
            if (timeNanos - newest <= Math.max(0L, maxExtrapolateNanos)) {
                return sample(count - 1);
            }
            return null;
        }
        if (timeNanos < oldest) {
            return null;
        }
        if (timeNanos == newest) {
            return sample(count - 1);
        }
        int lower = lowerBracket(timeNanos);
        long t0 = timeAt(lower);
        long t1 = timeAt(lower + 1);
        if (t1 == t0) {
            return sample(lower + 1);
        }
        double fraction = (double) (timeNanos - t0) / (double) (t1 - t0);
        int a = physical(lower);
        int b = physical(lower + 1);
        double x = xs[a] + (xs[b] - xs[a]) * fraction;
        double y = ys[a] + (ys[b] - ys[a]) * fraction;
        double heading = headings[a] + (headings[b] - headings[a]) * fraction;
        return new PlanarPose(x, y, heading);
    }

    public synchronized PlanarPose newest() {
        return count == 0 ? null : sample(count - 1);
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

    private PlanarPose sample(int logical) {
        int index = physical(logical);
        return new PlanarPose(xs[index], ys[index], headings[index]);
    }

    private void append(long timeNanos, double x, double y, double heading) {
        int index;
        if (count < times.length) {
            index = physical(count);
            count++;
        } else {
            index = start;
            start = (start + 1) % times.length;
        }
        times[index] = timeNanos;
        xs[index] = x;
        ys[index] = y;
        headings[index] = heading;
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
}
