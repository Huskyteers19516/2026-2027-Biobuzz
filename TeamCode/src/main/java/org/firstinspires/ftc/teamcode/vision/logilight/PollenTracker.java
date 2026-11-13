package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

public final class PollenTracker {

    public static final int MAX_FRAMES = 120;
    private static final double NANOS_PER_SECOND = 1e9;

    public static final class Ball {
        public final double x;
        public final double y;
        public final double forward;
        public final double left;
        public final double distance;
        public final double bearingLeftDegrees;
        public final long captureNanos;

        public Ball(double x, double y, double forward, double left, double distance, double bearingLeftDegrees,
                    long captureNanos) {
            this.x = x;
            this.y = y;
            this.forward = forward;
            this.left = left;
            this.distance = distance;
            this.bearingLeftDegrees = bearingLeftDegrees;
            this.captureNanos = captureNanos;
        }

        public double distanceTo(double px, double py) {
            return Math.hypot(x - px, y - py);
        }
    }

    public static final class Cluster {
        public final double x;
        public final double y;
        public final int frames;
        public final long newestNanos;
        final List<Ball> members;
        double sortKey;

        Cluster(double x, double y, List<Ball> members) {
            this.x = x;
            this.y = y;
            this.members = members;
            this.frames = members.size();
            long newest = 0L;
            for (Ball ball : members) {
                newest = Math.max(newest, ball.captureNanos);
            }
            this.newestNanos = newest;
        }

        public double distanceTo(double px, double py) {
            return Math.hypot(x - px, y - py);
        }
    }

    private static final class Frame {
        final long nanos;
        final List<Ball> balls;

        Frame(long nanos, List<Ball> balls) {
            this.nanos = nanos;
            this.balls = balls;
        }
    }

    private final ArrayDeque<Frame> frames = new ArrayDeque<Frame>();
    private final ArrayDeque<Ball> track = new ArrayDeque<Ball>();
    private Frame newest;
    private boolean locked;
    private double estimateX = Double.NaN;
    private double estimateY = Double.NaN;
    private int framesSeen;
    private long lastSeenNanos;
    private long lockNanos;
    private Ball lastAssociated;

    public synchronized boolean addFrame(long captureNanos, List<Ball> balls, PollenSeekConfig config) {
        if (newest != null && captureNanos <= newest.nanos) {
            return false;
        }
        List<Ball> copy = balls == null ? new ArrayList<Ball>() : new ArrayList<Ball>(balls);
        Frame frame = new Frame(captureNanos, Collections.unmodifiableList(copy));
        frames.addLast(frame);
        newest = frame;
        long keepNanos = (long) (Math.max(config.confirmWindowSeconds, 1.0) * NANOS_PER_SECOND);
        while (frames.size() > MAX_FRAMES || (!frames.isEmpty() && captureNanos - frames.peekFirst().nanos > keepNanos)) {
            frames.removeFirst();
        }
        if (locked) {
            associate(frame, config);
        }
        return true;
    }

    private void associate(Frame frame, PollenSeekConfig config) {
        Ball best = nearest(frame.balls, estimateX, estimateY, config.associateRadiusInches);
        if (best == null) {
            return;
        }
        track.addLast(best);
        while (track.size() > Math.max(1, config.averageFrames)) {
            track.removeFirst();
        }
        recomputeEstimate();
        framesSeen++;
        lastSeenNanos = frame.nanos;
        lastAssociated = best;
    }

    private void recomputeEstimate() {
        double sumX = 0.0;
        double sumY = 0.0;
        for (Ball ball : track) {
            sumX += ball.x;
            sumY += ball.y;
        }
        if (!track.isEmpty()) {
            estimateX = sumX / track.size();
            estimateY = sumY / track.size();
        }
    }

    public synchronized List<Cluster> confirmedClusters(PollenSeekConfig config, long nowNanos, final double robotX,
                                                        final double robotY) {
        long window = (long) (config.confirmWindowSeconds * NANOS_PER_SECOND);
        List<Frame> recent = new ArrayList<Frame>();
        Iterator<Frame> it = frames.descendingIterator();
        while (it.hasNext()) {
            Frame frame = it.next();
            if (nowNanos - frame.nanos > window) {
                break;
            }
            recent.add(frame);
        }
        List<Cluster> clusters = new ArrayList<Cluster>();
        if (recent.size() < Math.max(1, config.confirmFrames)) {
            return clusters;
        }
        double radius = config.clusterRadiusInches;
        for (Frame seedFrame : recent) {
            for (Ball seed : seedFrame.balls) {
                boolean covered = false;
                for (Cluster existing : clusters) {
                    if (existing.distanceTo(seed.x, seed.y) <= radius) {
                        covered = true;
                        break;
                    }
                }
                if (covered) {
                    continue;
                }
                List<Ball> members = new ArrayList<Ball>();
                double sumX = 0.0;
                double sumY = 0.0;
                for (Frame frame : recent) {
                    Ball match = nearest(frame.balls, seed.x, seed.y, radius);
                    if (match != null) {
                        members.add(match);
                        sumX += match.x;
                        sumY += match.y;
                    }
                }
                if (members.size() >= config.confirmFrames) {
                    clusters.add(new Cluster(sumX / members.size(), sumY / members.size(), members));
                }
            }
        }
        for (Cluster cluster : clusters) {
            cluster.sortKey = cluster.distanceTo(robotX, robotY);
        }
        Collections.sort(clusters, new Comparator<Cluster>() {
            @Override
            public int compare(Cluster a, Cluster b) {
                return Double.compare(a.sortKey, b.sortKey);
            }
        });
        return clusters;
    }

    public synchronized void lock(Cluster cluster, PollenSeekConfig config, long nowNanos) {
        if (cluster == null) {
            return;
        }
        locked = true;
        track.clear();
        List<Ball> members = new ArrayList<Ball>(cluster.members);
        Collections.sort(members, new Comparator<Ball>() {
            @Override
            public int compare(Ball a, Ball b) {
                return Long.compare(a.captureNanos, b.captureNanos);
            }
        });
        int keep = Math.max(1, config.averageFrames);
        for (int i = Math.max(0, members.size() - keep); i < members.size(); i++) {
            track.addLast(members.get(i));
        }
        recomputeEstimate();
        framesSeen = cluster.frames;
        lastSeenNanos = cluster.newestNanos;
        lastAssociated = members.isEmpty() ? null : members.get(members.size() - 1);
        lockNanos = nowNanos;
    }

    public synchronized void unlock() {
        locked = false;
        track.clear();
        estimateX = Double.NaN;
        estimateY = Double.NaN;
        framesSeen = 0;
        lastSeenNanos = 0L;
        lastAssociated = null;
        lockNanos = 0L;
    }

    public synchronized void clear() {
        frames.clear();
        newest = null;
        unlock();
    }

    public synchronized boolean isLocked() {
        return locked;
    }

    public synchronized double estimateX() {
        return estimateX;
    }

    public synchronized double estimateY() {
        return estimateY;
    }

    public synchronized int framesSeen() {
        return framesSeen;
    }

    public synchronized long lastSeenNanos() {
        return lastSeenNanos;
    }

    public synchronized long lockNanos() {
        return lockNanos;
    }

    public synchronized Ball lastAssociated() {
        return lastAssociated;
    }

    public synchronized long newestFrameNanos() {
        return newest == null ? 0L : newest.nanos;
    }

    public synchronized List<Ball> newestBalls() {
        return newest == null ? Collections.<Ball>emptyList() : newest.balls;
    }

    public synchronized Ball newestNearestTo(double x, double y, double maxDistance) {
        return newest == null ? null : nearest(newest.balls, x, y, maxDistance);
    }

    public synchronized Ball newestNearestToRobot() {
        if (newest == null) {
            return null;
        }
        Ball best = null;
        for (Ball ball : newest.balls) {
            if (best == null || ball.distance < best.distance) {
                best = ball;
            }
        }
        return best;
    }

    static Ball nearest(List<Ball> balls, double x, double y, double maxDistance) {
        Ball best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (Ball ball : balls) {
            double d = ball.distanceTo(x, y);
            if (d <= maxDistance && d < bestDistance) {
                best = ball;
                bestDistance = d;
            }
        }
        return best;
    }
}
