package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class PollenRoutePlanner {

    public static final double FIELD_SIZE_INCHES = PollenSeekConfig.FIELD_SIZE_INCHES;
    public static final String HIVE_NAME = "HIVE";
    public static final double HIVE_MIN_X = 47.3;
    public static final double HIVE_MIN_Y = 52.5;
    public static final double HIVE_MAX_X = 96.7;
    public static final double HIVE_MAX_Y = 91.5;
    public static final String[] FLOWER_NAMES = {"red FLOWER", "rear FLOWER", "blue FLOWER", "audience FLOWER"};
    public static final double[][] FLOWER_BOXES = {
            {0.0, 45.0, 5.0, 51.0},
            {45.0, 139.0, 51.0, 144.0},
            {139.0, 93.0, 144.0, 99.0},
            {93.0, 0.0, 99.0, 5.0}
    };

    private static final double EPSILON = 1e-9;
    private static final double MIN_LEG_INCHES = 1.0;
    private static final double LEG_ADVANCE_T = 1e-6;

    public enum Status { OK, ARRIVED, UNREACHABLE }

    public static final class Box {
        public final String name;
        public final double minX;
        public final double minY;
        public final double maxX;
        public final double maxY;

        public Box(String name, double minX, double minY, double maxX, double maxY) {
            this.name = name == null ? "keep-out" : name;
            this.minX = Math.min(minX, maxX);
            this.minY = Math.min(minY, maxY);
            this.maxX = Math.max(minX, maxX);
            this.maxY = Math.max(minY, maxY);
        }

        public Box inflated(double amount) {
            return new Box(name, minX - amount, minY - amount, maxX + amount, maxY + amount);
        }

        public boolean contains(double x, double y) {
            return x > minX + EPSILON && x < maxX - EPSILON && y > minY + EPSILON && y < maxY - EPSILON;
        }

        public boolean containsClosed(double x, double y) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }

        public boolean intersectsSegment(double ax, double ay, double bx, double by) {
            double dx = bx - ax;
            double dy = by - ay;
            double t0 = 0.0;
            double t1 = 1.0;
            double[] p = {-dx, dx, -dy, dy};
            double[] q = {ax - minX, maxX - ax, ay - minY, maxY - ay};
            for (int i = 0; i < 4; i++) {
                if (Math.abs(p[i]) < EPSILON) {
                    if (q[i] <= EPSILON) {
                        return false;
                    }
                    continue;
                }
                double r = q[i] / p[i];
                if (p[i] < 0.0) {
                    if (r > t1) {
                        return false;
                    }
                    if (r > t0) {
                        t0 = r;
                    }
                } else {
                    if (r < t0) {
                        return false;
                    }
                    if (r < t1) {
                        t1 = r;
                    }
                }
            }
            return t1 - t0 > EPSILON;
        }
    }

    public static final class Plan {
        public final Status status;
        public final String reason;
        public final double ballX;
        public final double ballY;
        public final double approachX;
        public final double approachY;
        public final double approachHeading;
        public final double[] xs;
        public final double[] ys;
        public final double length;
        public final boolean clamped;

        Plan(Status status, String reason, double ballX, double ballY, double approachX, double approachY,
             double approachHeading, double[] xs, double[] ys, boolean clamped) {
            this.status = status;
            this.reason = reason == null ? "" : reason;
            this.ballX = ballX;
            this.ballY = ballY;
            this.approachX = approachX;
            this.approachY = approachY;
            this.approachHeading = approachHeading;
            this.xs = xs == null ? new double[0] : xs;
            this.ys = ys == null ? new double[0] : ys;
            this.length = polylineLength(this.xs, this.ys);
            this.clamped = clamped;
        }

        public int pointCount() {
            return xs.length;
        }

        public int legCount() {
            return Math.max(0, xs.length - 1);
        }

        public int viaCount() {
            return Math.max(0, xs.length - 2);
        }

        public double firstHeading() {
            if (xs.length >= 2) {
                return Math.atan2(ys[1] - ys[0], xs[1] - xs[0]);
            }
            return approachHeading;
        }

        public double finalLegHeading() {
            int n = xs.length;
            if (n >= 2) {
                return Math.atan2(ys[n - 1] - ys[n - 2], xs[n - 1] - xs[n - 2]);
            }
            return approachHeading;
        }

        public boolean reachable() {
            return status != Status.UNREACHABLE;
        }

        public String describe() {
            if (status == Status.UNREACHABLE) {
                return "UNREACHABLE - " + reason;
            }
            if (status == Status.ARRIVED) {
                return "ARRIVED - already within reach" + (clamped ? " (approach clamped to the wall margin)" : "");
            }
            String kind = viaCount() == 0 ? "direct" : String.format(Locale.US, "via %d point%s", viaCount(),
                    viaCount() == 1 ? "" : "s");
            return String.format(Locale.US, "%s  %.1f in  stop at (%.1f, %.1f) facing %.0f deg%s", kind, length,
                    approachX, approachY, Math.toDegrees(approachHeading), clamped ? "  (clamped to the wall margin)" : "");
        }
    }

    public static final class Progress {
        public final double remaining;
        public final int legIndex;

        Progress(double remaining, int legIndex) {
            this.remaining = remaining;
            this.legIndex = legIndex;
        }
    }

    private final PollenSeekConfig config;
    private final Box hive;
    private final List<Box> keepOuts;
    private final List<double[]> corners;

    public PollenRoutePlanner(PollenSeekConfig config) {
        this.config = config == null ? new PollenSeekConfig() : config;
        this.hive = new Box(HIVE_NAME, HIVE_MIN_X, HIVE_MIN_Y, HIVE_MAX_X, HIVE_MAX_Y);
        List<Box> boxes = new ArrayList<Box>();
        if (this.config.avoidHive) {
            boxes.add(hive);
        }
        if (this.config.avoidFlowers) {
            for (int i = 0; i < FLOWER_BOXES.length; i++) {
                double[] b = FLOWER_BOXES[i];
                boxes.add(new Box(FLOWER_NAMES[i], b[0], b[1], b[2], b[3]));
            }
        }
        boxes.addAll(parseKeepOuts(this.config.extraKeepOuts));
        this.keepOuts = Collections.unmodifiableList(boxes);
        List<double[]> list = new ArrayList<double[]>();
        if (this.config.avoidHive) {
            double out = softInflation() + this.config.cornerPushInches;
            list.add(new double[]{hive.minX - out, hive.minY - out});
            list.add(new double[]{hive.maxX + out, hive.minY - out});
            list.add(new double[]{hive.maxX + out, hive.maxY + out});
            list.add(new double[]{hive.minX - out, hive.maxY + out});
        }
        this.corners = Collections.unmodifiableList(list);
    }

    public List<Box> keepOuts() {
        return keepOuts;
    }

    public List<double[]> viaCorners() {
        return corners;
    }

    public double lowBound() {
        return config.robotRadiusInches + config.wallMarginInches;
    }

    public double highBound() {
        return FIELD_SIZE_INCHES - lowBound();
    }

    public double hardInflation() {
        return config.robotRadiusInches;
    }

    public double softInflation() {
        return config.robotRadiusInches + config.obstacleMarginInches;
    }

    public boolean inBounds(double x, double y) {
        double lo = lowBound();
        double hi = highBound();
        return x >= lo - EPSILON && x <= hi + EPSILON && y >= lo - EPSILON && y <= hi + EPSILON;
    }

    public Box keepOutContaining(double x, double y) {
        for (Box box : keepOuts) {
            if (box.containsClosed(x, y)) {
                return box;
            }
        }
        return null;
    }

    public Plan plan(double robotX, double robotY, double robotHeading, double ballX, double ballY) {
        if (!finite(robotX) || !finite(robotY) || !finite(ballX) || !finite(ballY)) {
            return unreachable("pose or ball position is not a number", ballX, ballY);
        }
        if (robotX < 0.0 || robotX > FIELD_SIZE_INCHES || robotY < 0.0 || robotY > FIELD_SIZE_INCHES) {
            return unreachable(String.format(Locale.US, "robot pose (%.1f, %.1f) is outside the field", robotX, robotY),
                    ballX, ballY);
        }
        if (ballX < 0.0 || ballX > FIELD_SIZE_INCHES || ballY < 0.0 || ballY > FIELD_SIZE_INCHES) {
            return unreachable(String.format(Locale.US, "ball (%.1f, %.1f) is outside the field", ballX, ballY),
                    ballX, ballY);
        }
        Box holding = keepOutContaining(ballX, ballY);
        if (holding != null) {
            String where = HIVE_NAME.equals(holding.name) ? "under the HIVE" : "inside the " + holding.name;
            return unreachable(String.format(Locale.US, "ball (%.1f, %.1f) is %s", ballX, ballY, where), ballX, ballY);
        }
        for (Box box : keepOuts) {
            if (box.inflated(hardInflation()).contains(robotX, robotY)) {
                return unreachable("robot is against the " + box.name + " - drive away from it first", ballX, ballY);
            }
        }
        double toBall = Math.hypot(ballX - robotX, ballY - robotY);
        if (toBall <= config.stopShortInches + config.minPathInches) {
            double heading = toBall > EPSILON ? Math.atan2(ballY - robotY, ballX - robotX) : robotHeading;
            return new Plan(Status.ARRIVED, "", ballX, ballY, robotX, robotY, heading,
                    new double[]{robotX}, new double[]{robotY}, false);
        }
        List<double[][]> candidates = new ArrayList<double[][]>();
        candidates.add(new double[0][]);
        int n = corners.size();
        for (int i = 0; i < n; i++) {
            candidates.add(new double[][]{corners.get(i)});
        }
        for (int i = 0; i < n; i++) {
            candidates.add(new double[][]{corners.get(i), corners.get((i + 1) % n)});
            candidates.add(new double[][]{corners.get(i), corners.get((i + n - 1) % n)});
        }
        Plan best = null;
        String directReason = null;
        for (double[][] vias : candidates) {
            String[] why = new String[1];
            Plan candidate = tryRoute(robotX, robotY, ballX, ballY, vias, why);
            if (vias.length == 0 && candidate == null) {
                directReason = why[0];
            }
            if (candidate != null && (best == null || candidate.length < best.length - EPSILON)) {
                best = candidate;
            }
        }
        if (best == null) {
            String reason = directReason == null ? "no route found" : directReason;
            return unreachable(reason + (corners.isEmpty() ? "" : " and no route around the HIVE corners works"),
                    ballX, ballY);
        }
        if (best.length < config.minPathInches) {
            return new Plan(Status.ARRIVED, "", ballX, ballY, robotX, robotY, best.approachHeading,
                    new double[]{robotX}, new double[]{robotY}, best.clamped);
        }
        return best;
    }

    private Plan tryRoute(double robotX, double robotY, double ballX, double ballY, double[][] vias, String[] why) {
        List<double[]> points = new ArrayList<double[]>();
        points.add(new double[]{robotX, robotY});
        for (double[] via : vias) {
            if (!inBounds(via[0], via[1])) {
                why[0] = "via point outside the wall margin";
                return null;
            }
            double[] last = points.get(points.size() - 1);
            if (Math.hypot(via[0] - last[0], via[1] - last[1]) >= MIN_LEG_INCHES) {
                points.add(via);
            }
        }
        double[] last = points.get(points.size() - 1);
        double dx = ballX - last[0];
        double dy = ballY - last[1];
        double distance = Math.hypot(dx, dy);
        if (distance <= config.stopShortInches + EPSILON) {
            if (points.size() > 1) {
                why[0] = "via point too close to the ball";
                return null;
            }
        }
        double ux = distance > EPSILON ? dx / distance : 0.0;
        double uy = distance > EPSILON ? dy / distance : 0.0;
        double ax = ballX - ux * config.stopShortInches;
        double ay = ballY - uy * config.stopShortInches;
        double lo = lowBound();
        double hi = highBound();
        double cx = clamp(ax, lo, hi);
        double cy = clamp(ay, lo, hi);
        boolean clamped = Math.abs(cx - ax) > EPSILON || Math.abs(cy - ay) > EPSILON;
        double toBallX = ballX - cx;
        double toBallY = ballY - cy;
        double heading = Math.hypot(toBallX, toBallY) > EPSILON ? Math.atan2(toBallY, toBallX) : Math.atan2(uy, ux);
        for (Box box : keepOuts) {
            if (box.inflated(softInflation()).contains(cx, cy)) {
                why[0] = String.format(Locale.US, "stop point (%.1f, %.1f) would be too close to the %s", cx, cy, box.name);
                return null;
            }
        }
        double[] tail = points.get(points.size() - 1);
        if (Math.hypot(cx - tail[0], cy - tail[1]) >= MIN_LEG_INCHES || points.size() == 1) {
            points.add(new double[]{cx, cy});
        } else {
            points.set(points.size() - 1, new double[]{cx, cy});
        }
        for (int i = 0; i + 1 < points.size(); i++) {
            double[] a = points.get(i);
            double[] b = points.get(i + 1);
            String blocked = blockedBy(a[0], a[1], b[0], b[1]);
            if (blocked != null) {
                why[0] = "path blocked by the " + blocked;
                return null;
            }
        }
        double[] xs = new double[points.size()];
        double[] ys = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            xs[i] = points.get(i)[0];
            ys[i] = points.get(i)[1];
        }
        return new Plan(Status.OK, "", ballX, ballY, cx, cy, heading, xs, ys, clamped);
    }

    public String blockedBy(double ax, double ay, double bx, double by) {
        for (Box box : keepOuts) {
            Box hard = box.inflated(hardInflation());
            if (hard.intersectsSegment(ax, ay, bx, by)) {
                return box.name;
            }
            Box soft = box.inflated(softInflation());
            if (!soft.contains(ax, ay) && soft.intersectsSegment(ax, ay, bx, by)) {
                return box.name;
            }
        }
        return null;
    }

    public static Progress progress(Plan plan, double x, double y, int fromLeg) {
        if (plan == null || plan.xs.length == 0) {
            return new Progress(Double.NaN, 0);
        }
        if (plan.xs.length == 1) {
            return new Progress(Math.hypot(plan.xs[0] - x, plan.ys[0] - y), 0);
        }
        int legs = plan.xs.length - 1;
        int first = Math.max(0, Math.min(legs - 1, fromLeg));
        int last = Math.min(legs - 1, first + 1);
        double[] tails = new double[legs];
        double running = 0.0;
        for (int i = legs - 1; i >= 0; i--) {
            tails[i] = running;
            running += Math.hypot(plan.xs[i + 1] - plan.xs[i], plan.ys[i + 1] - plan.ys[i]);
        }
        double[] current = legCost(plan, first, x, y, tails);
        if (last > first) {
            double[] next = legCost(plan, last, x, y, tails);
            boolean pastCorner = next[1] > LEG_ADVANCE_T && next[0] < current[0] - EPSILON;
            boolean endReached = current[1] >= 1.0 - LEG_ADVANCE_T;
            if (pastCorner || endReached) {
                return new Progress(next[0], last);
            }
        }
        return new Progress(current[0], first);
    }

    private static double[] legCost(Plan plan, int i, double x, double y, double[] tails) {
        double ax = plan.xs[i];
        double ay = plan.ys[i];
        double dx = plan.xs[i + 1] - ax;
        double dy = plan.ys[i + 1] - ay;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared > EPSILON ? ((x - ax) * dx + (y - ay) * dy) / lengthSquared : 1.0;
        t = clamp(t, 0.0, 1.0);
        double px = ax + dx * t;
        double py = ay + dy * t;
        double cost = Math.hypot(x - px, y - py) + (1.0 - t) * Math.sqrt(lengthSquared) + tails[i];
        return new double[]{cost, t};
    }

    public static List<Box> parseKeepOuts(String text) {
        List<Box> boxes = new ArrayList<Box>();
        if (text == null || text.trim().isEmpty()) {
            return boxes;
        }
        String[] parts = text.split(";");
        int index = 1;
        for (String part : parts) {
            String[] numbers = part.trim().split(",");
            if (numbers.length != 4) {
                continue;
            }
            try {
                double minX = Double.parseDouble(numbers[0].trim());
                double minY = Double.parseDouble(numbers[1].trim());
                double maxX = Double.parseDouble(numbers[2].trim());
                double maxY = Double.parseDouble(numbers[3].trim());
                if (finite(minX) && finite(minY) && finite(maxX) && finite(maxY)) {
                    boxes.add(new Box("keep-out " + index, minX, minY, maxX, maxY));
                    index++;
                }
            } catch (NumberFormatException ignored) {
            }
        }
        return boxes;
    }

    private static Plan unreachable(String reason, double ballX, double ballY) {
        return new Plan(Status.UNREACHABLE, reason, ballX, ballY, Double.NaN, Double.NaN, Double.NaN, null, null, false);
    }

    static double polylineLength(double[] xs, double[] ys) {
        double total = 0.0;
        for (int i = 0; i + 1 < xs.length && i + 1 < ys.length; i++) {
            total += Math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]);
        }
        return total;
    }

    private static double clamp(double value, double low, double high) {
        return value < low ? low : (value > high ? high : value);
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
