package org.firstinspires.ftc.teamtestcode.other.robot.launcher;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

public final class ShotTable {

    public enum Model { AUTO, INTERPOLATE, LINEAR, QUADRATIC }

    public static final Model MODEL = Model.AUTO;

    public static final double EXTRAPOLATE_INCHES = 12.0;

    public static final double MAX_DIP_TICKS = 40.0;

    public static final double MERGE_DISTANCE_INCHES = 1.0;

    public static final double[][] SHOT_DATA = {
            {24.0, 1600.0},
            {144.0, 1600.0},
    };

    public static final class Point {
        public final double distance;
        public final double velocity;
        public final int samples;

        Point(double distance, double velocity, int samples) {
            this.distance = distance;
            this.velocity = velocity;
            this.samples = samples;
        }
    }

    public static final class Fit {
        public final Model model;
        public final String note;
        public final double[] coefficients;
        public final List<Point> points;
        public final double minDistance;
        public final double maxDistance;
        public final double rmse;

        Fit(Model model, String note, double[] coefficients, List<Point> points) {
            this.model = model;
            this.note = note;
            this.coefficients = coefficients;
            this.points = points;
            this.minDistance = points.get(0).distance;
            this.maxDistance = points.get(points.size() - 1).distance;
            double sum = 0.0;
            int count = 0;
            for (double[] row : SHOT_DATA) {
                double error = rawValue(row[0]) - row[1];
                sum += error * error;
                count++;
            }
            this.rmse = Math.sqrt(sum / count);
        }

        public double velocity(double inches) {
            double low = minDistance - EXTRAPOLATE_INCHES;
            double high = maxDistance + EXTRAPOLATE_INCHES;
            double d = Math.max(low, Math.min(high, inches));
            double v;
            if (d < minDistance) {
                v = rawValue(minDistance) - slope(minDistance, true) * (minDistance - d);
            } else if (d > maxDistance) {
                v = rawValue(maxDistance) + slope(maxDistance, false) * (d - maxDistance);
            } else {
                v = rawValue(d);
            }
            return Math.max(Launcher.MIN_TARGET_VELOCITY, Math.min(Launcher.MAX_TARGET_VELOCITY, v));
        }

        public String formula() {
            switch (model) {
                case INTERPOLATE:
                    return "straight lines between the " + points.size() + " measured distances";
                case QUADRATIC:
                    return String.format(Locale.US, "v = %.1f %+.3f*d %+.5f*d^2",
                            coefficients[0], coefficients[1], coefficients[2]);
                case LINEAR:
                    return String.format(Locale.US, "v = %.1f %+.3f*d", coefficients[0], coefficients[1]);
                default:
                    return String.format(Locale.US, "v = %.1f", coefficients[0]);
            }
        }

        double rawValue(double d) {
            if (model == Model.INTERPOLATE) {
                return interpolate(points, d);
            }
            double v = 0.0;
            double power = 1.0;
            for (double c : coefficients) {
                v += c * power;
                power *= d;
            }
            return v;
        }

        private double slope(double edge, boolean lowEdge) {
            if (model == Model.INTERPOLATE) {
                int n = points.size();
                if (n < 2) {
                    return 0.0;
                }
                Point a = lowEdge ? points.get(0) : points.get(n - 1);
                Point b = lowEdge ? points.get(n - 1) : points.get(0);
                for (int i = 1; i < n; i++) {
                    Point p = lowEdge ? points.get(i) : points.get(n - 1 - i);
                    if (Math.abs(p.distance - a.distance) >= EXTRAPOLATE_INCHES) {
                        b = p;
                        break;
                    }
                }
                return Math.max(0.0, (b.velocity - a.velocity) / (b.distance - a.distance));
            }
            double s = 0.0;
            double power = 1.0;
            for (int i = 1; i < coefficients.length; i++) {
                s += i * coefficients[i] * power;
                power *= edge;
            }
            return s;
        }
    }

    private static final String PROBLEM;
    private static final Fit FIT;

    static {
        String problem;
        Fit fit = null;
        try {
            problem = findProblem();
            if (problem == null) {
                fit = buildFit();
            }
        } catch (RuntimeException e) {
            problem = "SHOT_DATA cannot be fitted: " + e.getMessage();
            fit = null;
        }
        PROBLEM = problem;
        FIT = fit;
    }

    private ShotTable() {
    }

    public static String problem() {
        return PROBLEM;
    }

    public static Fit fit() {
        return FIT;
    }

    public static double velocityForDistance(double inches) {
        if (FIT == null || Double.isNaN(inches) || Double.isInfinite(inches)) {
            return Launcher.DEFAULT_TARGET_VELOCITY;
        }
        return FIT.velocity(inches);
    }

    public static String summary() {
        if (FIT == null) {
            return "BROKEN: " + PROBLEM;
        }
        return String.format(Locale.US, "%s %s  (%d shots, %.0f-%.0f in, rms error %.0f)",
                FIT.model, FIT.note, SHOT_DATA.length, FIT.minDistance, FIT.maxDistance, FIT.rmse);
    }

    private static String findProblem() {
        if (SHOT_DATA.length == 0) {
            return "SHOT_DATA is empty - add at least one {distance, velocity} row";
        }
        for (int i = 0; i < SHOT_DATA.length; i++) {
            double[] row = SHOT_DATA[i];
            if (row == null || row.length != 2) {
                return "SHOT_DATA row " + (i + 1) + " must be {distance, velocity}";
            }
            if (!(row[0] > 0.0) || Double.isInfinite(row[0])) {
                return "SHOT_DATA row " + (i + 1) + " distance must be more than 0 inches";
            }
            if (!(row[1] >= Launcher.MIN_TARGET_VELOCITY && row[1] <= Launcher.MAX_TARGET_VELOCITY)) {
                return String.format(Locale.US, "SHOT_DATA row %d velocity must be %.0f-%.0f",
                        i + 1, Launcher.MIN_TARGET_VELOCITY, Launcher.MAX_TARGET_VELOCITY);
            }
        }
        if (MODEL == Model.LINEAR && distinctPoints().size() < 2) {
            return "MODEL LINEAR needs shots at 2 or more different distances";
        }
        if (MODEL == Model.QUADRATIC && distinctPoints().size() < 3) {
            return "MODEL QUADRATIC needs shots at 3 or more different distances";
        }
        return null;
    }

    private static Fit buildFit() {
        List<Point> points = distinctPoints();
        int distinct = points.size();
        switch (MODEL) {
            case INTERPOLATE:
                return new Fit(Model.INTERPOLATE, "", new double[0], points);
            case LINEAR:
                return new Fit(Model.LINEAR, "", polyfit(1), points);
            case QUADRATIC:
                return new Fit(Model.QUADRATIC, "", polyfit(2), points);
            default:
                break;
        }
        if (distinct == 1) {
            return new Fit(Model.LINEAR, "[one distance only: same velocity everywhere]",
                    new double[] {points.get(0).velocity, 0.0}, points);
        }
        if (distinct == 2) {
            return new Fit(Model.LINEAR, "[2 distances: straight line]", polyfit(1), points);
        }
        double[] q = polyfit(2);
        double lo = points.get(0).distance;
        double hi = points.get(distinct - 1).distance;
        double atLo = q[0] + q[1] * lo + q[2] * lo * lo;
        double atHi = q[0] + q[1] * hi + q[2] * hi * hi;
        double drop;
        if (q[2] == 0.0) {
            drop = atLo - atHi;
        } else {
            double vertex = Math.max(lo, Math.min(hi, -q[1] / (2.0 * q[2])));
            double atVertex = q[0] + q[1] * vertex + q[2] * vertex * vertex;
            drop = q[2] > 0.0 ? atLo - atVertex : atVertex - atHi;
        }
        if (drop > MAX_DIP_TICKS) {
            return new Fit(Model.INTERPOLATE, "[fitted curve goes DOWN with distance, using straight lines between your points - check your data]",
                    new double[0], points);
        }
        return new Fit(Model.QUADRATIC, "", q, points);
    }

    private static List<Point> distinctPoints() {
        double[][] rows = SHOT_DATA.clone();
        Arrays.sort(rows, Comparator.comparingDouble(r -> r[0]));
        List<Point> points = new ArrayList<>();
        int i = 0;
        while (i < rows.length) {
            double first = rows[i][0];
            double sumD = 0.0;
            double sumV = 0.0;
            int n = 0;
            while (i < rows.length && rows[i][0] - first <= MERGE_DISTANCE_INCHES) {
                sumD += rows[i][0];
                sumV += rows[i][1];
                n++;
                i++;
            }
            points.add(new Point(sumD / n, sumV / n, n));
        }
        return points;
    }

    private static double[] polyfit(int degree) {
        double mean = 0.0;
        for (double[] row : SHOT_DATA) {
            mean += row[0];
        }
        mean /= SHOT_DATA.length;
        double scale = 0.0;
        for (double[] row : SHOT_DATA) {
            scale = Math.max(scale, Math.abs(row[0] - mean));
        }
        if (scale == 0.0) {
            scale = 1.0;
        }
        int size = degree + 1;
        double[][] a = new double[size][size + 1];
        for (double[] row : SHOT_DATA) {
            double x = (row[0] - mean) / scale;
            double[] powers = new double[2 * degree + 1];
            powers[0] = 1.0;
            for (int k = 1; k < powers.length; k++) {
                powers[k] = powers[k - 1] * x;
            }
            for (int r = 0; r < size; r++) {
                for (int c = 0; c < size; c++) {
                    a[r][c] += powers[r + c];
                }
                a[r][size] += powers[r] * row[1];
            }
        }
        double[] scaled = solve(a);
        double[] result = new double[size];
        for (int k = 0; k < size; k++) {
            double[] expanded = expandShifted(k, mean, scale);
            for (int j = 0; j <= k; j++) {
                result[j] += scaled[k] * expanded[j];
            }
        }
        return result;
    }

    private static double[] expandShifted(int k, double mean, double scale) {
        double[] coefficients = new double[k + 1];
        double factor = Math.pow(scale, -k);
        for (int j = 0; j <= k; j++) {
            coefficients[j] = factor * binomial(k, j) * Math.pow(-mean, k - j);
        }
        return coefficients;
    }

    private static double binomial(int n, int k) {
        double result = 1.0;
        for (int i = 1; i <= k; i++) {
            result = result * (n - k + i) / i;
        }
        return result;
    }

    private static double[] solve(double[][] a) {
        int n = a.length;
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) {
                    pivot = r;
                }
            }
            double[] tmp = a[col];
            a[col] = a[pivot];
            a[pivot] = tmp;
            if (Math.abs(a[col][col]) < 1e-12) {
                throw new IllegalStateException("shot data cannot be fitted");
            }
            for (int r = 0; r < n; r++) {
                if (r == col) {
                    continue;
                }
                double f = a[r][col] / a[col][col];
                for (int c = col; c <= n; c++) {
                    a[r][c] -= f * a[col][c];
                }
            }
        }
        double[] x = new double[n];
        for (int i = 0; i < n; i++) {
            x[i] = a[i][n] / a[i][i];
        }
        return x;
    }

    private static double interpolate(List<Point> points, double d) {
        if (points.size() == 1 || d <= points.get(0).distance) {
            return points.get(0).velocity;
        }
        for (int i = 1; i < points.size(); i++) {
            Point a = points.get(i - 1);
            Point b = points.get(i);
            if (d <= b.distance) {
                double t = (d - a.distance) / (b.distance - a.distance);
                return a.velocity + t * (b.velocity - a.velocity);
            }
        }
        return points.get(points.size() - 1).velocity;
    }
}
