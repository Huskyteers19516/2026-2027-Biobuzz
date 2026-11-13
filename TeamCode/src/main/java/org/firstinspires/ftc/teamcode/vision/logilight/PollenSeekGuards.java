package org.firstinspires.ftc.teamcode.vision.logilight;

import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;
import org.firstinspires.ftc.teamcode.vision.logilight.math.TargetGeometry;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;

import java.util.Locale;

public final class PollenSeekGuards {

    private static final double NANOS_PER_SECOND = 1e9;

    private PollenSeekGuards() {
    }

    public static final class ProgressMonitor {
        private boolean active;
        private double best;
        private double last;
        private double markBest;
        private long markNanos;
        private boolean diverged;

        public void start(double remaining, long nowNanos) {
            diverged = false;
            active = isFinite(remaining);
            best = remaining;
            last = remaining;
            markBest = remaining;
            markNanos = nowNanos;
        }

        public void replanned(double newRemaining) {
            if (!active || !isFinite(newRemaining)) {
                return;
            }
            double excess = Math.max(0.0, last - best);
            double newBest = newRemaining - excess;
            markBest += newBest - best;
            best = newBest;
            last = newRemaining;
        }

        public void resume(long nowNanos) {
            markBest = best;
            markNanos = nowNanos;
        }

        public void stop() {
            active = false;
        }

        public boolean isActive() {
            return active;
        }

        public boolean diverged() {
            return diverged;
        }

        public double best() {
            return best;
        }

        public double excess() {
            return active ? Math.max(0.0, last - best) : 0.0;
        }

        public double secondsSinceProgress(long nowNanos) {
            return active ? (nowNanos - markNanos) / NANOS_PER_SECOND : 0.0;
        }

        public String update(double remaining, long nowNanos, PollenSeekConfig config) {
            if (!active) {
                return null;
            }
            if (!isFinite(remaining)) {
                return "!! remaining distance is not a number - pose invalid";
            }
            last = remaining;
            if (remaining < best) {
                best = remaining;
            }
            if (markBest - best >= config.stallProgressInches) {
                markBest = best;
                markNanos = nowNanos;
            }
            double excess = remaining - best;
            if (excess > config.divergeInches) {
                diverged = true;
                return String.format(Locale.US,
                        "!! moving away from the target (+%.1f in from the closest point) - localizer sign? push-test the Pinpoint",
                        excess);
            }
            double stalled = (nowNanos - markNanos) / NANOS_PER_SECOND;
            if (stalled > config.stallSeconds) {
                return String.format(Locale.US,
                        "!! no progress (< %.1f in) for %.1f s - robot stuck or localizer wrong",
                        config.stallProgressInches, stalled);
            }
            return null;
        }
    }

    public static final class CrossCheck {
        private boolean armed;
        private double anchorX;
        private double anchorY;
        private double anchorDistance;
        private double tolerance;
        private int requiredFrames;
        private int consecutive;
        private double lastError = Double.NaN;
        private double worstError = 0.0;
        private int checks;

        public void arm(double x, double y, double distanceInches, PollenSeekConfig config) {
            armed = isFinite(x) && isFinite(y);
            anchorX = x;
            anchorY = y;
            anchorDistance = isFinite(distanceInches) ? Math.max(0.0, distanceInches) : 0.0;
            tolerance = config.crossCheckInches + config.crossCheckPerInch * anchorDistance;
            requiredFrames = Math.max(1, config.crossCheckFrames);
            consecutive = 0;
            lastError = Double.NaN;
            worstError = 0.0;
            checks = 0;
        }

        public void disarm() {
            armed = false;
            consecutive = 0;
        }

        public boolean isArmed() {
            return armed;
        }

        public double anchorX() {
            return anchorX;
        }

        public double anchorY() {
            return anchorY;
        }

        public double tolerance() {
            return tolerance;
        }

        public double gate(PollenSeekConfig config) {
            return tolerance * config.crossCheckGateFactor;
        }

        public double lastError() {
            return lastError;
        }

        public double worstError() {
            return worstError;
        }

        public int consecutive() {
            return consecutive;
        }

        public int checks() {
            return checks;
        }

        public double predictionError(PlanarPose poseAtCapture, double measuredForward, double measuredLeft) {
            Vec3 predicted = TargetGeometry.fieldPointToRobot(poseAtCapture, new Vec3(anchorX, anchorY, 0.0));
            if (predicted == null) {
                return Double.NaN;
            }
            return Math.hypot(predicted.x - measuredForward, predicted.y - measuredLeft);
        }

        public boolean check(PlanarPose poseAtCapture, double measuredForward, double measuredLeft) {
            if (!armed || poseAtCapture == null) {
                return false;
            }
            double error = predictionError(poseAtCapture, measuredForward, measuredLeft);
            if (!isFinite(error)) {
                return false;
            }
            checks++;
            lastError = error;
            worstError = Math.max(worstError, error);
            consecutive = error > tolerance ? consecutive + 1 : 0;
            return consecutive >= requiredFrames;
        }

        public double estimateDrift(double estimateX, double estimateY) {
            if (!armed || !isFinite(estimateX) || !isFinite(estimateY)) {
                return 0.0;
            }
            return Math.hypot(estimateX - anchorX, estimateY - anchorY);
        }

        public boolean estimateDrifted(double estimateX, double estimateY) {
            return armed && estimateDrift(estimateX, estimateY) > tolerance;
        }

        public String tripMessage() {
            return String.format(Locale.US,
                    "!! odometry disagrees with camera (%.1f in > %.1f in for %d frames) - localizer sign? push-test the Pinpoint",
                    lastError, tolerance, consecutive);
        }
    }

    public static final class PoseSanity {
        private boolean has;
        private double lastX;
        private double lastY;

        public void reset() {
            has = false;
        }

        public String sample(double x, double y, double heading, PollenSeekConfig config) {
            if (!isFinite(x) || !isFinite(y) || !isFinite(heading)) {
                has = false;
                return "!! pose is not a number (NaN) - Pinpoint fault";
            }
            String fault = null;
            if (has) {
                double jump = Math.hypot(x - lastX, y - lastY);
                if (jump > config.poseJumpInches) {
                    fault = String.format(Locale.US, "!! pose jumped %.1f in in one loop - Pinpoint fault", jump);
                }
            }
            if (fault == null && outsideField(x, y)) {
                fault = String.format(Locale.US, "!! pose (%.1f, %.1f) left the field", x, y);
            }
            has = true;
            lastX = x;
            lastY = y;
            return fault;
        }

        public static boolean outsideField(double x, double y) {
            return x < 0.0 || y < 0.0 || x > PollenSeekConfig.FIELD_SIZE_INCHES || y > PollenSeekConfig.FIELD_SIZE_INCHES;
        }
    }

    public static double turnPower(double headingErrorRad, PollenSeekConfig config) {
        if (!isFinite(headingErrorRad) || headingErrorRad == 0.0) {
            return 0.0;
        }
        double magnitude = Math.abs(headingErrorRad) * config.turnKpPerRadian;
        magnitude = Math.max(config.minTurnPower, Math.min(config.maxTurnPower, magnitude));
        return Math.copySign(magnitude, headingErrorRad);
    }

    public static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
