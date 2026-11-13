package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import java.util.Locale;

public final class PushTestGate {

    public static final String X_POD_FIX = "!! PINPOINT X POD REVERSED - flip PINPOINT_X_POD_DIRECTION in Constants";
    public static final String Y_POD_FIX = "!! PINPOINT Y POD REVERSED - flip PINPOINT_Y_POD_DIRECTION in Constants";
    public static final String PODS_SWAPPED_FIX =
            "!! forward push reads SIDEWAYS - X/Y pod cables swapped (or robot pushed sideways) - fix and restart the OpMode";
    public static final String DIAGONAL_WARNING = "!! push reads DIAGONAL - push the robot straight forward";
    public static final String LEFT_REQUIRED =
            "forward OK - now push LEFT about 6 in (REQUIRED), then put the robot back and press BACK";

    public enum Verdict { WAITING, FORWARD_OK, LEFT_OK, PASSED, X_REVERSED, Y_REVERSED, PODS_SWAPPED }

    private static final double LEFT_START_INCHES = 1.0;

    private final double thresholdInches;
    private final double lateralRatio;

    private boolean begun;
    private double originX;
    private double originY;
    private double originHeading;
    private double forward;
    private double lateral;
    private boolean forwardPassed;
    private double forwardAtPass;
    private double lateralAtPass;
    private boolean leftPassed;
    private boolean xReversed;
    private boolean yReversed;
    private boolean podsSwapped;
    private boolean rezeroed;
    private boolean diagonal;
    private boolean poseValid;

    public PushTestGate(double thresholdInches, double lateralRatio) {
        this.thresholdInches = thresholdInches > 0.0 ? thresholdInches : 4.0;
        this.lateralRatio = lateralRatio > 0.0 ? lateralRatio : 0.5;
    }

    public void begin(double x, double y, double heading) {
        if (!finite(x) || !finite(y) || !finite(heading)) {
            begun = false;
            return;
        }
        begun = true;
        originX = x;
        originY = y;
        originHeading = heading;
        forward = 0.0;
        lateral = 0.0;
        forwardPassed = false;
        forwardAtPass = 0.0;
        lateralAtPass = 0.0;
        leftPassed = false;
        xReversed = false;
        yReversed = false;
        podsSwapped = false;
        rezeroed = false;
        diagonal = false;
        poseValid = true;
    }

    public void rezero(double x, double y, double heading) {
        if (failed()) {
            return;
        }
        if (forwardPassed && leftPassed) {
            rezeroed = true;
            return;
        }
        if (forwardPassed && begun && finite(x) && finite(y) && finite(heading)) {
            originX = x;
            originY = y;
            originHeading = heading;
            forward = 0.0;
            lateral = 0.0;
            forwardAtPass = 0.0;
            lateralAtPass = 0.0;
            diagonal = false;
            return;
        }
        begin(x, y, heading);
    }

    public void update(double x, double y, double heading) {
        if (!begun) {
            begin(x, y, heading);
            return;
        }
        if (rezeroed) {
            return;
        }
        if (!finite(x) || !finite(y)) {
            poseValid = false;
            return;
        }
        poseValid = true;
        double dx = x - originX;
        double dy = y - originY;
        double c = Math.cos(originHeading);
        double s = Math.sin(originHeading);
        forward = dx * c + dy * s;
        lateral = -dx * s + dy * c;
        if (failed()) {
            diagonal = false;
            return;
        }
        if (!forwardPassed) {
            double absForward = Math.abs(forward);
            double absLateral = Math.abs(lateral);
            if (forward <= -thresholdInches && absLateral <= lateralRatio * absForward) {
                xReversed = true;
                diagonal = false;
                return;
            }
            if (absLateral >= thresholdInches && absForward <= lateralRatio * absLateral) {
                podsSwapped = true;
                diagonal = false;
                return;
            }
            if (forward >= thresholdInches && absLateral <= lateralRatio * forward) {
                forwardPassed = true;
                forwardAtPass = forward;
                lateralAtPass = lateral;
                diagonal = false;
                return;
            }
            diagonal = Math.max(absForward, absLateral) >= thresholdInches;
            return;
        }
        if (leftPassed) {
            return;
        }
        double leftDelta = lateral - lateralAtPass;
        if (Math.abs(leftDelta) < LEFT_START_INCHES) {
            forwardAtPass = forward;
            diagonal = false;
            return;
        }
        double forwardDelta = forward - forwardAtPass;
        if (Math.abs(leftDelta) >= thresholdInches) {
            if (Math.abs(forwardDelta) <= lateralRatio * Math.abs(leftDelta)) {
                diagonal = false;
                if (leftDelta > 0.0) {
                    leftPassed = true;
                } else {
                    yReversed = true;
                }
                return;
            }
            diagonal = true;
            return;
        }
        diagonal = false;
    }

    public boolean driveEnabled() {
        return forwardPassed && leftPassed && rezeroed && !failed();
    }

    public boolean failed() {
        return xReversed || yReversed || podsSwapped;
    }

    public Verdict verdict() {
        if (xReversed) {
            return Verdict.X_REVERSED;
        }
        if (yReversed) {
            return Verdict.Y_REVERSED;
        }
        if (podsSwapped) {
            return Verdict.PODS_SWAPPED;
        }
        if (forwardPassed && leftPassed && rezeroed) {
            return Verdict.PASSED;
        }
        if (forwardPassed && leftPassed) {
            return Verdict.LEFT_OK;
        }
        if (forwardPassed) {
            return Verdict.FORWARD_OK;
        }
        return Verdict.WAITING;
    }

    public boolean forwardPassed() {
        return forwardPassed;
    }

    public boolean leftPassed() {
        return leftPassed;
    }

    public boolean podsSwapped() {
        return podsSwapped;
    }

    public boolean rezeroed() {
        return rezeroed;
    }

    public double forwardInches() {
        return forward;
    }

    public double lateralInches() {
        return lateral;
    }

    public double leftPushInches() {
        return forwardPassed ? lateral - lateralAtPass : 0.0;
    }

    public String liveLine() {
        if (!begun) {
            return "push test: no Pinpoint pose";
        }
        if (!poseValid) {
            return "push test: !! pose is NaN";
        }
        String verdict;
        if (xReversed || yReversed) {
            verdict = "WRONG SIGN";
        } else if (podsSwapped) {
            verdict = "WRONG (reads sideways)";
        } else if (rezeroed) {
            verdict = "OK (done)";
        } else if (leftPassed) {
            verdict = "forward + left OK";
        } else if (forwardPassed) {
            verdict = diagonal ? "WRONG (diagonal)" : String.format(Locale.US, "forward OK, left push %+.1f in (needs >= +%.1f)",
                    leftPushInches(), thresholdInches);
        } else if (diagonal) {
            verdict = "WRONG (diagonal)";
        } else if (forward < 0.0 && Math.abs(forward) >= 1.0) {
            verdict = "reading BACKWARD";
        } else {
            verdict = "waiting";
        }
        return String.format(Locale.US, "push test: forward %+.1f in, lateral %+.1f in -> %s", forward, lateral, verdict);
    }

    public String instruction() {
        if (xReversed) {
            return X_POD_FIX + "  (driving stays OFF; if you pushed BACKWARD by mistake, restart the OpMode)";
        }
        if (yReversed) {
            return Y_POD_FIX + "  (driving stays OFF; if you pushed RIGHT by mistake, restart the OpMode)";
        }
        if (podsSwapped) {
            return PODS_SWAPPED_FIX + "  (driving stays OFF)";
        }
        if (rezeroed) {
            return "push test PASSED (forward + left) - pose re-zeroed, driving allowed after START";
        }
        if (leftPassed) {
            return "forward + left OK - put the robot back on the start pose and press BACK to re-zero";
        }
        if (forwardPassed) {
            return LEFT_REQUIRED;
        }
        return String.format(Locale.US,
                "PUSH TEST: push the robot FORWARD by hand about 6 in (needs >= +%.1f in, sideways <= %.0f%%), then LEFT about 6 in",
                thresholdInches, lateralRatio * 100.0);
    }

    public String warning() {
        if (xReversed) {
            return X_POD_FIX;
        }
        if (yReversed) {
            return Y_POD_FIX;
        }
        if (podsSwapped) {
            return PODS_SWAPPED_FIX;
        }
        if (diagonal) {
            return forwardPassed ? "!! push reads DIAGONAL - push the robot straight LEFT" : DIAGONAL_WARNING;
        }
        return "";
    }

    public String shortStatus() {
        switch (verdict()) {
            case PASSED:
                return "PASSED";
            case LEFT_OK:
                return "forward + left OK - put the robot back and press BACK to re-zero";
            case FORWARD_OK:
                return LEFT_REQUIRED;
            case X_REVERSED:
                return "FAILED - X pod reads reversed";
            case Y_REVERSED:
                return "FAILED - Y pod reads reversed";
            case PODS_SWAPPED:
                return "FAILED - forward push reads sideways (X/Y pod cables swapped?)";
            case WAITING:
            default:
                return "not done - push the robot FORWARD, then LEFT, in INIT";
        }
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
