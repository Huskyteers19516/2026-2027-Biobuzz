package org.firstinspires.ftc.teamtestcode.Opmode;

/**
 * Times a burst of shots for MatchAuto, with no hardware in it.
 *
 * Call update() every loop with the current time and whether the outtake is at speed;
 * it answers "should the transfer feed right now?". The interlock is checked on every
 * call, so if the flywheel slows down in the middle of a shot the feed stops at once and
 * only resumes when the speed is back. It gives up if the flywheel stays slow for
 * recoverSeconds, and it always finishes by deadlineSeconds.
 */
public final class ShotSequencer {

    private enum Phase {
        WAITING_FOR_SPEED,
        FEEDING,
        GAP
    }

    private final int shots;
    private final double feedSeconds;
    private final double gapSeconds;
    private final double recoverSeconds;
    private final double deadlineSeconds;

    private Phase phase = Phase.WAITING_FOR_SPEED;
    private boolean started = false;
    private double lastTime;
    private double phaseStart;
    private double fedSeconds;
    private int fired;
    private String result;

    public ShotSequencer(int shots, double feedSeconds, double gapSeconds,
                         double recoverSeconds, double deadlineSeconds) {
        this.shots = shots;
        this.feedSeconds = feedSeconds;
        this.gapSeconds = gapSeconds;
        this.recoverSeconds = recoverSeconds;
        this.deadlineSeconds = deadlineSeconds;
        if (shots <= 0) {
            result = "no shots asked";
        }
    }

    /** @return true if the transfer should feed during this loop */
    public boolean update(double nowSeconds, boolean outtakeReady) {
        if (!started) {
            started = true;
            lastTime = nowSeconds;
            phaseStart = nowSeconds;
        }
        double dt = Math.max(0.0, nowSeconds - lastTime);
        lastTime = nowSeconds;

        if (done()) {
            return false;
        }
        if (nowSeconds >= deadlineSeconds) {
            result = "TIME LIMIT";
            return false;
        }

        switch (phase) {
            case FEEDING:
                if (!outtakeReady) {
                    enter(Phase.WAITING_FOR_SPEED, nowSeconds);
                    return false;
                }
                fedSeconds += dt;
                if (fedSeconds < feedSeconds) {
                    return true;
                }
                fired++;
                fedSeconds = 0.0;
                if (fired >= shots) {
                    result = "ok";
                } else {
                    enter(Phase.GAP, nowSeconds);
                }
                return false;

            case GAP:
                if (nowSeconds - phaseStart >= gapSeconds) {
                    enter(Phase.WAITING_FOR_SPEED, nowSeconds);
                }
                return false;

            case WAITING_FOR_SPEED:
            default:
                // Checked first: once the wait is over we give up, even if the speed
                // happens to come back on this very loop.
                if (nowSeconds - phaseStart >= recoverSeconds) {
                    result = "OUTTAKE NOT AT SPEED";
                    return false;
                }
                if (outtakeReady) {
                    enter(Phase.FEEDING, nowSeconds);
                    return true;
                }
                return false;
        }
    }

    private void enter(Phase next, double nowSeconds) {
        phase = next;
        phaseStart = nowSeconds;
    }

    public boolean done() {
        return result != null;
    }

    public int fired() {
        return fired;
    }

    public String summary() {
        return "shot " + fired + " / " + shots + (result == null || result.equals("ok") ? "" : " (" + result + ")");
    }
}
