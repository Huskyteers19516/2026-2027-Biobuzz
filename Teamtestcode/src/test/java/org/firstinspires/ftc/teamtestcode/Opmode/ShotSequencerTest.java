package org.firstinspires.ftc.teamtestcode.Opmode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotSequencerTest {

    private static final double DT = 0.02;
    private static final double FEED = 0.6;
    private static final double GAP = 0.4;
    private static final double RECOVER = 3.0;
    private static final double DEADLINE = 8.0;

    private interface Speed {
        boolean readyAt(double seconds);
    }

    /** Runs the sequencer like the op mode loop does and returns how long the transfer fed. */
    private static double run(ShotSequencer shots, Speed speed, double maxSeconds) {
        double fedSeconds = 0.0;
        for (double t = 0.0; t <= maxSeconds && !shots.done(); t += DT) {
            boolean ready = speed.readyAt(t);
            boolean feed = shots.update(t, ready);
            assertFalse("fed while the outtake was not at speed, t=" + t, feed && !ready);
            if (feed) {
                fedSeconds += DT;
            }
        }
        return fedSeconds;
    }

    @Test
    public void firesEveryShotWhenTheOuttakeStaysAtSpeed() {
        ShotSequencer shots = new ShotSequencer(3, FEED, GAP, RECOVER, DEADLINE);
        double fed = run(shots, t -> true, 30.0);

        assertTrue(shots.done());
        assertEquals(3, shots.fired());
        assertEquals("shot 3 / 3", shots.summary());
        assertEquals(3 * FEED, fed, 0.1);
    }

    @Test
    public void neverFeedsWhenTheOuttakeNeverReachesSpeed() {
        ShotSequencer shots = new ShotSequencer(3, FEED, GAP, RECOVER, DEADLINE);
        double fed = run(shots, t -> false, 30.0);

        assertTrue(shots.done());
        assertEquals(0, shots.fired());
        assertEquals(0.0, fed, 1e-9);
        assertEquals("shot 0 / 3 (OUTTAKE NOT AT SPEED)", shots.summary());
    }

    // Codex review: speed read 1500 once and 0 forever after must not fire 2 / 2.
    @Test
    public void stopsFeedingAsSoonAsTheOuttakeLosesSpeed() {
        ShotSequencer shots = new ShotSequencer(2, FEED, GAP, RECOVER, DEADLINE);
        double fed = run(shots, t -> t < DT / 2, 30.0);

        assertTrue(shots.done());
        assertEquals(0, shots.fired());
        assertTrue(fed <= DT + 1e-9);
    }

    @Test
    public void resumesTheSameShotAfterTheOuttakeRecovers() {
        ShotSequencer shots = new ShotSequencer(1, FEED, GAP, RECOVER, DEADLINE);
        // At speed, then slow for one second in the middle of the shot, then back.
        double fed = run(shots, t -> t < 0.3 || t > 1.3, 30.0);

        assertEquals(1, shots.fired());
        assertEquals("shot 1 / 1", shots.summary());
        assertEquals(FEED, fed, 0.1);
    }

    @Test
    public void wholeBurstEndsAtTheDeadlineEvenWithTooManyShots() {
        ShotSequencer shots = new ShotSequencer(10, FEED, GAP, RECOVER, DEADLINE);
        double finishedAt = 0.0;
        for (double t = 0.0; t <= 30.0 && !shots.done(); t += DT) {
            shots.update(t, true);
            finishedAt = t;
        }

        assertTrue(shots.done());
        assertTrue(finishedAt <= DEADLINE + DT);
        assertTrue(shots.fired() < 10);
        assertFalse(shots.update(finishedAt + DT, true));
    }

    @Test
    public void zeroShotsOrZeroTimeNeverFeeds() {
        ShotSequencer none = new ShotSequencer(0, FEED, GAP, RECOVER, DEADLINE);
        assertTrue(none.done());
        assertFalse(none.update(0.0, true));

        // The autonomous budget is already spent, so the deadline is 0 seconds.
        ShotSequencer late = new ShotSequencer(3, FEED, GAP, RECOVER, 0.0);
        assertFalse(late.update(0.0, true));
        assertTrue(late.done());
        assertEquals(0, late.fired());
    }

    // Codex review: slow at t=0, back at speed at t=3.1 is past the 3 s wait, so no feed.
    @Test
    public void givesUpOnceTheRecoveryWaitIsOverEvenIfSpeedComesBack() {
        ShotSequencer shots = new ShotSequencer(1, FEED, GAP, RECOVER, DEADLINE);
        assertFalse(shots.update(0.0, false));
        assertFalse(shots.update(RECOVER + 0.1, true));
        assertTrue(shots.done());
        assertEquals(0, shots.fired());
    }
}
