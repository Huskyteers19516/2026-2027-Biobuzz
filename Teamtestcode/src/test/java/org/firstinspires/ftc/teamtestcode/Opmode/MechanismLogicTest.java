package org.firstinspires.ftc.teamtestcode.Opmode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MechanismLogicTest {

    private static final double EPS = 1e-9;

    @Test
    public void idleCommandsNothing() {
        MechanismLogic.Command c = MechanismLogic.decide(false, false, false, true);
        assertEquals(0.0, c.intakePower, EPS);
        assertEquals(0.0, c.transferPower, EPS);
        assertFalse(c.servoFire);
    }

    @Test
    public void fireIsBlockedUntilOuttakeIsAtSpeed() {
        MechanismLogic.Command c = MechanismLogic.decide(false, false, true, false);
        assertEquals(0.0, c.transferPower, EPS);
        assertFalse(c.servoFire);
    }

    @Test
    public void fireFeedsWhenOuttakeIsAtSpeed() {
        MechanismLogic.Command c = MechanismLogic.decide(false, false, true, true);
        assertEquals(RobotTeleOp.TRANSFER_FEED_POWER, c.transferPower, EPS);
        assertTrue(c.servoFire);
    }

    @Test
    public void ejectBeatsEverythingElse() {
        MechanismLogic.Command c = MechanismLogic.decide(true, true, true, true);
        assertEquals(RobotTeleOp.EJECT_POWER, c.intakePower, EPS);
        assertEquals(RobotTeleOp.EJECT_POWER, c.transferPower, EPS);
        assertFalse(c.servoFire);
    }

    @Test
    public void intakeAloneNeverFiresServo() {
        MechanismLogic.Command c = MechanismLogic.decide(true, false, false, true);
        assertEquals(RobotTeleOp.INTAKE_POWER, c.intakePower, EPS);
        assertEquals(RobotTeleOp.TRANSFER_POWER_WHILE_INTAKING, c.transferPower, EPS);
        assertFalse(c.servoFire);
    }

    @Test
    public void everyCommandStaysInsideMotorLimits() {
        for (int bits = 0; bits < 16; bits++) {
            boolean outtakeReady = (bits & 8) != 0;
            MechanismLogic.Command c = MechanismLogic.decide(
                    (bits & 1) != 0, (bits & 2) != 0, (bits & 4) != 0, outtakeReady);
            assertTrue(Math.abs(c.intakePower) <= 1.0);
            assertTrue(Math.abs(c.transferPower) <= 1.0);
            // The servo may only fire while the transfer is feeding forward.
            if (c.servoFire) {
                assertTrue(c.transferPower > 0.0);
            }
            // No combination of buttons feeds a ball into a flywheel that is not at speed.
            if (!outtakeReady) {
                assertTrue(c.transferPower <= RobotTeleOp.TRANSFER_POWER_WHILE_INTAKING);
                assertFalse(c.servoFire);
            }
        }
    }

    @Test
    public void outtakeReadyNeedsMotorOnAndSpeedInTolerance() {
        assertTrue(MechanismLogic.outtakeReady(true, 1450.0, 1500.0, 100.0));
        assertFalse(MechanismLogic.outtakeReady(true, 1300.0, 1500.0, 100.0));
        assertFalse(MechanismLogic.outtakeReady(false, 1500.0, 1500.0, 100.0));
        assertFalse(MechanismLogic.outtakeReady(true, 0.0, 0.0, 100.0));
        assertFalse(MechanismLogic.outtakeReady(true, Double.NaN, 1500.0, 100.0));
    }

    @Test
    public void deadzoneAndClip() {
        assertEquals(0.0, MechanismLogic.deadzone(0.04, 0.05), EPS);
        assertEquals(-0.5, MechanismLogic.deadzone(-0.5, 0.05), EPS);
        assertEquals(1.0, MechanismLogic.deadzone(1.7, 0.05), EPS);
        assertEquals(500.0, MechanismLogic.clip(Double.NaN, 500.0, 2600.0), EPS);
        assertEquals(2600.0, MechanismLogic.clip(9999.0, 500.0, 2600.0), EPS);
    }

    @Test
    public void slowTriggerBlendsDriveScale() {
        assertEquals(1.0, MechanismLogic.driveScale(0.0, 1.0, 0.35), EPS);
        assertEquals(0.35, MechanismLogic.driveScale(1.0, 1.0, 0.35), EPS);
        assertEquals(0.35, MechanismLogic.driveScale(5.0, 1.0, 0.35), EPS);
    }

    @Test
    public void timedActionsNeverRunPastTheBudget() {
        assertEquals(1.5, MechanismLogic.boundedSeconds(1.5, 8.0, 10.0, 29.0), EPS);
        assertEquals(8.0, MechanismLogic.boundedSeconds(60.0, 8.0, 10.0, 29.0), EPS);
        assertEquals(1.0, MechanismLogic.boundedSeconds(5.0, 8.0, 28.0, 29.0), EPS);
        assertEquals(0.0, MechanismLogic.boundedSeconds(5.0, 8.0, 31.0, 29.0), EPS);
        assertEquals(0.0, MechanismLogic.boundedSeconds(-3.0, 8.0, 0.0, 29.0), EPS);
    }

    @Test
    public void timedOutPathOnlyCountsWhenRobotIsAtTheTarget() {
        double tolIn = 2.0;
        double tolRad = Math.toRadians(5.0);
        assertTrue(MechanismLogic.arrived(1.0, Math.toRadians(2.0), tolIn, tolRad));
        assertFalse(MechanismLogic.arrived(10.0, 0.0, tolIn, tolRad));
        assertFalse(MechanismLogic.arrived(0.5, Math.toRadians(20.0), tolIn, tolRad));
        // 359 degrees of error is really 1 degree.
        assertTrue(MechanismLogic.arrived(0.5, Math.toRadians(359.0), tolIn, tolRad));
        assertFalse(MechanismLogic.arrived(Double.NaN, 0.0, tolIn, tolRad));
    }

    @Test
    public void stoppingKeepsGoingWhenOneDeviceThrows() {
        final int[] stopped = {0};
        Runnable ok = () -> stopped[0]++;
        Runnable broken = () -> {
            throw new IllegalStateException("device unplugged");
        };

        RuntimeException failure = MechanismLogic.runAll(broken, ok, broken, ok, ok);

        assertEquals(3, stopped[0]);
        assertEquals("device unplugged", failure.getMessage());
        assertEquals(null, MechanismLogic.runAll(ok));
    }
}
