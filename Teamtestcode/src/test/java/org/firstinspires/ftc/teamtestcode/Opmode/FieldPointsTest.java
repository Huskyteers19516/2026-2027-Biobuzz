package org.firstinspires.ftc.teamtestcode.Opmode;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.List;

public class FieldPointsTest {

    @Test
    public void missionBuildsAndEveryStepIsBounded() {
        List<FieldPoints.Step> mission = FieldPoints.mission();
        assertFalse(mission.isEmpty());

        for (FieldPoints.Step step : mission) {
            assertNotNull(step.name, step.path);
            assertNotNull(step.name, step.target);
            assertTrue(step.name, step.seconds >= 0.0 && step.seconds <= 30.0);
            assertTrue(step.name, step.shots >= 0 && step.shots <= 10);
            if (step.action == FieldPoints.Action.SHOOT) {
                assertTrue(step.name, step.shots > 0);
            }
        }
    }

    @Test
    public void missionStartsAtTheStartPoseAndEveryPathEndsAtItsTarget() {
        List<FieldPoints.Step> mission = FieldPoints.mission();
        assertTrue(mission.get(0).path.get(0.0).distance(FieldPoints.START_POSE) < 1e-6);

        for (FieldPoints.Step step : mission) {
            assertTrue(step.name, step.path.endPose().distance(step.target) < 1e-6);
        }
    }
}
