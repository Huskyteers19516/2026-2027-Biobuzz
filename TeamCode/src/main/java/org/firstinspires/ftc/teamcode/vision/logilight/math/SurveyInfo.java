package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.Locale;

public final class SurveyInfo {
    public final PlanarPose robotPose;
    public final double forwardInches;
    public final double leftInches;
    public final double upInches;
    public final double yawLeftDegrees;
    public final double tiltUpDegrees;
    public final double rollClockwiseDegrees;

    public SurveyInfo(PlanarPose robotPose, double forwardInches, double leftInches, double upInches,
                      double yawLeftDegrees, double tiltUpDegrees, double rollClockwiseDegrees) {
        this.robotPose = robotPose;
        this.forwardInches = forwardInches;
        this.leftInches = leftInches;
        this.upInches = upInches;
        this.yawLeftDegrees = yawLeftDegrees;
        this.tiltUpDegrees = tiltUpDegrees;
        this.rollClockwiseDegrees = rollClockwiseDegrees;
    }

    public static SurveyInfo of(PlanarPose robotPose, CameraModel mount) {
        if (robotPose == null || mount == null) {
            return null;
        }
        return new SurveyInfo(robotPose, mount.forwardInches, mount.leftInches, mount.upInches,
                mount.yawLeftDegrees, mount.tiltUpDegrees, mount.rollClockwiseDegrees);
    }

    public boolean isFinite() {
        return robotPose != null && robotPose.isFinite()
                && finite(forwardInches) && finite(leftInches) && finite(upInches)
                && finite(yawLeftDegrees) && finite(tiltUpDegrees) && finite(rollClockwiseDegrees);
    }

    public CameraModel mountOn(CameraModel model) {
        if (model == null) {
            return null;
        }
        return model.withMount(forwardInches, leftInches, upInches, yawLeftDegrees, tiltUpDegrees, rollClockwiseDegrees);
    }

    public Vec3 opticalPointOf(Vec3 originField, CameraModel model) {
        CameraModel surveyMount = mountOn(model);
        if (surveyMount == null || originField == null || robotPose == null) {
            return null;
        }
        return surveyMount.robotPointToOptical(TargetGeometry.fieldPointToRobot(robotPose, originField));
    }

    public boolean sameMountAs(CameraModel model, double toleranceInches, double toleranceDegrees) {
        if (model == null) {
            return false;
        }
        return Math.abs(model.forwardInches - forwardInches) <= toleranceInches
                && Math.abs(model.leftInches - leftInches) <= toleranceInches
                && Math.abs(model.upInches - upInches) <= toleranceInches
                && Math.abs(model.yawLeftDegrees - yawLeftDegrees) <= toleranceDegrees
                && Math.abs(model.tiltUpDegrees - tiltUpDegrees) <= toleranceDegrees
                && Math.abs(model.rollClockwiseDegrees - rollClockwiseDegrees) <= toleranceDegrees;
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "from (%.1f, %.1f, %.1f deg)  mount fwd %.2f left %.2f up %.2f yaw %.2f tilt %.2f roll %.2f",
                robotPose == null ? Double.NaN : robotPose.x, robotPose == null ? Double.NaN : robotPose.y,
                robotPose == null ? Double.NaN : robotPose.headingDegrees(),
                forwardInches, leftInches, upInches, yawLeftDegrees, tiltUpDegrees, rollClockwiseDegrees);
    }
}
