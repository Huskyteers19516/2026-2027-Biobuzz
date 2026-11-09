package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class FieldTargetState {
    public final String label;
    public final Vec3 originField;
    public final Mat3 rotationField;
    public final SurveyInfo survey;

    public FieldTargetState(String label, Vec3 originField, Mat3 rotationField) {
        this(label, originField, rotationField, null);
    }

    public FieldTargetState(String label, Vec3 originField, Mat3 rotationField, SurveyInfo survey) {
        this.label = label == null ? "" : label;
        this.originField = originField == null ? Vec3.ZERO : originField;
        this.rotationField = rotationField == null ? Mat3.IDENTITY : rotationField;
        this.survey = survey;
    }

    public static FieldTargetState fromDegrees(String label, double x, double y, double z,
                                               double yawDeg, double pitchDeg, double rollDeg) {
        return new FieldTargetState(label, new Vec3(x, y, z),
                Mat3.fromYawPitchRoll(Math.toRadians(yawDeg), Math.toRadians(pitchDeg), Math.toRadians(rollDeg)));
    }

    public double yawDegrees() {
        return Math.toDegrees(rotationField.yaw());
    }

    public double pitchDegrees() {
        return Math.toDegrees(rotationField.pitch());
    }

    public double rollDegrees() {
        return Math.toDegrees(rotationField.roll());
    }

    public FieldTargetState withLabel(String newLabel) {
        return new FieldTargetState(newLabel, originField, rotationField, survey);
    }

    public FieldTargetState withSurvey(SurveyInfo info) {
        return new FieldTargetState(label, originField, rotationField, info);
    }

    public boolean hasSurvey() {
        return survey != null && survey.isFinite();
    }

    public Vec3 fieldPointOf(Vec3 pointInTarget) {
        return originField.plus(rotationField.times(pointInTarget == null ? Vec3.ZERO : pointInTarget));
    }

    public Vec3 normalInField() {
        return rotationField.column(2);
    }

    public boolean isFinite() {
        return originField.isFinite() && rotationField.isFinite();
    }

    public static FieldTargetState average(String label, List<FieldTargetState> samples) {
        if (samples == null || samples.isEmpty()) {
            return null;
        }
        List<Vec3> origins = new ArrayList<Vec3>();
        List<Mat3> rotations = new ArrayList<Mat3>();
        for (FieldTargetState sample : samples) {
            if (sample == null || !sample.isFinite()) {
                continue;
            }
            origins.add(sample.originField);
            rotations.add(sample.rotationField);
        }
        if (origins.isEmpty()) {
            return null;
        }
        return new FieldTargetState(label, Vec3.average(origins), Mat3.average(rotations));
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "%s  (%.2f, %.2f, %.2f)  yaw %.1f  pitch %.1f  roll %.1f deg",
                label, originField.x, originField.y, originField.z, yawDegrees(), pitchDegrees(), rollDegrees());
    }
}
