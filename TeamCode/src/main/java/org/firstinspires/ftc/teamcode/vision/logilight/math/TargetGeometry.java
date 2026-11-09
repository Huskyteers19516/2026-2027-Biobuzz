package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.List;

public final class TargetGeometry {
    private static final Vec3 UP = Vec3.UNIT_Z;
    private static final double CIRCULAR_MEAN_EPSILON = 1e-12;

    private TargetGeometry() {
    }

    public static Vec3 opticalFromFtcPose(double ftcX, double ftcY, double ftcZ) {
        return new Vec3(ftcX, -ftcZ, ftcY);
    }

    public static PlanarPose solveKnownHeading(Vec3 targetInRobot, Vec3 originField, double headingRad) {
        if (targetInRobot == null || originField == null || !Vec3.isFiniteValue(headingRad)) {
            return null;
        }
        double c = Math.cos(headingRad);
        double s = Math.sin(headingRad);
        double rotatedX = c * targetInRobot.x - s * targetInRobot.y;
        double rotatedY = s * targetInRobot.x + c * targetInRobot.y;
        return new PlanarPose(originField.x - rotatedX, originField.y - rotatedY, headingRad);
    }

    public static double headingFromRotations(Mat3 targetRotationInRobot, Mat3 stateRotationField) {
        if (targetRotationInRobot == null || stateRotationField == null) {
            return Double.NaN;
        }
        Mat3 robotToField = stateRotationField.times(targetRotationInRobot.transpose());
        return Math.atan2(robotToField.m10, robotToField.m00);
    }

    public static PlanarPose solveFull(Vec3 targetInRobot, Mat3 targetRotationInRobot, FieldTargetState state) {
        if (targetInRobot == null || targetRotationInRobot == null || state == null) {
            return null;
        }
        double heading = headingFromRotations(targetRotationInRobot, state.rotationField);
        if (!Vec3.isFiniteValue(heading)) {
            return null;
        }
        return solveKnownHeading(targetInRobot, state.originField, heading);
    }

    public static double heightErrorInches(Vec3 targetInRobot, Vec3 originField) {
        if (targetInRobot == null || originField == null) {
            return Double.NaN;
        }
        return targetInRobot.z - originField.z;
    }

    public static double tiltMismatchRadians(Mat3 targetRotationInRobot, Mat3 stateRotationField) {
        if (targetRotationInRobot == null || stateRotationField == null) {
            return Double.NaN;
        }
        Vec3 upObserved = targetRotationInRobot.transposeTimes(UP).normalized();
        Vec3 upExpected = stateRotationField.transposeTimes(UP).normalized();
        if (upObserved.norm() == 0.0 || upExpected.norm() == 0.0) {
            return Double.NaN;
        }
        return upObserved.angleTo(upExpected);
    }

    public static double rotationMismatchRadians(Mat3 targetRotationInRobot, double headingRad, Mat3 stateRotationField) {
        if (targetRotationInRobot == null || stateRotationField == null || !Vec3.isFiniteValue(headingRad)) {
            return Double.NaN;
        }
        return Mat3.rotZ(headingRad).times(targetRotationInRobot).angleTo(stateRotationField);
    }

    public static FieldTargetState survey(String label, PlanarPose robotPose, Vec3 targetInRobot, Mat3 targetRotationInRobot) {
        if (robotPose == null || targetInRobot == null || targetRotationInRobot == null || !robotPose.isFinite()) {
            return null;
        }
        Mat3 headingRotation = Mat3.rotZ(robotPose.heading);
        Vec3 origin = new Vec3(robotPose.x, robotPose.y, 0.0).plus(headingRotation.times(targetInRobot));
        return new FieldTargetState(label, origin, headingRotation.times(targetRotationInRobot));
    }

    public static Vec3 robotPointToField(PlanarPose robotPose, Vec3 pRobot) {
        if (robotPose == null || pRobot == null) {
            return null;
        }
        return new Vec3(robotPose.x, robotPose.y, 0.0).plus(Mat3.rotZ(robotPose.heading).times(pRobot));
    }

    public static Vec3 fieldPointToRobot(PlanarPose robotPose, Vec3 pField) {
        if (robotPose == null || pField == null) {
            return null;
        }
        return Mat3.rotZ(robotPose.heading).transposeTimes(pField.minus(new Vec3(robotPose.x, robotPose.y, 0.0)));
    }

    public static double[] stddevXYH(double distanceInches, double baseInches, double perInchSquared,
                                     double headingBaseDegrees, double headingPerInchDegrees) {
        double distance = Vec3.isFiniteValue(distanceInches) ? Math.abs(distanceInches) : 0.0;
        double sigmaXY = baseInches + perInchSquared * distance * distance;
        double sigmaHeading = Math.toRadians(headingBaseDegrees + headingPerInchDegrees * distance);
        return new double[]{sigmaXY, sigmaXY, sigmaHeading};
    }

    public static PlanarPose weightedAverage(List<PlanarPose> poses, List<Double> weights) {
        if (poses == null || poses.isEmpty()) {
            return null;
        }
        double totalWeight = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumSin = 0.0;
        double sumCos = 0.0;
        double headingWeight = 0.0;
        double heaviestHeading = Double.NaN;
        double heaviestHeadingWeight = -1.0;
        for (int i = 0; i < poses.size(); i++) {
            PlanarPose pose = poses.get(i);
            if (pose == null || !pose.isPositionFinite()) {
                continue;
            }
            double weight = 1.0;
            if (weights != null && i < weights.size()) {
                Double value = weights.get(i);
                weight = value == null ? 1.0 : value.doubleValue();
            }
            if (!Vec3.isFiniteValue(weight) || weight <= 0.0) {
                continue;
            }
            totalWeight += weight;
            sumX += weight * pose.x;
            sumY += weight * pose.y;
            if (pose.hasHeading()) {
                sumSin += weight * Math.sin(pose.heading);
                sumCos += weight * Math.cos(pose.heading);
                headingWeight += weight;
                if (weight > heaviestHeadingWeight) {
                    heaviestHeadingWeight = weight;
                    heaviestHeading = pose.heading;
                }
            }
        }
        if (totalWeight <= 0.0) {
            return null;
        }
        double heading = Double.NaN;
        if (headingWeight > 0.0) {
            heading = Math.hypot(sumSin, sumCos) > CIRCULAR_MEAN_EPSILON * headingWeight
                    ? Math.atan2(sumSin, sumCos)
                    : heaviestHeading;
        }
        return new PlanarPose(sumX / totalWeight, sumY / totalWeight, heading);
    }

    public static double horizontalBearingLeftDegrees(Vec3 pRobot) {
        if (pRobot == null) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan2(pRobot.y, pRobot.x));
    }

    public static double horizontalDistanceInches(Vec3 pRobot) {
        if (pRobot == null) {
            return Double.NaN;
        }
        return Math.hypot(pRobot.x, pRobot.y);
    }

    public static double polygonArea(double[] uvPairs) {
        if (uvPairs == null || uvPairs.length < 6) {
            return 0.0;
        }
        int count = uvPairs.length / 2;
        double twiceArea = 0.0;
        for (int i = 0; i < count; i++) {
            int next = (i + 1) % count;
            twiceArea += uvPairs[2 * i] * uvPairs[2 * next + 1] - uvPairs[2 * next] * uvPairs[2 * i + 1];
        }
        double area = Math.abs(twiceArea) / 2.0;
        return Vec3.isFiniteValue(area) ? area : 0.0;
    }
}
