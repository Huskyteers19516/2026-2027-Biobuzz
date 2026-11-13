package org.firstinspires.ftc.teamcode.vision.logilight;

public class PollenSeekConfig {

    public static final double MAX_POWER_LIMIT = 0.6;
    public static final double MAX_CREEP_POWER_LIMIT = 0.3;
    public static final double MAX_TURN_POWER_LIMIT = 0.5;
    public static final double MAX_VELOCITY_LIMIT_INCHES_PER_SECOND = 60.0;
    public static final double MAX_SEEK_TIMEOUT_SECONDS = 10.0;
    public static final double MAX_INTAKE_SECONDS = 3.0;
    public static final double MAX_DIVERGE_INCHES = 12.0;
    public static final double MAX_STALL_SECONDS = 3.0;
    public static final double MAX_CROSS_CHECK_INCHES = 16.0;
    public static final double MAX_CROSS_CHECK_PER_INCH = 0.3;
    public static final double MAX_POSE_JUMP_INCHES = 24.0;
    public static final double MAX_TURN_DRIFT_INCHES = 8.0;
    public static final double MAX_RANGE_LIMIT_INCHES = 96.0;
    public static final double FIELD_SIZE_INCHES = 144.0;

    public double minRangeInches = 6.0;
    public double maxRangeInches = 60.0;
    public double maxFrameAgeMs = 300.0;
    public double poseLookupSlackMs = 60.0;
    public double minCircularity = 0.5;
    public double borderMarginPixels = 2.0;
    public double minSizeRatio = 0.5;
    public double maxSizeRatio = 2.0;
    public double ballDiameterInches = 2.8;

    public int confirmFrames = 3;
    public double confirmWindowSeconds = 0.5;
    public double clusterRadiusInches = 4.0;
    public int averageFrames = 5;
    public double associateRadiusInches = 6.0;

    public double stopShortInches = 10.0;
    public double minPathInches = 1.0;
    public double turnFirstDegrees = 25.0;
    public double turnToleranceDegrees = 6.0;
    public double turnTimeoutSeconds = 1.0;
    public double turnKpPerRadian = 0.8;
    public double minTurnPower = 0.10;
    public double maxTurnPower = 0.35;
    public double replanInches = 4.0;
    public double replanMinSeconds = 0.4;
    public double finalApproachInches = 16.0;
    public double maxPower = 0.45;
    public double maxVelocityInchesPerSecond = 20.0;
    public double arriveToleranceInches = 2.5;

    public boolean runIntakeOnArrive = false;
    public double intakeSeconds = 1.5;
    public double intakeCreepInches = 6.0;
    public double creepPower = 0.2;
    public boolean useConstantsIntakePower = true;
    public double intakePower = 1.0;
    public double intakeBackwardAbortInches = 2.0;

    public double divergeInches = 6.0;
    public double stallSeconds = 1.5;
    public double stallProgressInches = 1.0;
    public double seekTimeoutSeconds = 7.0;
    public double lostSeconds = 1.0;
    public double crossCheckInches = 8.0;
    public double crossCheckPerInch = 0.15;
    public int crossCheckFrames = 3;
    public double crossCheckGateFactor = 2.5;
    public double poseJumpInches = 12.0;
    public double extraTravelInches = 24.0;
    public double turnDriftAbortInches = 4.0;

    public double wallMarginInches = 3.0;
    public double robotRadiusInches = 9.0;
    public double obstacleMarginInches = 2.0;
    public double cornerPushInches = 2.0;
    public boolean avoidHive = true;
    public boolean avoidFlowers = true;
    public String extraKeepOuts = "";

    public double pushTestForwardInches = 4.0;
    public double pushTestLateralRatio = 0.5;

    public PollenSeekConfig() {
    }

    public PollenSeekConfig copy() {
        PollenSeekConfig c = new PollenSeekConfig();
        c.minRangeInches = minRangeInches;
        c.maxRangeInches = maxRangeInches;
        c.maxFrameAgeMs = maxFrameAgeMs;
        c.poseLookupSlackMs = poseLookupSlackMs;
        c.minCircularity = minCircularity;
        c.borderMarginPixels = borderMarginPixels;
        c.minSizeRatio = minSizeRatio;
        c.maxSizeRatio = maxSizeRatio;
        c.ballDiameterInches = ballDiameterInches;
        c.confirmFrames = confirmFrames;
        c.confirmWindowSeconds = confirmWindowSeconds;
        c.clusterRadiusInches = clusterRadiusInches;
        c.averageFrames = averageFrames;
        c.associateRadiusInches = associateRadiusInches;
        c.stopShortInches = stopShortInches;
        c.minPathInches = minPathInches;
        c.turnFirstDegrees = turnFirstDegrees;
        c.turnToleranceDegrees = turnToleranceDegrees;
        c.turnTimeoutSeconds = turnTimeoutSeconds;
        c.turnKpPerRadian = turnKpPerRadian;
        c.minTurnPower = minTurnPower;
        c.maxTurnPower = maxTurnPower;
        c.replanInches = replanInches;
        c.replanMinSeconds = replanMinSeconds;
        c.finalApproachInches = finalApproachInches;
        c.maxPower = maxPower;
        c.maxVelocityInchesPerSecond = maxVelocityInchesPerSecond;
        c.arriveToleranceInches = arriveToleranceInches;
        c.runIntakeOnArrive = runIntakeOnArrive;
        c.intakeSeconds = intakeSeconds;
        c.intakeCreepInches = intakeCreepInches;
        c.creepPower = creepPower;
        c.useConstantsIntakePower = useConstantsIntakePower;
        c.intakePower = intakePower;
        c.intakeBackwardAbortInches = intakeBackwardAbortInches;
        c.divergeInches = divergeInches;
        c.stallSeconds = stallSeconds;
        c.stallProgressInches = stallProgressInches;
        c.seekTimeoutSeconds = seekTimeoutSeconds;
        c.lostSeconds = lostSeconds;
        c.crossCheckInches = crossCheckInches;
        c.crossCheckPerInch = crossCheckPerInch;
        c.crossCheckFrames = crossCheckFrames;
        c.crossCheckGateFactor = crossCheckGateFactor;
        c.poseJumpInches = poseJumpInches;
        c.extraTravelInches = extraTravelInches;
        c.turnDriftAbortInches = turnDriftAbortInches;
        c.wallMarginInches = wallMarginInches;
        c.robotRadiusInches = robotRadiusInches;
        c.obstacleMarginInches = obstacleMarginInches;
        c.cornerPushInches = cornerPushInches;
        c.avoidHive = avoidHive;
        c.avoidFlowers = avoidFlowers;
        c.extraKeepOuts = extraKeepOuts;
        c.pushTestForwardInches = pushTestForwardInches;
        c.pushTestLateralRatio = pushTestLateralRatio;
        return c;
    }

    public PollenSeekConfig sanitized() {
        PollenSeekConfig d = new PollenSeekConfig();
        PollenSeekConfig c = copy();
        c.minRangeInches = clamp(minRangeInches, 0.0, 24.0, d.minRangeInches);
        c.maxRangeInches = clamp(maxRangeInches, c.minRangeInches + 1.0, MAX_RANGE_LIMIT_INCHES, d.maxRangeInches);
        c.maxFrameAgeMs = clamp(maxFrameAgeMs, 50.0, 500.0, d.maxFrameAgeMs);
        c.poseLookupSlackMs = clamp(poseLookupSlackMs, 0.0, 150.0, d.poseLookupSlackMs);
        c.minCircularity = clamp(minCircularity, 0.0, 1.0, d.minCircularity);
        c.borderMarginPixels = clamp(borderMarginPixels, 0.0, 50.0, d.borderMarginPixels);
        c.minSizeRatio = clamp(minSizeRatio, 0.05, 1.0, d.minSizeRatio);
        c.maxSizeRatio = clamp(maxSizeRatio, 1.0, 20.0, d.maxSizeRatio);
        c.ballDiameterInches = clamp(ballDiameterInches, 0.5, 10.0, d.ballDiameterInches);
        c.confirmFrames = clampInt(confirmFrames, 1, 30);
        c.confirmWindowSeconds = clamp(confirmWindowSeconds, 0.05, 3.0, d.confirmWindowSeconds);
        c.clusterRadiusInches = clamp(clusterRadiusInches, 0.5, 24.0, d.clusterRadiusInches);
        c.averageFrames = clampInt(averageFrames, 1, 30);
        c.associateRadiusInches = clamp(associateRadiusInches, 0.5, 24.0, d.associateRadiusInches);
        c.stopShortInches = clamp(stopShortInches, 0.0, 30.0, d.stopShortInches);
        c.minPathInches = clamp(minPathInches, 1.0, 12.0, d.minPathInches);
        c.turnFirstDegrees = clamp(turnFirstDegrees, 5.0, 90.0, d.turnFirstDegrees);
        c.turnToleranceDegrees = clamp(turnToleranceDegrees, 1.0, c.turnFirstDegrees, d.turnToleranceDegrees);
        c.turnTimeoutSeconds = clamp(turnTimeoutSeconds, 0.2, 3.0, d.turnTimeoutSeconds);
        c.turnKpPerRadian = clamp(turnKpPerRadian, 0.0, 5.0, d.turnKpPerRadian);
        c.maxTurnPower = clamp(maxTurnPower, 0.05, MAX_TURN_POWER_LIMIT, d.maxTurnPower);
        c.minTurnPower = clamp(minTurnPower, 0.0, c.maxTurnPower, d.minTurnPower);
        c.replanInches = clamp(replanInches, 0.5, 24.0, d.replanInches);
        c.replanMinSeconds = clamp(replanMinSeconds, 0.1, 5.0, d.replanMinSeconds);
        c.finalApproachInches = clamp(finalApproachInches, 0.0, 48.0, d.finalApproachInches);
        c.maxPower = clamp(maxPower, 0.05, MAX_POWER_LIMIT, d.maxPower);
        c.maxVelocityInchesPerSecond = clamp(maxVelocityInchesPerSecond, 0.0, MAX_VELOCITY_LIMIT_INCHES_PER_SECOND,
                d.maxVelocityInchesPerSecond);
        c.arriveToleranceInches = clamp(arriveToleranceInches, 0.5, 12.0, d.arriveToleranceInches);
        c.intakeSeconds = clamp(intakeSeconds, 0.0, MAX_INTAKE_SECONDS, d.intakeSeconds);
        c.intakeCreepInches = clamp(intakeCreepInches, 0.0, 12.0, d.intakeCreepInches);
        c.creepPower = clamp(creepPower, 0.0, Math.min(MAX_CREEP_POWER_LIMIT, c.maxPower), d.creepPower);
        c.intakePower = clamp(intakePower, -1.0, 1.0, d.intakePower);
        c.intakeBackwardAbortInches = clamp(intakeBackwardAbortInches, 0.5, 6.0, d.intakeBackwardAbortInches);
        c.divergeInches = clamp(divergeInches, 2.0, MAX_DIVERGE_INCHES, d.divergeInches);
        c.stallSeconds = clamp(stallSeconds, 0.5, MAX_STALL_SECONDS, d.stallSeconds);
        c.stallProgressInches = clamp(stallProgressInches, 0.25, 3.0, d.stallProgressInches);
        c.seekTimeoutSeconds = clamp(seekTimeoutSeconds, 1.0, MAX_SEEK_TIMEOUT_SECONDS, d.seekTimeoutSeconds);
        c.lostSeconds = clamp(lostSeconds, 0.25, 3.0, d.lostSeconds);
        c.crossCheckInches = clamp(crossCheckInches, 3.0, MAX_CROSS_CHECK_INCHES, d.crossCheckInches);
        c.crossCheckPerInch = clamp(crossCheckPerInch, 0.0, MAX_CROSS_CHECK_PER_INCH, d.crossCheckPerInch);
        c.crossCheckFrames = clampInt(crossCheckFrames, 1, 10);
        c.crossCheckGateFactor = clamp(crossCheckGateFactor, 1.5, 5.0, d.crossCheckGateFactor);
        c.poseJumpInches = clamp(poseJumpInches, 2.0, MAX_POSE_JUMP_INCHES, d.poseJumpInches);
        c.extraTravelInches = clamp(extraTravelInches, 6.0, 48.0, d.extraTravelInches);
        c.turnDriftAbortInches = clamp(turnDriftAbortInches, 2.0, MAX_TURN_DRIFT_INCHES, d.turnDriftAbortInches);
        c.wallMarginInches = clamp(wallMarginInches, 0.0, 12.0, d.wallMarginInches);
        c.robotRadiusInches = clamp(robotRadiusInches, 6.0, 14.0, d.robotRadiusInches);
        c.obstacleMarginInches = clamp(obstacleMarginInches, 0.0, 12.0, d.obstacleMarginInches);
        c.cornerPushInches = clamp(cornerPushInches, 0.5, 12.0, d.cornerPushInches);
        c.extraKeepOuts = extraKeepOuts == null ? "" : extraKeepOuts;
        c.pushTestForwardInches = clamp(pushTestForwardInches, 2.0, 12.0, d.pushTestForwardInches);
        c.pushTestLateralRatio = clamp(pushTestLateralRatio, 0.1, 1.0, d.pushTestLateralRatio);
        return c;
    }

    static double clamp(double value, double low, double high, double fallback) {
        double v = Double.isNaN(value) || Double.isInfinite(value) ? fallback : value;
        if (v < low) {
            return low;
        }
        if (v > high) {
            return high;
        }
        return v;
    }

    static int clampInt(int value, int low, int high) {
        return Math.max(low, Math.min(high, value));
    }
}
