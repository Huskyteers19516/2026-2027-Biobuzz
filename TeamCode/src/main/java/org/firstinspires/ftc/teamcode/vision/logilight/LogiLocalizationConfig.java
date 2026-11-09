package org.firstinspires.ftc.teamcode.vision.logilight;

public class LogiLocalizationConfig {

    public boolean enabled = true;
    public boolean useUnsurveyedTargets = false;
    public double maxTargetDistanceInches = 110.0;
    public int minPercentClusterFound = 50;
    public double maxHeightErrorInches = 6.0;
    public double maxTiltMismatchDegrees = 12.0;
    public double maxRotationMismatchDegrees = 15.0;
    public double maxAngularVelocityDegPerSec = 180.0;
    public double headingMaxExtrapolateMs = 60.0;

    public double stddevBaseInches = 1.0;
    public double stddevPerInchSquared = 0.0008;
    public double headingStddevBaseDegrees = 3.0;
    public double headingStddevPerInchDegrees = 0.05;

    public double maxJumpInches = 24.0;
    public int jumpConfirmFrames = 4;

    public double initialVarianceXY = 4.0;
    public double initialVarianceHeading = 0.01;
    public double processVarianceXY = 0.02;
    public double processVarianceHeading = 0.0005;
    public double measurementVarianceXY = 4.0;
    public double measurementVarianceHeading = 0.02;
    public int fusionBufferSize = 150;
    public boolean fuseMt1Heading = false;

    public LogiLocalizationConfig() {
    }

    public LogiLocalizationConfig copy() {
        LogiLocalizationConfig copy = new LogiLocalizationConfig();
        copy.enabled = enabled;
        copy.useUnsurveyedTargets = useUnsurveyedTargets;
        copy.maxTargetDistanceInches = maxTargetDistanceInches;
        copy.minPercentClusterFound = minPercentClusterFound;
        copy.maxHeightErrorInches = maxHeightErrorInches;
        copy.maxTiltMismatchDegrees = maxTiltMismatchDegrees;
        copy.maxRotationMismatchDegrees = maxRotationMismatchDegrees;
        copy.maxAngularVelocityDegPerSec = maxAngularVelocityDegPerSec;
        copy.headingMaxExtrapolateMs = headingMaxExtrapolateMs;
        copy.stddevBaseInches = stddevBaseInches;
        copy.stddevPerInchSquared = stddevPerInchSquared;
        copy.headingStddevBaseDegrees = headingStddevBaseDegrees;
        copy.headingStddevPerInchDegrees = headingStddevPerInchDegrees;
        copy.maxJumpInches = maxJumpInches;
        copy.jumpConfirmFrames = jumpConfirmFrames;
        copy.initialVarianceXY = initialVarianceXY;
        copy.initialVarianceHeading = initialVarianceHeading;
        copy.processVarianceXY = processVarianceXY;
        copy.processVarianceHeading = processVarianceHeading;
        copy.measurementVarianceXY = measurementVarianceXY;
        copy.measurementVarianceHeading = measurementVarianceHeading;
        copy.fusionBufferSize = fusionBufferSize;
        copy.fuseMt1Heading = fuseMt1Heading;
        return copy;
    }
}
