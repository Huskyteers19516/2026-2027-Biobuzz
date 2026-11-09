package org.firstinspires.ftc.teamcode.vision.logilight;

public class LogiAimConfig {

    public double turnKp = 0.02;
    public double minTurnPower = 0.05;
    public double maxTurnPower = 0.5;
    public double toleranceDegrees = 1.5;

    public double driveKp = 0.03;
    public double maxDrivePower = 0.5;
    public double stopDistanceInches = 12.0;
    public double arriveOnLossInches = 18.0;

    public double searchTurnPower = 0.2;
    public double driveOnlyWhenTxBelowDegrees = 10.0;

    public LogiAimConfig() {
    }

    public LogiAimConfig copy() {
        LogiAimConfig copy = new LogiAimConfig();
        copy.turnKp = turnKp;
        copy.minTurnPower = minTurnPower;
        copy.maxTurnPower = maxTurnPower;
        copy.toleranceDegrees = toleranceDegrees;
        copy.driveKp = driveKp;
        copy.maxDrivePower = maxDrivePower;
        copy.stopDistanceInches = stopDistanceInches;
        copy.arriveOnLossInches = arriveOnLossInches;
        copy.searchTurnPower = searchTurnPower;
        copy.driveOnlyWhenTxBelowDegrees = driveOnlyWhenTxBelowDegrees;
        return copy;
    }

    public double turnLeftPowerForTx(double txDegrees) {
        if (Double.isNaN(txDegrees) || Math.abs(txDegrees) <= toleranceDegrees) {
            return 0.0;
        }
        double magnitude = Math.abs(txDegrees) * turnKp;
        magnitude = Math.max(minTurnPower, Math.min(maxTurnPower, magnitude));
        return -Math.signum(txDegrees) * magnitude;
    }

    public double forwardPowerForDistance(double distanceInches, double txDegrees) {
        if (Double.isNaN(distanceInches) || Double.isNaN(txDegrees)
                || Math.abs(txDegrees) > driveOnlyWhenTxBelowDegrees) {
            return 0.0;
        }
        double error = distanceInches - stopDistanceInches;
        if (error <= 0.0) {
            return 0.0;
        }
        return Math.min(maxDrivePower, error * driveKp);
    }
}
