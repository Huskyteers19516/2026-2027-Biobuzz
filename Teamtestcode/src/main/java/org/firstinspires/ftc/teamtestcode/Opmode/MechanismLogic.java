package org.firstinspires.ftc.teamtestcode.Opmode;

/**
 * The decisions for intake / transfer / launcher servo, with no hardware in it, so the
 * same rules run in RobotTeleOp and MatchAuto and can be unit tested on a laptop.
 *
 * Rules (highest priority first):
 *   1. EJECT  - intake and transfer run backwards, servo at rest.
 *   2. FIRE   - transfer feeds ONLY while the outtake is at speed. There is no override.
 *   3. INTAKE - intake runs in.
 * The servo only goes to its fire position while the transfer is feeding.
 */
public final class MechanismLogic {

    private MechanismLogic() {
    }

    public static final class Command {
        public final double intakePower;
        public final double transferPower;
        public final boolean servoFire;
        public final String state;

        Command(double intakePower, double transferPower, boolean servoFire, String state) {
            this.intakePower = intakePower;
            this.transferPower = transferPower;
            this.servoFire = servoFire;
            this.state = state;
        }
    }

    public static Command decide(boolean intakeHeld, boolean ejectHeld, boolean fireHeld,
                                 boolean outtakeReady) {
        if (ejectHeld) {
            return new Command(RobotTeleOp.EJECT_POWER, RobotTeleOp.EJECT_POWER, false, "EJECT");
        }

        double intakePower = intakeHeld ? RobotTeleOp.INTAKE_POWER : 0.0;

        if (fireHeld && outtakeReady) {
            return new Command(intakePower, RobotTeleOp.TRANSFER_FEED_POWER, true, "FIRING");
        }
        if (fireHeld) {
            return new Command(intakePower, 0.0, false, "WAITING FOR OUTTAKE SPEED");
        }
        if (intakeHeld) {
            return new Command(intakePower, RobotTeleOp.TRANSFER_POWER_WHILE_INTAKING, false, "INTAKE");
        }
        return new Command(0.0, 0.0, false, "IDLE");
    }

    public static boolean outtakeReady(boolean outtakeOn, double velocity, double target, double tolerance) {
        return outtakeOn && target > 0.0 && Math.abs(velocity - target) <= tolerance;
    }

    public static double clip(double value, double min, double max) {
        if (Double.isNaN(value)) {
            return min;
        }
        return Math.max(min, Math.min(max, value));
    }

    public static double deadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0.0 : clip(value, -1.0, 1.0);
    }

    /** Slow-mode trigger 0..1 blends the drive scale from normalScale down to slowScale. */
    public static double driveScale(double slowTrigger, double normalScale, double slowScale) {
        double t = clip(slowTrigger, 0.0, 1.0);
        return normalScale + (slowScale - normalScale) * t;
    }

    /** A path that timed out only counts as arrived if the robot really is at the target. */
    public static boolean arrived(double distanceInches, double headingErrorRadians,
                                  double toleranceInches, double toleranceRadians) {
        return distanceInches <= toleranceInches && Math.abs(angleWrap(headingErrorRadians)) <= toleranceRadians;
    }

    public static double angleWrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /**
     * Runs every action even if an earlier one throws, so one dead device can not keep
     * the others from being stopped. Returns the first failure, or null if all worked.
     */
    public static RuntimeException runAll(Runnable... actions) {
        RuntimeException first = null;
        for (Runnable action : actions) {
            try {
                action.run();
            } catch (RuntimeException e) {
                if (first == null) {
                    first = e;
                }
            }
        }
        return first;
    }

    /**
     * How long a timed action may still run: never longer than its own limit and never
     * past the end of the autonomous budget. Returns 0 when the budget is already spent.
     */
    public static double boundedSeconds(double wantedSeconds, double limitSeconds,
                                        double elapsedSeconds, double budgetSeconds) {
        double remaining = budgetSeconds - elapsedSeconds;
        return clip(Math.min(Math.min(wantedSeconds, limitSeconds), remaining), 0.0, limitSeconds);
    }
}
