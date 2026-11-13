package org.firstinspires.ftc.teamcode.vision.logilight;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.HashMap;
import java.util.Map;

public final class CappedDrivetrain implements Drivetrain {

    private final Drivetrain inner;
    private volatile double cap;
    private volatile double lastPeakWheel;

    public CappedDrivetrain(Drivetrain inner, double cap) {
        if (inner == null) {
            throw new IllegalArgumentException("CappedDrivetrain needs a drivetrain");
        }
        this.inner = inner;
        setCap(cap);
    }

    public void setCap(double newCap) {
        if (Double.isNaN(newCap) || Double.isInfinite(newCap) || newCap <= 0.0) {
            cap = 0.0;
            return;
        }
        cap = Math.min(newCap, PollenSeekConfig.MAX_POWER_LIMIT);
    }

    public double cap() {
        return cap;
    }

    public double lastPeakWheelPower() {
        return lastPeakWheel;
    }

    public Drivetrain inner() {
        return inner;
    }

    public static double[] wheelMix(double forward, double strafe, double turn) {
        return new double[]{forward - strafe - turn, forward + strafe + turn, forward + strafe - turn, forward - strafe + turn};
    }

    public static DrivePowers capped(DrivePowers powers, double cap) {
        if (powers == null) {
            return DrivePowers.zero();
        }
        double f = powers.forward();
        double s = powers.strafe();
        double t = powers.turn();
        if (!finite(f) || !finite(s) || !finite(t) || !(cap > 0.0)) {
            return DrivePowers.zero();
        }
        double largest = 0.0;
        for (double wheel : wheelMix(f, s, t)) {
            largest = Math.max(largest, Math.abs(wheel));
        }
        double limit = Math.min(cap, PollenSeekConfig.MAX_POWER_LIMIT);
        if (largest <= limit) {
            return new DrivePowers(f, s, t);
        }
        double scale = limit / largest;
        return new DrivePowers(f * scale, s * scale, t * scale);
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        DrivePowers limited = capped(powers, cap);
        double peak = 0.0;
        for (double wheel : wheelMix(limited.forward(), limited.strafe(), limited.turn())) {
            peak = Math.max(peak, Math.abs(wheel));
        }
        lastPeakWheel = peak;
        inner.drive(limited, manual);
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        return inner.maxScaling(current, delta);
    }

    @Override
    public void stop() {
        lastPeakWheel = 0.0;
        inner.stop();
    }

    @Override
    public void stop(boolean brake) {
        lastPeakWheel = 0.0;
        inner.stop(brake);
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<String, Object>();
        Map<String, Object> innerDebug = inner.debug();
        if (innerDebug != null) {
            map.putAll(innerDebug);
        }
        map.put("powerCap", cap);
        map.put("peakWheelPower", lastPeakWheel);
        return map;
    }

    @Override
    public double interpolateAcceleration(double xRadius, double yRadius, double theta) {
        return inner.interpolateAcceleration(xRadius, yRadius, theta);
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return inner.interpolateVelocity(xRadius, yRadius, theta);
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
