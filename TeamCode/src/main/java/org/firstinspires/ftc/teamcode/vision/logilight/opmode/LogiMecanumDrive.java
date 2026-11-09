package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

public final class LogiMecanumDrive {

    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;
    private final String fault;
    private final double[] lastPowers = new double[4];

    public LogiMecanumDrive(HardwareMap hardwareMap) {
        DcMotor lf = null;
        DcMotor lb = null;
        DcMotor rf = null;
        DcMotor rb = null;
        String problem = "";
        try {
            lf = initMotor(hardwareMap, Constants.FRONT_LEFT_MOTOR_NAME, true);
            lb = initMotor(hardwareMap, Constants.BACK_LEFT_MOTOR_NAME, true);
            rf = initMotor(hardwareMap, Constants.FRONT_RIGHT_MOTOR_NAME, false);
            rb = initMotor(hardwareMap, Constants.BACK_RIGHT_MOTOR_NAME, false);
        } catch (Throwable t) {
            lf = null;
            lb = null;
            rf = null;
            rb = null;
            problem = "NOT AVAILABLE - " + LogiOpModeTools.describe(t);
        }
        leftFront = lf;
        leftBack = lb;
        rightFront = rf;
        rightBack = rb;
        fault = problem;
    }

    public boolean isAvailable() {
        return leftFront != null && leftBack != null && rightFront != null && rightBack != null;
    }

    public String fault() {
        return fault;
    }

    public void drive(double forward, double strafeLeft, double turnLeft) {
        drive(forward, strafeLeft, turnLeft, 1.0, 0.0, 0.0);
    }

    public void drive(double forward, double strafeLeft, double turnLeft, double scale,
                      double assistForward, double assistTurnLeft) {
        double axial = finiteOrZero(forward);
        double lateral = -finiteOrZero(strafeLeft);
        double yaw = -finiteOrZero(turnLeft);
        double[] powers = {
                axial + lateral + yaw,
                axial - lateral + yaw,
                axial - lateral - yaw,
                axial + lateral - yaw
        };
        normalize(powers);
        double factor = Double.isNaN(scale) || Double.isInfinite(scale) ? 1.0 : Math.max(0.0, Math.min(1.0, scale));
        double assistAxial = finiteOrZero(assistForward);
        double assistYaw = -finiteOrZero(assistTurnLeft);
        powers[0] = powers[0] * factor + assistAxial + assistYaw;
        powers[1] = powers[1] * factor + assistAxial + assistYaw;
        powers[2] = powers[2] * factor + assistAxial - assistYaw;
        powers[3] = powers[3] * factor + assistAxial - assistYaw;
        normalize(powers);
        apply(powers);
    }

    private static void normalize(double[] powers) {
        double max = 1.0;
        for (double power : powers) {
            max = Math.max(max, Math.abs(power));
        }
        for (int i = 0; i < powers.length; i++) {
            powers[i] /= max;
        }
    }

    public void stop() {
        apply(new double[]{0.0, 0.0, 0.0, 0.0});
    }

    public String powerSummary() {
        return String.format(Locale.US, "lf %+.2f  lb %+.2f  rf %+.2f  rb %+.2f",
                lastPowers[0], lastPowers[1], lastPowers[2], lastPowers[3]);
    }

    private void apply(double[] powers) {
        if (!isAvailable()) {
            return;
        }
        try {
            leftFront.setPower(powers[0]);
            leftBack.setPower(powers[1]);
            rightFront.setPower(powers[2]);
            rightBack.setPower(powers[3]);
            System.arraycopy(powers, 0, lastPowers, 0, lastPowers.length);
        } catch (Throwable ignored) {
        }
    }

    private static DcMotor initMotor(HardwareMap hardwareMap, String name, boolean left) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(left ? Constants.LEFT_SIDE_DIRECTION : Constants.RIGHT_SIDE_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private static double finiteOrZero(double value) {
        return Double.isNaN(value) || Double.isInfinite(value) ? 0.0 : value;
    }
}
