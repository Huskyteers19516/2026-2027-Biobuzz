package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeeker;

public final class PollenIntakeMotor implements PollenSeeker.IntakeControl {

    private final DcMotor motor;
    private double lastPower;

    private PollenIntakeMotor(DcMotor motor) {
        this.motor = motor;
    }

    public static PollenIntakeMotor create(HardwareMap hardwareMap, StringBuilder fault) {
        try {
            DcMotor motor = hardwareMap.get(DcMotor.class, Constants.INTAKE);
            motor.setDirection(Constants.INTAKE_DIRECTION);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
            return new PollenIntakeMotor(motor);
        } catch (Throwable t) {
            if (fault != null) {
                fault.append("intake \"").append(Constants.INTAKE).append("\" NOT AVAILABLE - ")
                        .append(LogiOpModeTools.describe(t));
            }
            return null;
        }
    }

    @Override
    public void run(double power) {
        double clipped = Double.isNaN(power) || Double.isInfinite(power) ? 0.0 : Range.clip(power, -1.0, 1.0);
        motor.setPower(clipped);
        lastPower = clipped;
    }

    @Override
    public void stop() {
        motor.setPower(0.0);
        lastPower = 0.0;
    }

    public double lastPower() {
        return lastPower;
    }
}
