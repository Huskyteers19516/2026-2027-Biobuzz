package org.firstinspires.ftc.teamtestcode.functiontest.launchertesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamtestcode.other.robot.launcher.Launcher;
import org.firstinspires.ftc.teamtestcode.other.robot.pedroPathing.Constants;

@TeleOp(name = "New Launcher Tester", group = "TeleOp")
public class NewLauncherTester extends OpMode {

    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final double SMALL_STEP = 50.0;
    private static final double BIG_STEP = 250.0;

    private DcMotorEx motor;
    private VoltageSensor voltageSensor;
    private double target = Launcher.DEFAULT_TARGET_VELOCITY;
    private boolean on = false;
    private double integral = 0.0;
    private boolean boosting = false;
    private double power = 0.0;
    private double lastTime = 0.0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, Constants.OUTTAKE);
        motor.setDirection(DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (voltageSensor == null && sensor.getVoltage() > 7.0) {
                voltageSensor = sensor;
            }
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Actual", "%.0f ticks/s  (spin the wheel by hand, it must go positive)", motor.getVelocity());
    }

    @Override
    public void start() {
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            on = !on;
            integral = 0.0;
            boosting = false;
        }
        if (gamepad1.dpadUpWasPressed()) {
            target += SMALL_STEP;
        }
        if (gamepad1.dpadDownWasPressed()) {
            target -= SMALL_STEP;
        }
        if (gamepad1.dpadRightWasPressed()) {
            target += BIG_STEP;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            target -= BIG_STEP;
        }
        target = Range.clip(target, Launcher.MIN_TARGET_VELOCITY, Launcher.MAX_TARGET_VELOCITY);

        double now = getRuntime();
        double dt = now - lastTime;
        lastTime = now;
        double actual = motor.getVelocity();
        double error = target - actual;

        if (on) {
            double voltage = voltageSensor == null ? Launcher.NOMINAL_VOLTAGE : Math.max(voltageSensor.getVoltage(), 7.0);
            double feedforward = (Launcher.DEFAULT_KS + Launcher.DEFAULT_KV * target) * Launcher.NOMINAL_VOLTAGE / voltage;
            if (Math.abs(error) <= Launcher.INTEGRAL_ZONE) {
                integral = Range.clip(integral + Launcher.DEFAULT_KI * error * dt, -Launcher.INTEGRAL_LIMIT, Launcher.INTEGRAL_LIMIT);
            }
            if (error > Launcher.RECOVERY_BOOST_ENTER) {
                boosting = true;
            } else if (error < Launcher.RECOVERY_BOOST_EXIT) {
                boosting = false;
            }
            power = boosting ? 1.0 : Range.clip(feedforward + Launcher.DEFAULT_KP * error + integral, 0.0, 1.0);
        } else {
            power = 0.0;
        }
        motor.setPower(power);

        telemetry.addData("Launcher", on ? "ON" : "OFF  (Y = on)");
        telemetry.addData("Target", "%.0f ticks/s", target);
        telemetry.addData("Actual", "%.0f ticks/s", actual);
        telemetry.addData("Error", on ? String.format("%+.0f ticks/s", error) : "-");
        telemetry.addData("Power", "%.2f", power);
        telemetry.addLine("Y on/off   dpad up/down +/-50   dpad right/left +/-250");
    }

    @Override
    public void stop() {
        motor.setPower(0.0);
    }
}
