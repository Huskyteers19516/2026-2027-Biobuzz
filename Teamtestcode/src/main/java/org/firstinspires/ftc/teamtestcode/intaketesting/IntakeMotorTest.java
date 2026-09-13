package org.firstinspires.ftc.teamtestcode.intaketesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Intake Motor Test", group = "Testing")
public class IntakeMotorTest extends OpMode {

    private static final String MOTOR_NAME = "intake_motor";
    private static final boolean INTAKE_REVERSED = false;

    private static final double TICKS_PER_REV_OVERRIDE = 0.0;
    private static final double EXTERNAL_GEAR_RATIO = 1.0;

    private static final double TRIGGER_DEADZONE = 0.05;
    private static final double POWER_STEP = 0.05;
    private static final double DEFAULT_SET_POWER = 0.5;

    private DcMotorEx intake;

    private double ticksPerRev;
    private double maxRpm;

    private double setPower = DEFAULT_SET_POWER;
    private boolean running = false;

    private double peakRpm = 0.0;
    private double peakAmps = 0.0;

    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;
    private boolean upWasPressed = false;
    private boolean downWasPressed = false;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        intake.setDirection(INTAKE_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ticksPerRev = TICKS_PER_REV_OVERRIDE > 0.0
                ? TICKS_PER_REV_OVERRIDE
                : intake.getMotorType().getTicksPerRev();
        maxRpm = intake.getMotorType().getMaxRPM();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Intake Motor Test");
        telemetry.addData("Motor type", intake.getMotorType().getName());
        telemetry.addData("Ticks per rev", "%.1f", ticksPerRev);
        telemetry.addData("Rated free speed", "%.0f RPM", maxRpm);
        if (ticksPerRev <= 0.0) {
            telemetry.addLine("WARNING: ticks per rev is 0. Set the motor type in the");
            telemetry.addLine("robot configuration or set TICKS_PER_REV_OVERRIDE.");
        }
        telemetry.addLine();
        telemetry.addLine("Right trigger: intake");
        telemetry.addLine("A: run/stop at set power   B: stop");
        telemetry.addLine("Dpad up/down: set power   X: reset peaks");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (a && !aWasPressed) {
            running = !running;
        }
        if (b && !bWasPressed) {
            running = false;
        }
        if (x && !xWasPressed) {
            peakRpm = 0.0;
            peakAmps = 0.0;
        }
        if (up && !upWasPressed) {
            setPower = Range.clip(setPower + POWER_STEP, 0.0, 1.0);
        }
        if (down && !downWasPressed) {
            setPower = Range.clip(setPower - POWER_STEP, 0.0, 1.0);
        }

        aWasPressed = a;
        bWasPressed = b;
        xWasPressed = x;
        upWasPressed = up;
        downWasPressed = down;

        double triggerPower = gamepad1.right_trigger;
        boolean triggerActive = triggerPower > TRIGGER_DEADZONE;

        String source;
        double power;

        if (triggerActive) {
            power = triggerPower;
            source = "triggers";
        } else if (running) {
            power = setPower;
            source = "set power";
        } else {
            power = 0.0;
            source = "stopped";
        }

        intake.setPower(Range.clip(power, 0.0, 1.0));

        double ticksPerSecond = intake.getVelocity();
        double motorRpm = ticksPerRev > 0.0 ? ticksPerSecond / ticksPerRev * 60.0 : 0.0;
        double rollerRpm = motorRpm * EXTERNAL_GEAR_RATIO;
        double amps = intake.getCurrent(CurrentUnit.AMPS);
        double percentOfFree = maxRpm > 0.0 ? Math.abs(motorRpm) / maxRpm * 100.0 : 0.0;

        peakRpm = Math.max(peakRpm, Math.abs(motorRpm));
        peakAmps = Math.max(peakAmps, amps);

        telemetry.addData("Control", source);
        telemetry.addData("Set power", "%.2f", setPower);
        telemetry.addData("Output power", "%.2f", power);
        telemetry.addLine();
        telemetry.addData("Motor speed", "%.0f RPM", motorRpm);
        telemetry.addData("Roller speed", "%.0f RPM", rollerRpm);
        telemetry.addData("Encoder velocity", "%.0f ticks/s", ticksPerSecond);
        telemetry.addData("Of rated free speed", "%.0f %%", percentOfFree);
        telemetry.addData("Current", "%.2f A", amps);
        telemetry.addLine();
        telemetry.addData("Peak speed", "%.0f RPM", peakRpm);
        telemetry.addData("Peak current", "%.2f A", peakAmps);
        telemetry.addData("Encoder position", intake.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
    }
}
