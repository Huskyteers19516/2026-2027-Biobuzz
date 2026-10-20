package org.firstinspires.ftc.teamtestcode.intaketesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Intake Motor Test", group = "Testing")
public class IntakeMotorTest extends OpMode {

    private static final String MOTOR_NAME = "intake_motor";
    private static final boolean INTAKE_REVERSED = false;

    private static final double TICKS_PER_REV_OVERRIDE = 0.0;
    private static final double EXTERNAL_GEAR_RATIO = 1.0;

    private static final double TRIGGER_DEADZONE = 0.05;
    private static final double DEFAULT_TARGET_VELOCITY = 1000.0;
    private static final double VELOCITY_STEP = 100.0;
    private static final double VELOCITY_FINE_STEP = 25.0;
    private static final double MAX_TARGET_VELOCITY = 6000.0;
    private static final double VELOCITY_REACHED_TOLERANCE = 50.0;
    private static final double STALL_WARN_SECONDS = 1.5;

    private DcMotorEx intake;

    private double ticksPerRev;
    private double maxRpm;

    private double targetVelocity = DEFAULT_TARGET_VELOCITY;
    private boolean running = false;

    private double peakRpm = 0.0;
    private double peakAmps = 0.0;

    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;
    private boolean yWasPressed = false;
    private boolean upWasPressed = false;
    private boolean downWasPressed = false;

    private final ElapsedTime stallTimer = new ElapsedTime();

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        intake.setDirection(INTAKE_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        telemetry.addLine("A / B: target velocity +/- 100 ticks/s");
        telemetry.addLine("Dpad up/down: fine +/- 25 ticks/s");
        telemetry.addLine("Y: run/stop   Right trigger: hold to run");
        telemetry.addLine("X: reset peaks");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (a && !aWasPressed) {
            targetVelocity = Range.clip(targetVelocity + VELOCITY_STEP, 0.0, MAX_TARGET_VELOCITY);
        }
        if (b && !bWasPressed) {
            targetVelocity = Range.clip(targetVelocity - VELOCITY_STEP, 0.0, MAX_TARGET_VELOCITY);
        }
        if (up && !upWasPressed) {
            targetVelocity = Range.clip(targetVelocity + VELOCITY_FINE_STEP, 0.0, MAX_TARGET_VELOCITY);
        }
        if (down && !downWasPressed) {
            targetVelocity = Range.clip(targetVelocity - VELOCITY_FINE_STEP, 0.0, MAX_TARGET_VELOCITY);
        }
        if (y && !yWasPressed) {
            running = !running;
        }
        if (x && !xWasPressed) {
            peakRpm = 0.0;
            peakAmps = 0.0;
        }

        aWasPressed = a;
        bWasPressed = b;
        xWasPressed = x;
        yWasPressed = y;
        upWasPressed = up;
        downWasPressed = down;

        boolean triggerHeld = gamepad1.right_trigger > TRIGGER_DEADZONE;
        boolean commanded = triggerHeld || running;
        double commandedVelocity = commanded ? targetVelocity : 0.0;

        intake.setVelocity(commandedVelocity);

        double ticksPerSecond = intake.getVelocity();
        double motorRpm = ticksPerRev > 0.0 ? ticksPerSecond / ticksPerRev * 60.0 : 0.0;
        double rollerRpm = motorRpm * EXTERNAL_GEAR_RATIO;
        double amps = intake.getCurrent(CurrentUnit.AMPS);
        double percentOfFree = maxRpm > 0.0 ? Math.abs(motorRpm) / maxRpm * 100.0 : 0.0;
        double error = commandedVelocity - ticksPerSecond;

        peakRpm = Math.max(peakRpm, Math.abs(motorRpm));
        peakAmps = Math.max(peakAmps, amps);

        boolean movingEnough = Math.abs(ticksPerSecond) > VELOCITY_REACHED_TOLERANCE;

        if (commandedVelocity <= 0.0 || movingEnough) {
            stallTimer.reset();
        }

        telemetry.addData("Control", triggerHeld ? "trigger held" : (running ? "running" : "stopped"));
        telemetry.addData("Target velocity", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Commanded", "%.0f ticks/s", commandedVelocity);
        telemetry.addData("Measured", "%.0f ticks/s", ticksPerSecond);
        telemetry.addData("Error", "%.0f ticks/s%s", error,
                commanded && Math.abs(error) <= VELOCITY_REACHED_TOLERANCE ? "  AT SPEED" : "");
        telemetry.addLine();
        telemetry.addData("Motor speed", "%.0f RPM", motorRpm);
        telemetry.addData("Roller speed", "%.0f RPM", rollerRpm);
        telemetry.addData("Of rated free speed", "%.0f %%", percentOfFree);
        telemetry.addData("Current", "%.2f A", amps);
        telemetry.addLine();
        telemetry.addData("Peak speed", "%.0f RPM", peakRpm);
        telemetry.addData("Peak current", "%.2f A", peakAmps);
        telemetry.addData("Encoder position", intake.getCurrentPosition());

        if (commandedVelocity > 0.0 && !movingEnough && stallTimer.seconds() > STALL_WARN_SECONDS) {
            telemetry.addLine();
            telemetry.addLine("!! COMMANDED BUT NOT MOVING");
            telemetry.addLine("Encoder cable unplugged, or the intake is jammed.");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setPower(0.0);
    }
}
