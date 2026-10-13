package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.LAUNCHER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Launcher TeleOp", group = "Testing")
public class LauncherTeleOp extends OpMode {

    private static final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double START_VELOCITY = 2000.0;
    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 6000.0;
    private static final double SMALL_STEP = 100.0;
    private static final double BIG_STEP = 500.0;

    private DcMotorEx launcher;

    private double targetVelocity = START_VELOCITY;
    private boolean running = false;

    private boolean lastA = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastRight = false;
    private boolean lastLeft = false;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER);
        launcher.setDirection(LAUNCHER_DIRECTION);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean right = gamepad1.dpad_right;
        boolean left = gamepad1.dpad_left;

        if (a && !lastA) {
            running = !running;
        }
        if (up && !lastUp) {
            targetVelocity += SMALL_STEP;
        }
        if (down && !lastDown) {
            targetVelocity -= SMALL_STEP;
        }
        if (right && !lastRight) {
            targetVelocity += BIG_STEP;
        }
        if (left && !lastLeft) {
            targetVelocity -= BIG_STEP;
        }
        targetVelocity = Range.clip(targetVelocity, MIN_VELOCITY, MAX_VELOCITY);

        lastA = a;
        lastUp = up;
        lastDown = down;
        lastRight = right;
        lastLeft = left;

        if (running) {
            launcher.setVelocity(targetVelocity);
        } else {
            launcher.setVelocity(0);
        }

        telemetry.addData("Launcher", running ? "ON" : "OFF");
        telemetry.addData("Target", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Measured", "%.0f ticks/s", launcher.getVelocity());
        telemetry.addData("Power", "%.2f", launcher.getPower());
        telemetry.addLine("A = on/off   dpad up/down = +-100   dpad right/left = +-500");
    }

    @Override
    public void stop() {
        launcher.setVelocity(0);
    }
}
