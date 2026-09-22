package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum TeleOp", group = "Meet0")
public class MecanumTeleOp extends OpMode {

    private static final double LAUNCHER_VELOCITY = 2000.0;

    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double STICK_DEADZONE = 0.05;

    private static final boolean LEFT_SIDE_REVERSED = false;
    private static final boolean LAUNCHER_REVERSED = false;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotorEx launcher;

    private boolean launcherOn = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        DcMotorSimple.Direction leftDirection = LEFT_SIDE_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rightDirection = LEFT_SIDE_REVERSED
                ? DcMotorSimple.Direction.FORWARD
                : DcMotorSimple.Direction.REVERSE;

        leftFront.setDirection(leftDirection);
        leftBack.setDirection(leftDirection);
        rightFront.setDirection(rightDirection);
        rightBack.setDirection(rightDirection);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setDirection(LAUNCHER_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAll();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double axial = deadzone(-gamepad1.left_stick_y);
        double lateral = deadzone(gamepad1.left_stick_x);
        double yaw = deadzone(gamepad1.right_stick_x);

        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower * DRIVE_POWER_SCALE);
        leftBack.setPower(leftBackPower * DRIVE_POWER_SCALE);
        rightFront.setPower(rightFrontPower * DRIVE_POWER_SCALE);
        rightBack.setPower(rightBackPower * DRIVE_POWER_SCALE);

        if (gamepad1.aWasPressed()) {
            launcherOn = !launcherOn;
        }
        if (gamepad1.bWasPressed()) {
            launcherOn = false;
        }

        if (launcherOn) {
            launcher.setVelocity(LAUNCHER_VELOCITY);
        } else {
            launcher.setVelocity(0.0);
        }

        telemetry.addData("Drive", "axial %.2f  lateral %.2f  yaw %.2f", axial, lateral, yaw);
        telemetry.addData("Wheels", "LF %.2f  RF %.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Wheels", "LB %.2f  RB %.2f", leftBackPower, rightBackPower);
        telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
        telemetry.addData("Launcher target", "%.0f ticks/s", launcherOn ? LAUNCHER_VELOCITY : 0.0);
        telemetry.addData("Launcher actual", "%.0f ticks/s", launcher.getVelocity());
    }

    @Override
    public void stop() {
        stopAll();
    }

    private double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : Range.clip(value, -1.0, 1.0);
    }

    private void stopAll() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        launcher.setPower(0.0);
    }
}
