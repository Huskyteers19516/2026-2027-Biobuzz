package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE_BLOCKER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.launcher.Launcher;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Shoot 3 Then Drive", group = "Meet0")
public class ShootThenDriveAuto extends LinearOpMode {

    private static final int SHOTS = 3;
    private static final double SHOOT_TIME_LIMIT_SECONDS = 13.0;
    private static final double DRIVE_SECONDS = 1.5;
    private static final double DRIVE_POWER = 0.3;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private Launcher launcher;
    private Servo blocker;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private String shootResult = "not started";

    @Override
    public void runOpMode() {
        PoseHandoff.clear();
        leftFront = initMotor(FRONT_LEFT_MOTOR_NAME, Constants.LEFT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = initMotor(BACK_LEFT_MOTOR_NAME, Constants.LEFT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = initMotor(FRONT_RIGHT_MOTOR_NAME, Constants.RIGHT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack = initMotor(BACK_RIGHT_MOTOR_NAME, Constants.RIGHT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        intake = initMotor(INTAKE, Constants.INTAKE_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);

        launcher = new Launcher(hardwareMap);
        launcher.setReadyTimeoutSeconds(Launcher.AUTO_READY_TIMEOUT_SECONDS);
        launcher.setBlockerControlEnabled(false);
        blocker = hardwareMap.get(Servo.class, OUTTAKE_BLOCKER);
        telemetry.addData("Outtake velocity", "%.0f ticks/s", () -> launcher.measuredVelocity());

        while (opModeInInit()) {
            launcher.sense();
            telemetry.addLine("Shoot 3 Then Drive");
            telemetry.addData("Plan", "shoot %d (max %.0f s), then drive forward %.1f s at %.2f",
                    SHOTS, SHOOT_TIME_LIMIT_SECONDS, DRIVE_SECONDS, DRIVE_POWER);
            telemetry.addData("Target velocity", "%.0f", launcher.targetVelocity());
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        try {
            shootTimer.reset();
            shoot();
            launcher.stop();
            intake.setPower(0.0);

            if (opModeIsActive()) {
                driveForward();
            }
        } finally {
            setDrivePower(0.0);
            intake.setPower(0.0);
            launcher.shutdown();
        }

        telemetry.addData("Status", "Finished");
        telemetry.addData("Shoot result", shootResult);
        Launcher.Shot last = launcher.lastShot();
        if (last != null) {
            telemetry.addData("Last shot", last.summary());
        }
        telemetry.update();
    }

    private void shoot() {
        launcher.spinUp();
        holdBlocker(false, Launcher.PULSE_SPINUP_SECONDS, "spinning up", true, 0);

        int pulses = 0;
        for (int ball = 1; ball <= SHOTS && canShoot(); ball++) {
            holdBlocker(true, Launcher.PULSE_OPEN_SECONDS, "blocker open", false, ball);
            pulses = ball;
            if (ball < SHOTS) {
                holdBlocker(false, Launcher.PULSE_RECOVER_SECONDS, "recovering", false, ball);
            }
        }
        holdBlocker(false, 0.0, "done", false, pulses);

        shootResult = String.format("%d / %d blocker pulses, %.1f s%s", pulses, SHOTS, shootTimer.seconds(),
                shootTimer.seconds() >= SHOOT_TIME_LIMIT_SECONDS ? " (time limit)" : "");
    }

    private void holdBlocker(boolean open, double seconds, String phase, boolean stopWhenReady, int ball) {
        launcher.setManualBlockerOpen(open);
        launcher.setStagingTransferPower(open ? Launcher.PULSE_TRANSFER_POWER : 0.0);
        intake.setPower(open ? Launcher.PULSE_INTAKE_POWER : 0.0);
        ElapsedTime timer = new ElapsedTime();
        do {
            launcher.update();
            blocker.setPosition(open ? Constants.BLOCKER_OPEN_POSITION : Constants.BLOCKER_CLOSED_POSITION);
            if (stopWhenReady && launcher.isReady()) {
                break;
            }
            telemetry.addData("Phase", "%s  ball %d / %d", phase, ball, SHOTS);
            launcher.addTelemetry(telemetry);
            telemetry.addData("Blocker", open ? "OPEN" : "CLOSED");
            telemetry.addData("Shoot time", "%.1f / %.1f s", shootTimer.seconds(), SHOOT_TIME_LIMIT_SECONDS);
            telemetry.update();
        } while (canShoot() && timer.seconds() < seconds);
        if (!open) {
            return;
        }
        launcher.setManualBlockerOpen(false);
        launcher.setStagingTransferPower(0.0);
        intake.setPower(0.0);
        blocker.setPosition(Constants.BLOCKER_CLOSED_POSITION);
    }

    private boolean canShoot() {
        return opModeIsActive() && shootTimer.seconds() < SHOOT_TIME_LIMIT_SECONDS;
    }

    private void driveForward() {
        setDrivePower(DRIVE_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            launcher.update();
            blocker.setPosition(Constants.BLOCKER_CLOSED_POSITION);
            telemetry.addData("Phase", "driving forward");
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), DRIVE_SECONDS);
            telemetry.addData("Shoot result", shootResult);
            telemetry.update();
        }
        setDrivePower(0.0);
    }

    private void setDrivePower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private DcMotor initMotor(String name, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior behavior) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(behavior);
        motor.setPower(0.0);
        return motor;
    }
}
