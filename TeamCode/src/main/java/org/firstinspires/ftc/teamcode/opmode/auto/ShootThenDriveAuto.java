package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.launcher.Launcher;
import org.firstinspires.ftc.teamcode.opmode.teleOp.RobotTeleOp;

@Autonomous(name = "Shoot 3 Then Drive", group = "Meet0")
public class ShootThenDriveAuto extends LinearOpMode {

    private static final int SHOTS = 3;
    private static final double SHOOT_TIME_LIMIT_SECONDS = 13.0;
    private static final double DRIVE_SECONDS = 2.0;
    private static final double DRIVE_POWER = 0.4;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private Launcher launcher;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private String shootResult = "not started";

    @Override
    public void runOpMode() {
        leftFront = initMotor(FRONT_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = initMotor(BACK_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = initMotor(FRONT_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack = initMotor(BACK_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        intake = initMotor(INTAKE, RobotTeleOp.INTAKE_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);

        launcher = new Launcher(hardwareMap);
        launcher.setReadyTimeoutSeconds(Launcher.AUTO_READY_TIMEOUT_SECONDS);
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
        int detectedAtStart = launcher.shotsDetected();
        int attemptsAtStart = launcher.shotsAttempted();
        int maxAttempts = SHOTS + Launcher.AUTO_EXTRA_ATTEMPTS;

        launcher.setStagingTransferPower(Launcher.AUTO_STAGING_TRANSFER_POWER);
        launcher.spinUp();

        while (canShoot()) {
            int detected = launcher.shotsDetected() - detectedAtStart;
            int attempts = launcher.shotsAttempted() - attemptsAtStart;
            if (!launcher.isBusy() && (detected >= SHOTS || attempts >= maxAttempts || !launcher.canFire())) {
                break;
            }
            if (launcher.queuedShots() == 0 && !launcher.isBusy()) {
                launcher.requestShots(1);
            }
            launcher.update();
            intake.setPower(launcher.isFeeding() ? Launcher.AUTO_FEED_INTAKE_POWER : 0.0);

            telemetry.addData("Phase", "shooting  %d / %d detected  %d attempts", detected, SHOTS, attempts);
            launcher.addTelemetry(telemetry);
            telemetry.addData("Shoot time", "%.1f / %.1f s", shootTimer.seconds(), SHOOT_TIME_LIMIT_SECONDS);
            telemetry.update();
        }

        launcher.cancelShots();
        int detected = launcher.shotsDetected() - detectedAtStart;
        int attempts = launcher.shotsAttempted() - attemptsAtStart;
        shootResult = String.format("%d / %d shots detected, %d feeds, %.1f s%s",
                detected, SHOTS, attempts, shootTimer.seconds(),
                shootTimer.seconds() >= SHOOT_TIME_LIMIT_SECONDS ? " (time limit)" : "");
    }

    private boolean canShoot() {
        return opModeIsActive() && shootTimer.seconds() < SHOOT_TIME_LIMIT_SECONDS;
    }

    private void driveForward() {
        setDrivePower(DRIVE_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            launcher.update();
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
