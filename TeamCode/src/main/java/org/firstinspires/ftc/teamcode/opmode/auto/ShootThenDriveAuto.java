package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE_BLOCKER;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.TRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.teleOp.RobotTeleOp;

@Autonomous(name = "Shoot 3 Then Drive", group = "Meet0")
public class ShootThenDriveAuto extends LinearOpMode {

    private static final int SHOTS = 3;
    private static final double SHOOT_TIME_LIMIT_SECONDS = 13.0;
    private static final double DRIVE_SECONDS = 2.0;
    private static final double DRIVE_POWER = 0.4;

    private static final double SHOOT_VELOCITY = RobotTeleOp.OUTTAKE_VELOCITY;
    private static final double SHOOT_TOLERANCE = RobotTeleOp.OUTTAKE_TOLERANCE;
    private static final double SPINUP_TIMEOUT_SECONDS = RobotTeleOp.SPINUP_TIMEOUT_SECONDS;
    private static final double FEED_SECONDS = RobotTeleOp.FEED_SECONDS;
    private static final double FEED_TRANSFER_POWER = RobotTeleOp.FEED_POWER;
    private static final double FEED_INTAKE_POWER = 1.0;
    private static final double SHOT_GAP_SECONDS = 0.4;

    private static final double BLOCKER_CLOSED_POSITION = RobotTeleOp.BLOCKER_CLOSED_POSITION;
    private static final double BLOCKER_OPEN_POSITION = RobotTeleOp.BLOCKER_OPEN_POSITION;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo blocker;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private int shotsFired = 0;
    private String shootResult = "not started";

    @Override
    public void runOpMode() {
        leftFront = initMotor(FRONT_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = initMotor(BACK_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = initMotor(FRONT_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack = initMotor(BACK_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        intake = initMotor(INTAKE, RobotTeleOp.INTAKE_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);
        transfer = initMotor(TRANSFER, RobotTeleOp.TRANSFER_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = hardwareMap.get(DcMotorEx.class, OUTTAKE);
        outtake.setDirection(RobotTeleOp.OUTTAKE_DIRECTION);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        blocker = hardwareMap.get(Servo.class, OUTTAKE_BLOCKER);

        while (opModeInInit()) {
            telemetry.addLine("Shoot 3 Then Drive");
            telemetry.addData("Plan", "shoot %d (max %.0f s), then drive forward %.1f s at %.2f",
                    SHOTS, SHOOT_TIME_LIMIT_SECONDS, DRIVE_SECONDS, DRIVE_POWER);
            telemetry.addData("Outtake velocity", "%.0f", SHOOT_VELOCITY);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        try {
            blocker.setPosition(BLOCKER_CLOSED_POSITION);
            shootTimer.reset();
            shoot();
            stopShooter();

            if (opModeIsActive()) {
                driveForward();
            }
        } finally {
            stopAll();
        }

        telemetry.addData("Status", "Finished");
        telemetry.addData("Shots fed", "%d / %d", shotsFired, SHOTS);
        telemetry.addData("Shoot result", shootResult);
        telemetry.update();
    }

    private void shoot() {
        outtake.setVelocity(SHOOT_VELOCITY);

        for (int shot = 1; shot <= SHOTS; shot++) {
            ElapsedTime spinUp = new ElapsedTime();
            boolean atSpeed = false;
            while (canShoot() && spinUp.seconds() < SPINUP_TIMEOUT_SECONDS) {
                atSpeed = Math.abs(outtake.getVelocity() - SHOOT_VELOCITY) <= SHOOT_TOLERANCE;
                if (atSpeed) {
                    break;
                }
                showShootTelemetry(shot, "spinning up");
            }
            if (!canShoot()) {
                shootResult = String.format("time limit reached before shot %d", shot);
                return;
            }

            blocker.setPosition(BLOCKER_OPEN_POSITION);
            transfer.setPower(FEED_TRANSFER_POWER);
            intake.setPower(FEED_INTAKE_POWER);

            ElapsedTime feed = new ElapsedTime();
            while (canShoot() && feed.seconds() < FEED_SECONDS) {
                showShootTelemetry(shot, atSpeed ? "feeding" : "feeding (spin-up timed out)");
            }

            transfer.setPower(0.0);
            intake.setPower(0.0);
            blocker.setPosition(BLOCKER_CLOSED_POSITION);
            shotsFired = shot;

            if (!canShoot()) {
                shootResult = String.format("time limit reached during shot %d", shot);
                return;
            }

            ElapsedTime gap = new ElapsedTime();
            while (shot < SHOTS && canShoot() && gap.seconds() < SHOT_GAP_SECONDS) {
                showShootTelemetry(shot, "gap");
            }
        }
        shootResult = String.format("all %d shots fed in %.1f s", SHOTS, shootTimer.seconds());
    }

    private boolean canShoot() {
        return opModeIsActive() && shootTimer.seconds() < SHOOT_TIME_LIMIT_SECONDS;
    }

    private void driveForward() {
        setDrivePower(DRIVE_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            telemetry.addData("Phase", "driving forward");
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), DRIVE_SECONDS);
            telemetry.addData("Shoot result", shootResult);
            telemetry.update();
        }
        setDrivePower(0.0);
    }

    private void showShootTelemetry(int shot, String phase) {
        telemetry.addData("Phase", "shot %d / %d  %s", shot, SHOTS, phase);
        telemetry.addData("Outtake", "target %.0f  measured %.0f", SHOOT_VELOCITY, outtake.getVelocity());
        telemetry.addData("Shoot time", "%.1f / %.1f s", shootTimer.seconds(), SHOOT_TIME_LIMIT_SECONDS);
        telemetry.update();
    }

    private void setDrivePower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void stopShooter() {
        transfer.setPower(0.0);
        intake.setPower(0.0);
        blocker.setPosition(BLOCKER_CLOSED_POSITION);
        outtake.setVelocity(0.0);
    }

    private void stopAll() {
        setDrivePower(0.0);
        stopShooter();
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
