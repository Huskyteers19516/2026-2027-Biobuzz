package org.firstinspires.ftc.teamtestcode.OpModeForTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Forward Back Auto", group = "Testing")
public class ForwardBackAuto extends LinearOpMode {

    private static final double FORWARD_INCHES = 24.0;
    private static final double BACKWARD_INCHES = 24.0;
    private static final double DRIVE_SPEED = 0.4;
    private static final double PAUSE_SECONDS = 0.5;

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double LEG_TIMEOUT_BASE_SECONDS = 2.0;
    private static final double LEG_TIMEOUT_SECONDS_PER_INCH = 0.25;
    private static final double LEG_TIMEOUT_REFERENCE_SPEED = 0.4;

    private static final double INTAKE_POWER = 0.0;
    private static final boolean INTAKE_DURING_FORWARD = false;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;

    private final ElapsedTime legTimer = new ElapsedTime();
    private String legResult = "none";

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            telemetry.addLine("Forward Back Auto");
            telemetry.addData("Forward", "%.1f in", FORWARD_INCHES);
            telemetry.addData("Backward", "%.1f in", BACKWARD_INCHES);
            telemetry.addData("Speed", "%.2f", DRIVE_SPEED);
            telemetry.addLine("Encoder cables must be plugged in.");
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        try {
            if (INTAKE_DURING_FORWARD) {
                intake.setPower(INTAKE_POWER);
            }

            driveInches(FORWARD_INCHES, "forward");

            intake.setPower(0.0);
            sleepSeconds(PAUSE_SECONDS);

            driveInches(-BACKWARD_INCHES, "backward");
        } finally {
            stopAll();
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Done.");
        telemetry.addData("Last leg", legResult);
        telemetry.update();
    }

    private void driveInches(double inches, String label) {
        if (!opModeIsActive()) {
            return;
        }

        int counts = (int) (inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + counts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + counts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + counts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + counts);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(Math.abs(DRIVE_SPEED));

        double timeout = legTimeoutSeconds(inches, Math.abs(DRIVE_SPEED));
        legTimer.reset();

        while (opModeIsActive() && motorsBusy() && legTimer.seconds() < timeout) {
            telemetry.addData("Step", label);
            telemetry.addData("Target", "%.1f in", inches);
            telemetry.addData("Elapsed", "%.1f / %.1f s", legTimer.seconds(), timeout);
            telemetry.addData("Position", "LF %d  RF %d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.update();
        }

        legResult = motorsBusy()
                ? String.format("%s TIMEOUT at %.1f s", label, legTimer.seconds())
                : String.format("%s arrived in %.1f s", label, legTimer.seconds());

        setDrivePower(0.0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double legTimeoutSeconds(double inches, double speed) {
        double safeSpeed = speed > 0.05 ? speed : 0.05;

        return LEG_TIMEOUT_BASE_SECONDS
                + Math.abs(inches) * LEG_TIMEOUT_SECONDS_PER_INCH
                * (LEG_TIMEOUT_REFERENCE_SPEED / safeSpeed);
    }

    private void sleepSeconds(double seconds) {
        legTimer.reset();

        while (opModeIsActive() && legTimer.seconds() < seconds) {
            telemetry.addData("Step", "pause");
            telemetry.addData("Elapsed", "%.1f / %.1f s", legTimer.seconds(), seconds);
            telemetry.update();
        }
    }

    private boolean motorsBusy() {
        return leftFront.isBusy() || leftBack.isBusy()
                || rightFront.isBusy() || rightBack.isBusy();
    }

    private void setDrivePower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void stopAll() {
        setDrivePower(0.0);
        intake.setPower(0.0);
    }

    private void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }
}
