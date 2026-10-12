package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple park auto for the basic 4-motor chassis.
 *
 * Path: drive forward, turn toward the parking zone, drive into the zone.
 * During init, press X for BLUE or B for RED. RED mirrors the turn direction.
 * Measure the field and tune the distances below before using it in a match.
 */
@Autonomous(name = "Simple Park Auto", group = "Meet0")
public class SimpleParkAuto extends LinearOpMode {

    // Path. Set TURN_DEGREES to 0 to drive straight into the zone.
    private static final double START_DELAY_SECONDS = 0.0;
    private static final double FORWARD_INCHES = 24.0;
    private static final double TURN_DEGREES = 90.0;   // positive = left on BLUE
    private static final double PARK_INCHES = 24.0;

    private static final double DRIVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.3;

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Distance between left and right wheels. Tune until a 90 degree turn is 90 degrees.
    private static final double TRACK_WIDTH_INCHES = 14.0;

    private static final double LEG_TIMEOUT_BASE_SECONDS = 2.0;
    private static final double LEG_TIMEOUT_SECONDS_PER_INCH = 0.25;
    private static final double LEG_TIMEOUT_REFERENCE_SPEED = 0.4;
    private static final double PAUSE_SECONDS = 0.25;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private final ElapsedTime legTimer = new ElapsedTime();
    private boolean isRed = false;
    private String legResult = "none";

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(0.0);

        while (opModeInInit()) {
            if (gamepad1.x) {
                isRed = false;
            } else if (gamepad1.b) {
                isRed = true;
            }

            telemetry.addLine("Simple Park Auto");
            telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
            telemetry.addLine("Press X for BLUE, B for RED.");
            telemetry.addData("Path", "fwd %.0f in, turn %.0f deg, park %.0f in",
                    FORWARD_INCHES, isRed ? -TURN_DEGREES : TURN_DEGREES, PARK_INCHES);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        try {
            runPark();
        } finally {
            setPower(0.0);
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Status", "Finished");
        telemetry.addData("Last leg", legResult);
        telemetry.update();
    }

    private void runPark() {
        if (START_DELAY_SECONDS > 0) {
            sleep((long) (START_DELAY_SECONDS * 1000));
        }

        if (!drive("Driving forward", FORWARD_INCHES)) return;
        pause();

        double turn = isRed ? -TURN_DEGREES : TURN_DEGREES;
        if (!turn("Turning toward the parking zone", turn)) return;
        pause();

        drive("Parking", PARK_INCHES);
    }

    private boolean drive(String step, double inches) {
        int counts = (int) Math.round(inches * COUNTS_PER_INCH);
        return move(step, counts, counts, DRIVE_SPEED, legTimeoutSeconds(inches, DRIVE_SPEED));
    }

    /** Positive degrees turns left (counter-clockwise). */
    private boolean turn(String step, double degrees) {
        double arcInches = Math.PI * TRACK_WIDTH_INCHES * (degrees / 360.0);
        int counts = (int) Math.round(arcInches * COUNTS_PER_INCH);
        return move(step, -counts, counts, TURN_SPEED, legTimeoutSeconds(arcInches, TURN_SPEED));
    }

    private boolean move(String step, int leftCounts, int rightCounts, double speed,
                         double timeoutSeconds) {
        if (!opModeIsActive()) return false;
        if (leftCounts == 0 && rightCounts == 0) return true;

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + leftCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rightCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rightCounts);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        legTimer.reset();
        setPower(Math.abs(speed));

        while (opModeIsActive() && motorsBusy() && legTimer.seconds() < timeoutSeconds) {
            telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
            telemetry.addData("Step", step);
            telemetry.addData("Target", "L %d  R %d",
                    leftFront.getTargetPosition(), rightFront.getTargetPosition());
            telemetry.addData("Actual", "L %d  R %d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.update();
        }

        boolean arrived = opModeIsActive() && !motorsBusy();
        if (!opModeIsActive()) {
            legResult = step + ": stopped";
        } else if (!arrived) {
            legResult = String.format("%s: TIMEOUT at %.1f s", step, timeoutSeconds);
        } else {
            legResult = String.format("%s: arrived in %.1f s", step, legTimer.seconds());
        }

        setPower(0.0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return arrived;
    }

    private double legTimeoutSeconds(double inches, double speed) {
        return LEG_TIMEOUT_BASE_SECONDS
                + Math.abs(inches) * LEG_TIMEOUT_SECONDS_PER_INCH
                * (LEG_TIMEOUT_REFERENCE_SPEED / Math.max(Math.abs(speed), 0.05));
    }

    private void pause() {
        if (opModeIsActive()) {
            sleep((long) (PAUSE_SECONDS * 1000));
        }
    }

    private boolean motorsBusy() {
        return leftFront.isBusy() || leftBack.isBusy()
                || rightFront.isBusy() || rightBack.isBusy();
    }

    private void setPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
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
