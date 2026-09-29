package org.firstinspires.ftc.teamtestcode.mecanumdriveauto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Mecanum Drive Auto (Square)", group = "Testing")
public class MecanumDriveAuto extends LinearOpMode {

    private static final boolean USE_TURNS = false;
    private static final boolean STRAFE_CALIBRATION = false;

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double STRAFE_MULTIPLIER = 1.15;

    private static final double SIDE_LENGTH_INCHES = 24.0;

    private static final double DRIVE_SPEED = 0.4;
    private static final double STRAFE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.3;

    private static final double HEADING_THRESHOLD = 1.0;
    private static final double P_TURN_GAIN = 0.02;
    private static final double P_DRIVE_GAIN = 0.03;
    private static final double MAX_CORRECTION_SCALE = 0.5;

    private static final double LEG_TIMEOUT_BASE_SECONDS = 2.0;
    private static final double LEG_TIMEOUT_SECONDS_PER_INCH = 0.25;
    private static final double LEG_TIMEOUT_REFERENCE_SPEED = 0.4;
    private static final double TURN_TIMEOUT_SECONDS = 4.0;
    private static final double SETTLE_SECONDS = 0.3;

    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private IMU imu;

    private final ElapsedTime timer = new ElapsedTime();

    private double headingError = 0.0;
    private String step = "idle";
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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION)));

        while (opModeInInit()) {
            telemetry.addLine("Mecanum Drive Auto - Square");
            telemetry.addData("Mode", USE_TURNS ? "drive + turn" : "strafe, no turns");
            if (!USE_TURNS && STRAFE_CALIBRATION) {
                telemetry.addLine("STRAFE_CALIBRATION is ON: stops after leg 2");
            }
            telemetry.addData("Side length", "%.1f in", SIDE_LENGTH_INCHES);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addLine("Place the robot, then press START.");
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        imu.resetYaw();

        try {
            if (USE_TURNS) {
                runSquareWithTurns();
            } else {
                runSquareWithStrafes();
            }
        } finally {
            stopMotors();
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Step", step);
        if (!USE_TURNS && STRAFE_CALIBRATION) {
            telemetry.addLine("Stopped after the strafe leg. Measure the sideways distance.");
        } else {
            telemetry.addLine("Square complete. Back at start.");
        }
        telemetry.addData("Last leg", legResult);
        telemetry.addData("Final heading", "%.1f deg", getHeading());
        telemetry.update();
    }

    private void runSquareWithTurns() {
        double heading = 0.0;

        for (int side = 0; side < 4 && opModeIsActive(); side++) {
            step = String.format("turn mode: side %d of 4", side + 1);

            driveStraight(DRIVE_SPEED, SIDE_LENGTH_INCHES, heading);

            heading = normalize(heading - 90.0);

            turnToHeading(TURN_SPEED, heading);
            holdHeading(TURN_SPEED, heading, SETTLE_SECONDS);
        }
    }

    private void runSquareWithStrafes() {
        double heading = 0.0;

        step = "strafe mode: 1 of 4 forward";
        driveStraight(DRIVE_SPEED, SIDE_LENGTH_INCHES, heading);
        holdHeading(TURN_SPEED, heading, SETTLE_SECONDS);

        step = "strafe mode: 2 of 4 right";
        strafe(STRAFE_SPEED, SIDE_LENGTH_INCHES, heading);
        holdHeading(TURN_SPEED, heading, SETTLE_SECONDS);

        if (STRAFE_CALIBRATION) {
            step = "strafe mode: stopped after leg 2 for calibration";
            return;
        }

        step = "strafe mode: 3 of 4 backward";
        driveStraight(DRIVE_SPEED, -SIDE_LENGTH_INCHES, heading);
        holdHeading(TURN_SPEED, heading, SETTLE_SECONDS);

        step = "strafe mode: 4 of 4 left";
        strafe(STRAFE_SPEED, -SIDE_LENGTH_INCHES, heading);
        holdHeading(TURN_SPEED, heading, SETTLE_SECONDS);
    }

    private void driveStraight(double maxSpeed, double inches, double heading) {
        int counts = (int) (inches * COUNTS_PER_INCH);
        runToTargets(maxSpeed, counts, counts, counts, counts, heading, "driving",
                legTimeoutSeconds(inches, maxSpeed));
    }

    private void strafe(double maxSpeed, double inches, double heading) {
        int counts = (int) (inches * COUNTS_PER_INCH * STRAFE_MULTIPLIER);
        runToTargets(maxSpeed, counts, -counts, -counts, counts, heading, "strafing",
                legTimeoutSeconds(inches, maxSpeed));
    }

    private double legTimeoutSeconds(double inches, double maxSpeed) {
        double speed = Math.max(Math.abs(maxSpeed), 0.05);
        return LEG_TIMEOUT_BASE_SECONDS
                + Math.abs(inches) * LEG_TIMEOUT_SECONDS_PER_INCH
                * (LEG_TIMEOUT_REFERENCE_SPEED / speed);
    }

    private void runToTargets(double maxSpeed, int deltaLeftFront, int deltaRightFront,
                              int deltaLeftBack, int deltaRightBack, double heading, String mode,
                              double timeoutSeconds) {
        if (!opModeIsActive()) {
            return;
        }

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + deltaLeftFront);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + deltaRightFront);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + deltaLeftBack);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + deltaRightBack);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double speed = Math.abs(maxSpeed);
        double limit = speed * MAX_CORRECTION_SCALE;

        double baseLeftFront = Math.signum(deltaLeftFront) * speed;
        double baseRightFront = Math.signum(deltaRightFront) * speed;
        double baseLeftBack = Math.signum(deltaLeftBack) * speed;
        double baseRightBack = Math.signum(deltaRightBack) * speed;

        setWheelPowers(baseLeftFront, baseRightFront, baseLeftBack, baseRightBack);

        timer.reset();

        while (opModeIsActive() && motorsBusy() && timer.seconds() < timeoutSeconds) {
            double correction = getSteeringCorrection(heading, P_DRIVE_GAIN);
            correction = Range.clip(correction, -limit, limit);

            setWheelPowers(
                    baseLeftFront - correction,
                    baseRightFront + correction,
                    baseLeftBack - correction,
                    baseRightBack + correction);

            telemetry.addData("Step", step);
            telemetry.addData("Mode", mode);
            telemetry.addData("Last leg", legResult);
            telemetry.addData("Leg timeout", "%.1f s", timeoutSeconds);
            telemetry.addData("Target heading", "%.1f deg", heading);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addData("Error", "%.1f deg", headingError);
            telemetry.addData("Targets", "LF %d  RF %d",
                    leftFront.getTargetPosition(), rightFront.getTargetPosition());
            telemetry.addData("Targets", "LB %d  RB %d",
                    leftBack.getTargetPosition(), rightBack.getTargetPosition());
            telemetry.addData("Actual", "LF %d  RF %d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.addData("Actual", "LB %d  RB %d",
                    leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            telemetry.update();
        }

        if (!opModeIsActive()) {
            legResult = mode + ": stopped";
        } else if (motorsBusy()) {
            legResult = String.format("%s: TIMEOUT at %.1f s, leg cut short", mode, timeoutSeconds);
        } else {
            legResult = String.format("%s: arrived in %.1f s", mode, timer.seconds());
        }

        setWheelPowers(0.0, 0.0, 0.0, 0.0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnToHeading(double maxSpeed, double heading) {
        if (!opModeIsActive()) {
            return;
        }

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();
        getSteeringCorrection(heading, P_TURN_GAIN);

        while (opModeIsActive()
                && Math.abs(headingError) > HEADING_THRESHOLD
                && timer.seconds() < TURN_TIMEOUT_SECONDS) {

            double correction = getSteeringCorrection(heading, P_TURN_GAIN);
            correction = Range.clip(correction, -maxSpeed, maxSpeed);

            setTurnPower(correction);

            telemetry.addData("Step", step);
            telemetry.addData("Mode", "turning");
            telemetry.addData("Last leg", legResult);
            telemetry.addData("Target heading", "%.1f deg", heading);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addData("Error", "%.1f deg", headingError);
            telemetry.update();
        }

        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    private void holdHeading(double maxSpeed, double heading, double seconds) {
        if (!opModeIsActive()) {
            return;
        }

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();

        while (opModeIsActive() && timer.seconds() < seconds) {
            double correction = getSteeringCorrection(heading, P_TURN_GAIN);
            correction = Range.clip(correction, -maxSpeed, maxSpeed);

            setTurnPower(correction);

            telemetry.addData("Step", step);
            telemetry.addData("Mode", "holding");
            telemetry.addData("Last leg", legResult);
            telemetry.addData("Target heading", "%.1f deg", heading);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addData("Error", "%.1f deg", headingError);
            telemetry.update();
        }

        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    private double getSteeringCorrection(double targetHeading, double gain) {
        headingError = normalize(targetHeading - getHeading());
        return Range.clip(headingError * gain, -1.0, 1.0);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double normalize(double degrees) {
        degrees %= 360.0;

        if (degrees > 180.0) {
            degrees -= 360.0;
        } else if (degrees < -180.0) {
            degrees += 360.0;
        }

        return degrees;
    }

    private void setTurnPower(double correction) {
        setWheelPowers(-correction, correction, -correction, correction);
    }

    private void setWheelPowers(double leftFrontPower, double rightFrontPower,
                                double leftBackPower, double rightBackPower) {
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    private void stopMotors() {
        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    private boolean motorsBusy() {
        return leftFront.isBusy() || leftBack.isBusy()
                || rightFront.isBusy() || rightBack.isBusy();
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
