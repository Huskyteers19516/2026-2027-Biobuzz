package org.firstinspires.ftc.teamtestcode.tankdriveauto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Tank Drive Auto (Square)", group = "Testing")
public class TankDriveAuto extends LinearOpMode {

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double SIDE_LENGTH_INCHES = 24.0;

    private static final double DRIVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.3;

    private static final double HEADING_THRESHOLD = 1.0;
    private static final double P_TURN_GAIN = 0.02;
    private static final double P_DRIVE_GAIN = 0.03;

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
            telemetry.addLine("Tank Drive Auto - Square");
            telemetry.addData("Side length", "%.1f in", SIDE_LENGTH_INCHES);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addLine("Place the robot, then press START.");
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        imu.resetYaw();

        double heading = 0.0;

        for (int side = 0; side < 4 && opModeIsActive(); side++) {
            telemetry.addData("Side", "%d of 4", side + 1);
            telemetry.update();

            driveStraight(DRIVE_SPEED, SIDE_LENGTH_INCHES, heading);

            heading = normalize(heading - 90.0);

            turnToHeading(TURN_SPEED, heading);
            holdHeading(TURN_SPEED, heading, SETTLE_SECONDS);
        }

        stopMotors();

        telemetry.addLine("Square complete. Back at start.");
        telemetry.addData("Final heading", "%.1f deg", getHeading());
        telemetry.update();
    }

    private void driveStraight(double maxSpeed, double inches, double heading) {
        if (!opModeIsActive()) {
            return;
        }

        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + moveCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + moveCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + moveCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + moveCounts);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double speed = Math.abs(maxSpeed);
        setDrivePower(speed, speed);

        while (opModeIsActive() && motorsBusy()) {
            double correction = getSteeringCorrection(heading, P_DRIVE_GAIN);

            if (inches < 0.0) {
                correction *= -1.0;
            }

            setDrivePower(speed - correction, speed + correction);

            telemetry.addData("Mode", "driving");
            telemetry.addData("Target heading", "%.1f deg", heading);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addData("Error", "%.1f deg", headingError);
            telemetry.update();
        }

        setDrivePower(0.0, 0.0);
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

            setDrivePower(-correction, correction);

            telemetry.addData("Mode", "turning");
            telemetry.addData("Target heading", "%.1f deg", heading);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addData("Error", "%.1f deg", headingError);
            telemetry.update();
        }

        setDrivePower(0.0, 0.0);
    }

    private void holdHeading(double maxSpeed, double heading, double seconds) {
        if (!opModeIsActive()) {
            return;
        }

        timer.reset();

        while (opModeIsActive() && timer.seconds() < seconds) {
            double correction = getSteeringCorrection(heading, P_TURN_GAIN);
            correction = Range.clip(correction, -maxSpeed, maxSpeed);

            setDrivePower(-correction, correction);

            telemetry.addData("Mode", "holding");
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addData("Error", "%.1f deg", headingError);
            telemetry.update();
        }

        setDrivePower(0.0, 0.0);
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

    private void setDrivePower(double leftPower, double rightPower) {
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
    }

    private void stopMotors() {
        setDrivePower(0.0, 0.0);
    }

    private boolean motorsBusy() {
        return leftFront.isBusy() && leftBack.isBusy()
                && rightFront.isBusy() && rightBack.isBusy();
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
