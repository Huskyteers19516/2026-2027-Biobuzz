package org.firstinspires.ftc.teamtestcode.swervedrivetesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Swerve Drive Testing (1 module)", group = "Testing")
public class swerveDriveTesting extends OpMode {

    private static final double ENCODER_OFFSET_DEGREES = 0.0;
    private static final boolean STEER_REVERSED = false;

    private static final double STEER_KP = 0.012;
    private static final double STEER_KD = 0.0006;
    private static final double STEER_KS = 0.05;
    private static final double STEER_TOLERANCE_DEGREES = 2.0;
    private static final double STEER_MAX_POWER = 1.0;

    private static final double STICK_DEADZONE = 0.15;
    private static final boolean SCALE_DRIVE_BY_ERROR = true;
    private static final double TRIM_STEP_DEGREES = 0.5;

    private DcMotor driveMotor;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private CRServo steerServo;
    private AnalogInput steerEncoder;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double heldAngle = 0.0;
    private double offsetTrim = ENCODER_OFFSET_DEGREES;
    private double previousError = 0.0;
    private boolean dpadWasPressed = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class ,"left_front");
        leftBack = hardwareMap.get(DcMotor.class,"left_back");
        rightFront = hardwareMap.get(DcMotor.class,"right_front");
        rightBack= hardwareMap.get(DcMotor.class,"right_back");
        driveMotor = hardwareMap.get(DcMotor.class, "swerve_motor");
        steerServo = hardwareMap.get(CRServo.class, "swerve_servo");
        steerEncoder = hardwareMap.get(AnalogInput.class, "swerve_encoder");

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        steerServo.setDirection(STEER_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Ready. Hold A to read the encoder for calibration.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        previousError = 0.0;
    }

    @Override
    public void loop() {
        double moduleAngle = readModuleAngle();

        if (gamepad1.a) {
            handleTrim();
            steerServo.setPower(0.0);
            driveMotor.setPower(0.0);
            heldAngle = moduleAngle;
            previousError = 0.0;

            telemetry.addLine(">>> CALIBRATION MODE <<<");
            telemetry.addLine("Point the wheel straight forward by hand.");
            telemetry.addData("Raw encoder", "%.1f deg", readRawAngle());
            telemetry.addData("dpad left/right", "trim the offset");
            telemetry.addData("Offset", "%.1f deg", offsetTrim);
            telemetry.addLine("When the wheel is straight, copy Raw encoder");
            telemetry.addLine("into ENCODER_OFFSET_DEGREES.");
            telemetry.addData("Reads zero now?", "%.1f deg", moduleAngle);
            telemetry.update();
            return;
        }

        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;

        double magnitude = Range.clip(Math.hypot(stickX, stickY), 0.0, 1.0);
        boolean stickReleased = magnitude < STICK_DEADZONE;

        double targetAngle = heldAngle;
        boolean reversed = false;

        if (!stickReleased) {
            targetAngle = Math.toDegrees(Math.atan2(stickX, stickY));

            if (Math.abs(normalize(targetAngle - moduleAngle)) > 90.0) {
                targetAngle = normalize(targetAngle + 180.0);
                reversed = true;
            }

            heldAngle = targetAngle;
        }

        double error = normalize(targetAngle - moduleAngle);

        handleTrim();

        double steerPower;
        double drivePower;

        if (stickReleased) {
            steerPower = 0.0;
            drivePower = 0.0;
            previousError = error;
        } else {
            steerPower = computeSteerPower(error);
            drivePower = reversed ? -magnitude : magnitude;

            if (SCALE_DRIVE_BY_ERROR) {
                drivePower *= Math.cos(Math.toRadians(error));
            }
        }

        steerServo.setPower(steerPower);
        driveMotor.setPower(drivePower);

        telemetry.addData("Stick", "x %.2f  y %.2f", stickX, stickY);
        telemetry.addData("Module angle", "%.1f deg", moduleAngle);
        telemetry.addData("Target angle", "%.1f deg", targetAngle);
        telemetry.addData("Error", "%.1f deg", error);
        telemetry.addData("Steer power", "%.2f", steerPower);
        telemetry.addData("Drive power", "%.2f%s", drivePower, reversed ? "  (reversed)" : "");
        telemetry.addData("Offset", "%.1f deg", offsetTrim);
        telemetry.update();
    }

    @Override
    public void stop() {
        driveMotor.setPower(0.0);
        steerServo.setPower(0.0);
    }

    private double readRawAngle() {
        return steerEncoder.getVoltage() / steerEncoder.getMaxVoltage() * 360.0;
    }

    private double readModuleAngle() {
        return normalize(readRawAngle() - offsetTrim);
    }

    private double computeSteerPower(double error) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (Math.abs(error) < STEER_TOLERANCE_DEGREES) {
            previousError = error;
            return 0.0;
        }

        double derivative = dt > 1e-4 ? (error - previousError) / dt : 0.0;
        previousError = error;

        double power = (STEER_KP * error)
                + (STEER_KD * derivative)
                + (STEER_KS * Math.signum(error));

        return Range.clip(power, -STEER_MAX_POWER, STEER_MAX_POWER);
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

    private void handleTrim() {
        boolean pressed = gamepad1.dpad_left || gamepad1.dpad_right;

        if (pressed && !dpadWasPressed) {
            offsetTrim = normalize(offsetTrim
                    + (gamepad1.dpad_right ? TRIM_STEP_DEGREES : -TRIM_STEP_DEGREES));
        }

        dpadWasPressed = pressed;
    }
}
