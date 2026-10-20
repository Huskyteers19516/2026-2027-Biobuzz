package org.firstinspires.ftc.teamtestcode.swervedrivetesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Swerve Servo Bench Test", group = "Testing")
public class SwerveServoBenchTest extends OpMode {

    private static final double START_TEST_POWER = 0.30;
    private static final double POWER_STEP = 0.05;
    private static final double DRIVE_TEST_POWER = 0.20;
    private static final double MOVED_THRESHOLD_DEGREES = 3.0;
    private static final double STICK_DEADZONE = 0.10;

    private CRServo steerServo;
    private AnalogInput steerEncoder;
    private DcMotor driveMotor;

    private double testPower = START_TEST_POWER;
    private double voltsMin = Double.MAX_VALUE;
    private double voltsMax = -Double.MAX_VALUE;
    private double angleAtPress = 0.0;
    private double maxTravelSeen = 0.0;
    private boolean wasCommanding = false;

    @Override
    public void init() {
        steerServo = hardwareMap.get(CRServo.class, "swerve_servo");
        steerEncoder = hardwareMap.get(AnalogInput.class, "swerve_encoder");
        driveMotor = hardwareMap.get(DcMotor.class, "swerve_motor");

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        steerServo.setPower(0.0);
        driveMotor.setPower(0.0);
    }

    @Override
    public void init_loop() {
        double volts = steerEncoder.getVoltage();
        track(volts);

        telemetry.addLine("SWERVE SERVO BENCH TEST");
        telemetry.addLine("Nothing is powered during INIT.");
        telemetry.addData("Encoder volts", "%.3f V", volts);
        telemetry.addData("Raw angle", "%.1f deg", angleOf(volts));
        telemetry.addData("Volts seen", "%.3f - %.3f V", voltsMin, voltsMax);
        telemetry.addLine();
        telemetry.addLine("TEST 1 - turn the module by hand now.");
        telemetry.addLine("Volts must change. If not, the encoder is the fault.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double volts = steerEncoder.getVoltage();
        double angle = angleOf(volts);
        track(volts);

        if (gamepad1.dpadUpWasPressed()) {
            testPower = Range.clip(testPower + POWER_STEP, 0.0, 1.0);
        }
        if (gamepad1.dpadDownWasPressed()) {
            testPower = Range.clip(testPower - POWER_STEP, 0.0, 1.0);
        }

        double servoPower = 0.0;
        String mode = "idle (all power off)";
        double stick = gamepad1.left_stick_x;

        if (gamepad1.a) {
            mode = "hand mode - all power off";
        } else if (gamepad1.b) {
            servoPower = testPower;
            mode = "servo + power";
        } else if (gamepad1.x) {
            servoPower = -testPower;
            mode = "servo - power";
        } else if (Math.abs(stick) > STICK_DEADZONE) {
            servoPower = stick;
            mode = "manual steer from left stick";
        }

        boolean commanding = servoPower != 0.0;
        if (commanding && !wasCommanding) {
            angleAtPress = angle;
            maxTravelSeen = 0.0;
        }
        if (commanding) {
            maxTravelSeen = Math.max(maxTravelSeen, Math.abs(normalize(angle - angleAtPress)));
        }
        wasCommanding = commanding;

        double drivePower = gamepad1.right_trigger > 0.1 ? DRIVE_TEST_POWER : 0.0;

        steerServo.setPower(servoPower);
        driveMotor.setPower(drivePower);

        telemetry.addLine("SWERVE SERVO BENCH TEST - open loop, no PID");
        telemetry.addData("Mode", mode);
        telemetry.addData("Servo power", "%.2f", servoPower);
        telemetry.addData("Test power", "%.2f  (dpad up/down)", testPower);
        telemetry.addLine();
        telemetry.addData("Encoder volts", "%.3f V", volts);
        telemetry.addData("Raw angle", "%.1f deg", angle);
        telemetry.addData("Volts seen", "%.3f - %.3f V", voltsMin, voltsMax);

        if (commanding) {
            telemetry.addData("Moved since press", "%.1f deg", maxTravelSeen);
            telemetry.addLine(maxTravelSeen > MOVED_THRESHOLD_DEGREES
                    ? "ENCODER IS MOVING while the servo is powered"
                    : "NO ENCODER MOVEMENT - watch the wheel itself");
        }

        telemetry.addLine();
        telemetry.addData("Left stick x", "%.2f", stick);
        telemetry.addLine("Left stick L/R steers directly, no feedback loop.");
        telemetry.addLine("A hand mode | B servo+ | X servo- | trigger drive motor");
        telemetry.addLine("TEST 2: hold B. Does the WHEEL physically turn?");
        telemetry.update();
    }

    @Override
    public void stop() {
        steerServo.setPower(0.0);
        driveMotor.setPower(0.0);
    }

    private void track(double volts) {
        voltsMin = Math.min(voltsMin, volts);
        voltsMax = Math.max(voltsMax, volts);
    }

    private double angleOf(double volts) {
        double max = steerEncoder.getMaxVoltage();
        return max > 0.0 ? volts / max * 360.0 : 0.0;
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
}
