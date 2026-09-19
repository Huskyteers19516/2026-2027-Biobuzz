package org.firstinspires.ftc.teamtestcode.pedropathing.swerve;

import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Swerve Pod Calibration", group = "Pedro")
public class SwervePodCalibration extends OpMode {

    private static final double FORWARD_WHEEL_THETA_RAD = Math.PI / 2.0;
    private static final double SERVO_TEST_POWER = 0.2;
    private static final double DRIVE_TEST_MAX_POWER = 0.3;
    private static final double MOVEMENT_THRESHOLD_DEG = 0.5;
    private static final double ALIGNED_TOLERANCE_DEG = 3.0;

    private CoaxialPod leftPod;
    private CoaxialPod rightPod;
    private AnalogInput leftEncoder;
    private AnalogInput rightEncoder;

    private double leftPreviousRawRad = 0.0;
    private double rightPreviousRawRad = 0.0;
    private double leftNetChangeDeg = 0.0;
    private double rightNetChangeDeg = 0.0;
    private boolean wasMeasuring = false;

    private double leftMinVoltage = Double.MAX_VALUE;
    private double leftMaxVoltage = -Double.MAX_VALUE;
    private double rightMinVoltage = Double.MAX_VALUE;
    private double rightMaxVoltage = -Double.MAX_VALUE;

    @Override
    public void init() {
        leftPod = SwerveConstants.createLeftPod(hardwareMap);
        rightPod = SwerveConstants.createRightPod(hardwareMap);
        leftEncoder = hardwareMap.get(AnalogInput.class, SwerveConstants.LEFT_ENCODER_NAME);
        rightEncoder = hardwareMap.get(AnalogInput.class, SwerveConstants.RIGHT_ENCODER_NAME);

        leftPreviousRawRad = leftPod.getRawAngleRad();
        rightPreviousRawRad = rightPod.getRawAngleRad();
    }

    @Override
    public void init_loop() {
        updateVoltageRange();
        telemetry.addLine("Swerve Pod Calibration (wheels OFF the ground)");
        telemetry.addLine("No button: all power OFF");
        telemetry.addLine("Hold A: power off, turn wheels by hand");
        telemetry.addLine("Hold B: servos spin with small + power");
        telemetry.addLine("Hold X: pods hold forward, right trigger = slow drive");
        telemetry.addLine("Back: reset the seen min/max voltage");
        addPodTelemetry("LEFT", leftPod, leftEncoder, leftNetChangeDeg, leftMinVoltage, leftMaxVoltage);
        addPodTelemetry("RIGHT", rightPod, rightEncoder, rightNetChangeDeg, rightMinVoltage, rightMaxVoltage);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.back) {
            resetVoltageRange();
        }
        updateVoltageRange();

        boolean measuring = gamepad1.a || gamepad1.b;

        if (measuring && !wasMeasuring) {
            leftNetChangeDeg = 0.0;
            rightNetChangeDeg = 0.0;
            leftPreviousRawRad = leftPod.getRawAngleRad();
            rightPreviousRawRad = rightPod.getRawAngleRad();
        }
        wasMeasuring = measuring;

        if (measuring) {
            leftNetChangeDeg += rawChangeDeg(leftPod.getRawAngleRad(), leftPreviousRawRad);
            rightNetChangeDeg += rawChangeDeg(rightPod.getRawAngleRad(), rightPreviousRawRad);
        }
        leftPreviousRawRad = leftPod.getRawAngleRad();
        rightPreviousRawRad = rightPod.getRawAngleRad();

        if (gamepad1.a) {
            setAllPower(0.0, 0.0);
            telemetry.addLine(">>> HAND MODE: power is off <<<");
            telemetry.addLine("1) Spin each wheel one full turn slowly.");
            telemetry.addLine("   Copy 'seen min/max V' into <POD>_ANALOG_MIN/MAX_VOLTAGE.");
            telemetry.addLine("2) Turn each wheel COUNTER-CLOCKWISE (seen from above).");
            telemetry.addLine("   Raw INCREASING -> ENCODER_REVERSED = true");
            telemetry.addLine("   Raw DECREASING -> ENCODER_REVERSED = false");
            telemetry.addLine("3) Point each wheel straight FORWARD.");
            telemetry.addLine("   Copy the matching 'offset' line into <POD>_POD_ANGLE_OFFSET_RAD.");
            addDirectionHint("LEFT", leftNetChangeDeg, true);
            addDirectionHint("RIGHT", rightNetChangeDeg, true);
        } else if (gamepad1.b) {
            setAllPower(SERVO_TEST_POWER, 0.0);
            telemetry.addLine(">>> SERVO TEST: +" + SERVO_TEST_POWER + " servo power <<<");
            telemetry.addLine("Raw must INCREASE with + power.");
            telemetry.addLine("If it DECREASES, flip <POD>_SERVO_DIRECTION.");
            addDirectionHint("LEFT", leftNetChangeDeg, false);
            addDirectionHint("RIGHT", rightNetChangeDeg, false);
        } else if (gamepad1.x) {
            double drivePower = gamepad1.right_trigger * DRIVE_TEST_MAX_POWER;
            leftPod.move(FORWARD_WHEEL_THETA_RAD, drivePower, false);
            rightPod.move(FORWARD_WHEEL_THETA_RAD, drivePower, false);
            telemetry.addLine(">>> HOLD FORWARD <<<");
            telemetry.addLine("Both wheels should point forward (angle after offset ~0 or ~180).");
            telemetry.addLine("Right trigger: both wheels must roll the robot FORWARD.");
            telemetry.addLine("If one rolls backward, flip its <POD>_DRIVE_DIRECTION.");
            telemetry.addData("Drive power", "%.2f", drivePower);
        } else {
            setAllPower(0.0, 0.0);
            telemetry.addLine(">>> IDLE: all power off <<<");
            telemetry.addLine("A = hand mode | B = servo test | X = hold forward");
        }

        addPodTelemetry("LEFT", leftPod, leftEncoder, leftNetChangeDeg, leftMinVoltage, leftMaxVoltage);
        addPodTelemetry("RIGHT", rightPod, rightEncoder, rightNetChangeDeg, rightMinVoltage, rightMaxVoltage);
        telemetry.update();
    }

    @Override
    public void stop() {
        setAllPower(0.0, 0.0);
    }

    private void setAllPower(double servoPower, double drivePower) {
        leftPod.setServoPower(servoPower);
        rightPod.setServoPower(servoPower);
        leftPod.setMotorPower(drivePower);
        rightPod.setMotorPower(drivePower);
    }

    private void updateVoltageRange() {
        double leftVoltage = leftEncoder.getVoltage();
        double rightVoltage = rightEncoder.getVoltage();
        leftMinVoltage = Math.min(leftMinVoltage, leftVoltage);
        leftMaxVoltage = Math.max(leftMaxVoltage, leftVoltage);
        rightMinVoltage = Math.min(rightMinVoltage, rightVoltage);
        rightMaxVoltage = Math.max(rightMaxVoltage, rightVoltage);
    }

    private void resetVoltageRange() {
        leftMinVoltage = Double.MAX_VALUE;
        leftMaxVoltage = -Double.MAX_VALUE;
        rightMinVoltage = Double.MAX_VALUE;
        rightMaxVoltage = -Double.MAX_VALUE;
    }

    private void addPodTelemetry(String label, CoaxialPod pod, AnalogInput encoder, double netChangeDeg,
                                 double minVoltage, double maxVoltage) {
        double rawRad = pod.getRawAngleRad();
        double offsetRad = pod.getOffsetAngleRad();
        double offsetSignedDeg = Math.toDegrees(Angle.normalizeSigned(offsetRad));
        boolean aligned = Math.abs(offsetSignedDeg) < ALIGNED_TOLERANCE_DEG
                || Math.abs(Math.abs(offsetSignedDeg) - 180.0) < ALIGNED_TOLERANCE_DEG;

        telemetry.addLine("---- " + label + " POD ----");
        telemetry.addData(label + " voltage", "%.3f V", encoder.getVoltage());
        telemetry.addData(label + " seen min/max V", "%.3f / %.3f", minVoltage, maxVoltage);
        telemetry.addData(label + " raw", "%.1f deg | %.4f rad", Math.toDegrees(rawRad), rawRad);
        telemetry.addData(label + " offset if ENCODER_REVERSED=false", "%.4f rad", rawRad);
        telemetry.addData(label + " offset if ENCODER_REVERSED=true", "%.4f rad", Angle.normalize(rawRad + Math.PI));
        telemetry.addData(label + " after offset", "%.1f deg | %.4f rad", offsetSignedDeg, Angle.normalizeSigned(offsetRad));
        telemetry.addData(label + " aligned fwd/back", aligned ? "YES" : "no");
        telemetry.addData(label + " net change", "%.1f deg", netChangeDeg);
    }

    private void addDirectionHint(String label, double netChangeDeg, boolean handMode) {
        if (Math.abs(netChangeDeg) < MOVEMENT_THRESHOLD_DEG) {
            telemetry.addData(label, "not moving yet");
            return;
        }

        boolean increasing = netChangeDeg > 0;
        if (handMode) {
            telemetry.addData(label, "raw %s -> if you turned CCW: ENCODER_REVERSED = %s",
                    increasing ? "INCREASING" : "DECREASING", increasing ? "true" : "false");
        } else {
            telemetry.addData(label, "raw %s -> SERVO_DIRECTION %s",
                    increasing ? "INCREASING" : "DECREASING", increasing ? "OK" : "must be flipped");
        }
    }

    private double rawChangeDeg(double currentRad, double previousRad) {
        return Math.toDegrees(Angle.normalizeSigned(currentRad - previousRad));
    }
}
