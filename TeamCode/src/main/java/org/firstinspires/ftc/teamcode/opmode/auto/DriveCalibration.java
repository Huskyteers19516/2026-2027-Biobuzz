package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive Calibration", group = "Test")
public class DriveCalibration extends LinearOpMode {

    private static final double CALIBRATION_DISTANCE_INCHES = 48.0;
    private static final double MIN_MEASURE_INCHES = 10.0;
    private static final int DIRECTION_CHECK_TICKS = 200;
    private static final double SIDE_REVERSED_INCHES = 2.0;
    private static final double TEST_DRIVE_POWER = 0.3;
    private static final double TEST_TURN_POWER = 0.25;
    private static final double SCALE_WARNING_LOW = 0.8;
    private static final double SCALE_WARNING_HIGH = 1.25;

    private static final int[] FORWARD_SIGNS = {1, 1, 1, 1};
    private static final int[] STRAFE_RIGHT_SIGNS = {1, -1, -1, 1};

    private WheelOdometry odometry;
    private boolean lastX = false;

    @Override
    public void runOpMode() {
        odometry = new WheelOdometry(hardwareMap);
        odometry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        odometry.reset(new Pose(0.0, 0.0, 0.0));

        while (!isStopRequested()) {
            if (gamepad1.x && !lastX) {
                stopDrive();
                odometry.reset(new Pose(0.0, 0.0, 0.0));
            }
            lastX = gamepad1.x;

            if (opModeIsActive()) {
                testDrive();
            }

            odometry.update();
            showTelemetry();
        }

        stopDrive();
    }

    private void testDrive() {
        double forward = 0.0;
        double strafe = 0.0;
        double turn = 0.0;
        if (gamepad1.dpad_up) forward += TEST_DRIVE_POWER;
        if (gamepad1.dpad_down) forward -= TEST_DRIVE_POWER;
        if (gamepad1.dpad_right) strafe += TEST_DRIVE_POWER;
        if (gamepad1.dpad_left) strafe -= TEST_DRIVE_POWER;
        if (gamepad1.right_bumper) turn += TEST_TURN_POWER;
        if (gamepad1.left_bumper) turn -= TEST_TURN_POWER;

        odometry.leftFront.setPower(forward + strafe + turn);
        odometry.leftBack.setPower(forward - strafe + turn);
        odometry.rightFront.setPower(forward - strafe - turn);
        odometry.rightBack.setPower(forward + strafe - turn);
    }

    private void stopDrive() {
        odometry.leftFront.setPower(0.0);
        odometry.leftBack.setPower(0.0);
        odometry.rightFront.setPower(0.0);
        odometry.rightBack.setPower(0.0);
    }

    private void showTelemetry() {
        double rawForward = odometry.rawForwardInches();
        double rawStrafe = odometry.rawStrafeInches();
        int[] ticks = odometry.ticksSinceReset();

        telemetry.addLine("DRIVE CALIBRATION  (gamepad1 X = zero)");
        telemetry.addData("Tape distance", "%.0f in", CALIBRATION_DISTANCE_INCHES);
        if (!odometry.imuReady()) {
            telemetry.addLine("!! IMU FAILED TO INITIALIZE");
        }
        telemetry.addLine();

        telemetry.addLine("1) FORWARD: press X, push the robot straight forward exactly the tape distance");
        telemetry.addData("Forward", "raw %.2f in  scaled %.2f in  (FORWARD_SCALE now %.4f)",
                rawForward, odometry.forwardInches(), WheelOdometry.FORWARD_SCALE);
        if (Math.abs(rawForward) >= MIN_MEASURE_INCHES) {
            telemetry.addData(">> Set FORWARD_SCALE", "%.4f", suggestedScale(rawForward));
            addScaleWarning(rawForward, "forward");
        }
        telemetry.addLine();

        telemetry.addLine("2) STRAFE: press X, push the robot straight RIGHT exactly the tape distance");
        telemetry.addData("Strafe", "raw %.2f in  scaled %.2f in  (STRAFE_SCALE now %.4f)",
                rawStrafe, odometry.strafeInches(), WheelOdometry.STRAFE_SCALE);
        if (Math.abs(rawStrafe) >= MIN_MEASURE_INCHES) {
            telemetry.addData(">> Set STRAFE_SCALE", "%.4f", suggestedScale(rawStrafe));
            addScaleWarning(rawStrafe, "strafe right");
        }
        telemetry.addLine();

        telemetry.addLine("3) HEADING: press X, turn the robot 90 deg LEFT by hand");
        telemetry.addData("IMU yaw", "%.1f deg  (should read about +90)", odometry.yawDegrees());
        telemetry.addLine();

        telemetry.addData("Wheel ticks since zero", "LF %d  LB %d  RF %d  RB %d",
                ticks[0], ticks[1], ticks[2], ticks[3]);
        addDirectionCheck(ticks, rawForward, rawStrafe);
        telemetry.addLine();

        telemetry.addLine("4) POWERED TEST (after START): dpad = drive 0.3, bumpers = turn");
        telemetry.addLine("Drive the tape distance with the dpad and compare to the tape - more = wheel slip.");
        telemetry.addLine("Put the >> values into WheelOdometry.java, then run this again to check.");
        telemetry.update();
    }

    private double suggestedScale(double rawInches) {
        return CALIBRATION_DISTANCE_INCHES / Math.abs(rawInches);
    }

    private void addScaleWarning(double rawInches, String direction) {
        if (rawInches < 0) {
            telemetry.addLine("!! Reading is NEGATIVE while pushing " + direction
                    + " - drive motor directions are wrong");
        }
        double scale = suggestedScale(rawInches);
        if (scale < SCALE_WARNING_LOW || scale > SCALE_WARNING_HIGH) {
            telemetry.addLine("!! Scale is far from 1.0 - check DRIVE_TICKS_PER_REV"
                    + " (312 rpm = 537.7, 435 rpm = 384.5) and WHEEL_DIAMETER_INCHES");
        }
    }

    private void addDirectionCheck(int[] ticks, double rawForward, double rawStrafe) {
        int moved = 0;
        for (int tick : ticks) {
            moved += Math.abs(tick);
        }
        if (moved < DIRECTION_CHECK_TICKS) {
            telemetry.addData("Direction check", "push the robot to check each wheel");
            return;
        }
        telemetry.addLine("Only scales from a direction check that says OK are valid.");
        telemetry.addData("If pushed FORWARD", wrongWheels(ticks, FORWARD_SIGNS));
        telemetry.addData("If pushed RIGHT", wrongWheels(ticks, STRAFE_RIGHT_SIGNS));
        if (Math.abs(rawForward) < SIDE_REVERSED_INCHES && Math.abs(rawStrafe) < SIDE_REVERSED_INCHES) {
            telemetry.addLine("!! Wheels turn but forward and strafe stay near 0 - one whole side is reversed");
        }
    }

    private static String wrongWheels(int[] ticks, int[] expected) {
        StringBuilder wrong = new StringBuilder();
        for (int i = 0; i < ticks.length; i++) {
            int sign = Integer.signum(ticks[i]);
            if (sign != expected[i]) {
                if (wrong.length() > 0) wrong.append(", ");
                wrong.append(WheelOdometry.WHEEL_NAMES[i]).append(sign == 0 ? " not counting" : " backwards");
            }
        }
        return wrong.length() == 0 ? "all 4 wheels OK" : "!! " + wrong;
    }
}
