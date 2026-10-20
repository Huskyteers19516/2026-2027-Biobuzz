package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Frank Auto", group = "Auto")
public class FrankAuto extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;


    private static final double FORWARD_INCHES = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        follower follower = Constants.createFollower(hardwareMap);

        Pose start = new Pose(0.0, 0.0, Math.toRadians(0.0));
        follower.setPose(start);

        telemetry.addData("Forward distance (inches)", FORWARD_INCHES);
        telemetry.addLine("Zero distance means no movement.");
        telemetry.update();

        waitForStart();

        if (isStopRequested() || FORWARD_INCHES <= 0.0) {
            return;
        }

        Pose end = new Pose(
                FORWARD_INCHES, 0.0, Math.toRadians(0.0));

        try{
            follower.follow(Paths.line(start, end).constant(start));

            resetRuntime();

            while (opModeIsActive()
                    && follower.isBusy()
                    && getRuntime() < 10) {

                follower.update();
                idle();
            }
        } finally {
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
        }
    }
}


package org.firstinspires.ftc.teamtestcode.autodesign;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Independent, robot-centric mecanum TeleOp.
 * Hardware names and motor directions follow the team's supplied configuration.
 * This OpMode does not run Pedro or require a Pinpoint localizer.
 */
@TeleOp(name = "Frank Drive Control", group = "Test")
public class FrankDriveControl extends LinearOpMode {

    private static final double NORMAL_SPEED = 0.50;
    private static final double SLOW_SPEED = 0.25;
    private static final double STICK_DEADBAND = 0.05;
    private static final double POWER_CHANGE_PER_SECOND = 2.0;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double appliedFrontLeft;
    private double appliedFrontRight;
    private double appliedBackLeft;
    private double appliedBackRight;
    private boolean emergencyStopped;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initializeDrive();

            telemetry.addLine("Left stick: forward / strafe");
            telemetry.addLine("Right stick X: turn");
            telemetry.addLine("Hold left bumper: slow mode");
            telemetry.addLine("B: latch stop; center sticks and press A to reset");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            ElapsedTime cycleTimer = new ElapsedTime();
            cycleTimer.reset();

            while (opModeIsActive()) {
                double dt = Range.clip(cycleTimer.seconds(), 0.0, 0.10);
                cycleTimer.reset();

                double forward = applyDeadband(-gamepad1.left_stick_y);
                double strafe = applyDeadband(gamepad1.left_stick_x);
                double turn = applyDeadband(gamepad1.right_stick_x);
                double speed = gamepad1.left_bumper ? SLOW_SPEED : NORMAL_SPEED;

                // B has priority over A, including when both are held.
                if (gamepad1.b) {
                    emergencyStopped = true;
                } else if (gamepad1.a && sticksCentered()) {
                    emergencyStopped = false;
                }

                if (emergencyStopped) {
                    stopDrive();
                } else {
                    drive(forward, strafe, turn, speed, dt);
                }

                displayStatus(speed);
                idle();
            }
        } finally {

            stopDrive();
        }
    }

    private void initializeDrive() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        configureMotor(frontLeft, DcMotorSimple.Direction.FORWARD);
        configureMotor(backLeft, DcMotorSimple.Direction.FORWARD);
        configureMotor(frontRight, DcMotorSimple.Direction.REVERSE);
        configureMotor(backRight, DcMotorSimple.Direction.REVERSE);

        emergencyStopped = false;
        stopDrive();
    }
}