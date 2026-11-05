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




package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public final class FrankAutoDesign {

    private FrankAutoDesign() { }

    public enum Action {
        MOVE, INTAKE, SHOOT, PARK
    }

    public static Pose startPose() {
        return FieldPoints.START_POSE;
    }

    public static double budgetSeconds() {
        return FieldPoints.AUTONOMOUS_BUDGET_SECONDS;
    }

    // Advisory review status; an executor must explicitly check it.
    public static boolean isReviewed() {
        return FieldPoints.POINTS_CONFIRMED
                && FieldPoints.GOALS_CONFIRMED
                && FieldPoints.PARK_ROUTE_CONFIRMED;
    }

    public static List<Step> steps() {
        List<Step> route = new ArrayList<>();

        route.add(new Step(
                "Drive to shooting point 1 and shoot preload",
                FieldPoints.START_POSE,
                FieldPoints.SHOOT_POSE_1,
                Action.SHOOT,
                FieldPoints.SHOTS_AT_SHOOT_POSE_1,
                false, 0.0,
                FieldPoints.DEFAULT_PATH_TIMEOUT_SECONDS));

        route.add(new Step(
                "Move to flower 1 start",
                FieldPoints.SHOOT_POSE_1,
                FieldPoints.FLOWER_1_START,
                Action.MOVE, 0, false, 0.0,
                FieldPoints.DEFAULT_PATH_TIMEOUT_SECONDS));

        route.add(new Step(
                "Intake along flower 1 and briefly hold at its end",
                FieldPoints.FLOWER_1_START,
                FieldPoints.FLOWER_1_END,
                Action.INTAKE, 0, true,
                FieldPoints.INTAKE_SECONDS_AT_FLOWER,
                FieldPoints.DEFAULT_PATH_TIMEOUT_SECONDS));

        route.add(new Step(
                "Return to shooting point 1 and shoot collected artifact",
                FieldPoints.FLOWER_1_END,
                FieldPoints.SHOOT_POSE_1,
                Action.SHOOT,
                FieldPoints.SHOTS_AFTER_FLOWER,
                false, 0.0,
                FieldPoints.DEFAULT_PATH_TIMEOUT_SECONDS));

        route.add(new Step(
                "Move to candidate rack bypass waypoint 1",
                FieldPoints.SHOOT_POSE_1,
                FieldPoints.RACK_AROUND_1,
                Action.MOVE, 0, false, 0.0,
                FieldPoints.PARK_SEGMENT_TIMEOUT_SECONDS));

        route.add(new Step(
                "Move to candidate rack bypass waypoint 2",
                FieldPoints.RACK_AROUND_1,
                FieldPoints.RACK_AROUND_2,
                Action.MOVE, 0, false, 0.0,
                FieldPoints.PARK_SEGMENT_TIMEOUT_SECONDS));

        route.add(new Step(
                "Move to the designated park pose",
                FieldPoints.RACK_AROUND_2,
                FieldPoints.PARK_POSE,
                Action.PARK, 0, false, 0.0,
                FieldPoints.PARK_SEGMENT_TIMEOUT_SECONDS));

        return Collections.unmodifiableList(route);
    }

    public static final class Step {
        public final String name;
        public final Pose from;
        public final Pose to;

        // MOVE/PARK: drive only. SHOOT: drive, then aim and shoot.
        // INTAKE: intake while driving if enabled, then hold for the dwell.
        public final Action action;
        public final int shots;
        public final boolean intakeWhileDriving;
        public final double intakeDwellSeconds;
        public final double pathTimeoutSeconds;

        private Step(String name, Pose from, Pose to, Action action, int shots,
                     boolean intakeWhileDriving, double intakeDwellSeconds,
                     double pathTimeoutSeconds) {
            this.name = name;
            this.from = from;
            this.to = to;
            this.action = action;
            this.shots = shots;
            this.intakeWhileDriving = intakeWhileDriving;