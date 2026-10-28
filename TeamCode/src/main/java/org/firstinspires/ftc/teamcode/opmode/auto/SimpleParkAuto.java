package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE_BLOCKER;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.TRANSFER;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.teleOp.RobotTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Simple Park Auto", group = "Meet0")
public class SimpleParkAuto extends LinearOpMode {

    private static final Pose START = FieldPoints.START_POSE;

    private List<Step> route() {
        List<Step> steps = new ArrayList<>();

        steps.add(step("Drive to shoot 1")
                .from(FieldPoints.START_POSE)
                .to(FieldPoints.SHOOT_POSE_1)
                .shoot(1.5)
                .pause(0.5));

        steps.add(step("Go to angle 1")
                .from(FieldPoints.SHOOT_POSE_1)
                .to(FieldPoints.ANGLE_1));

        steps.add(step("Park")
                .from(FieldPoints.ANGLE_1)
                .to(FieldPoints.PARK_POSE));

        return steps;
    }

    private static final double DEFAULT_TIMEOUT_SECONDS = FieldPoints.DEFAULT_PATH_TIMEOUT_SECONDS;
    private static final double AUTO_BUDGET_SECONDS = FieldPoints.AUTONOMOUS_BUDGET_SECONDS;
    private static final double MIN_PATH_INCHES = FieldPoints.MIN_PATH_LENGTH_INCHES;
    private static final double HEADING_TOLERANCE_DEGREES = 2.0;

    private static final DcMotorSimple.Direction INTAKE_DIRECTION = RobotTeleOp.INTAKE_DIRECTION;
    private static final DcMotorSimple.Direction TRANSFER_DIRECTION = RobotTeleOp.TRANSFER_DIRECTION;
    private static final DcMotorSimple.Direction OUTTAKE_DIRECTION = RobotTeleOp.OUTTAKE_DIRECTION;

    private static final double SHOOT_VELOCITY = RobotTeleOp.OUTTAKE_VELOCITY;
    private static final double SHOOT_TOLERANCE = RobotTeleOp.OUTTAKE_TOLERANCE;
    private static final double SPINUP_TIMEOUT_SECONDS = RobotTeleOp.SPINUP_TIMEOUT_SECONDS;
    private static final double FEED_TRANSFER_POWER = RobotTeleOp.FEED_POWER;
    private static final double FEED_INTAKE_POWER = 1.0;
    private static final double INTAKE_POWER = RobotTeleOp.INTAKE_POWER;

    private static final double BLOCKER_CLOSED_POSITION = RobotTeleOp.BLOCKER_CLOSED_POSITION;
    private static final double BLOCKER_OPEN_POSITION = RobotTeleOp.BLOCKER_OPEN_POSITION;

    private Follower follower;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo blocker;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private String lastResult = "none";

    @Override
    public void runOpMode() {
        List<Step> steps = route();
        for (int i = 0; i < steps.size(); i++) {
            if (steps.get(i).to == null) {
                throw new IllegalStateException("Step " + (i + 1) + " \"" + steps.get(i).name + "\" has no .to(...)");
            }
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START);

        intake = initRoller(INTAKE, INTAKE_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);
        transfer = initRoller(TRANSFER, TRANSFER_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        outtake = initOuttake(OUTTAKE, OUTTAKE_DIRECTION);
        blocker = hardwareMap.get(Servo.class, OUTTAKE_BLOCKER);

        while (opModeInInit()) {
            follower.localizer.update();
            telemetry.addLine("Simple Park Auto");
            telemetry.addData("Start", formatPose(START));
            telemetry.addData("Pose now", formatPose(follower.pose()));
            for (int i = 0; i < steps.size(); i++) {
                telemetry.addData("Step " + (i + 1), steps.get(i).summary());
            }
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        autoTimer.reset();
        blocker.setPosition(BLOCKER_CLOSED_POSITION);

        try {
            for (int i = 0; i < steps.size(); i++) {
                if (!runStep(i + 1, steps.size(), steps.get(i))) {
                    break;
                }
            }
        } finally {
            stopEverything();
        }

        telemetry.addData("Status", "Finished");
        telemetry.addData("Last result", lastResult);
        telemetry.addData("Final pose", formatPose(follower.pose()));
        telemetry.addData("Time used", "%.1f s", autoTimer.seconds());
        telemetry.update();
    }

    private boolean runStep(int number, int total, Step step) {
        String label = String.format("%d/%d %s", number, total, step.name);

        if (!drive(label, step)) {
            return false;
        }
        follower.hold(step.to);

        if (step.shootSeconds > 0 && !shoot(label, step.shootSeconds)) {
            return false;
        }
        if (step.intakeSeconds > 0 && !runIntake(label, step.intakeSeconds)) {
            return false;
        }
        if (step.pauseSeconds > 0 && !holdFor(label, "pause", step.pauseSeconds)) {
            return false;
        }

        lastResult = label + ": done";
        return true;
    }

    private boolean drive(String label, Step step) {
        Pose start = step.from != null ? step.from : follower.pose();

        if (step.controls.isEmpty() && start.distance(step.to) < MIN_PATH_INCHES) {
            return turnInPlace(label, step);
        }

        Path path;
        if (step.controls.isEmpty()) {
            path = Paths.line(start, step.to);
        } else {
            Pose[] points = new Pose[step.controls.size() + 2];
            points[0] = start;
            for (int i = 0; i < step.controls.size(); i++) {
                points[i + 1] = step.controls.get(i);
            }
            points[points.length - 1] = step.to;
            path = Paths.curve(points);
        }
        follower.follow(path.linear(start, step.to));

        if (step.intakeWhileDriving) {
            intake.setPower(INTAKE_POWER);
        }

        ElapsedTime timer = new ElapsedTime();
        boolean timedOut = false;
        while (canRun() && (follower.following() || follower.isBusy())) {
            if (timer.seconds() >= step.timeoutSeconds) {
                timedOut = true;
                break;
            }
            follower.update();
            telemetry.addData("Step", label);
            telemetry.addData("Phase", "driving");
            telemetry.addData("Target", formatPose(step.to));
            telemetry.addData("Pose now", formatPose(follower.pose()));
            telemetry.addData("Remaining", "%.1f in", follower.remainingDistance());
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), step.timeoutSeconds);
            telemetry.update();
        }

        if (step.intakeWhileDriving) {
            intake.setPower(0.0);
        }

        if (timedOut) {
            lastResult = String.format("%s: TIMEOUT after %.1f s", label, timer.seconds());
            follower.hold(follower.pose());
            follower.update();
            return false;
        }
        if (!canRun()) {
            lastResult = label + ": stopped";
            return false;
        }
        return true;
    }

    private boolean turnInPlace(String label, Step step) {
        follower.hold(step.to);
        if (step.intakeWhileDriving) {
            intake.setPower(INTAKE_POWER);
        }

        ElapsedTime timer = new ElapsedTime();
        boolean timedOut = false;
        while (canRun() && headingErrorDegrees(step.to) > HEADING_TOLERANCE_DEGREES) {
            if (timer.seconds() >= step.timeoutSeconds) {
                timedOut = true;
                break;
            }
            follower.update();
            telemetry.addData("Step", label);
            telemetry.addData("Phase", "turning");
            telemetry.addData("Heading error", "%.1f deg", headingErrorDegrees(step.to));
            telemetry.addData("Pose now", formatPose(follower.pose()));
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), step.timeoutSeconds);
            telemetry.update();
        }

        if (step.intakeWhileDriving) {
            intake.setPower(0.0);
        }

        if (timedOut) {
            lastResult = String.format("%s: TURN TIMEOUT after %.1f s", label, timer.seconds());
            return false;
        }
        if (!canRun()) {
            lastResult = label + ": stopped";
            return false;
        }
        return true;
    }

    private double headingErrorDegrees(Pose target) {
        double error = target.heading() - follower.pose().heading();
        return Math.abs(Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error))));
    }

    private boolean shoot(String label, double feedSeconds) {
        setOuttakeVelocity(SHOOT_VELOCITY);

        ElapsedTime timer = new ElapsedTime();
        boolean upToSpeed = false;
        while (canRun() && timer.seconds() < SPINUP_TIMEOUT_SECONDS) {
            follower.update();
            upToSpeed = Math.abs(outtake.getVelocity() - SHOOT_VELOCITY) <= SHOOT_TOLERANCE;
            if (upToSpeed) {
                break;
            }
            showShootTelemetry(label, "spinning up", timer.seconds(), SPINUP_TIMEOUT_SECONDS);
        }
        if (!canRun()) {
            return false;
        }

        blocker.setPosition(BLOCKER_OPEN_POSITION);
        transfer.setPower(FEED_TRANSFER_POWER);
        intake.setPower(FEED_INTAKE_POWER);

        timer.reset();
        while (canRun() && timer.seconds() < feedSeconds) {
            follower.update();
            showShootTelemetry(label, upToSpeed ? "feeding" : "feeding (spin-up timed out)",
                    timer.seconds(), feedSeconds);
        }

        transfer.setPower(0.0);
        intake.setPower(0.0);
        blocker.setPosition(BLOCKER_CLOSED_POSITION);
        setOuttakeVelocity(0.0);
        return canRun();
    }

    private boolean runIntake(String label, double seconds) {
        intake.setPower(INTAKE_POWER);
        boolean ok = holdFor(label, "intaking", seconds);
        intake.setPower(0.0);
        return ok;
    }

    private boolean holdFor(String label, String phase, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (canRun() && timer.seconds() < seconds) {
            follower.update();
            telemetry.addData("Step", label);
            telemetry.addData("Phase", phase);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.addData("Pose now", formatPose(follower.pose()));
            telemetry.update();
        }
        return canRun();
    }

    private void showShootTelemetry(String label, String phase, double time, double limit) {
        telemetry.addData("Step", label);
        telemetry.addData("Phase", phase);
        telemetry.addData("Outtake", "target %.0f  measured %.0f",
                SHOOT_VELOCITY, outtake.getVelocity());
        telemetry.addData("Time", "%.1f / %.1f s", time, limit);
        telemetry.update();
    }

    private boolean canRun() {
        return opModeIsActive() && autoTimer.seconds() < AUTO_BUDGET_SECONDS;
    }

    private void stopEverything() {
        follower.stop();
        follower.update();
        intake.setPower(0.0);
        transfer.setPower(0.0);
        setOuttakeVelocity(0.0);
    }

    private void setOuttakeVelocity(double velocity) {
        outtake.setVelocity(velocity);
    }

    private DcMotor initRoller(String name, DcMotorSimple.Direction direction,
                               DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setPower(0.0);
        return motor;
    }

    private DcMotorEx initOuttake(String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    private static String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    private static Step step(String name) {
        return new Step(name);
    }

    private static final class Step {
        final String name;
        Pose from;
        Pose to;
        final List<Pose> controls = new ArrayList<>();
        double pauseSeconds = 0.0;
        double shootSeconds = 0.0;
        double intakeSeconds = 0.0;
        boolean intakeWhileDriving = false;
        double timeoutSeconds = DEFAULT_TIMEOUT_SECONDS;

        Step(String name) {
            this.name = name;
        }

        Step from(Pose pose) {
            from = pose;
            return this;
        }

        Step to(Pose pose) {
            to = pose;
            return this;
        }

        Step control(Pose... poses) {
            controls.addAll(Arrays.asList(poses));
            return this;
        }

        Step pause(double seconds) {
            pauseSeconds = seconds;
            return this;
        }

        Step shoot(double feedSeconds) {
            shootSeconds = feedSeconds;
            return this;
        }

        Step intake(double seconds) {
            intakeSeconds = seconds;
            return this;
        }

        Step intakeWhileDriving() {
            intakeWhileDriving = true;
            return this;
        }

        Step timeout(double seconds) {
            timeoutSeconds = seconds;
            return this;
        }

        String summary() {
            StringBuilder text = new StringBuilder(name);
            text.append(" -> ").append(formatPose(to));
            if (!controls.isEmpty()) text.append("  cp ").append(controls.size());
            if (intakeWhileDriving) text.append("  intake-drive");
            if (shootSeconds > 0) text.append(String.format("  shoot %.1fs", shootSeconds));
            if (intakeSeconds > 0) text.append(String.format("  intake %.1fs", intakeSeconds));
            if (pauseSeconds > 0) text.append(String.format("  pause %.1fs", pauseSeconds));
            return text.toString();
        }
    }
}
