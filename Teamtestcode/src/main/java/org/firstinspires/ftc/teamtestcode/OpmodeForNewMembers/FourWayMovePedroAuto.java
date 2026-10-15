package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamtestcode.pedropathing.Constants;

import java.util.Locale;
import java.util.Map;

// Uses only the four drive motors: position comes from the wheel encoders.
@Autonomous(name = "Four Way Move Pedro Auto", group = "Testing")
public class FourWayMovePedroAuto extends LinearOpMode {

    Follower follower;
    ElapsedTime timer = new ElapsedTime();

    // heading 0: +x is forward, +y is left
    Pose start = new Pose(0.0, 0.0, 0.0);
    Pose front = new Pose(24.0, 0.0, 0.0);
    Pose right = new Pose(0.0, -24.0, 0.0);
    Pose end = new Pose(6.0, 0.0, 0.0);

    double maxTime = 6;
    double pause = 0.5;

    // same cap as Costume Bot Drive; the costume bot tips over when it goes too fast
    double maxPower = 0.4;

    // safety limits: stop everything if the robot does something it should not
    double maxOffLine = 6.0;       // inches away from the straight line to the target
    double noMoveTime = 1.5;       // seconds without moving 1 inch = encoders not counting

    boolean stopped = false;
    String stopReason = "";

    @Override
    public void runOpMode() {
        try {
            follower = new Follower(new WheelLocalizer(hardwareMap),
                    new CappedDrive(Constants.createDrivetrain(hardwareMap), maxPower),
                    Constants.createAlgorithm());
        } catch (RuntimeException e) {
            // usually: a drive motor is missing from the robot config
            while (!isStopRequested()) {
                telemetry.addLine("CANNOT START: drive motor missing from the config");
                telemetry.addLine("needs left_front, left_back, right_front, right_back");
                telemetry.addData("error", e.getMessage());
                telemetry.update();
            }
            return;
        }
        follower.setPose(start);

        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addLine("ready");
            telemetry.addLine("forward, back, right, left, then forward a little");
            telemetry.addLine("check before START by pushing the robot by hand:");
            telemetry.addLine("  push forward -> x goes up");
            telemetry.addLine("  push left -> y goes up");
            telemetry.addData("pose", poseText(follower.pose()));
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        follower.setPose(start);

        goTo(front);
        waitTime(pause);
        goTo(start);
        waitTime(pause);
        goTo(right);
        waitTime(pause);
        goTo(start);
        waitTime(pause);
        goTo(end);

        follower.stop();
        follower.update();

        while (opModeIsActive()) {
            if (stopped) {
                telemetry.addLine("STOPPED: " + stopReason);
            } else {
                telemetry.addLine("done");
            }
            telemetry.addData("pose", poseText(follower.pose()));
            telemetry.update();
            idle();
        }
    }

    void goTo(Pose target) {
        if (stopped) {
            return;
        }

        Pose here = follower.pose();
        Path path = Paths.line(here, target).linear(here, target);
        follower.follow(path);

        timer.reset();
        while (opModeIsActive() && timer.seconds() < maxTime
                && (follower.following() || follower.isBusy())) {
            follower.update();

            String problem = safetyProblem(here, target, follower.pose(), timer.seconds());
            if (problem != null) {
                stopped = true;
                stopReason = problem;
                follower.stop();
                follower.update();
                return;
            }

            telemetry.addData("target", poseText(target));
            telemetry.addData("pose", poseText(follower.pose()));
            telemetry.addData("time", "%.1f", timer.seconds());
            telemetry.update();
        }

        follower.hold(timer.seconds() < maxTime ? target : follower.pose());
        follower.update();
    }

    String safetyProblem(Pose from, Pose to, Pose now, double seconds) {
        if (Double.isNaN(now.x()) || Double.isNaN(now.y()) || Double.isNaN(now.heading())) {
            return "wheel encoders give no position";
        }

        double dx = to.x() - from.x();
        double dy = to.y() - from.y();
        double length = Math.hypot(dx, dy);
        double moved = Math.hypot(now.x() - from.x(), now.y() - from.y());

        if (length > 0.1) {
            // distance from the straight line from -> to
            double offLine = Math.abs(dx * (now.y() - from.y()) - dy * (now.x() - from.x())) / length;
            if (offLine > maxOffLine) {
                return String.format(Locale.US, "drifted %.1f in off the path - check motor directions", offLine);
            }

            // how far along the line; below 0 = going the wrong way (a reversed pod
            // makes the robot speed up instead of slow down), above length = overshoot
            double along = (dx * (now.x() - from.x()) + dy * (now.y() - from.y())) / length;
            if (along < -maxOffLine) {
                return "moving away from the target - check motor directions";
            }
            if (along > length + maxOffLine) {
                return String.format(Locale.US, "overshot the target by %.1f in", along - length);
            }
        }

        if (length > 2.0 && seconds > noMoveTime && moved < 1.0) {
            return "position not changing - encoders not counting, or the robot is stuck";
        }

        return null;
    }

    void waitTime(double seconds) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            if (!stopped) {
                follower.update();
            }
        }
    }

    String poseText(Pose pose) {
        return String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    // Tracks the robot position from the four drive wheel encoders (mecanum kinematics).
    // Only x and y are tracked; heading stays where it was set, so the robot is not
    // turned back if it rotates a little. Wheels slip, so expect an inch or two
    // of error per move, more when strafing.
    static class WheelLocalizer implements Localizer {
        // goBILDA 312 rpm motor (537.7 ticks per turn) with 96 mm wheels, same as FourWayMoveAuto
        static final double TICKS_PER_INCH = 537.7 / (3.78 * Math.PI);
        // wheels turn about 15% more than the robot moves when strafing
        static final double STRAFE_FIX = 1.15;

        final DcMotor lf, lb, rf, rb;
        int lfLast, lbLast, rfLast, rbLast;
        Pose pose = Pose.zero();
        Velocity velocity = Velocity.zero();
        final ElapsedTime dtTimer = new ElapsedTime();

        WheelLocalizer(HardwareMap hardwareMap) {
            lf = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_MOTOR_NAME);
            lb = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_MOTOR_NAME);
            rf = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_MOTOR_NAME);
            rb = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_MOTOR_NAME);

            // same directions as the drivetrain, so a wheel rolling forward counts up
            lf.setDirection(DcMotorSimple.Direction.FORWARD);
            lb.setDirection(DcMotorSimple.Direction.FORWARD);
            rf.setDirection(DcMotorSimple.Direction.REVERSE);
            rb.setDirection(DcMotorSimple.Direction.REVERSE);

            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            reset();
        }

        @Override
        public void setPose(Pose newPose) {
            pose = newPose;
        }

        @Override
        public void update() {
            int lfNow = lf.getCurrentPosition();
            int lbNow = lb.getCurrentPosition();
            int rfNow = rf.getCurrentPosition();
            int rbNow = rb.getCurrentPosition();

            double dLf = (lfNow - lfLast) / TICKS_PER_INCH;
            double dLb = (lbNow - lbLast) / TICKS_PER_INCH;
            double dRf = (rfNow - rfLast) / TICKS_PER_INCH;
            double dRb = (rbNow - rbLast) / TICKS_PER_INCH;
            lfLast = lfNow;
            lbLast = lbNow;
            rfLast = rfNow;
            rbLast = rbNow;

            // robot frame: forward, left
            double forward = (dLf + dLb + dRf + dRb) / 4.0;
            double left = (-dLf + dLb + dRf - dRb) / 4.0 / STRAFE_FIX;

            double h = pose.heading();
            double dx = forward * Math.cos(h) - left * Math.sin(h);
            double dy = forward * Math.sin(h) + left * Math.cos(h);
            pose = new Pose(pose.x() + dx, pose.y() + dy, h);

            double dt = dtTimer.seconds();
            dtTimer.reset();
            if (dt > 1e-4) {
                velocity = new Velocity(dx / dt, dy / dt, 0.0);
            }
        }

        @Override
        public MotionState state() {
            return MotionState.ofVelocity(pose, velocity);
        }

        @Override
        public void reset() {
            lfLast = lf.getCurrentPosition();
            lbLast = lb.getCurrentPosition();
            rfLast = rf.getCurrentPosition();
            rbLast = rb.getCurrentPosition();
            pose = Pose.zero();
            velocity = Velocity.zero();
            dtTimer.reset();
        }
    }

    // Wraps Pedro's Mecanum so no wheel ever gets more than maxPower.
    static class CappedDrive implements Drivetrain {
        final Mecanum mecanum;
        final double maxPower;

        CappedDrive(Mecanum mecanum, double maxPower) {
            this.mecanum = mecanum;
            this.maxPower = maxPower;
        }

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            double biggest = 1.0;
            for (double p : mecanum.computeWheelPowersUnnormalized(powers)) {
                biggest = Math.max(biggest, Math.abs(p));
            }
            double scale = maxPower / biggest;
            mecanum.drive(new DrivePowers(powers.forward() * scale, powers.strafe() * scale,
                    powers.turn() * scale), manual);
        }

        @Override
        public double maxScaling(DrivePowers current, DrivePowers delta) {
            return mecanum.maxScaling(current, delta);
        }

        @Override
        public void stop() {
            mecanum.stop();
        }

        @Override
        public void stop(boolean brake) {
            mecanum.stop(brake);
        }

        @Override
        public Map<String, Object> debug() {
            return mecanum.debug();
        }

        @Override
        public double interpolateAcceleration(double xRadius, double yRadius, double theta) {
            return mecanum.interpolateAcceleration(xRadius, yRadius, theta);
        }

        @Override
        public double interpolateVelocity(double xRadius, double yRadius, double theta) {
            return mecanum.interpolateVelocity(xRadius, yRadius, theta);
        }
    }
}
