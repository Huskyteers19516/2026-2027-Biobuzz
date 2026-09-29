package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Match Auto", group = "Competition")
public class MatchAuto extends LinearOpMode {

    private static final double OUTTAKE_VELOCITY = RobotTeleOp.OUTTAKE_VELOCITY;
    private static final double OUTTAKE_TOLERANCE = RobotTeleOp.OUTTAKE_TOLERANCE;
    private static final double SPINUP_TIMEOUT_SECONDS = RobotTeleOp.SPINUP_TIMEOUT_SECONDS;
    private static final double FEED_POWER = RobotTeleOp.FEED_POWER;
    private static final double FEED_SECONDS = RobotTeleOp.FEED_SECONDS;
    private static final boolean KEEP_OUTTAKE_SPINNING = RobotTeleOp.KEEP_OUTTAKE_SPINNING;
    private static final double INTAKE_POWER = RobotTeleOp.INTAKE_POWER;

    private static final double SHOT_GAP_SECONDS = 0.4;
    private static final double MAX_ACTION_SECONDS = 8.0;
    private static final double SETTLE_SECONDS_BEFORE_SHOOT = 0.2;

    private static final double AUTONOMOUS_BUDGET_SECONDS = FieldPoints.AUTONOMOUS_BUDGET_SECONDS;
    private static final double MIN_PATH_LENGTH_INCHES = FieldPoints.MIN_PATH_LENGTH_INCHES;

    private static final double AIM_DEFAULT_POSITION = RobotTeleOp.AIM_START_POSITION;

    private Follower follower;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo launcherServo;

    private final ElapsedTime runtime = new ElapsedTime();

    private String lastPathResult = "none";
    private String lastShotResult = "none";
    private String stopReason = "mission complete";

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, RobotTeleOp.INTAKE_NAME);
        transfer = hardwareMap.get(DcMotor.class, RobotTeleOp.TRANSFER_NAME);
        outtake = hardwareMap.get(DcMotorEx.class, RobotTeleOp.OUTTAKE_NAME);
        launcherServo = hardwareMap.get(Servo.class, RobotTeleOp.LAUNCHER_SERVO_NAME);

        intake.setDirection(RobotTeleOp.INTAKE_DIRECTION);
        transfer.setDirection(RobotTeleOp.TRANSFER_DIRECTION);
        outtake.setDirection(RobotTeleOp.OUTTAKE_DIRECTION);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower.setPose(FieldPoints.START_POSE);

        List<FieldPoints.Step> mission = FieldPoints.mission();

        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addLine("Match Auto");
            telemetry.addData("Steps", mission.size());
            telemetry.addData("Budget", "%.1f s", AUTONOMOUS_BUDGET_SECONDS);
            telemetry.addData("Start pose", formatPose(FieldPoints.START_POSE));
            telemetry.addData("Measured pose", formatPose(follower.pose()));
            telemetry.addData("Localizer", Constants.PINPOINT_NAME);
            telemetry.addData("Outtake ticks", outtake.getCurrentPosition());
            if (!FieldPoints.POINTS_CONFIRMED) {
                telemetry.addLine("!! FIELD POINTS NOT CONFIRMED - AUTO WILL NOT MOVE");
                telemetry.addLine("Measure the real coordinates in FieldPoints.java,");
                telemetry.addLine("then set POINTS_CONFIRMED = true.");
            }

            telemetry.addLine("Nothing is commanded until START");
            telemetry.update();
        }

        if (!FieldPoints.POINTS_CONFIRMED) {
            stopAll();

            while (opModeIsActive()) {
                telemetry.addLine("!! FIELD POINTS NOT CONFIRMED - REFUSING TO DRIVE");
                telemetry.addLine("The poses in FieldPoints.java are placeholders.");
                telemetry.addLine("Measure them, then set POINTS_CONFIRMED = true.");
                telemetry.update();
            }

            return;
        }

        if (isStopRequested()) {
            return;
        }

        try {
            setAim(AIM_DEFAULT_POSITION);

            follower.setPose(FieldPoints.START_POSE);
            follower.update();
            runtime.reset();

            for (int index = 0; index < mission.size(); index++) {
                if (!opModeIsActive()) {
                    stopReason = "stop requested";
                    break;
                }
                if (budgetExhausted()) {
                    stopReason = String.format("budget %.1f s reached at step %d",
                            AUTONOMOUS_BUDGET_SECONDS, index + 1);
                    break;
                }
                runStep(index, mission.size(), mission.get(index));
            }
        } finally {
            stopAll();
        }

        telemetry.addData("Status", "Finished");
        telemetry.addData("Reason", stopReason);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("Final pose", formatPose(follower.pose()));
        telemetry.addData("Last path", lastPathResult);
        telemetry.addData("Last shot", lastShotResult);
        telemetry.update();
    }

    private boolean budgetExhausted() {
        return runtime.seconds() >= AUTONOMOUS_BUDGET_SECONDS;
    }

    private boolean canKeepRunning() {
        return opModeIsActive() && !budgetExhausted();
    }

    private void setAim(double position) {
        launcherServo.setPosition(Range.clip(position,
                RobotTeleOp.AIM_MIN_POSITION, RobotTeleOp.AIM_MAX_POSITION));
    }

    private void runStep(int index, int total, FieldPoints.Step step) {
        Pose startPose = follower.pose();
        double legLength = startPose.distance(step.target);
        boolean timedOut = false;
        ElapsedTime pathTimer = new ElapsedTime();

        if (legLength < MIN_PATH_LENGTH_INCHES) {
            follower.hold(step.target);
            follower.update();
            lastPathResult = String.format("step %d already on target (%.2f in)", index + 1, legLength);
        } else {
            Path path = Paths.line(startPose, step.target).linear(startPose, step.target);
            follower.follow(path);

            if (step.intakeWhileDriving) {
                intake.setPower(INTAKE_POWER);
            }

            while (canKeepRunning() && (follower.following() || follower.isBusy())) {
                if (pathTimer.seconds() >= step.pathTimeoutSeconds) {
                    timedOut = true;
                    break;
                }
                follower.update();
                addStepTelemetry(index, total, step, "following");
                telemetry.addData("Remaining on path", "%.1f in", follower.remainingDistance());
                telemetry.addData("Path time", "%.1f / %.1f s", pathTimer.seconds(), step.pathTimeoutSeconds);
                telemetry.update();
            }

            intake.setPower(0.0);

            lastPathResult = timedOut
                    ? String.format("step %d TIMEOUT after %.1f s", index + 1, pathTimer.seconds())
                    : String.format("step %d arrived in %.1f s", index + 1, pathTimer.seconds());
        }

        if (!opModeIsActive()) {
            return;
        }

        follower.hold(timedOut ? follower.pose() : step.target);
        follower.update();

        addStepTelemetry(index, total, step, timedOut ? "TIMED OUT" : "arrived");
        if (timedOut) {
            telemetry.addLine("WARNING path timed out - holding the measured pose, not the target");
        }
        telemetry.update();

        if (budgetExhausted()) {
            return;
        }

        switch (step.action) {
            case SHOOT:
                holdFor(SETTLE_SECONDS_BEFORE_SHOOT);
                runShoot(index, total, step);
                break;
            case INTAKE:
                runIntake(index, total, step);
                break;
            case NONE:
            default:
                break;
        }
    }

    private void runShoot(int index, int total, FieldPoints.Step step) {
        int shots = step.shots;

        if (shots <= 0) {
            lastShotResult = String.format("step %d requested 0 shots - skipped", index + 1);
            return;
        }

        setAim(step.aim);

        for (int shot = 0; shot < shots; shot++) {
            if (!canKeepRunning()) {
                break;
            }

            outtake.setVelocity(OUTTAKE_VELOCITY);

            ElapsedTime spinUpTimer = new ElapsedTime();
            boolean atSpeed = false;

            while (canKeepRunning() && spinUpTimer.seconds() < SPINUP_TIMEOUT_SECONDS) {
                follower.update();
                double velocity = outtake.getVelocity();
                if (Math.abs(velocity - OUTTAKE_VELOCITY) <= OUTTAKE_TOLERANCE) {
                    atSpeed = true;
                    break;
                }
                addStepTelemetry(index, total, step, "spinning up");
                telemetry.addData("Shot", "%d / %d", shot + 1, shots);
                telemetry.addData("Aim", "%.2f", step.aim);
                telemetry.addData("Outtake vel", "measured %.0f / target %.0f  tol %.0f",
                        velocity, OUTTAKE_VELOCITY, OUTTAKE_TOLERANCE);
                telemetry.addData("Spin-up time", "%.2f / %.2f s",
                        spinUpTimer.seconds(), SPINUP_TIMEOUT_SECONDS);
                telemetry.update();
            }

            lastShotResult = atSpeed
                    ? String.format("shot %d at speed after %.2f s", shot + 1, spinUpTimer.seconds())
                    : String.format("shot %d SPIN-UP TIMEOUT at %.0f tps", shot + 1, outtake.getVelocity());

            if (!opModeIsActive()) {
                break;
            }

            if (!atSpeed && budgetExhausted()) {
                lastShotResult = String.format("shot %d abandoned - budget %.1f s reached",
                        shot + 1, AUTONOMOUS_BUDGET_SECONDS);
                break;
            }

            transfer.setPower(FEED_POWER);

            ElapsedTime feedTimer = new ElapsedTime();
            while (opModeIsActive() && feedTimer.seconds() < FEED_SECONDS) {
                follower.update();
                addStepTelemetry(index, total, step, "feeding");
                telemetry.addData("Shot", "%d / %d", shot + 1, shots);
                telemetry.addData("Feed time", "%.2f / %.2f s", feedTimer.seconds(), FEED_SECONDS);
                telemetry.addData("Outtake vel", "%.0f", outtake.getVelocity());
                telemetry.update();
            }

            transfer.setPower(0.0);

            if (shot < shots - 1 && canKeepRunning()) {
                holdFor(SHOT_GAP_SECONDS);
            }
        }

        transfer.setPower(0.0);

        if (!KEEP_OUTTAKE_SPINNING) {
            outtake.setVelocity(0.0);
            outtake.setPower(0.0);
        }
    }

    private void runIntake(int index, int total, FieldPoints.Step step) {
        double seconds = Math.min(step.seconds, MAX_ACTION_SECONDS);
        boolean clamped = step.seconds > MAX_ACTION_SECONDS;

        intake.setPower(INTAKE_POWER);

        ElapsedTime intakeTimer = new ElapsedTime();
        while (canKeepRunning() && intakeTimer.seconds() < seconds) {
            follower.update();
            addStepTelemetry(index, total, step, "intaking");
            telemetry.addData("Intake time", "%.1f / %.1f s", intakeTimer.seconds(), seconds);
            if (clamped) {
                telemetry.addData("WARNING", "intake clamped from %.1f to %.1f s",
                        step.seconds, MAX_ACTION_SECONDS);
            }
            telemetry.update();
        }

        intake.setPower(0.0);
    }

    private void holdFor(double seconds) {
        double limit = Math.min(seconds, MAX_ACTION_SECONDS);
        ElapsedTime timer = new ElapsedTime();
        while (canKeepRunning() && timer.seconds() < limit) {
            follower.update();
        }
    }

    private void addStepTelemetry(int index, int total, FieldPoints.Step step, String phase) {
        Pose pose = follower.pose();
        telemetry.addData("Step", "%d / %d  %s", index + 1, total, step.name);
        telemetry.addData("Phase", phase);
        telemetry.addData("Action", step.action);
        telemetry.addData("Target", formatPose(step.target));
        telemetry.addData("Current", formatPose(pose));
        telemetry.addData("Distance to target", "%.1f in", step.target.distance(pose));
        telemetry.addData("Runtime", "%.1f / %.1f s", runtime.seconds(), AUTONOMOUS_BUDGET_SECONDS);
    }

    private String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    private void stopAll() {
        intake.setPower(0.0);
        transfer.setPower(0.0);
        outtake.setVelocity(0.0);
        outtake.setPower(0.0);
        follower.stop();
        follower.update();
    }
}
