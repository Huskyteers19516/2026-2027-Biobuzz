package org.firstinspires.ftc.teamtestcode.OpModeForTest;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamtestcode.pedropathing.Constants;

import java.util.List;

@Autonomous(name = "Starter Bot Auto (Pedro)", group = "Pedro")
public class StarterBotAuto extends LinearOpMode {

    private static final double LAUNCHER_VELOCITY = 1500.0;
    private static final double LAUNCHER_TOLERANCE = 100.0;
    private static final double SPINUP_TIMEOUT_SECONDS = 3.0;
    private static final double FEED_SECONDS = 0.6;
    private static final double SHOT_GAP_SECONDS = 0.4;
    private static final int SHOTS_PER_STOP = 3;
    private static final boolean KEEP_LAUNCHER_SPINNING = true;
    private static final double PATH_TIMEOUT_SECONDS = 6.0;

    private static final double FEED_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double MAX_ACTION_SECONDS = 8.0;

    private static final String LAUNCHER_NAME = "launcher";
    private static final String LEFT_INTAKE_NAME = "left_intake";
    private static final String RIGHT_INTAKE_NAME = "right_intake";

    private Follower follower;
    private DcMotorEx launcher;
    private CRServo leftIntake;
    private CRServo rightIntake;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Paths.START_POSE);

        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_NAME);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftIntake = hardwareMap.get(CRServo.class, LEFT_INTAKE_NAME);
        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE_NAME);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        List<Paths.Step> mission = Paths.mission();

        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addLine("Starter Bot Auto (Pedro)");
            telemetry.addData("Steps", mission.size());
            telemetry.addData("Start pose", formatPose(Paths.START_POSE));
            telemetry.addData("Measured pose", formatPose(follower.pose()));
            telemetry.addLine("First run: wheels off the ground");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        follower.setPose(Paths.START_POSE);
        follower.update();
        runtime.reset();

        for (int index = 0; index < mission.size(); index++) {
            if (!opModeIsActive()) {
                break;
            }
            runStep(index, mission.size(), mission.get(index));
        }

        stopAll();

        telemetry.addData("Status", "Finished");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("Final pose", formatPose(follower.pose()));
        telemetry.update();
    }

    private void runStep(int index, int total, Paths.Step step) {
        follower.follow(step.path);

        if (step.intakeDuringPath) {
            setIntakePower(INTAKE_POWER);
        }

        ElapsedTime pathTimer = new ElapsedTime();
        boolean timedOut = false;

        while (opModeIsActive() && (follower.following() || follower.isBusy())) {
            if (pathTimer.seconds() >= PATH_TIMEOUT_SECONDS) {
                timedOut = true;
                break;
            }
            follower.update();
            addStepTelemetry(index, total, step, false);
            telemetry.addData("Path time", "%.1f / %.1f s", pathTimer.seconds(), PATH_TIMEOUT_SECONDS);
            telemetry.update();
        }

        setIntakePower(0.0);

        if (!opModeIsActive()) {
            return;
        }

        if (timedOut) {
            follower.hold(step.target);
            follower.update();
        }

        addStepTelemetry(index, total, step, timedOut);
        telemetry.update();

        switch (step.action) {
            case SHOOT:
                runShoot(index, total, step);
                break;
            case INTAKE:
                runIntake(index, total, step);
                break;
            case WAIT:
                holdFor(step.seconds);
                break;
            case NONE:
            default:
                break;
        }
    }

    private void runShoot(int index, int total, Paths.Step step) {
        int shots = step.shots > 0 ? step.shots : SHOTS_PER_STOP;

        launcher.setVelocity(LAUNCHER_VELOCITY);

        ElapsedTime spinupTimer = new ElapsedTime();
        boolean atSpeed = false;

        while (opModeIsActive() && spinupTimer.seconds() < SPINUP_TIMEOUT_SECONDS) {
            follower.update();
            double velocity = launcher.getVelocity();
            if (Math.abs(velocity - LAUNCHER_VELOCITY) <= LAUNCHER_TOLERANCE) {
                atSpeed = true;
                break;
            }
            addStepTelemetry(index, total, step, false);
            telemetry.addData("Spinup", "%.0f / %.0f ticks/s", velocity, LAUNCHER_VELOCITY);
            telemetry.addData("Spinup time", "%.1f / %.1f s", spinupTimer.seconds(), SPINUP_TIMEOUT_SECONDS);
            telemetry.update();
        }

        for (int shot = 0; shot < shots; shot++) {
            if (!opModeIsActive()) {
                break;
            }
            addStepTelemetry(index, total, step, false);
            telemetry.addData("Launcher", atSpeed ? "at speed" : "SPINUP TIMEOUT");
            telemetry.addData("Shot", "%d / %d", shot + 1, shots);
            telemetry.addData("Launcher velocity", "%.0f", launcher.getVelocity());
            telemetry.update();

            setIntakePower(FEED_POWER);
            holdFor(FEED_SECONDS);
            setIntakePower(0.0);

            if (shot < shots - 1) {
                holdFor(SHOT_GAP_SECONDS);
            }
        }

        setIntakePower(0.0);

        if (!KEEP_LAUNCHER_SPINNING) {
            launcher.setVelocity(0.0);
            launcher.setPower(0.0);
        }
    }

    private void runIntake(int index, int total, Paths.Step step) {
        double seconds = Math.min(step.seconds, MAX_ACTION_SECONDS);

        setIntakePower(INTAKE_POWER);

        ElapsedTime intakeTimer = new ElapsedTime();
        while (opModeIsActive() && intakeTimer.seconds() < seconds) {
            follower.update();
            addStepTelemetry(index, total, step, false);
            telemetry.addData("Intake", "%.1f / %.1f s", intakeTimer.seconds(), seconds);
            telemetry.update();
        }

        setIntakePower(0.0);
    }

    private void holdFor(double seconds) {
        double limit = Math.min(seconds, MAX_ACTION_SECONDS);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < limit) {
            follower.update();
        }
    }

    private void addStepTelemetry(int index, int total, Paths.Step step, boolean timedOut) {
        Pose pose = follower.pose();
        telemetry.addData("Step", "%d / %d  %s", index + 1, total, step.name);
        telemetry.addData("Action", step.action);
        telemetry.addData("Target", formatPose(step.target));
        telemetry.addData("Pinpoint", formatPose(pose));
        telemetry.addData("Distance to target", "%.1f in", step.target.distance(pose));
        telemetry.addData("Remaining on path", "%.1f in", follower.remainingDistance());
        telemetry.addData("Path", timedOut ? "TIMED OUT" : (follower.following() ? "following" : "done"));
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
    }

    private String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    private void setIntakePower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    private void stopAll() {
        setIntakePower(0.0);
        launcher.setVelocity(0.0);
        launcher.setPower(0.0);
        follower.stop();
        follower.update();
    }
}
