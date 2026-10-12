package org.firstinspires.ftc.teamtestcode.Opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamtestcode.pedropathing.Constants;

import java.util.List;

/**
 * Match autonomous: follows the mission in FieldPoints.java with Pedro Pathing and
 * shoots / intakes at each stop. Same 7 motors + 1 servo as RobotTeleOp, and the same
 * names, directions and speeds (they are read from RobotTeleOp).
 *
 * Every wait in here has a timeout AND stops when AUTONOMOUS_BUDGET_SECONDS runs out,
 * so the robot can never sit in a loop past the end of the autonomous period.
 */
@Autonomous(name = "Match Auto", group = "Match")
public class MatchAuto extends LinearOpMode {

    private static final double AUTONOMOUS_BUDGET_SECONDS = 29.0;
    private static final double PATH_TIMEOUT_SECONDS = 6.0;
    private static final double SPINUP_TIMEOUT_SECONDS = 3.0;
    private static final double FEED_SECONDS = 0.6;                     // UNCONFIRMED: time to push one ball through
    private static final double SHOT_GAP_SECONDS = 0.4;                 // UNCONFIRMED: time for the flywheel to recover
    private static final double MAX_ACTION_SECONDS = 8.0;
    private static final boolean KEEP_OUTTAKE_SPINNING = true;
    private static final double ARRIVE_TOLERANCE_INCHES = 2.0;          // UNCONFIRMED
    private static final double ARRIVE_TOLERANCE_RADIANS = Math.toRadians(5.0); // UNCONFIRMED

    private Follower follower;
    private DcMotor[] wheels = new DcMotor[4];
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo launcherServo;

    private final ElapsedTime runtime = new ElapsedTime();
    private final StringBuilder log = new StringBuilder();
    private boolean aborted = false;

    @Override
    public void runOpMode() {
        try {
            runAuto();
        } finally {
            stopAll();
        }
    }

    private void runAuto() {
        wheels = RobotTeleOp.driveMotors(hardwareMap);
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
            if (!RobotTeleOp.SERVO_POSITIONS_CONFIRMED) {
                telemetry.addLine("!! SERVO POSITIONS NOT CONFIRMED - servo is disabled");
            }
            telemetry.addLine("Nothing is commanded until START");
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        runtime.reset();

        if (!FieldPoints.POINTS_CONFIRMED) {
            while (running()) {
                telemetry.addLine("!! FIELD POINTS NOT CONFIRMED - REFUSING TO DRIVE");
                telemetry.addLine("The poses in FieldPoints.java are placeholders.");
                telemetry.addLine("Measure them, then set POINTS_CONFIRMED = true.");
                telemetry.update();
                idle();
            }

            return;
        }

        follower.setPose(FieldPoints.START_POSE);
        follower.update();
        setServo(false);

        for (int index = 0; index < mission.size() && running() && !aborted; index++) {
            runStep(index, mission.size(), mission.get(index));
        }

        stopAll();

        telemetry.addData("Status", aborted ? "ABORTED"
                : runtime.seconds() >= AUTONOMOUS_BUDGET_SECONDS ? "OUT OF TIME" : "Finished");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("Final pose", formatPose(follower.pose()));
        telemetry.addLine(log.toString());
        telemetry.update();
    }

    /** True while the op mode is active and the autonomous budget is not used up. */
    private boolean running() {
        return opModeIsActive() && runtime.seconds() < AUTONOMOUS_BUDGET_SECONDS;
    }

    private void runStep(int index, int total, FieldPoints.Step step) {
        follower.follow(step.path);

        if (step.intakeDuringPath) {
            intake.setPower(RobotTeleOp.INTAKE_POWER);
            transfer.setPower(RobotTeleOp.TRANSFER_POWER_WHILE_INTAKING);
        }

        ElapsedTime pathTimer = new ElapsedTime();
        boolean timedOut = false;

        while (running() && (follower.following() || follower.isBusy())) {
            if (pathTimer.seconds() >= PATH_TIMEOUT_SECONDS) {
                timedOut = true;
                break;
            }
            follower.update();
            addStepTelemetry(index, total, step);
            telemetry.addData("Path time", "%.1f / %.1f s", pathTimer.seconds(), PATH_TIMEOUT_SECONDS);
            telemetry.update();
        }

        stopFeeding();

        if (!running()) {
            log.append(index + 1).append(": ").append(step.name).append(" -> stopped\n");
            return;
        }

        if (timedOut) {
            Pose pose = follower.pose();
            boolean closeEnough = MechanismLogic.arrived(step.target.distance(pose),
                    step.target.heading() - pose.heading(),
                    ARRIVE_TOLERANCE_INCHES, ARRIVE_TOLERANCE_RADIANS);
            if (!closeEnough) {
                // Not where we think we are: shooting or driving on from here is a guess.
                aborted = true;
                stopAll();
                log.append(index + 1).append(": ").append(step.name)
                        .append(String.format(" -> PATH TIMEOUT %.1f in from target, AUTO ABORTED\n",
                                step.target.distance(pose)));
                return;
            }
            // Close enough: drop the unfinished path and just hold the target.
            follower.hold(step.target);
        }

        String result;
        switch (step.action) {
            case SHOOT:
                result = runShoot(index, total, step);
                break;
            case INTAKE:
                result = runIntake(index, total, step);
                break;
            case WAIT:
                holdFor(step.seconds);
                result = "waited";
                break;
            case NONE:
            default:
                result = "arrived";
                break;
        }

        log.append(index + 1).append(": ").append(step.name).append(" -> ")
                .append(timedOut ? "path timeout but in tolerance, " : "").append(result).append('\n');
    }

    // The flywheel speed is checked on every loop, the same interlock as RobotTeleOp:
    // the transfer only feeds while the outtake is at speed, and the whole burst has one
    // deadline (MAX_ACTION_SECONDS, and never past the autonomous budget).
    private String runShoot(int index, int total, FieldPoints.Step step) {
        double target = RobotTeleOp.OUTTAKE_TARGET_VELOCITY;
        outtake.setVelocity(target);

        ElapsedTime shootTimer = new ElapsedTime();
        ShotSequencer shots = new ShotSequencer(step.shots, FEED_SECONDS, SHOT_GAP_SECONDS,
                SPINUP_TIMEOUT_SECONDS, boundedSeconds(MAX_ACTION_SECONDS));

        while (running() && !shots.done()) {
            follower.update();
            double velocity = outtake.getVelocity();
            boolean ready = MechanismLogic.outtakeReady(true, velocity, target, RobotTeleOp.OUTTAKE_READY_TOLERANCE);
            boolean feed = shots.update(shootTimer.seconds(), ready);

            transfer.setPower(feed ? RobotTeleOp.TRANSFER_FEED_POWER : 0.0);
            setServo(feed);

            addStepTelemetry(index, total, step);
            telemetry.addData("Outtake", "%.0f / %.0f ticks/s %s", velocity, target, ready ? "READY" : "not ready");
            telemetry.addData("Shots", shots.summary());
            telemetry.update();
        }

        stopFeeding();

        if (!KEEP_OUTTAKE_SPINNING) {
            outtake.setPower(0.0);
        }

        return shots.summary();
    }

    private String runIntake(int index, int total, FieldPoints.Step step) {
        double seconds = boundedSeconds(step.seconds);

        intake.setPower(RobotTeleOp.INTAKE_POWER);
        transfer.setPower(RobotTeleOp.TRANSFER_POWER_WHILE_INTAKING);

        ElapsedTime intakeTimer = new ElapsedTime();
        while (running() && intakeTimer.seconds() < seconds) {
            follower.update();
            addStepTelemetry(index, total, step);
            telemetry.addData("Intake", "%.1f / %.1f s", intakeTimer.seconds(), seconds);
            telemetry.update();
        }

        stopFeeding();
        return String.format("intake %.1f s", intakeTimer.seconds());
    }

    private void holdFor(double seconds) {
        double limit = boundedSeconds(seconds);
        ElapsedTime timer = new ElapsedTime();
        while (running() && timer.seconds() < limit) {
            follower.update();
        }
    }

    private double boundedSeconds(double seconds) {
        return MechanismLogic.boundedSeconds(seconds, MAX_ACTION_SECONDS,
                runtime.seconds(), AUTONOMOUS_BUDGET_SECONDS);
    }

    private void addStepTelemetry(int index, int total, FieldPoints.Step step) {
        Pose pose = follower.pose();
        telemetry.addData("Step", "%d / %d  %s", index + 1, total, step.name);
        telemetry.addData("Action", step.action);
        telemetry.addData("Target", formatPose(step.target));
        telemetry.addData("Pinpoint", formatPose(pose));
        telemetry.addData("Distance to target", "%.1f in", step.target.distance(pose));
        telemetry.addData("Runtime", "%.1f / %.1f s", runtime.seconds(), AUTONOMOUS_BUDGET_SECONDS);
    }

    private String formatPose(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    private void setServo(boolean fire) {
        if (RobotTeleOp.SERVO_POSITIONS_CONFIRMED) {
            launcherServo.setPosition(fire ? RobotTeleOp.LAUNCHER_SERVO_FIRE : RobotTeleOp.LAUNCHER_SERVO_REST);
        }
    }

    private void stopFeeding() {
        intake.setPower(0.0);
        transfer.setPower(0.0);
        setServo(false);
    }

    // Also runs when hardware setup or the mission throws, so anything may be null here.
    // Each of the 7 motors is zeroed on its own and directly: follower.stop() alone only
    // takes effect on the next follower.update(), which needs a working localizer.
    private void stopAll() {
        RuntimeException failure = MechanismLogic.runAll(
                () -> RobotTeleOp.zero(wheels[0]),
                () -> RobotTeleOp.zero(wheels[1]),
                () -> RobotTeleOp.zero(wheels[2]),
                () -> RobotTeleOp.zero(wheels[3]),
                () -> RobotTeleOp.zero(intake),
                () -> RobotTeleOp.zero(transfer),
                () -> RobotTeleOp.zero(outtake),
                () -> { if (follower != null) follower.stop(); },
                () -> { if (follower != null) follower.drivetrain.stop(); });
        if (failure != null) {
            log.append("!! STOP FAILED: ").append(failure).append('\n');
        }
        RobotTeleOp.reportStopFailure("MatchAuto", failure, telemetry);
    }
}
