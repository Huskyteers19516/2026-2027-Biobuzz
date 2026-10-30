package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.IMU_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE_BLOCKER;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.TRANSFER;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.teleOp.RobotTeleOp;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Simple Park Auto", group = "Meet0")
public class SimpleParkAuto extends LinearOpMode {

    private List<Step> redRoute() {
        List<Step> steps = new ArrayList<>();

        steps.add(step("Shoot preload at shooting point 1")
                .to(FieldPoints.SHOOT_POSE_1)
                .shoot(FieldPoints.SHOTS_AT_SHOOT_POSE_1));

        steps.add(step("Go around the goal via angle 1")
                .to(FieldPoints.ANGLE_1));

        steps.add(step("Park")
                .to(FieldPoints.PARK_POSE)
                .timeout(PARK_TIMEOUT_SECONDS));

        return steps;
    }

    private List<Step> blueRoute() {
        List<Step> steps = new ArrayList<>();

        steps.add(step("Shoot preload at shooting point 1")
                .to(FieldPointsBlue.SHOOT_POSE_1)
                .shoot(FieldPoints.SHOTS_AT_SHOOT_POSE_1));

        steps.add(step("Go around the goal via angle 1")
                .to(FieldPointsBlue.ANGLE_1));

        steps.add(step("Park")
                .to(FieldPointsBlue.PARK_POSE)
                .timeout(PARK_TIMEOUT_SECONDS));

        return steps;
    }

    private static final double PARK_TIMEOUT_SECONDS = 10.0;

    private static final double DRIVE_KP = 0.05;
    private static final double TURN_KP = 0.02;
    private static final double MIN_DRIVE_POWER = 0.12;
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MIN_TURN_POWER = 0.10;
    private static final double MAX_TURN_POWER = 0.4;
    private static final double POSITION_TOLERANCE_INCHES = 1.0;
    private static final double HEADING_TOLERANCE_DEGREES = 2.0;

    private static final double STALL_SECONDS = 0.5;
    private static final int STALL_TICKS = 20;
    private static final double WHEEL_CHECK_POWER = 0.15;
    private static final int WHEEL_CHECK_TICKS = 10;
    private static final double HEADING_CHECK_TURN_POWER = 0.2;
    private static final double HEADING_CHECK_SECONDS = 0.75;
    private static final double HEADING_CHECK_MIN_DEGREES = 1.0;
    private static final double HEADING_WRONG_WAY_DEGREES = 5.0;

    private static final double DEFAULT_TIMEOUT_SECONDS = FieldPoints.DEFAULT_PATH_TIMEOUT_SECONDS;
    private static final double AUTO_BUDGET_SECONDS = FieldPoints.AUTONOMOUS_BUDGET_SECONDS;

    private static final DcMotorSimple.Direction INTAKE_DIRECTION = RobotTeleOp.INTAKE_DIRECTION;
    private static final DcMotorSimple.Direction TRANSFER_DIRECTION = RobotTeleOp.TRANSFER_DIRECTION;
    private static final DcMotorSimple.Direction OUTTAKE_DIRECTION = RobotTeleOp.OUTTAKE_DIRECTION;

    private static final double SHOOT_VELOCITY = RobotTeleOp.OUTTAKE_VELOCITY;
    private static final double SHOOT_TOLERANCE = RobotTeleOp.OUTTAKE_TOLERANCE;
    private static final double SPINUP_TIMEOUT_SECONDS = RobotTeleOp.SPINUP_TIMEOUT_SECONDS;
    private static final double FEED_TRANSFER_POWER = RobotTeleOp.FEED_POWER;
    private static final double FEED_SECONDS = RobotTeleOp.FEED_SECONDS;
    private static final boolean KEEP_OUTTAKE_SPINNING = RobotTeleOp.KEEP_OUTTAKE_SPINNING;
    private static final double FEED_INTAKE_POWER = 1.0;
    private static final double INTAKE_POWER = RobotTeleOp.INTAKE_POWER;
    private static final double SHOT_GAP_SECONDS = 0.4;
    private static final double SETTLE_SECONDS_BEFORE_SHOOT = 0.2;

    private static final double BLOCKER_CLOSED_POSITION = RobotTeleOp.BLOCKER_CLOSED_POSITION;
    private static final double BLOCKER_OPEN_POSITION = RobotTeleOp.BLOCKER_OPEN_POSITION;

    private static final String ARRIVED = "arrived";
    private static final String TIMEOUT = "TIMEOUT";
    private static final String STALLED = "STALLED - robot stuck, skipping to the next step";

    private WheelOdometry odometry;
    private final double[] wheelPowers = new double[4];
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo blocker;

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private String lastResult = "none";
    private String stopReason = "route complete";

    @Override
    public void runOpMode() {
        checkRoute("Red", redRoute());
        checkRoute("Blue", blueRoute());

        odometry = new WheelOdometry(hardwareMap);

        intake = initRoller(INTAKE, INTAKE_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);
        transfer = initRoller(TRANSFER, TRANSFER_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        outtake = initOuttake(OUTTAKE, OUTTAKE_DIRECTION);
        blocker = hardwareMap.get(Servo.class, OUTTAKE_BLOCKER);

        odometry.reset(startPose());
        List<Step> steps = route();

        while (opModeInInit()) {
            GoalScanner.Alliance picked = alliance;
            if (gamepad1.a && !gamepad1.start) {
                picked = GoalScanner.Alliance.BLUE;
            }
            if (gamepad1.b && !gamepad1.start) {
                picked = GoalScanner.Alliance.RED;
            }
            if (picked != alliance) {
                alliance = picked;
                steps = route();
                odometry.reset(startPose());
            }

            odometry.update();
            telemetry.addLine("Simple Park Auto - wheel encoders + IMU, no odometry");
            telemetry.addData("ALLIANCE", "%s   (gamepad1 A = BLUE, B = RED)", alliance);
            telemetry.addData("Points file", pointsFile());
            if (!odometry.imuReady()) {
                telemetry.addLine("!! IMU FAILED TO INITIALIZE - AUTO WILL NOT DRIVE");
            }
            if (!pointsConfirmed()) {
                telemetry.addLine("!! " + alliance + " POINTS NOT CONFIRMED in " + pointsFile());
            }
            telemetry.addData("Start", formatPose(startPose()));
            telemetry.addData("Pose now", odometry.poseText());
            telemetry.addData("Moved since INIT", "forward %.1f in  strafe %.1f in",
                    odometry.forwardInches(), odometry.strafeInches());
            telemetry.addLine("Place the robot exactly on the start pose before START.");
            telemetry.addLine("Run Drive Calibration first to set FORWARD_SCALE / STRAFE_SCALE.");
            telemetry.addData("Steps", steps.size());
            for (int i = 0; i < steps.size(); i++) {
                telemetry.addData("Step " + (i + 1), steps.get(i).summary());
            }
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        if (!odometry.imuReady()) {
            stopEverything();
            while (opModeIsActive()) {
                telemetry.addLine("!! IMU FAILED TO INITIALIZE - REFUSING TO DRIVE");
                telemetry.addData("Check", "a device named \"%s\" in the Robot Configuration", IMU_NAME);
                telemetry.update();
            }
            return;
        }

        odometry.reset(startPose());
        autoTimer.reset();
        blocker.setPosition(BLOCKER_CLOSED_POSITION);

        try {
            for (int i = 0; i < steps.size(); i++) {
                if (!canRun()) {
                    stopReason = opModeIsActive()
                            ? String.format("budget %.1f s reached at step %d", AUTO_BUDGET_SECONDS, i + 1)
                            : "stop requested";
                    break;
                }
                if (!runStep(i + 1, steps.size(), steps.get(i))) {
                    break;
                }
            }
        } finally {
            stopEverything();
        }

        telemetry.addData("Status", "Finished");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Reason", stopReason);
        telemetry.addData("Last result", lastResult);
        telemetry.addData("Final pose", odometry.poseText());
        telemetry.addData("Time used", "%.1f s", autoTimer.seconds());
        telemetry.update();
    }

    private boolean isBlue() {
        return alliance == GoalScanner.Alliance.BLUE;
    }

    private List<Step> route() {
        return isBlue() ? blueRoute() : redRoute();
    }

    private Pose startPose() {
        return isBlue() ? FieldPointsBlue.START_POSE : FieldPoints.START_POSE;
    }

    private boolean pointsConfirmed() {
        return isBlue() ? FieldPointsBlue.POINTS_CONFIRMED : FieldPoints.POINTS_CONFIRMED;
    }

    private String pointsFile() {
        return isBlue() ? "FieldPointsBlue.java" : "FieldPoints.java";
    }

    private static void checkRoute(String name, List<Step> steps) {
        for (int i = 0; i < steps.size(); i++) {
            if (steps.get(i).to == null) {
                throw new IllegalStateException(name + " route step " + (i + 1)
                        + " \"" + steps.get(i).name + "\" has no .to(...)");
            }
        }
    }

    private boolean runStep(int number, int total, Step step) {
        String label = String.format("%d/%d %s", number, total, step.name);

        String result = driveTo(label, step);
        if (!result.equals(ARRIVED) && !result.equals(TIMEOUT) && !result.equals(STALLED)) {
            stopReason = label + ": " + result;
            return false;
        }

        if (step.shots > 0) {
            if (!holdFor(label, "settling", SETTLE_SECONDS_BEFORE_SHOOT) || !shoot(label, step.shots)) {
                return false;
            }
        }
        if (step.intakeSeconds > 0 && !runIntake(label, step.intakeSeconds)) {
            return false;
        }
        if (step.pauseSeconds > 0 && !holdFor(label, "pause", step.pauseSeconds)) {
            return false;
        }
        return true;
    }

    private String driveTo(String label, Step step) {
        if (step.intakeWhileDriving) {
            intake.setPower(INTAKE_POWER);
        }

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stallTimer = new ElapsedTime();
        int[] stallBase = odometry.ticks();
        int[] wheelBase = odometry.ticks();
        double[] wheelSince = new double[4];
        boolean headingWatch = false;
        double headingWatchStart = 0.0;
        double headingWatchBase = 0.0;
        double headingWatchError = 0.0;
        String result = "stopped";

        while (canRun()) {
            odometry.update();

            double heading = odometry.heading();
            double errorX = step.to.x() - odometry.x();
            double errorY = step.to.y() - odometry.y();
            double distance = Math.hypot(errorX, errorY);
            double headingError = WheelOdometry.normalizeRadians(step.to.heading() - heading);
            double headingErrorDegrees = Math.toDegrees(headingError);

            boolean positionDone = distance <= POSITION_TOLERANCE_INCHES;
            boolean headingDone = Math.abs(headingErrorDegrees) <= HEADING_TOLERANCE_DEGREES;
            if (positionDone && headingDone) {
                result = ARRIVED;
                break;
            }
            if (timer.seconds() >= step.timeoutSeconds) {
                result = TIMEOUT;
                break;
            }

            double forward = 0.0;
            double strafe = 0.0;
            if (!positionDone) {
                double power = Range.clip(DRIVE_KP * distance, MIN_DRIVE_POWER, MAX_DRIVE_POWER);
                double forwardError = errorX * Math.cos(heading) + errorY * Math.sin(heading);
                double strafeError = errorX * Math.sin(heading) - errorY * Math.cos(heading);
                forward = forwardError / distance * power;
                strafe = strafeError / distance * power;
            }

            double turn = 0.0;
            if (!headingDone) {
                double power = Range.clip(TURN_KP * Math.abs(headingErrorDegrees), MIN_TURN_POWER, MAX_TURN_POWER);
                turn = -Math.signum(headingError) * power;
            }

            setDrivePowers(forward, strafe, turn);

            int[] ticks = odometry.ticks();
            if (maxWheelPower() < WHEEL_CHECK_POWER || ticksMoved(stallBase, ticks) >= STALL_TICKS) {
                stallBase = ticks;
                stallTimer.reset();
            } else if (stallTimer.seconds() >= STALL_SECONDS) {
                result = STALLED;
                break;
            }

            String wheelFault = checkWheels(ticks, wheelBase, wheelSince, timer.seconds());
            if (wheelFault != null && stallTimer.seconds() < STALL_SECONDS / 2.0) {
                result = wheelFault;
                break;
            }

            if (Math.abs(turn) >= HEADING_CHECK_TURN_POWER) {
                if (!headingWatch) {
                    headingWatch = true;
                    headingWatchStart = timer.seconds();
                    headingWatchBase = heading;
                    headingWatchError = Math.abs(headingErrorDegrees);
                } else if (timer.seconds() - headingWatchStart >= HEADING_CHECK_SECONDS) {
                    double turned = Math.abs(Math.toDegrees(WheelOdometry.normalizeRadians(heading - headingWatchBase)));
                    if (turned < HEADING_CHECK_MIN_DEGREES) {
                        result = "IMU heading not changing while turning - check the imu";
                        break;
                    }
                    if (Math.abs(headingErrorDegrees) > headingWatchError + HEADING_WRONG_WAY_DEGREES) {
                        result = "IMU heading going the wrong way - check HUB_LOGO_DIRECTION / HUB_USB_DIRECTION";
                        break;
                    }
                    headingWatchStart = timer.seconds();
                    headingWatchBase = heading;
                    headingWatchError = Math.abs(headingErrorDegrees);
                }
            } else {
                headingWatch = false;
            }

            telemetry.addData("Step", label);
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Phase", "driving");
            telemetry.addData("Target", formatPose(step.to));
            telemetry.addData("Pose now", odometry.poseText());
            telemetry.addData("Error", "%.1f in  %.1f deg", distance, headingErrorDegrees);
            telemetry.addData("Time", "%.1f / %.1f s  (auto %.1f / %.1f s)",
                    timer.seconds(), step.timeoutSeconds, autoTimer.seconds(), AUTO_BUDGET_SECONDS);
            telemetry.update();
        }

        stopDrive();
        if (step.intakeWhileDriving) {
            intake.setPower(0.0);
        }

        lastResult = String.format("%s: %s after %.1f s", label, result, timer.seconds());
        return result;
    }

    private boolean shoot(String label, int shots) {
        for (int shot = 1; shot <= shots; shot++) {
            String phase = String.format("shot %d / %d", shot, shots);
            setOuttakeVelocity(SHOOT_VELOCITY);

            ElapsedTime timer = new ElapsedTime();
            boolean upToSpeed = false;
            while (canRun() && timer.seconds() < SPINUP_TIMEOUT_SECONDS) {
                odometry.update();
                upToSpeed = Math.abs(outtake.getVelocity() - SHOOT_VELOCITY) <= SHOOT_TOLERANCE;
                if (upToSpeed) {
                    break;
                }
                showShootTelemetry(label, phase + " spinning up", timer.seconds(), SPINUP_TIMEOUT_SECONDS);
            }
            if (!canRun()) {
                return false;
            }

            blocker.setPosition(BLOCKER_OPEN_POSITION);
            transfer.setPower(FEED_TRANSFER_POWER);
            intake.setPower(FEED_INTAKE_POWER);

            timer.reset();
            while (canRun() && timer.seconds() < FEED_SECONDS) {
                odometry.update();
                setOuttakeVelocity(SHOOT_VELOCITY + RobotTeleOp.shotBoost(timer.seconds()));
                showShootTelemetry(label, phase + (upToSpeed ? " feeding" : " feeding (spin-up timed out)"),
                        timer.seconds(), FEED_SECONDS);
            }

            transfer.setPower(0.0);
            intake.setPower(0.0);
            blocker.setPosition(BLOCKER_CLOSED_POSITION);
            setOuttakeVelocity(SHOOT_VELOCITY);

            if (shot < shots && !holdFor(label, "gap between shots", SHOT_GAP_SECONDS)) {
                return false;
            }
        }

        if (!KEEP_OUTTAKE_SPINNING) {
            setOuttakeVelocity(0.0);
        }
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
            odometry.update();
            telemetry.addData("Step", label);
            telemetry.addData("Phase", phase);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.addData("Pose now", odometry.poseText());
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

    private String checkWheels(int[] ticks, int[] base, double[] since, double now) {
        for (int i = 0; i < ticks.length; i++) {
            if (Math.abs(wheelPowers[i]) < WHEEL_CHECK_POWER || Math.abs(ticks[i] - base[i]) >= WHEEL_CHECK_TICKS) {
                base[i] = ticks[i];
                since[i] = now;
            } else if (now - since[i] >= STALL_SECONDS) {
                return "STALLED - " + WheelOdometry.WHEEL_NAMES[i]
                        + " encoder not counting (cable unplugged or wheel stuck)";
            }
        }
        return null;
    }

    private double maxWheelPower() {
        double max = 0.0;
        for (double power : wheelPowers) {
            max = Math.max(max, Math.abs(power));
        }
        return max;
    }

    private static int ticksMoved(int[] from, int[] to) {
        int moved = 0;
        for (int i = 0; i < from.length; i++) {
            moved += Math.abs(to[i] - from[i]);
        }
        return moved;
    }

    private void setDrivePowers(double forward, double strafe, double turn) {
        double leftFrontPower = forward + strafe + turn;
        double leftBackPower = forward - strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double rightBackPower = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        wheelPowers[0] = leftFrontPower / max;
        wheelPowers[1] = leftBackPower / max;
        wheelPowers[2] = rightFrontPower / max;
        wheelPowers[3] = rightBackPower / max;

        odometry.leftFront.setPower(wheelPowers[0]);
        odometry.leftBack.setPower(wheelPowers[1]);
        odometry.rightFront.setPower(wheelPowers[2]);
        odometry.rightBack.setPower(wheelPowers[3]);
    }

    private void stopDrive() {
        setDrivePowers(0.0, 0.0, 0.0);
    }

    private boolean canRun() {
        return opModeIsActive() && autoTimer.seconds() < AUTO_BUDGET_SECONDS;
    }

    private void stopEverything() {
        stopDrive();
        intake.setPower(0.0);
        transfer.setPower(0.0);
        blocker.setPosition(BLOCKER_CLOSED_POSITION);
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
        Pose to;
        int shots = 0;
        double pauseSeconds = 0.0;
        double intakeSeconds = 0.0;
        boolean intakeWhileDriving = false;
        double timeoutSeconds = DEFAULT_TIMEOUT_SECONDS;

        Step(String name) {
            this.name = name;
        }

        Step to(Pose pose) {
            to = pose;
            return this;
        }

        Step shoot(int count) {
            shots = count;
            return this;
        }

        Step pause(double seconds) {
            pauseSeconds = seconds;
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
            if (intakeWhileDriving) text.append("  intake-drive");
            if (shots > 0) text.append(String.format("  shoot %d", shots));
            if (intakeSeconds > 0) text.append(String.format("  intake %.1fs", intakeSeconds));
            if (pauseSeconds > 0) text.append(String.format("  pause %.1fs", pauseSeconds));
            return text.toString();
        }
    }
}
