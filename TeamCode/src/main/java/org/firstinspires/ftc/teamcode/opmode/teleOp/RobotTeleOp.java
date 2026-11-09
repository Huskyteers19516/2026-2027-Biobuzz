package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.launcher.Launcher;
import org.firstinspires.ftc.teamcode.launcher.ShotLog;
import org.firstinspires.ftc.teamcode.launcher.ShotZones;
import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.panels.PanelsBridge;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Robot TeleOp", group = "Testing")
public class RobotTeleOp extends OpMode {

    public static double SLOW_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.05;
    private static final double TRIGGER_PRESS_THRESHOLD = 0.5;
    private static final double TRIGGER_RELEASE_THRESHOLD = 0.3;

    public static double TELEOP_INTAKE_POWER = 0.8;
    public static double INTAKE_REVERSE_POWER = -1.0;

    public static double OUTTAKE_SMALL_STEP = 100.0;
    private static final double OUTTAKE_BIG_STEP = 500.0;
    private static final int READY_RUMBLE_MS = 200;
    private static final double MAX_VELOCITY_TRIM = Launcher.MAX_TARGET_VELOCITY - Launcher.MIN_TARGET_VELOCITY;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private Servo flowerIntake;
    private Launcher launcher;
    private Localizer localizer;
    private String localizerFault = "";
    private GoalScanner scanner;
    private GoalScanner.GoalView goalView = GoalScanner.GoalView.NOT_SEEN;

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private boolean poseFromAuto = false;

    private boolean intakeOn = false;
    private boolean transferLatched = false;
    private int flowerIndex = Constants.FLOWER_START_INDEX;
    private boolean slowMode = false;

    private boolean leftTriggerLatched = false;
    private boolean rightTriggerLatched = false;
    private boolean singleTriggerLatched = false;
    private boolean wasReady = false;

    private double velocityTrim = 0.0;
    private int shotsLogged = 0;
    private double lastShotDistance = Double.NaN;
    private double lastShotVelocity = Double.NaN;
    private int lastShotNumber = -1;
    private int savedShotNumber = -1;
    private String shotLogMessage = "";
    private boolean burstThisHold = false;
    private boolean autoBurstPending = false;
    private int autoBurstEndShot = 0;
    private boolean wasAimHeld = false;
    private double alignedSince = -1.0;


    @Override
    public void init() {
        leftFront = initMotor(Constants.FRONT_LEFT_MOTOR_NAME, true);
        leftBack = initMotor(Constants.BACK_LEFT_MOTOR_NAME, true);
        rightFront = initMotor(Constants.FRONT_RIGHT_MOTOR_NAME, false);
        rightBack = initMotor(Constants.BACK_RIGHT_MOTOR_NAME, false);

        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE);
        intake.setDirection(Constants.INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flowerIntake = hardwareMap.get(Servo.class, Constants.FLOWER_INTAKE);
        launcher = new Launcher(hardwareMap);
        launcher.setQueueExpireSeconds(Launcher.TELEOP_QUEUE_EXPIRE_SECONDS);

        try {
            localizer = Constants.createLocalizer(hardwareMap);
        } catch (RuntimeException e) {
            localizer = null;
            localizerFault = e.getMessage();
        }

        alliance = PoseHandoff.alliance();
        Pose handoff = PoseHandoff.pose();
        poseFromAuto = handoff != null;
        if (localizer != null) {
            localizer.setPose(poseFromAuto ? handoff : GoalAim.startPose(alliance));
        }

        scanner = new GoalScanner(hardwareMap);

        telemetry.addData("Outtake velocity", "%.0f ticks/s", () -> launcher.measuredVelocity());
    }

    @Override
    public void init_loop() {
        launcher.sense();
        scanner.update();

        GoalScanner.Alliance picked = alliance;
        if (gamepad1.x) {
            picked = GoalScanner.Alliance.BLUE;
        }
        if (gamepad1.b && !gamepad1.start) {
            picked = GoalScanner.Alliance.RED;
        }
        if (picked != alliance) {
            alliance = picked;
            if (!poseFromAuto) {
                resetPose();
            }
        }
        if (gamepad1.backWasPressed()) {
            resetPose();
        }

        Pose pose = updatePose();
        PanelsBridge.drawRobot(pose);
        telemetry.addData("Status", "Init - nothing is commanded until START");
        telemetry.addData("ALLIANCE", "%s   (gp1 X = BLUE, B = RED)", alliance);
        telemetry.addData("Pose source", poseFromAuto ? "end of auto" : "start pose - put the robot on it, gp1 BACK = re-zero");
        addPoseTelemetry(pose);
        telemetry.addData("Camera", scanner.isAvailable() ? scanner.cameraState() : "NOT AVAILABLE - goal check off  " + scanner.fault());
        telemetry.addLine("Spin the outtake wheel by hand: Outtake velocity must change.");
    }

    @Override
    public void start() {
        PoseHandoff.clear();
        intakeOn = false;
        transferLatched = false;
        flowerIndex = Constants.FLOWER_START_INDEX;
        leftTriggerLatched = false;
        rightTriggerLatched = false;
        singleTriggerLatched = false;
        wasReady = false;
        velocityTrim = 0.0;
        burstThisHold = false;
        autoBurstPending = false;
        wasAimHeld = false;
        alignedSince = -1.0;
        shotsLogged = launcher.shotsAttempted();
        lastShotNumber = -1;
        savedShotNumber = -1;
        lastShotDistance = Double.NaN;
        lastShotVelocity = Double.NaN;
        shotLogMessage = "";
        launcher.setTargetVelocity(Launcher.DEFAULT_TARGET_VELOCITY);
        flowerIntake.setPosition(Constants.FLOWER_POSITIONS[flowerIndex]);
        launcher.update();
    }

    @Override
    public void loop() {
        if (gamepad1.backWasPressed()) {
            resetPose();
        }
        Pose pose = updatePose();
        PanelsBridge.drawRobot(pose);
        scanner.update();

        boolean slowTogglePressed = gamepad1.leftStickButtonWasPressed();
        boolean launcherTogglePressed = gamepad2.yWasPressed();
        boolean transferOnPressed = gamepad2.aWasPressed() && !gamepad2.start;
        boolean transferOffPressed = gamepad2.bWasPressed() && !gamepad2.start;
        boolean flowerUpPressed = gamepad2.rightBumperWasPressed();
        boolean flowerDownPressed = gamepad2.leftBumperWasPressed();
        boolean velocityUpPressed = gamepad2.dpadUpWasPressed();
        boolean velocityDownPressed = gamepad2.dpadDownWasPressed();
        boolean velocityBigUpPressed = gamepad2.dpadRightWasPressed();
        boolean velocityBigDownPressed = gamepad2.dpadLeftWasPressed();

        double leftTriggerValue = gamepad1.left_trigger;
        boolean intakeTogglePressed = triggerRisingEdge(leftTriggerValue, leftTriggerLatched);
        leftTriggerLatched = updateTriggerLatch(leftTriggerValue, leftTriggerLatched);
        double rightTriggerValue = gamepad2.right_trigger;
        boolean firePressed = triggerRisingEdge(rightTriggerValue, rightTriggerLatched);
        rightTriggerLatched = updateTriggerLatch(rightTriggerValue, rightTriggerLatched);
        double singleTriggerValue = gamepad2.left_trigger;
        boolean singleShotPressed = triggerRisingEdge(singleTriggerValue, singleTriggerLatched);
        singleTriggerLatched = updateTriggerLatch(singleTriggerValue, singleTriggerLatched);
        boolean reversing = gamepad1.left_bumper;
        boolean aimHeld = gamepad1.right_bumper;

        if (slowTogglePressed) {
            slowMode = !slowMode;
        }
        if (intakeTogglePressed) {
            intakeOn = !intakeOn;
        }
        if (transferOnPressed) {
            transferLatched = true;
        }
        if (transferOffPressed) {
            transferLatched = false;
        }
        if (launcherTogglePressed) {
            launcher.toggle();
        }
        if (flowerUpPressed) {
            flowerIndex = Math.min(flowerIndex + 1, Constants.FLOWER_POSITIONS.length - 1);
        }
        if (flowerDownPressed) {
            flowerIndex = Math.max(flowerIndex - 1, 0);
        }

        GoalAim.Solution aim = pose != null && GoalAim.goalsConfirmed(alliance) ? GoalAim.solveUp(alliance, pose, scanner) : null;
        GoalAim.Solution shotGoal = aim != null ? aim : pose != null ? GoalAim.solveUp(alliance, pose, scanner) : null;
        ShotZones.Result zone = shotGoal != null ? ShotZones.lookup(shotGoal.distance) : null;
        boolean aiming = aimHeld && aim != null;
        boolean aligned = updateAlignment(aiming, aim, pose);
        goalView = aiming ? scanner.goalView(aim.goalName) : scanner.facingView(alliance);
        boolean goalBlocked = scanner.blocksShot(goalView);

        double baseVelocity = zone != null ? zone.velocity : Launcher.DEFAULT_TARGET_VELOCITY;
        if (velocityUpPressed) {
            velocityTrim += OUTTAKE_SMALL_STEP;
        }
        if (velocityDownPressed) {
            velocityTrim -= OUTTAKE_SMALL_STEP;
        }
        if (velocityBigUpPressed) {
            velocityTrim += OUTTAKE_BIG_STEP;
        }
        if (velocityBigDownPressed) {
            velocityTrim -= OUTTAKE_BIG_STEP;
        }
        velocityTrim = Range.clip(velocityTrim, -MAX_VELOCITY_TRIM, MAX_VELOCITY_TRIM);
        launcher.setTargetVelocity(baseVelocity + velocityTrim);

        if (aimHeld && !wasAimHeld) {
            burstThisHold = launcher.queuedShots() > 0 || launcher.isBusy();
        }
        if (autoBurstPending && launcher.shotsAttempted() >= autoBurstEndShot) {
            autoBurstPending = false;
        }
        if (reversing) {
            launcher.cancelShots();
            autoBurstPending = false;
        } else {
            if (firePressed && launcher.queuedShots() == 0) {
                launcher.requestShots(Launcher.BURST_SIZE);
                burstThisHold = aimHeld;
            } else if (singleShotPressed && launcher.queuedShots() == 0) {
                launcher.requestShots(1);
                burstThisHold = aimHeld;
            }
            if (aiming && aligned && aim.inEntryAngle && !goalBlocked && !burstThisHold && launcher.queuedShots() == 0) {
                launcher.requestShots(Launcher.BURST_SIZE);
                burstThisHold = true;
                autoBurstPending = true;
                autoBurstEndShot = launcher.shotsAttempted() + Launcher.BURST_SIZE;
            }
        }
        if (!aimHeld && wasAimHeld) {
            if (autoBurstPending) {
                launcher.cancelShots();
            }
            autoBurstPending = false;
            burstThisHold = false;
        }
        wasAimHeld = aimHeld;
        if (goalBlocked) {
            launcher.cancelShots();
            autoBurstPending = false;
        }
        launcher.setFireAllowed((!aiming || aligned) && !goalBlocked);
        launcher.setManualBlockerOpen(gamepad2.x);

        double scale = slowMode ? SLOW_SCALE : 1.0;
        double manualYaw = aiming ? 0.0 : deadzone(gamepad1.right_stick_x);
        double aimYaw = aiming ? aimTurnPower(aim, pose) : 0.0;
        drive(deadzone(-gamepad1.left_stick_y), deadzone(gamepad1.left_stick_x), manualYaw, scale, aimYaw);

        double intakePower;
        if (reversing) {
            intakePower = INTAKE_REVERSE_POWER;
        } else {
            intakePower = intakeOn ? TELEOP_INTAKE_POWER : 0.0;
        }
        if (!reversing && (launcher.isFeeding() || gamepad2.x)) {
            intakePower = Constants.INTAKE_POWER;
        }
        intake.setPower(intakePower);

        double transferPower;
        if (reversing) {
            transferPower = -Constants.TRANSFER_POWER;
        } else {
            transferPower = (intakeOn || transferLatched || gamepad2.x) ? Constants.TRANSFER_POWER : 0.0;
        }
        launcher.setStagingTransferPower(transferPower);

        flowerIntake.setPosition(Constants.FLOWER_POSITIONS[flowerIndex]);
        launcher.update();
        updateShotLog(shotGoal);

        boolean ready = launcher.isReady();
        if (ready && !wasReady) {
            gamepad1.rumble(READY_RUMBLE_MS);
        }
        wasReady = ready;

        addTelemetry(pose, zone, aiming, aligned, baseVelocity, intakePower, transferPower);
    }

    @Override
    public void stop() {
        drive(0.0, 0.0, 0.0, 1.0, 0.0);
        intake.setPower(0.0);
        launcher.shutdown();
        scanner.close();
    }

    private String goalViewText() {
        if (!scanner.isAvailable()) {
            return "no camera - not checked";
        }
        String text;
        switch (goalView) {
            case OPEN:
                text = "OPEN - ok to shoot";
                break;
            case FACING_DOWN:
                text = "FACING DOWN - DON'T SHOOT";
                break;
            default:
                text = "not seen";
                break;
        }
        if (scanner.blocksShot(goalView)) {
            text += "  (shots cancelled, gp2 X still dumps)";
        } else if (!Constants.GOAL_CHECK_CALIBRATED) {
            text += "  (not calibrated - never blocks)";
        }
        return text;
    }

    private Pose updatePose() {
        if (localizer == null) {
            return null;
        }
        localizer.update();
        return localizer.pose();
    }

    private void resetPose() {
        if (localizer != null) {
            localizer.setPose(GoalAim.startPose(alliance));
        }
        poseFromAuto = false;
    }

    private boolean updateAlignment(boolean aiming, GoalAim.Solution aim, Pose pose) {
        if (!aiming || Math.abs(Math.toDegrees(aim.headingError(pose))) > Constants.AIM_TOLERANCE_DEGREES) {
            alignedSince = -1.0;
            return false;
        }
        if (alignedSince < 0) {
            alignedSince = getRuntime();
        }
        return getRuntime() - alignedSince >= Constants.AIM_SETTLE_SECONDS;
    }

    private double aimTurnPower(GoalAim.Solution aim, Pose pose) {
        double error = aim.headingError(pose);
        if (Math.abs(Math.toDegrees(error)) <= Constants.AIM_TOLERANCE_DEGREES) {
            return 0.0;
        }
        double power = Range.clip(Math.abs(error) * Constants.AIM_KP,
                Constants.AIM_MIN_TURN_POWER, Constants.AIM_MAX_TURN_POWER);
        return -Math.signum(error) * power;
    }

    private void drive(double axial, double lateral, double yaw, double scale, double aimYaw) {
        double[] powers = {
                axial + lateral + yaw,
                axial - lateral + yaw,
                axial - lateral - yaw,
                axial + lateral - yaw
        };
        normalize(powers);
        powers[0] = powers[0] * scale + aimYaw;
        powers[1] = powers[1] * scale + aimYaw;
        powers[2] = powers[2] * scale - aimYaw;
        powers[3] = powers[3] * scale - aimYaw;
        normalize(powers);

        leftFront.setPower(powers[0]);
        leftBack.setPower(powers[1]);
        rightFront.setPower(powers[2]);
        rightBack.setPower(powers[3]);
    }

    private static void normalize(double[] powers) {
        double max = 0.0;
        for (double power : powers) {
            max = Math.max(max, Math.abs(power));
        }
        if (max > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }
    }

    private void updateShotLog(GoalAim.Solution shotGoal) {
        if (launcher.shotsAttempted() != shotsLogged) {
            shotsLogged = launcher.shotsAttempted();
            Launcher.Shot shot = launcher.lastShot();
            if (shot != null && (shot.detected || launcher.encoderFault())) {
                lastShotDistance = shotGoal != null ? shotGoal.distance : Double.NaN;
                lastShotVelocity = shot.target - shot.burstBoost;
                lastShotNumber = shotsLogged;
            }
        }
        if (gamepad2.backWasPressed()) {
            if (Double.isNaN(lastShotDistance)) {
                shotLogMessage = "NOT saved - no confirmed shot with a distance yet";
            } else if (lastShotNumber == savedShotNumber) {
                shotLogMessage = "already saved this shot";
            } else {
                shotLogMessage = "saved " + ShotLog.record(lastShotDistance, lastShotVelocity);
                savedShotNumber = lastShotNumber;
                String error = ShotLog.lastError();
                if (!error.isEmpty()) {
                    shotLogMessage += "  FILE ERROR " + error;
                }
            }
        }
        if (gamepad2.rightStickButtonWasPressed()) {
            String removed = ShotLog.undo();
            shotLogMessage = removed == null ? "nothing to undo" : "removed " + removed;
            if (removed != null) {
                savedShotNumber = -1;
            }
            String undoError = ShotLog.lastError();
            if (!undoError.isEmpty()) {
                shotLogMessage += "  FILE ERROR " + undoError;
            }
        }
    }

    private void addPoseTelemetry(Pose pose) {
        if (pose == null) {
            telemetry.addData("Localizer", "NOT AVAILABLE - auto aim and auto velocity off  %s", localizerFault);
            return;
        }
        telemetry.addData("Pose", GoalAim.formatPose(pose));
        if (!GoalAim.goalsConfirmed(alliance)) {
            telemetry.addLine("!! " + alliance + " GOAL POINTS NOT CONFIRMED - auto aim off (zone velocity still on)");
        } else {
            telemetry.addData("Goal", GoalAim.solveUp(alliance, pose, scanner).summary(pose));
        }
        String zonesProblem = GoalAim.shotZonesProblem();
        if (zonesProblem != null) {
            telemetry.addLine("!! shot zones: " + zonesProblem);
        } else {
            telemetry.addData("Shot zones", ShotZones.summary());
        }
    }

    private void addTelemetry(Pose pose, ShotZones.Result zone, boolean aiming, boolean aligned,
                              double baseVelocity, double intakePower, double transferPower) {
        launcher.addTelemetry(telemetry);
        addPoseTelemetry(pose);
        telemetry.addData("Auto aim", aiming ? (aligned ? "ALIGNED" : "TURNING") : "off (hold gp1 RB)");
        telemetry.addData("Goal (camera)", goalViewText());
        if (zone != null) {
            telemetry.addData("Zone", zone.describe());
            telemetry.addData("Zone velocity", "%.0f  trim %+.0f  burst +%.0f when firing 3",
                    baseVelocity, velocityTrim, launcher.burstBoost);
        } else {
            telemetry.addData("Zone velocity", "no pose - default %.0f  trim %+.0f", baseVelocity, velocityTrim);
        }
        if (Double.isNaN(lastShotDistance)) {
            telemetry.addData("BACK saves", "no shot to save yet");
        } else {
            telemetry.addData("BACK saves", "%.1f in  at %.0f", lastShotDistance, lastShotVelocity);
        }
        telemetry.addData("Shot log", "%d saved  (gp2 BACK = last shot went in, gp2 R-stick = undo)  %s",
                ShotLog.count(), shotLogMessage);
        telemetry.addData("Flower", "position %d / %d  pos %.2f",
                flowerIndex + 1, Constants.FLOWER_POSITIONS.length, flowerIntake.getPosition());
        telemetry.addData("Intake", "%s  power %.2f%s",
                intakeOn ? "ON" : "OFF", intakePower,
                gamepad1.left_bumper ? "  REVERSE held" : "");
        telemetry.addData("Transfer", "latch %s  staging power %.2f",
                transferLatched ? "ON" : "OFF", transferPower);
        telemetry.addData("Slow mode", slowMode ? String.format("ON  %.2f", SLOW_SCALE) : "OFF");
        telemetry.addLine("gp1 RB hold = aim + fire 3   gp1 BACK = re-zero at start pose");
        telemetry.addLine("gp2 RT fire 3   gp2 LT fire 1   gp1 LB cancel   gp2 Y launcher   gp2 X hold = open blocker");
    }

    private DcMotor initMotor(String name, boolean left) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(left ? Constants.LEFT_SIDE_DIRECTION : Constants.RIGHT_SIDE_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private boolean triggerRisingEdge(double triggerValue, boolean latched) {
        return !latched && triggerValue >= TRIGGER_PRESS_THRESHOLD;
    }

    private boolean updateTriggerLatch(double triggerValue, boolean latched) {
        if (triggerValue >= TRIGGER_PRESS_THRESHOLD) {
            return true;
        }
        if (triggerValue <= TRIGGER_RELEASE_THRESHOLD) {
            return false;
        }
        return latched;
    }

    private double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
