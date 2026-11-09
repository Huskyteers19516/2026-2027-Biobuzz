package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.opmode.PoseHandoff;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiAimConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiBotPose;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiFiducial;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;
import org.firstinspires.ftc.teamcode.vision.logilight.math.HeadingHistory;
import org.firstinspires.ftc.teamcode.vision.logilight.math.PlanarPose;

import java.util.Locale;

@Configurable
@TeleOp(name = "LogiLight Aim Assist", group = "LogiLight")
public class LogiLightAimAssist extends OpMode {

    public static double SLOW_SCALE = 0.35;
    public static double AIM_SETTLE_SECONDS = 0.15;
    public static double AIM_MEMORY_SECONDS = 0.4;
    public static double SEARCH_SECONDS = 1.5;
    public static double MAX_RESULT_AGE_MS = LogiOpModeTools.DEFAULT_MAX_RESULT_AGE_MS;

    private static final int RUMBLE_MS = 200;
    private static final double MANUAL_TURN_OVERRIDE = 0.1;

    private enum Mode { DRIVE, AIM, CHASE }

    private LogiLight light;
    private LogiMecanumDrive drive;
    private Localizer localizer;
    private String localizerFault = "";
    private final HeadingHistory headingHistory = new HeadingHistory(LogiOpModeTools.HEADING_HISTORY_CAPACITY);

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private String pollenProfile = LogiOpModeTools.RED_PROFILE;
    private int selectedPipeline = 0;
    private int desiredPipeline = 0;
    private boolean slowMode = false;
    private Pose pose;

    private Mode mode = Mode.DRIVE;
    private String assistText = "off";
    private double alignedSince = -1.0;
    private boolean alignedRumbled = false;
    private double aimTargetHeading = Double.NaN;
    private double aimTargetTime = -1.0;
    private double lastSeenTime = -1.0;
    private double lastSeenTxSign = 0.0;
    private boolean arrivedRumbled = false;
    private double lastSeenDistance = Double.NaN;
    private boolean started = false;
    private String message = "";

    @Override
    public void init() {
        light = new LogiLight(hardwareMap);
        drive = new LogiMecanumDrive(hardwareMap);
        LogiOpModeTools.PinpointHandle pinpoint = LogiOpModeTools.createPinpoint(hardwareMap);
        localizer = pinpoint.localizer;
        localizerFault = pinpoint.fault;

        alliance = PoseHandoff.alliance();
        pollenProfile = LogiOpModeTools.allianceProfile(alliance);
        Pose handoff = PoseHandoff.pose();
        resetPose(handoff != null ? handoff : LogiOpModeTools.startPose(alliance));

        selectedPipeline = LogiOpModeTools.goalsPipelineIndex();
        desiredPipeline = selectedPipeline;
        light.pipelineSwitch(selectedPipeline);
    }

    @Override
    public void init_loop() {
        boolean xPressed = gamepad1.xWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean backPressed = gamepad1.backWasPressed();
        boolean leftBumper = gamepad1.left_bumper;

        String combo = LogiOpModeTools.settingsCombo(leftBumper, yPressed, xPressed);
        if (combo != null) {
            message = combo;
        }
        if (!leftBumper) {
            GoalScanner.Alliance picked = LogiOpModeTools.allianceFromButtons(alliance,
                    LogiOpModeTools.pressedWithoutStart(xPressed, gamepad1),
                    LogiOpModeTools.pressedWithoutStart(bPressed, gamepad1));
            if (picked != alliance) {
                alliance = picked;
                pollenProfile = LogiOpModeTools.allianceProfile(alliance);
                resetPose(LogiOpModeTools.startPose(alliance));
            }
        }
        if (backPressed) {
            resetPose(LogiOpModeTools.startPose(alliance));
        }

        updatePose();
        light.update();
        LogiResult result = light.getLatestResult();
        drawField(result);

        telemetry.addLine("LogiLight Aim Assist - nothing moves until START");
        telemetry.addData("Alliance", "%s   (gp1 X = BLUE, B = RED)   ball %s", alliance, pollenProfile);
        addCommonTelemetry(result);
    }

    @Override
    public void start() {
        PoseHandoff.clear();
        started = true;
        gamepad1.resetEdgeDetection();
        alignedSince = -1.0;
        alignedRumbled = false;
        arrivedRumbled = false;
        lastSeenTime = -1.0;
        lastSeenDistance = Double.NaN;
        aimTargetTime = -1.0;
        message = "";
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.aWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean xPressed = gamepad1.xWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean backPressed = gamepad1.backWasPressed();
        boolean slowPressed = gamepad1.leftStickButtonWasPressed();
        boolean chaseHeld = gamepad1.left_bumper;
        boolean aimHeld = gamepad1.right_bumper;

        if (aPressed) {
            selectedPipeline = LogiOpModeTools.goalsPipelineIndex();
        }
        if (yPressed) {
            selectPollen(LogiOpModeTools.YELLOW_PROFILE);
        }
        if (LogiOpModeTools.pressedWithoutStart(bPressed, gamepad1)) {
            selectPollen(LogiOpModeTools.RED_PROFILE);
        }
        if (xPressed) {
            selectPollen(LogiOpModeTools.BLUE_PROFILE);
        }
        if (backPressed) {
            resetPose(LogiOpModeTools.startPose(alliance));
            message = "pose re-zeroed at the " + alliance + " start pose";
        }
        if (slowPressed) {
            slowMode = !slowMode;
        }

        Mode wanted = aimHeld ? Mode.AIM : (chaseHeld ? Mode.CHASE : Mode.DRIVE);
        if (wanted != mode) {
            enterMode(wanted);
        }
        desiredPipeline = pipelineFor(mode);
        if (desiredPipeline >= 0 && desiredPipeline != light.getPipelineIndex()) {
            light.pipelineSwitch(desiredPipeline);
        }

        updatePose();
        light.update();
        LogiResult result = light.getLatestResult();
        boolean fresh = LogiOpModeTools.isFresh(result, desiredPipeline, MAX_RESULT_AGE_MS);

        double scale = slowMode ? SLOW_SCALE : 1.0;
        double forward = LogiOpModeTools.deadzone(-gamepad1.left_stick_y);
        double strafeLeft = -LogiOpModeTools.deadzone(gamepad1.left_stick_x);
        double manualTurnLeft = -LogiOpModeTools.deadzone(gamepad1.right_stick_x);
        double stickTurnLeft = manualTurnLeft;
        double assistForward = 0.0;
        double assistTurnLeft = 0.0;

        if (mode == Mode.AIM) {
            double turn = aimTurn(result, fresh);
            if (!Double.isNaN(turn)) {
                stickTurnLeft = 0.0;
                assistTurnLeft = turn;
            }
        } else if (mode == Mode.CHASE) {
            double[] command = chaseCommand(result, fresh, manualTurnLeft * scale);
            assistForward = command[0];
            if (!Double.isNaN(command[1])) {
                stickTurnLeft = 0.0;
                assistTurnLeft = command[1];
            }
        } else {
            assistText = "off  (hold gp1 RB = aim at goal, LB = chase ball)";
        }
        drive.drive(forward, strafeLeft, stickTurnLeft, scale, assistForward, assistTurnLeft);
        double turnLeft = stickTurnLeft * scale + assistTurnLeft;
        forward = forward * scale + assistForward;
        strafeLeft = strafeLeft * scale;

        drawField(result);
        telemetry.addData("Mode", "%s  %s", mode, assistText);
        telemetry.addData("Drive", "fwd %+.2f  left %+.2f  turnL %+.2f  %s", forward, strafeLeft, turnLeft,
                slowMode ? String.format(Locale.US, "SLOW %.2f", SLOW_SCALE) : "");
        addCommonTelemetry(result);
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (light != null) {
            light.close();
        }
    }

    private void selectPollen(String profile) {
        int index = LogiOpModeTools.pollenPipelineIndex(profile);
        if (index < 0) {
            message = "!! no COLOR pipeline uses profile \"" + profile + "\"";
            return;
        }
        if (!profile.equals(pollenProfile)) {
            lastSeenTime = -1.0;
            lastSeenTxSign = 0.0;
            lastSeenDistance = Double.NaN;
            arrivedRumbled = false;
        }
        pollenProfile = profile;
        selectedPipeline = index;
    }

    private int pipelineFor(Mode current) {
        if (current == Mode.AIM) {
            return LogiOpModeTools.seesTags(selectedPipeline) ? selectedPipeline : LogiOpModeTools.goalsPipelineIndex();
        }
        if (current == Mode.CHASE) {
            int index = LogiOpModeTools.pollenPipelineIndex(pollenProfile);
            return index >= 0 ? index : selectedPipeline;
        }
        return selectedPipeline;
    }

    private void enterMode(Mode next) {
        mode = next;
        alignedSince = -1.0;
        alignedRumbled = false;
        arrivedRumbled = false;
        aimTargetTime = -1.0;
        aimTargetHeading = Double.NaN;
        lastSeenTime = -1.0;
        lastSeenTxSign = 0.0;
        lastSeenDistance = Double.NaN;
    }

    private double aimTurn(LogiResult result, boolean fresh) {
        LogiAimConfig aim = aimConfig();
        double now = getRuntime();
        double headingNow = pose != null ? pose.heading() : Double.NaN;
        LogiFiducial goal = fresh ? LogiOpModeTools.chooseGoal(result, alliance, false) : null;
        double errorLeft = Double.NaN;
        String goalText;

        if (goal != null) {
            double bearing = LogiOpModeTools.goalBearingLeftDegrees(goal) + Constants.SHOOTER_HEADING_OFFSET_DEGREES;
            if (LogiOpModeTools.isFinite(headingNow)) {
                Double headingAtCapture = headingHistory.headingAt(goal.captureNanos, LogiOpModeTools.HEADING_LOOKUP_SLACK_NANOS);
                double base = headingAtCapture != null ? headingAtCapture : headingNow;
                aimTargetHeading = PlanarPose.normalizeRadians(base + Math.toRadians(bearing));
                aimTargetTime = now;
                errorLeft = Math.toDegrees(PlanarPose.normalizeRadians(aimTargetHeading - headingNow));
            } else {
                errorLeft = bearing;
            }
            goalText = String.format(Locale.US, "%s  d %.1f in  bL %+.1f", goal.name,
                    LogiOpModeTools.goalDistanceInches(goal), LogiOpModeTools.goalBearingLeftDegrees(goal));
        } else if (aimTargetTime >= 0.0 && now - aimTargetTime <= AIM_MEMORY_SECONDS && LogiOpModeTools.isFinite(headingNow)
                && LogiOpModeTools.isFinite(aimTargetHeading)) {
            errorLeft = Math.toDegrees(PlanarPose.normalizeRadians(aimTargetHeading - headingNow));
            goalText = "goal hidden - holding last aim";
        } else {
            alignedSince = -1.0;
            assistText = "no " + alliance + " goal visible - turn with the right stick";
            return Double.NaN;
        }

        double turn = aim.turnLeftPowerForTx(-errorLeft);
        boolean inTolerance = Math.abs(errorLeft) <= aim.toleranceDegrees;
        if (inTolerance) {
            if (alignedSince < 0.0) {
                alignedSince = now;
            }
        } else {
            alignedSince = -1.0;
        }
        boolean aligned = alignedSince >= 0.0 && now - alignedSince >= AIM_SETTLE_SECONDS;
        if (aligned && !alignedRumbled) {
            gamepad1.rumble(RUMBLE_MS);
            alignedRumbled = true;
        }
        if (!inTolerance) {
            alignedRumbled = false;
        }
        assistText = String.format(Locale.US, "%s  %s  err %+.1f deg  turn %+.2f", aligned ? "ALIGNED" : "TURNING",
                goalText, errorLeft, turn);
        return turn;
    }

    private double[] chaseCommand(LogiResult result, boolean fresh, double manualTurnLeft) {
        LogiAimConfig aim = aimConfig();
        double now = getRuntime();
        ColorTarget target = fresh ? result.getPrimaryColor() : null;
        if (target != null) {
            double headingNow = pose != null ? pose.heading() : Double.NaN;
            double tx = LogiOpModeTools.robotCenteredTx(target, headingHistory, result.getCaptureNanos(), headingNow);
            lastSeenTime = now;
            lastSeenTxSign = Math.signum(tx);
            lastSeenDistance = target.hasGroundPosition() ? target.groundDistanceInches : Double.NaN;
            double turn = aim.turnLeftPowerForTx(tx);
            double forward = target.hasGroundPosition() ? aim.forwardPowerForDistance(target.groundDistanceInches, tx) : 0.0;
            boolean arrived = target.hasGroundPosition() && target.groundDistanceInches <= aim.stopDistanceInches;
            if (arrived && !arrivedRumbled) {
                gamepad1.rumble(RUMBLE_MS);
                arrivedRumbled = true;
            }
            if (!arrived) {
                arrivedRumbled = false;
            }
            String range = target.hasGroundPosition()
                    ? String.format(Locale.US, "%.1f in", target.groundDistanceInches)
                    : "no ground fix (check mount tilt)";
            assistText = String.format(Locale.US, "%s %s  tx %+.1f  %s  fwd %+.2f", arrived ? "ARRIVED" : "CHASING",
                    target.profileName, tx, range, forward);
            return new double[]{forward, turn};
        }
        if (lastSeenTime >= 0.0 && LogiOpModeTools.isFinite(lastSeenDistance)
                && lastSeenDistance <= aim.arriveOnLossInches) {
            assistText = "ARRIVED (ball passed under the camera)";
            if (!arrivedRumbled) {
                gamepad1.rumble(RUMBLE_MS);
                arrivedRumbled = true;
            }
            return new double[]{0.0, Double.NaN};
        }
        boolean driverTurning = Math.abs(manualTurnLeft) > MANUAL_TURN_OVERRIDE;
        if (!driverTurning && lastSeenTime >= 0.0 && now - lastSeenTime <= SEARCH_SECONDS && lastSeenTxSign != 0.0) {
            assistText = String.format(Locale.US, "lost %s - searching %s", pollenProfile, lastSeenTxSign > 0 ? "right" : "left");
            return new double[]{0.0, -lastSeenTxSign * aim.searchTurnPower};
        }
        assistText = "no " + pollenProfile + " ball visible - turn with the right stick";
        return new double[]{0.0, Double.NaN};
    }

    private void updatePose() {
        if (localizer == null) {
            pose = null;
            return;
        }
        try {
            localizer.update();
            pose = localizer.pose();
        } catch (Throwable t) {
            pose = null;
            localizerFault = "!! Pinpoint read failed - " + LogiOpModeTools.describe(t);
        }
        if (pose != null && LogiOpModeTools.isFinite(pose.heading())) {
            long now = System.nanoTime();
            headingHistory.add(now, pose.heading());
            light.updateRobotOrientation(pose.heading(), now);
        }
    }

    private void resetPose(Pose start) {
        if (localizer == null || start == null) {
            return;
        }
        try {
            localizer.setPose(start);
            headingHistory.clear();
        } catch (Throwable t) {
            localizerFault = "!! Pinpoint setPose failed - " + LogiOpModeTools.describe(t);
        }
    }

    private void drawField(LogiResult result) {
        LogiLightField.drawResult(pose, result, null, light.getFieldMap());
    }

    private void addCommonTelemetry(LogiResult result) {
        telemetry.addData("Pipeline", "%s   selected %s", LogiOpModeTools.pipelineLabel(light.getPipelineIndex()),
                LogiOpModeTools.pipelineLabel(selectedPipeline));
        telemetry.addData("Ball color", "%s   (gp1 Y yellow, B red, X blue, A goals)", pollenProfile);
        LogiLightTelemetry.addCompact(telemetry, result);
        LogiFiducial goal = LogiOpModeTools.chooseGoal(result, alliance, false);
        if (goal != null) {
            telemetry.addData("Goal", LogiLightTelemetry.fiducialLine(goal));
        }
        ColorTarget pollen = result.getPrimaryColor();
        if (pollen != null) {
            telemetry.addData("Ball", LogiLightTelemetry.colorLine(pollen));
        }
        if (pose != null) {
            LogiBotPose best = result.getBestBotpose();
            telemetry.addData("Pose", "%s   vision %s", LogiLightTelemetry.formatPose(pose),
                    best == null ? "none" : LogiOpModeTools.poseError(best, pose) + " off");
        } else {
            telemetry.addData("Pinpoint", localizerFault.isEmpty() ? "NOT AVAILABLE - aim without latency compensation"
                    : localizerFault + "  (aim without latency compensation)");
        }
        if (!drive.isAvailable()) {
            telemetry.addLine("!! DRIVE MOTORS " + drive.fault());
        }
        telemetry.addData("LogiLight", light.isAvailable() ? light.getStatus().summaryLine() : light.fault());
        if (!message.isEmpty()) {
            telemetry.addData("Note", message);
        }
        telemetry.addLine("gp1 RB hold = aim at goal   LB hold = chase ball   L-stick btn = slow   BACK = re-zero");
        telemetry.addLine("gp1 A goals   Y yellow   B red   X blue" + (started ? "" : "   " + LogiOpModeTools.settingsHint()));
    }

    private static LogiAimConfig aimConfig() {
        LogiAimConfig aim = LogiLightSettings.AIM;
        return aim == null ? new LogiAimConfig() : aim;
    }
}
