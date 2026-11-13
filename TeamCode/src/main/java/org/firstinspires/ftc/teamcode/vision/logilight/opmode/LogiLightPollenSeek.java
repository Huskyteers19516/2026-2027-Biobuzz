package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.CappedDrivetrain;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightField;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeekConfig;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeekEngine;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeekSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeeker;

import java.util.Locale;

@TeleOp(name = "LogiLight Pollen Seek", group = "LogiLight")
public class LogiLightPollenSeek extends OpMode {

    private static final int REZERO_SETTLE_LOOPS = 3;
    private static final int ABORT_RUMBLE_MS = 600;
    private static final int DONE_RUMBLE_MS = 200;

    private LogiLight light;
    private Localizer pinpoint;
    private String pinpointFault = "";
    private PinpointStatusMonitor pinpointStatus;
    private CappedDrivetrain drivetrain;
    private Follower follower;
    private String followerFault = "";
    private PollenSeeker seeker;
    private PushTestGate pushTest;
    private PollenIntakeMotor intake;
    private String intakeFault = "";

    private GoalScanner.Alliance alliance = FieldPoints.DEFAULT_ALLIANCE;
    private boolean intakeOnArrive;
    private boolean cancelLatched;
    private boolean started;
    private boolean faulted;
    private String faultText = "";
    private String message = "";
    private int settleLoops;
    private PollenSeeker.State lastState = PollenSeeker.State.IDLE;

    @Override
    public void init() {
        PollenSeekConfig config = PollenSeekSettings.current();
        PollenSeekConfig safe = config.sanitized();
        pushTest = new PushTestGate(safe.pushTestForwardInches, safe.pushTestLateralRatio);
        light = new LogiLight(hardwareMap);
        LogiOpModeTools.PinpointHandle handle = LogiOpModeTools.createPinpoint(hardwareMap);
        pinpoint = handle.localizer;
        pinpointFault = handle.fault;
        pinpointStatus = PinpointStatusMonitor.create(hardwareMap, pinpoint != null);
        if (pinpoint != null) {
            try {
                drivetrain = new CappedDrivetrain(Constants.createDrivetrain(hardwareMap), safe.maxPower);
                follower = new Follower(pinpoint, drivetrain, Constants.createAlgorithm());
            } catch (Throwable t) {
                follower = null;
                drivetrain = null;
                followerFault = "Follower NOT AVAILABLE - " + LogiOpModeTools.describe(t);
            }
        }
        seeker = new PollenSeeker(light, follower, config);
        seeker.setPushTestGate(pushTest);
        intakeOnArrive = safe.runIntakeOnArrive;
        seeker.setIntakeOnArrive(intakeOnArrive);
        resetToStart();
    }

    @Override
    public void init_loop() {
        boolean xPressed = gamepad1.xWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean aPressed = gamepad1.aWasPressed();
        boolean backPressed = gamepad1.backWasPressed();

        GoalScanner.Alliance picked = LogiOpModeTools.allianceFromButtons(alliance,
                LogiOpModeTools.pressedWithoutStart(xPressed, gamepad1),
                LogiOpModeTools.pressedWithoutStart(bPressed, gamepad1));
        if (picked != alliance) {
            alliance = picked;
            resetToStart();
            message = "alliance " + alliance + " - pose set to its start pose";
        }
        if (backPressed) {
            resetToStart();
            message = "pose re-zeroed at the " + alliance + " start pose";
        }
        if (LogiOpModeTools.pressedWithoutStart(aPressed, gamepad1)) {
            toggleIntakeOnArrive();
        }

        updateLocalizerOnly();
        pinpointStatus.report(seeker);
        seeker.recordPose();
        light.update();
        seeker.observe();
        updatePushTest();
        draw();

        telemetry.addLine("LogiLight Pollen Seek - nothing moves until START");
        addVisionTelemetry();
        addPushTestTelemetry();
        telemetry.addData("Alliance", "%s   (gp1 X = BLUE, B = RED, BACK = re-zero at the start pose)", alliance);
        telemetry.addData("Start pose", LogiLightTelemetry.formatPose(LogiOpModeTools.startPose(alliance)));
        addDriveTelemetry();
        telemetry.addData("Intake on arrive", "%s   (gp1 A)", intakeOnArrive ? "ON" : "OFF");
        if (!message.isEmpty()) {
            telemetry.addData("Note", message);
        }
        telemetry.addLine("After START: hold gp1 RB = seek nearest POLLEN, release = stop, B = cancel, sticks = drive (capped)");
        telemetry.addLine("Nothing is commanded until START");
    }

    @Override
    public void start() {
        started = true;
        StringBuilder fault = new StringBuilder();
        intake = PollenIntakeMotor.create(hardwareMap, fault);
        intakeFault = fault.toString();
        seeker.setIntake(intake);
        gamepad1.resetEdgeDetection();
        message = "";
    }

    @Override
    public void loop() {
        if (faulted) {
            emergencyStop();
            telemetry.addLine("!! FAULT - all motors stopped: " + faultText);
            telemetry.addLine("Stop and restart the OpMode.");
            return;
        }
        try {
            runLoop();
        } catch (Throwable t) {
            faulted = true;
            faultText = LogiOpModeTools.describe(t);
            emergencyStop();
            telemetry.addLine("!! FAULT - all motors stopped: " + faultText);
        }
    }

    private void runLoop() {
        boolean aPressed = gamepad1.aWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean seekHeld = gamepad1.right_bumper;

        if (follower != null) {
            follower.update();
        }
        pinpointStatus.report(seeker);
        seeker.recordPose();
        light.update();
        seeker.observe();

        if (!seekHeld) {
            cancelLatched = false;
        }
        if (bPressed && seekHeld) {
            seeker.cancel();
            cancelLatched = true;
            message = "cancelled - release RB, then hold it again to seek";
        }
        if (aPressed) {
            toggleIntakeOnArrive();
        }

        boolean seekWanted = seekHeld && !cancelLatched;
        PollenSeeker.State state = seeker.step(seekWanted);

        if (!seekWanted && follower != null) {
            double forward = LogiOpModeTools.deadzone(-gamepad1.left_stick_y);
            double strafeLeft = -LogiOpModeTools.deadzone(gamepad1.left_stick_x);
            double turnLeft = -LogiOpModeTools.deadzone(gamepad1.right_stick_x);
            follower.manual(forward, strafeLeft, turnLeft);
        }
        if (state != lastState && (state == PollenSeeker.State.ABORTED || state == PollenSeeker.State.DONE)) {
            gamepad1.rumble(state == PollenSeeker.State.ABORTED ? ABORT_RUMBLE_MS : DONE_RUMBLE_MS);
        }
        lastState = state;

        draw();
        telemetry.addData("Mode", seekWanted ? "SEEKING (RB held)" : (cancelLatched ? "cancelled - release RB" : "manual drive"));
        telemetry.addData("State", "%s  %s", state, seeker.status());
        addVisionTelemetry();
        telemetry.addData("Plan", seeker.planNote());
        PollenSeekEngine.FieldBall target = seeker.currentTarget();
        telemetry.addData("Target", target == null ? "none" : target.describe());
        telemetry.addData("Guards", seeker.guardLine());
        if (!seeker.abortReason().isEmpty()) {
            telemetry.addData("Last abort", seeker.abortReason());
        }
        if (!seeker.lastEvent().isEmpty()) {
            telemetry.addData("Last event", seeker.lastEvent());
        }
        telemetry.addData("Push test", pushTest.shortStatus());
        addDriveTelemetry();
        telemetry.addData("Intake on arrive", "%s   (gp1 A)  %s", intakeOnArrive ? "ON" : "OFF",
                intake == null ? "!! " + intakeFault : String.format(Locale.US, "power %.2f", seeker.intakePower()));
        if (!message.isEmpty()) {
            telemetry.addData("Note", message);
        }
        telemetry.addLine("gp1 RB hold = seek nearest POLLEN   release = stop   B = cancel   A = intake on arrive   sticks = drive");
    }

    @Override
    public void stop() {
        if (started) {
            emergencyStop();
        }
        if (seeker != null) {
            try {
                seeker.releaseCamera();
            } catch (Throwable ignored) {
            }
        }
        if (light != null) {
            try {
                light.close();
            } catch (Throwable ignored) {
            }
        }
    }

    private void emergencyStop() {
        if (seeker != null) {
            try {
                seeker.shutdown();
            } catch (Throwable ignored) {
            }
        }
        if (follower != null) {
            try {
                follower.stop();
                follower.update();
            } catch (Throwable ignored) {
            }
        }
        if (drivetrain != null) {
            try {
                drivetrain.stop();
            } catch (Throwable ignored) {
            }
        }
        if (intake != null) {
            try {
                intake.stop();
            } catch (Throwable ignored) {
            }
        }
    }

    private void toggleIntakeOnArrive() {
        intakeOnArrive = !intakeOnArrive;
        seeker.setIntakeOnArrive(intakeOnArrive);
        message = "intake on arrive " + (intakeOnArrive ? "ON" : "OFF");
    }

    private void resetToStart() {
        Pose start = LogiOpModeTools.startPose(alliance);
        if (follower != null) {
            try {
                follower.setPose(start);
            } catch (Throwable t) {
                pinpointFault = "!! Pinpoint setPose failed - " + LogiOpModeTools.describe(t);
            }
        }
        seeker.resetPoseHistory();
        pushTest.rezero(start.x(), start.y(), start.heading());
        settleLoops = REZERO_SETTLE_LOOPS;
    }

    private void updateLocalizerOnly() {
        if (follower == null) {
            return;
        }
        try {
            follower.localizer.update();
        } catch (Throwable t) {
            pinpointFault = "!! Pinpoint read failed - " + LogiOpModeTools.describe(t);
        }
    }

    private void updatePushTest() {
        if (settleLoops > 0) {
            settleLoops--;
            return;
        }
        Pose pose = currentPose();
        if (pose != null) {
            pushTest.update(pose.x(), pose.y(), pose.heading());
        }
    }

    private Pose currentPose() {
        if (follower == null) {
            return null;
        }
        try {
            return follower.pose();
        } catch (Throwable t) {
            return null;
        }
    }

    private void draw() {
        if (LogiLightSettings.EVENT_MODE) {
            return;
        }
        PollenSeekEngine.FieldBall target = seeker.currentTarget();
        double ballX = target == null || follower == null ? Double.NaN : target.x;
        double ballY = target == null || follower == null ? Double.NaN : target.y;
        LogiLightField.drawSeek(currentPose(), ballX, ballY, seeker.plannedTarget(), seeker.plannedVia());
    }

    private void addVisionTelemetry() {
        telemetry.addData("POLLEN VISIBLE", seeker.visibilityLine());
        telemetry.addData("Detections", seeker.rejectLine());
        telemetry.addData("Camera", light.isAvailable() ? light.getStatus().summaryLine() : "!! " + light.fault());
        telemetry.addData("Pipeline", "%s  (%s)", seeker.pipelineLine(), seeker.frameNote());
        for (String warning : seeker.warnings()) {
            telemetry.addLine(warning);
        }
    }

    private void addPushTestTelemetry() {
        telemetry.addLine(pushTest.liveLine());
        telemetry.addLine(pushTest.instruction());
        String warning = pushTest.warning();
        if (!warning.isEmpty() && !pushTest.instruction().startsWith(warning)) {
            telemetry.addLine(warning);
        }
        if (!PollenSeekSettings.REQUIRE_PUSH_TEST) {
            telemetry.addLine("!! REQUIRE_PUSH_TEST is off");
        }
    }

    private void addDriveTelemetry() {
        telemetry.addData("Pose", LogiLightTelemetry.formatPose(currentPose()));
        if (pinpoint == null) {
            telemetry.addLine("!! PINPOINT NOT AVAILABLE - detection only, nothing will drive  " + pinpointFault);
        } else if (!pinpointFault.isEmpty()) {
            telemetry.addLine(pinpointFault);
        }
        if (follower == null && pinpoint != null) {
            telemetry.addLine("!! " + followerFault);
        }
        if (pinpoint != null) {
            telemetry.addData("Pinpoint status", pinpointStatus.line());
        }
        telemetry.addData("Cap", seeker.capLine());
        String refusal = seeker.refusalReason();
        telemetry.addData("Seeking", refusal == null ? "allowed (hold gp1 RB after START)" : "!! DISABLED - " + refusal);
    }
}
