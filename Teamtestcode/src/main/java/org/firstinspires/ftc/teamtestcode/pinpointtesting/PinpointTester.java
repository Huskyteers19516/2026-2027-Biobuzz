package org.firstinspires.ftc.teamtestcode.pinpointtesting;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayDeque;
import java.util.Deque;

@TeleOp(name = "Pinpoint Tester", group = "Testing")
public class PinpointTester extends OpMode {

    private static final String LEFT_DEVICE_NAME = "pinpoint_left";
    private static final String RIGHT_DEVICE_NAME = "pinpoint_right";

    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    private static final double SWINGARM_POD_TICKS_PER_MM = 13.26291192;
    private static final double FOUR_BAR_POD_TICKS_PER_MM = 19.89436789;

    private static final double LEFT_X_POD_OFFSET = -84.0;
    private static final double LEFT_Y_POD_OFFSET = -168.0;
    private static final DistanceUnit LEFT_OFFSET_UNIT = DistanceUnit.MM;
    private static final GoBildaPinpointDriver.EncoderDirection LEFT_X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection LEFT_Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final double RIGHT_X_POD_OFFSET = -84.0;
    private static final double RIGHT_Y_POD_OFFSET = 168.0;
    private static final DistanceUnit RIGHT_OFFSET_UNIT = DistanceUnit.MM;
    private static final GoBildaPinpointDriver.EncoderDirection RIGHT_X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection RIGHT_Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final double TEST_DISTANCE_INCHES = 48.0;
    private static final double SPIN_TURNS = 10.0;

    private static final double NOISE_WINDOW_SECONDS = 1.0;
    private static final int NOISE_MAX_SAMPLES = 400;

    private static final double NOISE_POSITION_TOLERANCE_IN = 0.08;
    private static final double NOISE_HEADING_TOLERANCE_DEG = 0.40;
    private static final double DIRECTION_MIN_TRAVEL_IN = 4.0;
    private static final double DIRECTION_MIN_TURN_DEG = 20.0;
    private static final double DISTANCE_TOLERANCE_PERCENT = 2.0;
    private static final double CROSS_AXIS_TOLERANCE_IN = 1.0;
    private static final double HEADING_DRIFT_TOLERANCE_DEG = 2.0;
    private static final double SPIN_TOLERANCE_PERCENT = 1.0;
    private static final double SPIN_MIN_TURN_DEG = 180.0;
    private static final double OFFSET_WANDER_TOLERANCE_IN = 1.0;
    private static final double OFFSET_MIN_TURN_DEG = 300.0;
    private static final double CLOSURE_TOLERANCE_IN = 2.0;
    private static final double DEVICE_AGREEMENT_TOLERANCE_IN = 1.0;
    private static final double DEVICE_AGREEMENT_TOLERANCE_DEG = 2.0;

    private static final double CALIBRATION_SETTLE_SECONDS = 1.0;

    private static final double YAW_SCALAR_MIN_PLAUSIBLE = 0.5;
    private static final double YAW_SCALAR_MAX_PLAUSIBLE = 2.0;
    private static final double YAW_SCALAR_RETRY_SECONDS = 1.0;

    private enum Mode {
        LIVE("LIVE"),
        DIRECTION("DIRECTION CHECK"),
        DISTANCE("DISTANCE TEST"),
        SPIN("SPIN TEST"),
        OFFSET("OFFSET TEST"),
        COMPARE("COMPARE");

        private final String label;

        Mode(String label) {
            this.label = label;
        }
    }

    private static final Mode[] MODES = Mode.values();

    private static class Unit {
        private final String name;
        private GoBildaPinpointDriver device;
        private boolean present;
        private boolean configured;
        private boolean dataValid;
        private String note = "";
        private String status = "no data";

        private double configXOffset;
        private double configYOffset;
        private DistanceUnit offsetUnit = DistanceUnit.MM;

        private double x;
        private double y;
        private double heading;
        private double unwrapped;
        private double frequency;
        private int loopTime;
        private int encoderX;
        private int encoderY;
        private double yawScalar;
        private boolean yawScalarKnown;
        private double yawScalarAttempt = -1000.0;

        private boolean seeded;
        private double lastHeading;
        private double lastX;
        private double lastY;

        private double baseX;
        private double baseY;
        private double baseUnwrapped;

        private double bodyX;
        private double bodyY;

        private double maxRelX;
        private double minRelX;
        private double maxRelY;
        private double minRelY;
        private double maxRelH;
        private double minRelH;
        private double maxBodyX;
        private double minBodyX;
        private double maxBodyY;
        private double minBodyY;
        private double maxRadius;
        private double maxTurn;

        private final Deque<double[]> noiseSamples = new ArrayDeque<double[]>();
        private double noiseX;
        private double noiseY;
        private double noiseHeading;

        private Unit(String name) {
            this.name = name;
        }

        private double relX() {
            return x - baseX;
        }

        private double relY() {
            return y - baseY;
        }

        private double relHeading() {
            return unwrapped - baseUnwrapped;
        }

        private void resetTest() {
            baseX = x;
            baseY = y;
            baseUnwrapped = unwrapped;
            bodyX = 0.0;
            bodyY = 0.0;
            maxRelX = 0.0;
            minRelX = 0.0;
            maxRelY = 0.0;
            minRelY = 0.0;
            maxRelH = 0.0;
            minRelH = 0.0;
            maxBodyX = 0.0;
            minBodyX = 0.0;
            maxBodyY = 0.0;
            minBodyY = 0.0;
            maxRadius = 0.0;
            maxTurn = 0.0;
        }

        private void dropSeed() {
            seeded = false;
            unwrapped = 0.0;
            baseUnwrapped = 0.0;
            noiseSamples.clear();
        }
    }

    private Unit left;
    private Unit right;

    private Mode mode = Mode.LIVE;
    private boolean primaryIsLeft = true;
    private boolean strafeVariant;

    private double maxPositionDisagreement;
    private double maxHeadingDisagreement;

    private boolean leftBumperWas;
    private boolean rightBumperWas;
    private boolean aWas;
    private boolean bWas;
    private boolean xWas;
    private boolean yWas;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime sinceTestReset = new ElapsedTime();
    private final ElapsedTime sinceCalibrate = new ElapsedTime();
    private boolean calibrating;

    @Override
    public void init() {
        left = new Unit(LEFT_DEVICE_NAME);
        right = new Unit(RIGHT_DEVICE_NAME);

        prepare(left, LEFT_X_POD_OFFSET, LEFT_Y_POD_OFFSET, LEFT_OFFSET_UNIT,
                LEFT_X_POD_DIRECTION, LEFT_Y_POD_DIRECTION);
        prepare(right, RIGHT_X_POD_OFFSET, RIGHT_Y_POD_OFFSET, RIGHT_OFFSET_UNIT,
                RIGHT_X_POD_DIRECTION, RIGHT_Y_POD_DIRECTION);

        runtime.reset();
        sinceTestReset.reset();
        sinceCalibrate.reset();
        calibrating = true;
    }

    @Override
    public void init_loop() {
        sampleAll();

        telemetry.addLine("=== PINPOINT TESTER ===");
        telemetry.addLine("Reads the two Pinpoints only. Nothing is ever driven.");
        telemetry.addLine("Every test is done by pushing or turning the robot by hand.");
        telemetry.addLine();
        addDeviceBlock();
        telemetry.addLine();
        if (!left.present && !right.present) {
            telemetry.addLine("NO PINPOINT FOUND AT ALL.");
            telemetry.addLine("Add two devices of type");
            telemetry.addLine("  goBILDA(R) Pinpoint Odometry Computer");
            telemetry.addLine("named " + LEFT_DEVICE_NAME + " and " + RIGHT_DEVICE_NAME + ",");
            telemetry.addLine("each on a DIFFERENT I2C bus (the address 0x31 is fixed).");
        } else {
            telemetry.addLine("Hold the robot still until status reads READY, then press START.");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        resetActiveTest();
    }

    @Override
    public void loop() {
        sampleAll();
        handleButtons();
        trackDisagreement();

        telemetry.addLine("=== PINPOINT TESTER ===");
        telemetry.addLine(String.format("MODE %d of %d:  %s",
                mode.ordinal() + 1, MODES.length, mode.label));
        telemetry.addData("Primary", "%s   (X switches)", primary().name);
        telemetry.addData("Since reset", "%.1f s", sinceTestReset.seconds());
        telemetry.addLine("Bumpers: change mode   A: reset this test   B: reset pose + IMU");
        if (calibrating) {
            telemetry.addLine("!! RESETTING POSE AND IMU - KEEP THE ROBOT COMPLETELY STILL");
            telemetry.addLine("!! measuring is paused until this clears");
        }
        telemetry.addLine();
        addDeviceBlock();
        telemetry.addLine();

        if (calibrating) {
            telemetry.addLine("Measurements resume as soon as the reset finishes.");
            telemetry.update();
            return;
        }

        switch (mode) {
            case LIVE:
                renderLive();
                break;
            case DIRECTION:
                renderDirection();
                break;
            case DISTANCE:
                renderDistance();
                break;
            case SPIN:
                renderSpin();
                break;
            case OFFSET:
                renderOffset();
                break;
            case COMPARE:
                renderCompare();
                break;
        }

        telemetry.update();
    }

    private void prepare(Unit unit, double xOffset, double yOffset, DistanceUnit offsetUnit,
                         GoBildaPinpointDriver.EncoderDirection xDirection,
                         GoBildaPinpointDriver.EncoderDirection yDirection) {
        unit.configXOffset = xOffset;
        unit.configYOffset = yOffset;
        unit.offsetUnit = offsetUnit;

        unit.device = hardwareMap.tryGet(GoBildaPinpointDriver.class, unit.name);
        if (unit.device == null) {
            unit.present = false;
            unit.note = "NOT IN THE ROBOT CONFIGURATION";
            unit.status = "missing";
            return;
        }

        unit.present = true;
        try {
            unit.device.setOffsets(xOffset, yOffset, offsetUnit);
            unit.device.setEncoderResolution(POD_TYPE);
            unit.device.setEncoderDirections(xDirection, yDirection);
            unit.device.resetPosAndIMU();
            unit.configured = true;
            unit.note = "";
        } catch (RuntimeException e) {
            unit.configured = false;
            unit.note = "SETUP FAILED: " + e.getClass().getSimpleName();
            unit.status = "setup failed";
        }
    }

    private void sampleAll() {
        if (calibrating && sinceCalibrate.seconds() >= CALIBRATION_SETTLE_SECONDS) {
            calibrating = false;
        }
        sample(left);
        sample(right);
    }

    private void sample(Unit unit) {
        if (!unit.present) {
            unit.dataValid = false;
            return;
        }

        try {
            unit.device.update();
            unit.x = unit.device.getPosX(DistanceUnit.INCH);
            unit.y = unit.device.getPosY(DistanceUnit.INCH);
            unit.heading = AngleUnit.normalizeDegrees(unit.device.getHeading(AngleUnit.DEGREES));
            unit.frequency = unit.device.getFrequency();
            unit.loopTime = unit.device.getLoopTime();
            unit.encoderX = unit.device.getEncoderX();
            unit.encoderY = unit.device.getEncoderY();
            unit.status = String.valueOf(unit.device.getDeviceStatus());
            unit.dataValid = true;
            unit.note = "";
        } catch (RuntimeException e) {
            unit.dataValid = false;
            unit.status = "READ FAILED";
            unit.note = e.getClass().getSimpleName();
            return;
        }

        if (calibrating) {
            unit.dropSeed();
            return;
        }

        if (!unit.yawScalarKnown) {
            readYawScalar(unit);
        }

        if (!unit.seeded) {
            unit.lastHeading = unit.heading;
            unit.unwrapped = unit.heading;
            unit.lastX = unit.x;
            unit.lastY = unit.y;
            unit.seeded = true;
            unit.resetTest();
        } else {
            unit.unwrapped += AngleUnit.normalizeDegrees(unit.heading - unit.lastHeading);
            unit.lastHeading = unit.heading;

            double dx = unit.x - unit.lastX;
            double dy = unit.y - unit.lastY;
            unit.lastX = unit.x;
            unit.lastY = unit.y;

            double radians = Math.toRadians(unit.heading);
            double cos = Math.cos(radians);
            double sin = Math.sin(radians);
            unit.bodyX += dx * cos + dy * sin;
            unit.bodyY += dy * cos - dx * sin;
        }

        double relX = unit.relX();
        double relY = unit.relY();
        double relH = unit.relHeading();

        unit.maxRelX = Math.max(unit.maxRelX, relX);
        unit.minRelX = Math.min(unit.minRelX, relX);
        unit.maxRelY = Math.max(unit.maxRelY, relY);
        unit.minRelY = Math.min(unit.minRelY, relY);
        unit.maxRelH = Math.max(unit.maxRelH, relH);
        unit.minRelH = Math.min(unit.minRelH, relH);
        unit.maxBodyX = Math.max(unit.maxBodyX, unit.bodyX);
        unit.minBodyX = Math.min(unit.minBodyX, unit.bodyX);
        unit.maxBodyY = Math.max(unit.maxBodyY, unit.bodyY);
        unit.minBodyY = Math.min(unit.minBodyY, unit.bodyY);
        unit.maxRadius = Math.max(unit.maxRadius, Math.hypot(relX, relY));
        unit.maxTurn = Math.max(unit.maxTurn, Math.abs(relH));

        updateNoise(unit);
    }

    private void readYawScalar(Unit unit) {
        double now = runtime.seconds();
        if (now - unit.yawScalarAttempt < YAW_SCALAR_RETRY_SECONDS) {
            return;
        }
        unit.yawScalarAttempt = now;
        try {
            double scalar = unit.device.getYawScalar();
            if (scalar > YAW_SCALAR_MIN_PLAUSIBLE && scalar < YAW_SCALAR_MAX_PLAUSIBLE) {
                unit.yawScalar = scalar;
                unit.yawScalarKnown = true;
            }
        } catch (RuntimeException e) {
            unit.yawScalarKnown = false;
        }
    }

    private void updateNoise(Unit unit) {
        double now = runtime.seconds();
        unit.noiseSamples.addLast(new double[]{now, unit.x, unit.y, unit.unwrapped});
        while (!unit.noiseSamples.isEmpty()
                && (now - unit.noiseSamples.peekFirst()[0] > NOISE_WINDOW_SECONDS
                || unit.noiseSamples.peekFirst()[0] > now
                || unit.noiseSamples.size() > NOISE_MAX_SAMPLES)) {
            unit.noiseSamples.removeFirst();
        }

        if (unit.noiseSamples.isEmpty()) {
            unit.noiseX = 0.0;
            unit.noiseY = 0.0;
            unit.noiseHeading = 0.0;
            return;
        }

        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        double minH = Double.MAX_VALUE;
        double maxH = -Double.MAX_VALUE;

        for (double[] sample : unit.noiseSamples) {
            minX = Math.min(minX, sample[1]);
            maxX = Math.max(maxX, sample[1]);
            minY = Math.min(minY, sample[2]);
            maxY = Math.max(maxY, sample[2]);
            minH = Math.min(minH, sample[3]);
            maxH = Math.max(maxH, sample[3]);
        }

        unit.noiseX = maxX - minX;
        unit.noiseY = maxY - minY;
        unit.noiseHeading = maxH - minH;
    }

    private void handleButtons() {
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        if (rightBumper && !rightBumperWas) {
            mode = MODES[(mode.ordinal() + 1) % MODES.length];
            resetActiveTest();
        }
        if (leftBumper && !leftBumperWas) {
            mode = MODES[(mode.ordinal() + MODES.length - 1) % MODES.length];
            resetActiveTest();
        }
        if (a && !aWas) {
            resetActiveTest();
        }
        if (b && !bWas) {
            resetPoseAndImu();
        }
        if (x && !xWas) {
            primaryIsLeft = !primaryIsLeft;
        }
        if (y && !yWas && mode == Mode.DISTANCE) {
            strafeVariant = !strafeVariant;
            resetActiveTest();
        }

        leftBumperWas = leftBumper;
        rightBumperWas = rightBumper;
        aWas = a;
        bWas = b;
        xWas = x;
        yWas = y;
    }

    private void resetActiveTest() {
        left.resetTest();
        right.resetTest();
        maxPositionDisagreement = 0.0;
        maxHeadingDisagreement = 0.0;
        sinceTestReset.reset();
    }

    private void resetPoseAndImu() {
        resetDevice(left);
        resetDevice(right);
        calibrating = true;
        sinceCalibrate.reset();
        resetActiveTest();
    }

    private void resetDevice(Unit unit) {
        if (!unit.present) {
            return;
        }
        try {
            unit.device.resetPosAndIMU();
            unit.x = 0.0;
            unit.y = 0.0;
            unit.baseX = 0.0;
            unit.baseY = 0.0;
            unit.dropSeed();
        } catch (RuntimeException e) {
            unit.note = "RESET FAILED: " + e.getClass().getSimpleName();
        }
    }

    private Unit primary() {
        return primaryIsLeft ? left : right;
    }

    private Unit secondary() {
        return primaryIsLeft ? right : left;
    }

    private static double presetTicksPerMm() {
        return POD_TYPE == GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                ? SWINGARM_POD_TICKS_PER_MM : FOUR_BAR_POD_TICKS_PER_MM;
    }

    private void addDeviceBlock() {
        addDeviceLines(left);
        addDeviceLines(right);
    }

    private void addDeviceLines(Unit unit) {
        if (!unit.present) {
            telemetry.addLine(String.format("%-15s FOUND: NO   %s", unit.name, unit.note));
            return;
        }
        if (!unit.dataValid) {
            telemetry.addLine(String.format("%-15s FOUND: YES  STATUS: %s  %s",
                    unit.name, unit.status, unit.note));
            return;
        }
        telemetry.addLine(String.format("%-15s FOUND: YES  STATUS: %s%s", unit.name, unit.status,
                unit.configured ? "" : "  (SETUP DID NOT RUN)"));
        telemetry.addLine(String.format("    x %8.2f in   y %8.2f in   h %8.2f deg   %5.0f Hz",
                unit.x, unit.y, unit.heading, unit.frequency));
    }

    private void renderLive() {
        telemetry.addLine("WHAT TO DO: nothing. Put the robot down and leave it still,");
        telemetry.addLine("then push it by hand and watch both readouts move together.");
        telemetry.addLine();

        telemetry.addLine("--- DIFFERENCE (left minus right) ---");
        if (bothValid()) {
            telemetry.addData("dx", "%.3f in", left.x - right.x);
            telemetry.addData("dy", "%.3f in", left.y - right.y);
            telemetry.addData("dheading", "%.3f deg",
                    AngleUnit.normalizeDegrees(left.heading - right.heading));
        } else {
            telemetry.addLine("Need both devices reading to compare.");
        }

        telemetry.addLine();
        telemetry.addLine(String.format("--- NOISE, peak to peak over %.1f s ---", NOISE_WINDOW_SECONDS));
        addNoiseLines(left);
        addNoiseLines(right);
        telemetry.addLine();
        telemetry.addLine(String.format("While still, expect under %.2f in and %.2f deg.",
                NOISE_POSITION_TOLERANCE_IN, NOISE_HEADING_TOLERANCE_DEG));
        telemetry.addLine("Exactly 0.000 in on an axis while you push it means a DEAD POD.");

        telemetry.addLine();
        telemetry.addLine("--- RAW ENCODERS AND TIMING ---");
        addRawLines(left);
        addRawLines(right);
    }

    private void addNoiseLines(Unit unit) {
        if (!unit.dataValid) {
            telemetry.addLine(String.format("%-15s no data", unit.name));
            return;
        }
        boolean quiet = unit.noiseX <= NOISE_POSITION_TOLERANCE_IN
                && unit.noiseY <= NOISE_POSITION_TOLERANCE_IN
                && unit.noiseHeading <= NOISE_HEADING_TOLERANCE_DEG;
        telemetry.addLine(String.format("%-15s x %.3f in  y %.3f in  h %.3f deg  %s",
                unit.name, unit.noiseX, unit.noiseY, unit.noiseHeading,
                quiet ? "QUIET" : "JITTERY (only valid if the robot is still)"));
    }

    private void addRawLines(Unit unit) {
        if (!unit.dataValid) {
            telemetry.addLine(String.format("%-15s no data", unit.name));
            return;
        }
        telemetry.addLine(String.format("%-15s encX %d  encY %d  loop %d us  yawScalar %s",
                unit.name, unit.encoderX, unit.encoderY, unit.loopTime,
                unit.yawScalarKnown ? String.format("%.5f", unit.yawScalar) : "UNREADABLE"));
    }

    private void renderDirection() {
        telemetry.addLine("WHAT TO DO, with the robot on the floor:");
        telemetry.addLine("  1. press B (robot still) then A to reset");
        telemetry.addLine("  2. push the robot straight FORWARD about 2 feet");
        telemetry.addLine("  3. push the robot straight LEFT about 2 feet");
        telemetry.addLine("  4. turn the robot COUNTER-CLOCKWISE about half a turn");
        telemetry.addLine("Travel is measured in the ROBOT frame, so the order does not");
        telemetry.addLine("matter, but do all three before reading the verdict.");
        telemetry.addLine();

        addDirectionReport(primary(), true);
        telemetry.addLine();
        addDirectionReport(secondary(), false);
    }

    private void addDirectionReport(Unit unit, boolean detailed) {
        telemetry.addLine(String.format("--- %s %s---", unit.name, detailed ? "(PRIMARY) " : ""));
        if (!unit.dataValid) {
            telemetry.addLine("    no data");
            return;
        }

        double xSwing = dominant(unit.maxBodyX, unit.minBodyX);
        double ySwing = dominant(unit.maxBodyY, unit.minBodyY);
        double hSwing = dominant(unit.maxRelH, unit.minRelH);

        telemetry.addLine(String.format("    live   fwd %7.2f in   left %7.2f in   turned %7.1f deg",
                unit.bodyX, unit.bodyY, unit.relHeading()));
        telemetry.addLine(String.format("    swing  fwd %7.2f in   left %7.2f in   h %7.1f deg",
                xSwing, ySwing, hSwing));
        telemetry.addLine();

        telemetry.addLine("    FORWARD push -> X should go UP");
        telemetry.addLine("      " + axisVerdict(xSwing, DIRECTION_MIN_TRAVEL_IN));
        if (xSwing < -DIRECTION_MIN_TRAVEL_IN) {
            telemetry.addLine("      FIX: for " + unit.name + " set the FIRST argument of");
            telemetry.addLine("      setEncoderDirections(...) to EncoderDirection.REVERSED");
            telemetry.addLine("      (Pedro: xPodDirection in Constants.pinpointConfig())");
        }

        telemetry.addLine("    LEFT push -> Y should go UP");
        telemetry.addLine("      " + axisVerdict(ySwing, DIRECTION_MIN_TRAVEL_IN));
        if (ySwing < -DIRECTION_MIN_TRAVEL_IN) {
            telemetry.addLine("      FIX: for " + unit.name + " set the SECOND argument of");
            telemetry.addLine("      setEncoderDirections(...) to EncoderDirection.REVERSED");
            telemetry.addLine("      (Pedro: yPodDirection in Constants.pinpointConfig())");
        }

        telemetry.addLine("    COUNTER-CLOCKWISE turn -> HEADING should go UP");
        telemetry.addLine("      " + axisVerdict(hSwing, DIRECTION_MIN_TURN_DEG));
        if (hSwing < -DIRECTION_MIN_TURN_DEG) {
            telemetry.addLine("      FIX: heading sign is NOT settable in software.");
            telemetry.addLine("      " + unit.name + " is mounted upside down. Flip the board over.");
        }
    }

    private String axisVerdict(double swing, double threshold) {
        if (Math.abs(swing) < threshold) {
            return String.format("NOT MOVED YET (%.2f) - push further, or this pod is dead", swing);
        }
        return swing > 0.0 ? "PASS" : "WRONG DIRECTION";
    }

    private double dominant(double high, double low) {
        return Math.abs(high) >= Math.abs(low) ? high : low;
    }

    private void renderDistance() {
        String axisName = strafeVariant ? "STRAFE (sideways, to the LEFT)" : "FORWARD";
        telemetry.addLine("WHAT TO DO:");
        telemetry.addLine("  1. line the robot up on a tape measure or the tile seams");
        telemetry.addLine("  2. press B (robot still) then A to zero everything");
        telemetry.addLine(String.format("  3. push the robot exactly %.1f in %s", TEST_DISTANCE_INCHES, axisName));
        telemetry.addLine("  4. do not let it rotate while you push");
        telemetry.addLine("Y switches between the forward run and the strafe run.");
        telemetry.addLine();
        telemetry.addData("Variant", strafeVariant ? "STRAFE, blames the Y (strafe) pod"
                : "FORWARD, blames the X (forward) pod");
        telemetry.addLine();

        addDistanceReport(primary(), true);
        telemetry.addLine();
        addDistanceReport(secondary(), false);
    }

    private void addDistanceReport(Unit unit, boolean detailed) {
        telemetry.addLine(String.format("--- %s %s---", unit.name, detailed ? "(PRIMARY) " : ""));
        if (!unit.dataValid) {
            telemetry.addLine("    no data");
            return;
        }

        double measured = strafeVariant ? unit.relY() : unit.relX();
        double cross = strafeVariant ? unit.relX() : unit.relY();
        double drift = unit.relHeading();
        double error = measured - TEST_DISTANCE_INCHES;
        double percent = TEST_DISTANCE_INCHES == 0.0 ? 0.0 : error / TEST_DISTANCE_INCHES * 100.0;

        telemetry.addLine(String.format("    commanded   %.2f in", TEST_DISTANCE_INCHES));
        telemetry.addLine(String.format("    measured    %.2f in   (%s)", measured,
                strafeVariant ? "Y" : "X"));
        telemetry.addLine(String.format("    cross axis  %.2f in   (%s, should be near 0)", cross,
                strafeVariant ? "X" : "Y"));
        telemetry.addLine(String.format("    heading     %.2f deg  (drift while pushing)", drift));
        telemetry.addLine(String.format("    error       %.2f in   %.2f %%", error, percent));

        if (measured < 1.0) {
            telemetry.addLine("    VERDICT: not pushed yet, or THIS POD IS REVERSED.");
            telemetry.addLine("    A negative reading is a direction fault, not a scale fault.");
            telemetry.addLine("    Go back to DIRECTION CHECK before scaling anything.");
            return;
        }

        double reportedFactor = TEST_DISTANCE_INCHES / measured;
        double resolutionMultiplier = measured / TEST_DISTANCE_INCHES;
        double preset = presetTicksPerMm();

        telemetry.addLine(String.format("    SUGGESTED SCALE CORRECTION  %.5f", reportedFactor));
        telemetry.addLine("    (that is commanded/measured: what the reported distance");
        telemetry.addLine("    has to be multiplied by to become the real distance)");
        telemetry.addLine(String.format("    TICKS-PER-MM MULTIPLIER     %.5f", resolutionMultiplier));
        telemetry.addLine("    Ticks per mm moves the OPPOSITE way to reported distance,");
        telemetry.addLine("    so MULTIPLY the resolution by the TICKS-PER-MM MULTIPLIER");
        telemetry.addLine("    (same thing as DIVIDING it by the scale correction).");
        telemetry.addLine(String.format("    from the %s preset %.5f  ->  %.5f ticks/mm",
                POD_TYPE, preset, preset * resolutionMultiplier));
        telemetry.addLine(String.format("    apply to %s with", unit.name));
        telemetry.addLine("    setEncoderResolution(ticksPerMm, DistanceUnit.MM)");
        telemetry.addLine("    (Pedro: ticksPerUnit + encoderResolutionUnit = MM)");

        boolean scaleOk = Math.abs(percent) <= DISTANCE_TOLERANCE_PERCENT;
        boolean crossOk = Math.abs(cross) <= CROSS_AXIS_TOLERANCE_IN;
        boolean driftOk = Math.abs(drift) <= HEADING_DRIFT_TOLERANCE_DEG;

        telemetry.addLine(String.format("    VERDICT: scale %s   cross axis %s   heading %s",
                scaleOk ? "PASS" : "FAIL", crossOk ? "PASS" : "FAIL", driftOk ? "PASS" : "FAIL"));
        if (!crossOk && driftOk) {
            telemetry.addLine("    cross axis is off but heading is not: the pods are not square");
            telemetry.addLine("    to the chassis, or you did not push in a straight line.");
        }
        if (!driftOk) {
            telemetry.addLine("    the robot rotated during the push, so this run does not count.");
        }
        if (!scaleOk) {
            telemetry.addLine(String.format("    blame the %s pod on %s.",
                    strafeVariant ? "Y (strafe)" : "X (forward)", unit.name));
        }
    }

    private void renderSpin() {
        telemetry.addLine("WHAT TO DO:");
        telemetry.addLine("  1. mark which way the robot faces, press B (still) then A");
        telemetry.addLine(String.format("  2. turn the robot in place exactly %.0f full turns", SPIN_TURNS));
        telemetry.addLine("     COUNTER-CLOCKWISE, slowly and smoothly");
        telemetry.addLine("  3. stop facing exactly the original direction");
        telemetry.addLine();

        addSpinReport(primary(), true);
        telemetry.addLine();
        addSpinReport(secondary(), false);
    }

    private void addSpinReport(Unit unit, boolean detailed) {
        telemetry.addLine(String.format("--- %s %s---", unit.name, detailed ? "(PRIMARY) " : ""));
        if (!unit.dataValid) {
            telemetry.addLine("    no data");
            return;
        }

        double expected = SPIN_TURNS * 360.0;
        double turned = unit.relHeading();
        double measured = Math.abs(turned);
        double turns = measured / 360.0;
        double error = measured - expected;
        double percent = expected == 0.0 ? 0.0 : error / expected * 100.0;

        telemetry.addLine(String.format("    turned      %.1f deg   (%.2f turns, %s)",
                turned, turns, turned >= 0.0 ? "CCW" : "CW"));
        telemetry.addLine(String.format("    expected    %.1f deg   (%.0f turns)", expected, SPIN_TURNS));
        telemetry.addLine(String.format("    error       %.1f deg   %.2f %%", error, percent));

        if (measured < SPIN_MIN_TURN_DEG) {
            telemetry.addLine("    VERDICT: keep turning");
            return;
        }

        double factor = expected / measured;
        telemetry.addLine(String.format("    HEADING CORRECTION FACTOR   %.5f", factor));
        if (unit.yawScalarKnown) {
            telemetry.addLine(String.format("    current yaw scalar %.5f  ->  NEW YAW SCALAR %.5f",
                    unit.yawScalar, unit.yawScalar * factor));
            telemetry.addLine("    apply with setYawScalar(newValue) right after you build");
            telemetry.addLine("    the driver, before resetPosAndIMU().");
        } else {
            telemetry.addLine("    yaw scalar could NOT be read from this device, so no");
            telemetry.addLine("    absolute value is shown. MULTIPLY whatever setYawScalar");
            telemetry.addLine("    value your code already uses by the factor above.");
            telemetry.addLine("    If your code never calls setYawScalar, use the factor");
            telemetry.addLine("    itself only after re-running with a clean I2C read.");
        }
        telemetry.addLine("    a factor outside 0.95 to 1.05 means a bad device - email goBILDA.");
        telemetry.addLine(String.format("    VERDICT: %s",
                Math.abs(percent) <= SPIN_TOLERANCE_PERCENT ? "PASS" : "FAIL"));
    }

    private void renderOffset() {
        telemetry.addLine("WHAT TO DO:");
        telemetry.addLine("  1. put the robot's tracking point over a mark on the floor");
        telemetry.addLine("  2. press B (still) then A");
        telemetry.addLine("  3. turn the robot in place, ON THE SPOT, two or three turns");
        telemetry.addLine("     keep the tracking point over the mark the whole time");
        telemetry.addLine("X and Y should stay near zero. A circle means a wrong pod offset.");
        telemetry.addLine();

        addOffsetReport(primary(), true);
        telemetry.addLine();
        addOffsetReport(secondary(), false);
    }

    private void addOffsetReport(Unit unit, boolean detailed) {
        telemetry.addLine(String.format("--- %s %s---", unit.name, detailed ? "(PRIMARY) " : ""));
        if (!unit.dataValid) {
            telemetry.addLine("    no data");
            return;
        }

        double centerX = (unit.maxRelX + unit.minRelX) / 2.0;
        double centerY = (unit.maxRelY + unit.minRelY) / 2.0;

        telemetry.addLine(String.format("    live        x %6.2f in   y %6.2f in", unit.relX(), unit.relY()));
        telemetry.addLine(String.format("    turned      %.1f deg", unit.relHeading()));
        telemetry.addLine(String.format("    MAX WANDER  %.2f in", unit.maxRadius));
        telemetry.addLine(String.format("    circle centre  x %6.2f in   y %6.2f in", centerX, centerY));

        if (unit.maxTurn < OFFSET_MIN_TURN_DEG) {
            telemetry.addLine("    VERDICT: keep turning, need about one full turn");
            return;
        }

        boolean ok = unit.maxRadius <= OFFSET_WANDER_TOLERANCE_IN;
        telemetry.addLine(String.format("    VERDICT: %s", ok ? "PASS" : "FAIL"));
        if (ok) {
            return;
        }

        if (Math.abs(centerY) >= Math.abs(centerX)) {
            double errorInches = centerY;
            double errorInUnit = unit.offsetUnit.fromInches(errorInches);
            telemetry.addLine("    signature: the circle centre sits on the Y axis.");
            telemetry.addLine("    That is the X (forward) pod's SIDEWAYS offset, the FIRST");
            telemetry.addLine("    argument of setOffsets(xOffset, yOffset, unit).");
            telemetry.addLine(String.format("    off by about %.2f in (%.1f mm)",
                    Math.abs(errorInches), Math.abs(errorInches) * 25.4));
            telemetry.addLine(String.format("    now %.2f %s  ->  TRY %.2f %s",
                    unit.configXOffset, unit.offsetUnit,
                    unit.configXOffset - errorInUnit, unit.offsetUnit));
        } else {
            double errorInches = centerX;
            double errorInUnit = unit.offsetUnit.fromInches(errorInches);
            telemetry.addLine("    signature: the circle centre sits on the X axis.");
            telemetry.addLine("    That is the Y (strafe) pod's FORWARD offset, the SECOND");
            telemetry.addLine("    argument of setOffsets(xOffset, yOffset, unit).");
            telemetry.addLine(String.format("    off by about %.2f in (%.1f mm)",
                    Math.abs(errorInches), Math.abs(errorInches) * 25.4));
            telemetry.addLine(String.format("    now %.2f %s  ->  TRY %.2f %s",
                    unit.configYOffset, unit.offsetUnit,
                    unit.configYOffset - errorInUnit, unit.offsetUnit));
        }
        telemetry.addLine("    re-run after the change. If MAX WANDER GREW, the sign was");
        telemetry.addLine("    wrong: go back and move it the other way by TWICE as much.");
    }

    private void renderCompare() {
        telemetry.addLine("WHAT TO DO:");
        telemetry.addLine("  1. park the robot on a marked spot, press B (still) then A");
        telemetry.addLine("  2. push the robot around a long loop by hand");
        telemetry.addLine("  3. bring it back onto the exact same spot, same heading");
        telemetry.addLine("Each device should read back near 0, 0. Whichever does not, lies.");
        telemetry.addLine();

        addClosureLine(left);
        addClosureLine(right);

        telemetry.addLine();
        telemetry.addLine("--- DISAGREEMENT BETWEEN THE TWO ---");
        if (bothValid()) {
            double nowPos = Math.hypot(left.relX() - right.relX(), left.relY() - right.relY());
            double nowHeading = Math.abs(left.relHeading() - right.relHeading());
            telemetry.addData("Now", "%.2f in   %.2f deg", nowPos, nowHeading);
            telemetry.addData("Running max", "%.2f in   %.2f deg",
                    maxPositionDisagreement, maxHeadingDisagreement);
            boolean agree = maxPositionDisagreement <= DEVICE_AGREEMENT_TOLERANCE_IN
                    && maxHeadingDisagreement <= DEVICE_AGREEMENT_TOLERANCE_DEG;
            telemetry.addLine(String.format("VERDICT: %s", agree ? "PASS, the two agree" : "FAIL, they disagree"));
            if (!agree) {
                telemetry.addLine("Run DISTANCE and SPIN on each one to find which is wrong.");
            }
        } else {
            telemetry.addLine("Need both devices reading to compare.");
        }
    }

    private void addClosureLine(Unit unit) {
        if (!unit.dataValid) {
            telemetry.addLine(String.format("%-15s no data", unit.name));
            return;
        }
        double closure = Math.hypot(unit.relX(), unit.relY());
        double headingClosure = AngleUnit.normalizeDegrees(unit.relHeading());
        telemetry.addLine(String.format("%-15s closure %.2f in   heading %.2f deg   %s",
                unit.name, closure, headingClosure,
                closure <= CLOSURE_TOLERANCE_IN ? "PASS" : "FAIL"));
        telemetry.addLine(String.format("    travelled at most %.2f in from the start", unit.maxRadius));
    }

    private void trackDisagreement() {
        if (calibrating || !bothValid()) {
            return;
        }
        maxPositionDisagreement = Math.max(maxPositionDisagreement,
                Math.hypot(left.relX() - right.relX(), left.relY() - right.relY()));
        maxHeadingDisagreement = Math.max(maxHeadingDisagreement,
                Math.abs(left.relHeading() - right.relHeading()));
    }

    private boolean bothValid() {
        return left.dataValid && right.dataValid;
    }
}
