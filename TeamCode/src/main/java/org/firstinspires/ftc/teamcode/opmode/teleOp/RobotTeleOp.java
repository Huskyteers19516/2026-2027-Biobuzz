package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Robot TeleOp", group = "Testing")
public class RobotTeleOp extends OpMode {

    private static final String LEFT_FRONT_NAME = "left_front";
    private static final String LEFT_BACK_NAME = "left_back";
    private static final String RIGHT_FRONT_NAME = "right_front";
    private static final String RIGHT_BACK_NAME = "right_back";
    public static final String INTAKE_NAME = "intake";
    public static final String TRANSFER_NAME = "transfer";
    public static final String OUTTAKE_NAME = "outtake";
//    public static final String LAUNCHER_SERVO_NAME = "launcher_servo";
    private static final String PINPOINT_LEFT_NAME = "pinpoint_left";
    private static final String PINPOINT_RIGHT_NAME = "pinpoint_right";
    private static final String FLOWER_INTAKE ="flower_intake";
    private static final String OUTTAKE_BLOCKER ="outtake_blocker";



    private static final DcMotorSimple.Direction LEFT_SIDE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_SIDE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double SLOW_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.05;
    private static final double TRIGGER_PRESS_THRESHOLD = 0.5;
    private static final double TRIGGER_RELEASE_THRESHOLD = 0.3;

    public static final double INTAKE_POWER = 1.0;
    private static final double TELEOP_INTAKE_POWER = 0.8;
    private static final double INTAKE_REVERSE_POWER = -1.0;
    public static final double TRANSFER_POWER = 1.0;

    public static final double OUTTAKE_VELOCITY = 2000.0;
    public static final double OUTTAKE_TOLERANCE = 100.0;
    public static final double SPINUP_TIMEOUT_SECONDS = 2.5;
    public static final double FEED_POWER = 1.0;
    public static final double FEED_SECONDS = 1.0;
    public static final boolean KEEP_OUTTAKE_SPINNING = false;

    private static final double OUTTAKE_MIN_VELOCITY = 0.0;
    private static final double OUTTAKE_MAX_VELOCITY = 6000.0;
    private static final double OUTTAKE_SMALL_STEP = 100.0;
    private static final double OUTTAKE_BIG_STEP = 500.0;

    private static final double SERVO_RANGE_DEGREES = 270.0;
    private static final double[] FLOWER_POSITIONS = {0.2, 0.5, 0.8};
    private static final int FLOWER_START_INDEX = 0;
    private static final double BLOCKER_CLOSED_POSITION = 0.5;
    private static final double BLOCKER_TURN_DEGREES = 50.0;
    private static final double BLOCKER_OPEN_POSITION =
            Range.clip(BLOCKER_CLOSED_POSITION + BLOCKER_TURN_DEGREES / SERVO_RANGE_DEGREES, 0.0, 1.0);

    private static final double OUTTAKE_ENCODER_STALL_TICKS = 50.0;
    private static final double OUTTAKE_ENCODER_WARN_SECONDS = 2.0;

    public static final double AIM_START_POSITION = 0.50;
    public static final double AIM_MIN_POSITION = 0.00;
    public static final double AIM_MAX_POSITION = 1.00;

    private static final boolean SLOW_MODE_DEFAULT = false;
    private static final boolean FIELD_CENTRIC_DEFAULT = false;

    private static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    private static final double PINPOINT_LEFT_X_OFFSET_MM = -84.0;
    private static final double PINPOINT_LEFT_Y_OFFSET_MM = -168.0;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_LEFT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_LEFT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final double PINPOINT_RIGHT_X_OFFSET_MM = -84.0;
    private static final double PINPOINT_RIGHT_Y_OFFSET_MM = 168.0;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_RIGHT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_RIGHT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final boolean PRIMARY_PINPOINT_IS_LEFT = true;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private GoBildaPinpointDriver pinpointLeft;
    private GoBildaPinpointDriver pinpointRight;
    private GoBildaPinpointDriver pinpointPrimary;
    private Servo outtakeBlocker;
    private Servo flowerIntake;

    private String headingResetResult = "not reset yet";

    private boolean intakeOn = false;
    private boolean transferLatched = false;
    private boolean outtakeOn = false;
    private double outtakeVelocity = OUTTAKE_VELOCITY;
    private int flowerIndex = FLOWER_START_INDEX;
    private boolean blockerOpen = false;
    private boolean slowMode = SLOW_MODE_DEFAULT;
    private boolean fieldCentric = FIELD_CENTRIC_DEFAULT;

    private boolean leftTriggerLatched = false;

    private double headingOffsetRadians = 0.0;
    private double commandedOuttakeVelocity = 0.0;

    private boolean outtakeEncoderWarning = false;
    private boolean primaryHeadingValid = false;
    private boolean fieldCentricActive = false;

    private final ElapsedTime outtakeEncoderTimer = new ElapsedTime();

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, LEFT_FRONT_NAME);
        leftBack = hardwareMap.get(DcMotor.class, LEFT_BACK_NAME);
        rightFront = hardwareMap.get(DcMotor.class, RIGHT_FRONT_NAME);
        rightBack = hardwareMap.get(DcMotor.class, RIGHT_BACK_NAME);
        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER_NAME);
        outtake = hardwareMap.get(DcMotorEx.class, OUTTAKE_NAME);
        pinpointLeft = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_LEFT_NAME);
        pinpointRight = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_RIGHT_NAME);
        pinpointPrimary = PRIMARY_PINPOINT_IS_LEFT ? pinpointLeft : pinpointRight;
        flowerIntake = hardwareMap.get(Servo.class, FLOWER_INTAKE);
        outtakeBlocker = hardwareMap.get(Servo.class, OUTTAKE_BLOCKER);

        leftFront.setDirection(LEFT_SIDE_DIRECTION);
        leftBack.setDirection(LEFT_SIDE_DIRECTION);
        rightFront.setDirection(RIGHT_SIDE_DIRECTION);
        rightBack.setDirection(RIGHT_SIDE_DIRECTION);
        intake.setDirection(INTAKE_DIRECTION);
        transfer.setDirection(TRANSFER_DIRECTION);
        outtake.setDirection(OUTTAKE_DIRECTION);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        configurePinpoint(pinpointLeft, PINPOINT_LEFT_X_OFFSET_MM, PINPOINT_LEFT_Y_OFFSET_MM,
                PINPOINT_LEFT_X_DIRECTION, PINPOINT_LEFT_Y_DIRECTION);
        configurePinpoint(pinpointRight, PINPOINT_RIGHT_X_OFFSET_MM, PINPOINT_RIGHT_Y_OFFSET_MM,
                PINPOINT_RIGHT_X_DIRECTION, PINPOINT_RIGHT_Y_DIRECTION);

        telemetry.addData("Status", "Initialized - nothing is commanded until START");
        telemetry.addData("Servos move at START", "flower pos %d, blocker closed", FLOWER_START_INDEX + 1);
    }

    @Override
    public void init_loop() {
        updatePinpoint(pinpointLeft);
        updatePinpoint(pinpointRight);

        telemetry.addData("Status", "Init - waiting for START");
        telemetry.addData("Primary", PRIMARY_PINPOINT_IS_LEFT ? PINPOINT_LEFT_NAME : PINPOINT_RIGHT_NAME);
        telemetry.addData("Pinpoint L", pinpointEncoderSummary(pinpointLeft));
        telemetry.addData("Pinpoint R", pinpointEncoderSummary(pinpointRight));
        telemetry.addData("Outtake ticks", outtake.getCurrentPosition());
        telemetry.addLine("Spin the outtake wheel by hand: the tick count must change.");
    }

    @Override
    public void start() {
        intakeOn = false;
        transferLatched = false;
        outtakeOn = false;
        outtakeVelocity = OUTTAKE_VELOCITY;
        flowerIndex = FLOWER_START_INDEX;
        blockerOpen = false;
        leftTriggerLatched = false;
        headingResetResult = "not reset yet";
        outtakeEncoderWarning = false;
        updateServos();
    }

    @Override
    public void loop() {
        boolean yPressed = gamepad1.yWasPressed();
        boolean backPressed = gamepad1.backWasPressed();
        boolean slowTogglePressed = gamepad1.leftStickButtonWasPressed();
        boolean outtakeTogglePressed = gamepad2.yWasPressed();
        boolean transferOnPressed = gamepad2.aWasPressed();
        boolean transferOffPressed = gamepad2.bWasPressed();
        boolean blockerTogglePressed = gamepad2.xWasPressed();
        boolean flowerUpPressed = gamepad2.rightBumperWasPressed();
        boolean flowerDownPressed = gamepad2.leftBumperWasPressed();
        boolean velocityUpPressed = gamepad2.dpadUpWasPressed();
        boolean velocityDownPressed = gamepad2.dpadDownWasPressed();
        boolean velocityBigUpPressed = gamepad2.dpadRightWasPressed();
        boolean velocityBigDownPressed = gamepad2.dpadLeftWasPressed();

        double leftTriggerValue = gamepad1.left_trigger;
        boolean intakeTogglePressed = triggerRisingEdge(leftTriggerValue, leftTriggerLatched);
        leftTriggerLatched = updateTriggerLatch(leftTriggerValue, leftTriggerLatched);

        updatePinpoint(pinpointLeft);
        updatePinpoint(pinpointRight);

        GoBildaPinpointDriver.DeviceStatus primaryStatus =
                pinpointPrimary == null ? null : pinpointPrimary.getDeviceStatus();
        primaryHeadingValid = primaryStatus == GoBildaPinpointDriver.DeviceStatus.READY;

        if (slowTogglePressed) {
            slowMode = !slowMode;
        }
        if (yPressed) {
            fieldCentric = !fieldCentric;
        }
        if (backPressed && primaryHeadingValid) {
            headingOffsetRadians = pinpointPrimary.getHeading(AngleUnit.RADIANS);
            headingResetResult = "zeroed";
        } else if (backPressed) {
            headingResetResult = "REFUSED - PRIMARY PINPOINT NOT READY";
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
        if (outtakeTogglePressed) {
            outtakeOn = !outtakeOn;
        }
        if (blockerTogglePressed) {
            blockerOpen = !blockerOpen;
        }
        if (flowerUpPressed) {
            flowerIndex = Math.min(flowerIndex + 1, FLOWER_POSITIONS.length - 1);
        }
        if (flowerDownPressed) {
            flowerIndex = Math.max(flowerIndex - 1, 0);
        }
        if (velocityUpPressed) {
            outtakeVelocity += OUTTAKE_SMALL_STEP;
        }
        if (velocityDownPressed) {
            outtakeVelocity -= OUTTAKE_SMALL_STEP;
        }
        if (velocityBigUpPressed) {
            outtakeVelocity += OUTTAKE_BIG_STEP;
        }
        if (velocityBigDownPressed) {
            outtakeVelocity -= OUTTAKE_BIG_STEP;
        }
        outtakeVelocity = Range.clip(outtakeVelocity, OUTTAKE_MIN_VELOCITY, OUTTAKE_MAX_VELOCITY);

        driveMecanum();

        double intakePower;
        if (gamepad1.left_bumper) {
            intakePower = INTAKE_REVERSE_POWER;
        } else {
            intakePower = intakeOn ? TELEOP_INTAKE_POWER : 0.0;
        }
        intake.setPower(intakePower);

        double transferPower;
        if (gamepad1.left_bumper) {
            transferPower = -TRANSFER_POWER;
        } else {
            transferPower = (intakeOn || transferLatched) ? TRANSFER_POWER : 0.0;
        }
        transfer.setPower(transferPower);

        commandedOuttakeVelocity = outtakeOn ? outtakeVelocity : 0.0;
        outtake.setVelocity(commandedOuttakeVelocity);

        updateServos();

        double measuredVelocity = outtake.getVelocity();
        updateOuttakeEncoderWatchdog(measuredVelocity);
        addTelemetry(measuredVelocity, intakePower, transferPower, primaryStatus);
    }

    @Override
    public void stop() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        intake.setPower(0.0);
        transfer.setPower(0.0);
        outtake.setVelocity(0.0);
        outtake.setPower(0.0);
    }

    private void updateServos() {
        flowerIntake.setPosition(FLOWER_POSITIONS[flowerIndex]);
        outtakeBlocker.setPosition(blockerOpen ? BLOCKER_OPEN_POSITION : BLOCKER_CLOSED_POSITION);
    }

    private void driveMecanum() {
        double axial = deadzone(-gamepad1.left_stick_y);
        double lateral = deadzone(gamepad1.left_stick_x);
        double yaw = deadzone(gamepad1.right_stick_x);

        fieldCentricActive = fieldCentric && primaryHeadingValid;
        if (fieldCentricActive) {
            double heading = pinpointPrimary.getHeading(AngleUnit.RADIANS) - headingOffsetRadians;
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotatedLateral = lateral * cos - axial * sin;
            double rotatedAxial = lateral * sin + axial * cos;
            lateral = rotatedLateral;
            axial = rotatedAxial;
        }

        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        double scale = DRIVE_POWER_SCALE * (slowMode ? SLOW_SCALE : 1.0);

        leftFront.setPower(leftFrontPower * scale);
        leftBack.setPower(leftBackPower * scale);
        rightFront.setPower(rightFrontPower * scale);
        rightBack.setPower(rightBackPower * scale);
    }

    private void updateOuttakeEncoderWatchdog(double measuredVelocity) {
        boolean commanding = Math.abs(commandedOuttakeVelocity) > 0.0;
        boolean stalled = Math.abs(measuredVelocity) < OUTTAKE_ENCODER_STALL_TICKS;
        if (!stalled) {
            outtakeEncoderTimer.reset();
            outtakeEncoderWarning = false;
        } else if (!commanding) {
            outtakeEncoderTimer.reset();
        } else if (outtakeEncoderTimer.seconds() >= OUTTAKE_ENCODER_WARN_SECONDS) {
            outtakeEncoderWarning = true;
        }
    }

    private void addTelemetry(double measuredVelocity, double intakePower, double transferPower,
                              GoBildaPinpointDriver.DeviceStatus primaryStatus) {
        telemetry.addData("Outtake", "%s  target %.0f  measured %.0f",
                outtakeOn ? "ON" : "OFF", outtakeVelocity, measuredVelocity);
        if (outtakeEncoderWarning) {
            telemetry.addLine("WARNING outtake commanded but measured velocity is ~0 - check the outtake encoder cable");
        }
        telemetry.addData("Blocker", "%s  pos %.2f",
                blockerOpen ? "OPEN" : "CLOSED", outtakeBlocker.getPosition());
        telemetry.addData("Flower", "position %d / %d  pos %.2f",
                flowerIndex + 1, FLOWER_POSITIONS.length, flowerIntake.getPosition());
        telemetry.addData("Intake", "%s  power %.2f%s",
                intakeOn ? "ON" : "OFF", intakePower,
                gamepad1.left_bumper ? "  REVERSE held" : "");
        telemetry.addData("Transfer", "latch %s  power %.2f",
                transferLatched ? "ON" : "OFF", transferPower);

        telemetry.addData("Slow mode", slowMode ? String.format("ON  %.2f", SLOW_SCALE) : "OFF");
        if (fieldCentric && !primaryHeadingValid) {
            telemetry.addLine("FIELD-CENTRIC REQUESTED BUT PRIMARY PINPOINT IS NOT READY - DRIVING ROBOT-CENTRIC");
        }
        telemetry.addData("Drive mode", fieldCentricActive ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Heading offset", "%.1f deg", Math.toDegrees(headingOffsetRadians));
        telemetry.addData("Heading reset", headingResetResult);

        telemetry.addData("Primary Pinpoint",
                PRIMARY_PINPOINT_IS_LEFT ? PINPOINT_LEFT_NAME : PINPOINT_RIGHT_NAME);
        telemetry.addData("Primary status", primaryStatus == null ? "NOT CONFIGURED" : primaryStatus);
        if (pinpointPrimary != null) {
            telemetry.addData("Primary freq", "%.1f Hz", pinpointPrimary.getFrequency());
        }
        telemetry.addData("Pinpoint L", pinpointPoseSummary(pinpointLeft));
        telemetry.addData("Pinpoint R", pinpointPoseSummary(pinpointRight));
    }

    private void updatePinpoint(GoBildaPinpointDriver pinpoint) {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    private String pinpointEncoderSummary(GoBildaPinpointDriver pinpoint) {
        if (pinpoint == null) {
            return "NOT CONFIGURED";
        }
        return String.format("%s  freq %.1f Hz  encX %d  encY %d",
                pinpoint.getDeviceStatus(), pinpoint.getFrequency(),
                pinpoint.getEncoderX(), pinpoint.getEncoderY());
    }

    private String pinpointPoseSummary(GoBildaPinpointDriver pinpoint) {
        if (pinpoint == null) {
            return "NOT CONFIGURED";
        }
        return String.format("x %.1f in  y %.1f in  h %.1f deg  %s",
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.DEGREES),
                pinpoint.getDeviceStatus());
    }

    private void configurePinpoint(GoBildaPinpointDriver pinpoint, double xOffsetMm, double yOffsetMm,
                                   GoBildaPinpointDriver.EncoderDirection xDirection,
                                   GoBildaPinpointDriver.EncoderDirection yDirection) {
        if (pinpoint == null) {
            return;
        }
        pinpoint.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PINPOINT_POD_TYPE);
        pinpoint.setEncoderDirections(xDirection, yDirection);
        pinpoint.resetPosAndIMU();
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
