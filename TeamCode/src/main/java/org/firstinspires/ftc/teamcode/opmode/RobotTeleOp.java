package org.firstinspires.ftc.teamcode.opmode;

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
    public static final String LAUNCHER_SERVO_NAME = "launcher_servo";
    private static final String PINPOINT_LEFT_NAME = "pinpoint_left";
    private static final String PINPOINT_RIGHT_NAME = "pinpoint_right";

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
    private static final double INTAKE_REVERSE_POWER = -1.0;
    public static final double TRANSFER_POWER = 1.0;

    public static final double OUTTAKE_VELOCITY = 2000.0;
    public static final double OUTTAKE_TOLERANCE = 100.0;
    public static final double SPINUP_TIMEOUT_SECONDS = 2.5;
    public static final double FEED_POWER = 1.0;
    public static final double FEED_SECONDS = 1.0;
    public static final boolean KEEP_OUTTAKE_SPINNING = false;

    private static final double OUTTAKE_ENCODER_STALL_TICKS = 50.0;
    private static final double OUTTAKE_ENCODER_WARN_SECONDS = 2.0;

    public static final double AIM_START_POSITION = 0.50;
    public static final double AIM_MIN_POSITION = 0.00;
    public static final double AIM_MAX_POSITION = 1.00;
    private static final double AIM_STEP_COARSE = 0.05;
    private static final double AIM_STEP_FINE = 0.01;

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

    private enum FireState { IDLE, SPIN_UP, FEED, FINISH }

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo launcherServo;
    private GoBildaPinpointDriver pinpointLeft;
    private GoBildaPinpointDriver pinpointRight;
    private GoBildaPinpointDriver pinpointPrimary;

    private FireState fireState = FireState.IDLE;
    private String spinUpResult = "n/a";
    private String fireWaitingFor = "nothing";
    private String headingResetResult = "not reset yet";

    private boolean intakeOn = false;
    private boolean transferLatched = false;
    private boolean outtakeHolding = false;
    private boolean slowMode = SLOW_MODE_DEFAULT;
    private boolean fieldCentric = FIELD_CENTRIC_DEFAULT;

    private boolean leftTriggerLatched = false;
    private boolean rightTriggerLatched = false;

    private double aimPosition = AIM_START_POSITION;
    private double headingOffsetRadians = 0.0;
    private double commandedOuttakeVelocity = 0.0;

    private boolean outtakeEncoderWarning = false;
    private boolean primaryHeadingValid = false;
    private boolean fieldCentricActive = false;

    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
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
        launcherServo = hardwareMap.get(Servo.class, LAUNCHER_SERVO_NAME);
        pinpointLeft = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_LEFT_NAME);
        pinpointRight = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_RIGHT_NAME);
        pinpointPrimary = PRIMARY_PINPOINT_IS_LEFT ? pinpointLeft : pinpointRight;

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
        telemetry.addData("Aim moves to", "%.3f at START", aimPosition);
    }

    @Override
    public void init_loop() {
        pinpointLeft.update();
        pinpointRight.update();

        telemetry.addData("Status", "Init - waiting for START");
        telemetry.addData("Primary", PRIMARY_PINPOINT_IS_LEFT ? PINPOINT_LEFT_NAME : PINPOINT_RIGHT_NAME);
        telemetry.addData("Pinpoint L", "%s  freq %.1f Hz  encX %d  encY %d",
                pinpointLeft.getDeviceStatus(), pinpointLeft.getFrequency(),
                pinpointLeft.getEncoderX(), pinpointLeft.getEncoderY());
        telemetry.addData("Pinpoint R", "%s  freq %.1f Hz  encX %d  encY %d",
                pinpointRight.getDeviceStatus(), pinpointRight.getFrequency(),
                pinpointRight.getEncoderX(), pinpointRight.getEncoderY());
        telemetry.addData("Outtake ticks", outtake.getCurrentPosition());
        telemetry.addLine("Spin the outtake wheel by hand: the tick count must change.");
    }

    @Override
    public void start() {
        fireState = FireState.IDLE;
        intakeOn = false;
        transferLatched = false;
        outtakeHolding = false;
        leftTriggerLatched = false;
        rightTriggerLatched = false;
        spinUpResult = "n/a";
        fireWaitingFor = "nothing";
        headingResetResult = "not reset yet";
        outtakeEncoderWarning = false;
        outtakeEncoderTimer.reset();
        aimPosition = Range.clip(AIM_START_POSITION, AIM_MIN_POSITION, AIM_MAX_POSITION);
        launcherServo.setPosition(aimPosition);
    }

    @Override
    public void loop() {
        boolean yPressed = gamepad1.yWasPressed();
        boolean backPressed = gamepad1.backWasPressed();
        boolean slowTogglePressed = gamepad1.leftStickButtonWasPressed();
        boolean cancelPressed = gamepad2.xWasPressed();
        boolean transferOnPressed = gamepad2.aWasPressed();
        boolean transferOffPressed = gamepad2.bWasPressed();
        boolean aimUpPressed = gamepad2.dpadUpWasPressed();
        boolean aimDownPressed = gamepad2.dpadDownWasPressed();
        boolean aimRightPressed = gamepad2.dpadRightWasPressed();
        boolean aimLeftPressed = gamepad2.dpadLeftWasPressed();

        double leftTriggerValue = gamepad1.left_trigger;
        double rightTriggerValue = gamepad1.right_trigger;
        boolean intakeTogglePressed = triggerRisingEdge(leftTriggerValue, leftTriggerLatched);
        leftTriggerLatched = updateTriggerLatch(leftTriggerValue, leftTriggerLatched);
        boolean firePressed = triggerRisingEdge(rightTriggerValue, rightTriggerLatched);
        rightTriggerLatched = updateTriggerLatch(rightTriggerValue, rightTriggerLatched);

        pinpointLeft.update();
        pinpointRight.update();

        GoBildaPinpointDriver.DeviceStatus primaryStatus = pinpointPrimary.getDeviceStatus();
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
        if (cancelPressed) {
            if (fireState != FireState.IDLE) {
                spinUpResult = "CANCELLED";
            }
            fireState = FireState.IDLE;
            transferLatched = false;
            outtakeHolding = false;
        }

        if (aimUpPressed) {
            aimPosition = Range.clip(aimPosition + AIM_STEP_COARSE, AIM_MIN_POSITION, AIM_MAX_POSITION);
        }
        if (aimDownPressed) {
            aimPosition = Range.clip(aimPosition - AIM_STEP_COARSE, AIM_MIN_POSITION, AIM_MAX_POSITION);
        }
        if (aimRightPressed) {
            aimPosition = Range.clip(aimPosition + AIM_STEP_FINE, AIM_MIN_POSITION, AIM_MAX_POSITION);
        }
        if (aimLeftPressed) {
            aimPosition = Range.clip(aimPosition - AIM_STEP_FINE, AIM_MIN_POSITION, AIM_MAX_POSITION);
        }
        launcherServo.setPosition(aimPosition);

        double measuredVelocity = outtake.getVelocity();
        advanceFireSequence(firePressed && !cancelPressed, measuredVelocity);

        driveMecanum();

        double intakePower;
        if (gamepad1.left_bumper) {
            intakePower = INTAKE_REVERSE_POWER;
        } else {
            intakePower = intakeOn ? INTAKE_POWER : 0.0;
        }
        intake.setPower(intakePower);

        double transferPower;
        if (fireState == FireState.SPIN_UP) {
            transferPower = 0.0;
        } else if (fireState == FireState.FEED) {
            transferPower = FEED_POWER;
        } else {
            transferPower = transferLatched ? TRANSFER_POWER : 0.0;
        }
        transfer.setPower(transferPower);

        if (fireState == FireState.SPIN_UP || fireState == FireState.FEED) {
            commandedOuttakeVelocity = OUTTAKE_VELOCITY;
        } else {
            commandedOuttakeVelocity = outtakeHolding ? OUTTAKE_VELOCITY : 0.0;
        }
        outtake.setVelocity(commandedOuttakeVelocity);

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

    private void advanceFireSequence(boolean fireRequested, double measuredVelocity) {
        switch (fireState) {
            case IDLE:
                fireWaitingFor = "right trigger";
                if (fireRequested) {
                    spinUpTimer.reset();
                    spinUpResult = "spinning up";
                    fireState = FireState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                fireWaitingFor = String.format("velocity within %.0f tps (%.1fs before timeout)",
                        OUTTAKE_TOLERANCE,
                        Math.max(0.0, SPINUP_TIMEOUT_SECONDS - spinUpTimer.seconds()));
                if (Math.abs(measuredVelocity - OUTTAKE_VELOCITY) <= OUTTAKE_TOLERANCE) {
                    spinUpResult = String.format("AT SPEED after %.2fs", spinUpTimer.seconds());
                    feedTimer.reset();
                    fireState = FireState.FEED;
                } else if (spinUpTimer.seconds() >= SPINUP_TIMEOUT_SECONDS) {
                    spinUpResult = String.format("TIMEOUT after %.2fs at %.0f tps",
                            spinUpTimer.seconds(), measuredVelocity);
                    feedTimer.reset();
                    fireState = FireState.FEED;
                }
                break;

            case FEED:
                fireWaitingFor = String.format("feed to finish (%.2fs left)",
                        Math.max(0.0, FEED_SECONDS - feedTimer.seconds()));
                if (feedTimer.seconds() >= FEED_SECONDS) {
                    fireState = FireState.FINISH;
                }
                break;

            case FINISH:
                outtakeHolding = KEEP_OUTTAKE_SPINNING;
                fireWaitingFor = "nothing";
                fireState = FireState.IDLE;
                break;
        }
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
        telemetry.addData("Fire state", fireState);
        telemetry.addData("Fire waiting for", fireWaitingFor);
        telemetry.addData("Spin-up result", spinUpResult);
        telemetry.addData("Outtake vel", "measured %.0f / target %.0f  tol %.0f",
                measuredVelocity, commandedOuttakeVelocity, OUTTAKE_TOLERANCE);
        telemetry.addData("Outtake hold", outtakeHolding ? "ON" : "OFF");
        if (outtakeEncoderWarning) {
            telemetry.addLine("WARNING outtake commanded but measured velocity is ~0 - check the outtake encoder cable");
        }

        telemetry.addData("Intake", "%s  power %.2f%s",
                intakeOn ? "ON" : "OFF", intakePower,
                gamepad1.left_bumper ? "  REVERSE held" : "");
        String transferOwner;
        if (fireState == FireState.SPIN_UP) {
            transferOwner = "  held off by fire sequence until at speed";
        } else if (fireState == FireState.FEED) {
            transferOwner = "  driven by fire sequence";
        } else {
            transferOwner = "";
        }
        telemetry.addData("Transfer", "latch %s  power %.2f%s",
                transferLatched ? "ON" : "OFF", transferPower, transferOwner);
        telemetry.addData("Aim position", "%.3f   range %.2f..%.2f",
                aimPosition, AIM_MIN_POSITION, AIM_MAX_POSITION);

        telemetry.addData("Slow mode", slowMode ? String.format("ON  %.2f", SLOW_SCALE) : "OFF");
        if (fieldCentric && !primaryHeadingValid) {
            telemetry.addLine("FIELD-CENTRIC REQUESTED BUT PRIMARY PINPOINT IS NOT READY - DRIVING ROBOT-CENTRIC");
        }
        telemetry.addData("Drive mode", fieldCentricActive ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Heading offset", "%.1f deg", Math.toDegrees(headingOffsetRadians));
        telemetry.addData("Heading reset", headingResetResult);

        telemetry.addData("Primary Pinpoint",
                PRIMARY_PINPOINT_IS_LEFT ? PINPOINT_LEFT_NAME : PINPOINT_RIGHT_NAME);
        telemetry.addData("Primary status", primaryStatus);
        telemetry.addData("Primary freq", "%.1f Hz", pinpointPrimary.getFrequency());
        telemetry.addData("Pinpoint L", "x %.1f in  y %.1f in  h %.1f deg  %s",
                pinpointLeft.getPosX(DistanceUnit.INCH),
                pinpointLeft.getPosY(DistanceUnit.INCH),
                pinpointLeft.getHeading(AngleUnit.DEGREES),
                pinpointLeft.getDeviceStatus());
        telemetry.addData("Pinpoint R", "x %.1f in  y %.1f in  h %.1f deg  %s",
                pinpointRight.getPosX(DistanceUnit.INCH),
                pinpointRight.getPosY(DistanceUnit.INCH),
                pinpointRight.getHeading(AngleUnit.DEGREES),
                pinpointRight.getDeviceStatus());
    }

    private void configurePinpoint(GoBildaPinpointDriver pinpoint, double xOffsetMm, double yOffsetMm,
                                   GoBildaPinpointDriver.EncoderDirection xDirection,
                                   GoBildaPinpointDriver.EncoderDirection yDirection) {
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
