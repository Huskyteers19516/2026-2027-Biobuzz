package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.launcher.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Robot TeleOp", group = "Testing")
public class RobotTeleOp extends OpMode {

    public static final DcMotorSimple.Direction LEFT_SIDE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_SIDE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double SLOW_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.05;
    private static final double TRIGGER_PRESS_THRESHOLD = 0.5;
    private static final double TRIGGER_RELEASE_THRESHOLD = 0.3;

    public static final double INTAKE_POWER = 1.0;
    private static final double TELEOP_INTAKE_POWER = 0.8;
    private static final double INTAKE_REVERSE_POWER = -1.0;
    public static final double TRANSFER_POWER = 1.0;

    private static final double OUTTAKE_SMALL_STEP = 100.0;
    private static final double OUTTAKE_BIG_STEP = 500.0;
    private static final int READY_RUMBLE_MS = 200;

    private static final double SERVO_RANGE_DEGREES = 270.0;
    public static final double[] FLOWER_POSITIONS = {0.2, 0.5, 0.8};
    public static final int FLOWER_START_INDEX = 0;
    public static final double BLOCKER_CLOSED_POSITION = 0.5;
    private static final double BLOCKER_TURN_DEGREES = -50.0;
    public static final double BLOCKER_OPEN_POSITION =
            Range.clip(BLOCKER_CLOSED_POSITION + BLOCKER_TURN_DEGREES / SERVO_RANGE_DEGREES, 0.0, 1.0);

    public static final double AIM_START_POSITION = 0.50;
    public static final double AIM_MIN_POSITION = 0.00;
    public static final double AIM_MAX_POSITION = 1.00;

    private static final boolean SLOW_MODE_DEFAULT = false;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private Servo flowerIntake;
    private Launcher launcher;

    private boolean intakeOn = false;
    private boolean transferLatched = false;
    private int flowerIndex = FLOWER_START_INDEX;
    private boolean slowMode = SLOW_MODE_DEFAULT;

    private boolean leftTriggerLatched = false;
    private boolean rightTriggerLatched = false;
    private boolean wasReady = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_MOTOR_NAME);
        leftBack = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_MOTOR_NAME);
        rightFront = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_MOTOR_NAME);
        rightBack = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_MOTOR_NAME);
        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE);
        flowerIntake = hardwareMap.get(Servo.class, Constants.FLOWER_INTAKE);
        launcher = new Launcher(hardwareMap);
        launcher.setQueueExpireSeconds(Launcher.TELEOP_QUEUE_EXPIRE_SECONDS);

        leftFront.setDirection(LEFT_SIDE_DIRECTION);
        leftBack.setDirection(LEFT_SIDE_DIRECTION);
        rightFront.setDirection(RIGHT_SIDE_DIRECTION);
        rightBack.setDirection(RIGHT_SIDE_DIRECTION);
        intake.setDirection(INTAKE_DIRECTION);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Outtake velocity", "%.0f ticks/s", () -> launcher.measuredVelocity());

        telemetry.addData("Status", "Initialized - nothing is commanded until START");
        telemetry.addData("Servos move at START", "flower pos %d, blocker closed", FLOWER_START_INDEX + 1);
    }

    @Override
    public void init_loop() {
        launcher.sense();
        telemetry.addData("Status", "Init - waiting for START");
        telemetry.addLine("Spin the outtake wheel by hand: Outtake velocity must change.");
    }

    @Override
    public void start() {
        intakeOn = false;
        transferLatched = false;
        flowerIndex = FLOWER_START_INDEX;
        leftTriggerLatched = false;
        rightTriggerLatched = false;
        wasReady = false;
        launcher.setTargetVelocity(Launcher.DEFAULT_TARGET_VELOCITY);
        flowerIntake.setPosition(FLOWER_POSITIONS[flowerIndex]);
        launcher.update();
    }

    @Override
    public void loop() {
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
        double rightTriggerValue = gamepad1.right_trigger;
        boolean firePressed = triggerRisingEdge(rightTriggerValue, rightTriggerLatched);
        rightTriggerLatched = updateTriggerLatch(rightTriggerValue, rightTriggerLatched);
        boolean singleShotPressed = gamepad1.rightBumperWasPressed();
        boolean reversing = gamepad1.left_bumper;

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
            flowerIndex = Math.min(flowerIndex + 1, FLOWER_POSITIONS.length - 1);
        }
        if (flowerDownPressed) {
            flowerIndex = Math.max(flowerIndex - 1, 0);
        }
        if (velocityUpPressed) {
            launcher.setTargetVelocity(launcher.targetVelocity() + OUTTAKE_SMALL_STEP);
        }
        if (velocityDownPressed) {
            launcher.setTargetVelocity(launcher.targetVelocity() - OUTTAKE_SMALL_STEP);
        }
        if (velocityBigUpPressed) {
            launcher.setTargetVelocity(launcher.targetVelocity() + OUTTAKE_BIG_STEP);
        }
        if (velocityBigDownPressed) {
            launcher.setTargetVelocity(launcher.targetVelocity() - OUTTAKE_BIG_STEP);
        }

        if (reversing) {
            launcher.cancelShots();
        } else {
            if (firePressed && launcher.queuedShots() == 0) {
                launcher.requestShots(Launcher.BURST_SIZE);
            } else if (singleShotPressed && launcher.queuedShots() == 0) {
                launcher.requestShots(1);
            }
        }
        launcher.setManualBlockerOpen(gamepad2.x);

        driveMecanum();

        double intakePower;
        if (reversing) {
            intakePower = INTAKE_REVERSE_POWER;
        } else {
            intakePower = intakeOn ? TELEOP_INTAKE_POWER : 0.0;
        }
        intake.setPower(intakePower);

        double transferPower;
        if (reversing) {
            transferPower = -TRANSFER_POWER;
        } else {
            transferPower = (intakeOn || transferLatched) ? TRANSFER_POWER : 0.0;
        }
        launcher.setStagingTransferPower(transferPower);

        flowerIntake.setPosition(FLOWER_POSITIONS[flowerIndex]);
        launcher.update();

        boolean ready = launcher.isReady();
        if (ready && !wasReady) {
            gamepad1.rumble(READY_RUMBLE_MS);
        }
        wasReady = ready;

        addTelemetry(intakePower, transferPower);
    }

    @Override
    public void stop() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        intake.setPower(0.0);
        launcher.shutdown();
    }

    private void driveMecanum() {
        double axial = deadzone(-gamepad1.left_stick_y);
        double lateral = deadzone(gamepad1.left_stick_x);
        double yaw = deadzone(gamepad1.right_stick_x);

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

    private void addTelemetry(double intakePower, double transferPower) {
        launcher.addTelemetry(telemetry);
        telemetry.addData("Flower", "position %d / %d  pos %.2f",
                flowerIndex + 1, FLOWER_POSITIONS.length, flowerIntake.getPosition());
        telemetry.addData("Intake", "%s  power %.2f%s",
                intakeOn ? "ON" : "OFF", intakePower,
                gamepad1.left_bumper ? "  REVERSE held" : "");
        telemetry.addData("Transfer", "latch %s  staging power %.2f",
                transferLatched ? "ON" : "OFF", transferPower);
        telemetry.addData("Slow mode", slowMode ? String.format("ON  %.2f", SLOW_SCALE) : "OFF");
        telemetry.addLine("gp1 RT fire 3   gp1 RB fire 1   gp1 LB cancel   gp2 Y launcher   gp2 X hold = open blocker");
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
