package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "No Encoder TeleOp", group = "Testing")
public class NoEncoderTeleOp extends OpMode {

    private static final double LAUNCHER_POWER = 0.9;
    private static final double SPINUP_SECONDS = 1.5;
    private static final double FEED_SECONDS = 0.3;
    private static final double CLOSE_SECONDS = 0.15;
    private static final double RECOVER_SECONDS = 0.8;
    private static final int BURST_SIZE = 3;
    private static final double FEED_TRANSFER_POWER = 1.0;
    private static final double STAGING_POWER = 0.4;

    private static final double SLOW_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.05;
    private static final double TRIGGER_PRESS_THRESHOLD = 0.5;
    private static final double TRIGGER_RELEASE_THRESHOLD = 0.3;
    private static final double TELEOP_INTAKE_POWER = 0.8;
    private static final int READY_RUMBLE_MS = 200;

    private enum Phase { IDLE, FEEDING, CLOSING, RECOVERING }

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotor outtake;
    private Servo blocker;
    private Servo flowerIntake;

    private boolean launcherOn = false;
    private final ElapsedTime launcherTimer = new ElapsedTime();
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private Phase phase = Phase.IDLE;
    private int queuedShots = 0;
    private int shotsFired = 0;

    private boolean intakeOn = false;
    private boolean transferLatched = false;
    private boolean slowMode = false;
    private int flowerIndex = RobotTeleOp.FLOWER_START_INDEX;
    private boolean leftTriggerLatched = false;
    private boolean rightTriggerLatched = false;
    private boolean wasSpunUp = false;

    @Override
    public void init() {
        leftFront = initMotor(Constants.FRONT_LEFT_MOTOR_NAME, true);
        leftBack = initMotor(Constants.BACK_LEFT_MOTOR_NAME, true);
        rightFront = initMotor(Constants.FRONT_RIGHT_MOTOR_NAME, false);
        rightBack = initMotor(Constants.BACK_RIGHT_MOTOR_NAME, false);

        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE);
        intake.setDirection(RobotTeleOp.INTAKE_DIRECTION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setPower(0.0);

        transfer = hardwareMap.get(DcMotor.class, Constants.TRANSFER);
        transfer.setDirection(RobotTeleOp.TRANSFER_DIRECTION);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPower(0.0);

        outtake = hardwareMap.get(DcMotor.class, Constants.OUTTAKE);
        outtake.setDirection(RobotTeleOp.OUTTAKE_DIRECTION);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setPower(0.0);

        blocker = hardwareMap.get(Servo.class, Constants.OUTTAKE_BLOCKER);
        flowerIntake = hardwareMap.get(Servo.class, Constants.FLOWER_INTAKE);

        telemetry.addData("Status", "No Encoder TeleOp - launcher runs at fixed power %.2f", LAUNCHER_POWER);
        telemetry.addLine("Servos move at START");
    }

    @Override
    public void start() {
        blocker.setPosition(RobotTeleOp.BLOCKER_CLOSED_POSITION);
        flowerIntake.setPosition(RobotTeleOp.FLOWER_POSITIONS[flowerIndex]);
    }

    @Override
    public void loop() {
        if (gamepad1.leftStickButtonWasPressed()) {
            slowMode = !slowMode;
        }
        if (gamepad2.yWasPressed()) {
            setLauncher(!launcherOn);
        }
        if (gamepad2.aWasPressed() && !gamepad2.start) {
            transferLatched = true;
        }
        if (gamepad2.bWasPressed() && !gamepad2.start) {
            transferLatched = false;
        }
        if (gamepad2.rightBumperWasPressed()) {
            flowerIndex = Math.min(flowerIndex + 1, RobotTeleOp.FLOWER_POSITIONS.length - 1);
        }
        if (gamepad2.leftBumperWasPressed()) {
            flowerIndex = Math.max(flowerIndex - 1, 0);
        }

        double leftTrigger = gamepad1.left_trigger;
        if (!leftTriggerLatched && leftTrigger >= TRIGGER_PRESS_THRESHOLD) {
            intakeOn = !intakeOn;
        }
        leftTriggerLatched = latch(leftTrigger, leftTriggerLatched);

        double rightTrigger = gamepad1.right_trigger;
        boolean burstPressed = !rightTriggerLatched && rightTrigger >= TRIGGER_PRESS_THRESHOLD;
        rightTriggerLatched = latch(rightTrigger, rightTriggerLatched);
        boolean singlePressed = gamepad1.rightBumperWasPressed();
        boolean reversing = gamepad1.left_bumper;

        if (reversing) {
            queuedShots = 0;
        } else if (queuedShots == 0 && (burstPressed || singlePressed)) {
            queuedShots = burstPressed ? BURST_SIZE : 1;
            if (!launcherOn) {
                setLauncher(true);
            }
        }

        boolean spunUp = launcherOn && launcherTimer.seconds() >= SPINUP_SECONDS;
        if (spunUp && !wasSpunUp) {
            gamepad1.rumble(READY_RUMBLE_MS);
        }
        wasSpunUp = spunUp;

        updatePhase(spunUp);

        double stagingPower;
        if (reversing) {
            stagingPower = -RobotTeleOp.TRANSFER_POWER;
        } else {
            stagingPower = (intakeOn || transferLatched) ? RobotTeleOp.TRANSFER_POWER : 0.0;
            if (queuedShots > 0) {
                stagingPower = Math.max(stagingPower, STAGING_POWER);
            }
        }

        double transferPower;
        if (phase == Phase.FEEDING) {
            transferPower = FEED_TRANSFER_POWER;
        } else if (phase == Phase.CLOSING) {
            transferPower = 0.0;
        } else {
            transferPower = stagingPower;
        }
        transfer.setPower(transferPower);

        double intakePower = reversing ? -1.0 : (intakeOn ? TELEOP_INTAKE_POWER : 0.0);
        intake.setPower(intakePower);

        boolean blockerOpen = phase == Phase.FEEDING || gamepad2.x;
        blocker.setPosition(blockerOpen ? RobotTeleOp.BLOCKER_OPEN_POSITION : RobotTeleOp.BLOCKER_CLOSED_POSITION);
        outtake.setPower(launcherOn ? LAUNCHER_POWER : 0.0);
        flowerIntake.setPosition(RobotTeleOp.FLOWER_POSITIONS[flowerIndex]);

        drive();

        telemetry.addData("Launcher", "%s  fixed power %.2f%s", launcherOn ? "ON" : "OFF", LAUNCHER_POWER,
                launcherOn && !spunUp ? String.format("  spinning up %.1f s", SPINUP_SECONDS - launcherTimer.seconds()) : "");
        telemetry.addData("Shooting", "%s  queued %d  fired %d", phase, queuedShots, shotsFired);
        telemetry.addData("Blocker", blockerOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Intake / Transfer", "%.2f / %.2f", intakePower, transferPower);
        telemetry.addData("Flower", "position %d / %d", flowerIndex + 1, RobotTeleOp.FLOWER_POSITIONS.length);
        telemetry.addData("Slow mode", slowMode ? "ON" : "OFF");
        telemetry.addLine("gp1 RT fire 3  RB fire 1  LB cancel/reverse  LT intake   gp2 Y launcher  X hold open blocker");
    }

    @Override
    public void stop() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        intake.setPower(0.0);
        transfer.setPower(0.0);
        outtake.setPower(0.0);
    }

    private void updatePhase(boolean spunUp) {
        if (!launcherOn) {
            phase = Phase.IDLE;
            queuedShots = 0;
            return;
        }
        switch (phase) {
            case IDLE:
                if (queuedShots > 0 && spunUp) {
                    startPhase(Phase.FEEDING);
                }
                break;
            case FEEDING:
                if (phaseTimer.seconds() >= FEED_SECONDS) {
                    queuedShots = Math.max(0, queuedShots - 1);
                    shotsFired++;
                    startPhase(Phase.CLOSING);
                }
                break;
            case CLOSING:
                if (phaseTimer.seconds() >= CLOSE_SECONDS) {
                    startPhase(Phase.RECOVERING);
                }
                break;
            case RECOVERING:
                if (phaseTimer.seconds() >= RECOVER_SECONDS) {
                    startPhase(Phase.IDLE);
                }
                break;
            default:
                break;
        }
    }

    private void startPhase(Phase next) {
        phase = next;
        phaseTimer.reset();
    }

    private void setLauncher(boolean on) {
        launcherOn = on;
        launcherTimer.reset();
        if (!on) {
            queuedShots = 0;
            phase = Phase.IDLE;
        }
    }

    private void drive() {
        double axial = deadzone(-gamepad1.left_stick_y);
        double lateral = deadzone(gamepad1.left_stick_x);
        double yaw = deadzone(gamepad1.right_stick_x);

        double lf = axial + lateral + yaw;
        double lb = axial - lateral + yaw;
        double rf = axial - lateral - yaw;
        double rb = axial + lateral - yaw;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb))));
        double scale = slowMode ? SLOW_SCALE : 1.0;

        leftFront.setPower(lf / max * scale);
        leftBack.setPower(lb / max * scale);
        rightFront.setPower(rf / max * scale);
        rightBack.setPower(rb / max * scale);
    }

    private DcMotor initMotor(String name, boolean left) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(left ? RobotTeleOp.LEFT_SIDE_DIRECTION : RobotTeleOp.RIGHT_SIDE_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
        return motor;
    }

    private static boolean latch(double value, boolean latched) {
        if (value >= TRIGGER_PRESS_THRESHOLD) {
            return true;
        }
        if (value <= TRIGGER_RELEASE_THRESHOLD) {
            return false;
        }
        return latched;
    }

    private static double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
