package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FLOWER_INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.launcher.Launcher;

@TeleOp(name = "Full Robot TeleOp", group = "Match")
public class FullRobotTeleOp extends OpMode {

    private static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = RobotTeleOp.LEFT_SIDE_DIRECTION;
    private static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = RobotTeleOp.RIGHT_SIDE_DIRECTION;
    private static final DcMotorSimple.Direction INTAKE_DIRECTION = RobotTeleOp.INTAKE_DIRECTION;

    private static final double STICK_DEADZONE = 0.05;
    private static final double TRIGGER_DEADZONE = 0.05;

    private static final double OUTTAKE_SMALL_STEP = 100.0;
    private static final double OUTTAKE_BIG_STEP = 500.0;
    private static final int READY_RUMBLE_MS = 200;

    private static final double[] FLOWER_POSITIONS = RobotTeleOp.FLOWER_POSITIONS;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private Servo flowerIntake;
    private Launcher launcher;

    private int flowerIndex = RobotTeleOp.FLOWER_START_INDEX;
    private boolean wasReady = false;

    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastRight = false;
    private boolean lastLeft = false;
    private boolean lastRT = false;

    @Override
    public void init() {
        leftFront = initDriveMotor(FRONT_LEFT_MOTOR_NAME, LEFT_DRIVE_DIRECTION);
        leftBack = initDriveMotor(BACK_LEFT_MOTOR_NAME, LEFT_DRIVE_DIRECTION);
        rightFront = initDriveMotor(FRONT_RIGHT_MOTOR_NAME, RIGHT_DRIVE_DIRECTION);
        rightBack = initDriveMotor(BACK_RIGHT_MOTOR_NAME, RIGHT_DRIVE_DIRECTION);

        intake = initRollerMotor(INTAKE, INTAKE_DIRECTION, DcMotor.ZeroPowerBehavior.FLOAT);

        launcher = new Launcher(hardwareMap);
        launcher.setQueueExpireSeconds(Launcher.TELEOP_QUEUE_EXPIRE_SECONDS);
        telemetry.addData("Outtake velocity", "%.0f ticks/s", () -> launcher.measuredVelocity());

        flowerIntake = hardwareMap.get(Servo.class, FLOWER_INTAKE);

        telemetry.addData("Status", "Initialized - servos move at START");
        showControls();
    }

    @Override
    public void init_loop() {
        launcher.sense();
    }

    @Override
    public void start() {
        launcher.setTargetVelocity(Launcher.DEFAULT_TARGET_VELOCITY);
        flowerIntake.setPosition(FLOWER_POSITIONS[flowerIndex]);
        launcher.update();
    }

    @Override
    public void loop() {
        drive();
        handleButtons();
        runIntakeAndTransfer();
        flowerIntake.setPosition(FLOWER_POSITIONS[flowerIndex]);
        launcher.update();

        boolean ready = launcher.isReady();
        if (ready && !wasReady) {
            gamepad1.rumble(READY_RUMBLE_MS);
        }
        wasReady = ready;

        showTelemetry();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        launcher.shutdown();
    }

    private void drive() {
        double forward = deadzone(-gamepad1.left_stick_y, STICK_DEADZONE);
        double strafe = deadzone(gamepad1.left_stick_x, STICK_DEADZONE);
        double turn = deadzone(gamepad1.right_stick_x, STICK_DEADZONE);

        double lf = forward + strafe + turn;
        double lb = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rb = forward + strafe - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));

        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }

    private void runIntakeAndTransfer() {
        double intakePower = deadzone(gamepad1.left_trigger, TRIGGER_DEADZONE);
        double transferPower = deadzone(gamepad1.right_trigger, TRIGGER_DEADZONE);
        if (gamepad1.left_bumper) {
            intakePower = -1.0;
        }
        if (gamepad1.right_bumper) {
            transferPower = -1.0;
        }
        intake.setPower(intakePower);
        launcher.setStagingTransferPower(transferPower);
    }

    private void handleButtons() {
        boolean a = gamepad1.a && !gamepad1.start;
        boolean x = gamepad1.x;
        boolean b = gamepad1.b && !gamepad1.start;
        boolean y = gamepad1.y;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean right = gamepad1.dpad_right;
        boolean left = gamepad1.dpad_left;
        boolean RT = gamepad1.right_bumper;

        if (RT && !lastRT) {
            launcher.toggle();
        }
        if (a && !lastA) {
            launcher.toggle();
        }
        if (x && !lastX) {
            flowerIndex = (flowerIndex + 1) % FLOWER_POSITIONS.length;
        }
        if (b && !lastB && launcher.queuedShots() == 0) {
            launcher.requestShots(Launcher.BURST_SIZE);
        } else if (y && !lastY && launcher.queuedShots() == 0) {
            launcher.requestShots(1);
        }
        if (up && !lastUp) {
            launcher.setTargetVelocity(launcher.targetVelocity() + OUTTAKE_SMALL_STEP);
        }
        if (down && !lastDown) {
            launcher.setTargetVelocity(launcher.targetVelocity() - OUTTAKE_SMALL_STEP);
        }
        if (right && !lastRight) {
            launcher.setTargetVelocity(launcher.targetVelocity() + OUTTAKE_BIG_STEP);
        }
        if (left && !lastLeft) {
            launcher.setTargetVelocity(launcher.targetVelocity() - OUTTAKE_BIG_STEP);
        }

        lastA = a;
        lastX = x;
        lastB = b;
        lastY = y;
        lastUp = up;
        lastDown = down;
        lastRight = right;
        lastLeft = left;
        lastRT = RT;
    }

    private void showTelemetry() {
        launcher.addTelemetry(telemetry);
        telemetry.addData("Intake / Transfer", "%.2f / %.2f", intake.getPower(), gamepad1.right_trigger);
        telemetry.addData("Flower", "position %d / %d  pos %.2f",
                flowerIndex + 1, FLOWER_POSITIONS.length, flowerIntake.getPosition());
        showControls();
    }

    private void showControls() {
        telemetry.addLine("Sticks: mecanum drive");
        telemetry.addLine("LT intake  RT transfer  LB/RB reverse them");
        telemetry.addLine("A or RB launcher on/off  dpad up/down +-100  right/left +-500");
        telemetry.addLine("X flower next position  B fire 3  Y fire 1");
    }

    private DcMotor initDriveMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        return motor;
    }

    private DcMotor initRollerMotor(String name, DcMotorSimple.Direction direction,
                                    DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setPower(0);
        return motor;
    }

    private static double deadzone(double value, double zone) {
        return Math.abs(value) < zone ? 0.0 : value;
    }
}
