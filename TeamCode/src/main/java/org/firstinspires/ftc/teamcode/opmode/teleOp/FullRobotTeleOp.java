package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FLOWER_INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE_BLOCKER;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.TRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Full Robot TeleOp", group = "Match")
public class FullRobotTeleOp extends OpMode {

    private static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = RobotTeleOp.LEFT_SIDE_DIRECTION;
    private static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = RobotTeleOp.RIGHT_SIDE_DIRECTION;
    private static final DcMotorSimple.Direction INTAKE_DIRECTION = RobotTeleOp.INTAKE_DIRECTION;
    private static final DcMotorSimple.Direction TRANSFER_DIRECTION = RobotTeleOp.TRANSFER_DIRECTION;
    private static final DcMotorSimple.Direction OUTTAKE_DIRECTION = RobotTeleOp.OUTTAKE_DIRECTION;

    private static final double STICK_DEADZONE = 0.05;
    private static final double TRIGGER_DEADZONE = 0.05;

    private static final double OUTTAKE_START_VELOCITY = RobotTeleOp.OUTTAKE_VELOCITY;
    private static final double OUTTAKE_MIN_VELOCITY = 0.0;
    private static final double OUTTAKE_MAX_VELOCITY = 6000.0;
    private static final double OUTTAKE_SMALL_STEP = 100.0;
    private static final double OUTTAKE_BIG_STEP = 500.0;

    private static final double[] FLOWER_POSITIONS = RobotTeleOp.FLOWER_POSITIONS;
    private static final double BLOCKER_CLOSED_POSITION = RobotTeleOp.BLOCKER_CLOSED_POSITION;
    private static final double BLOCKER_OPEN_POSITION = RobotTeleOp.BLOCKER_OPEN_POSITION;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo flowerIntake;
    private Servo blocker;

    private double outtakeVelocity = OUTTAKE_START_VELOCITY;
    private boolean outtakeOn = false;
    private int flowerIndex = RobotTeleOp.FLOWER_START_INDEX;
    private boolean blockerOpen = false;

    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastB = false;
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
        transfer = initRollerMotor(TRANSFER, TRANSFER_DIRECTION, DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = initOuttake(OUTTAKE, OUTTAKE_DIRECTION);

        flowerIntake = hardwareMap.get(Servo.class, FLOWER_INTAKE);
        blocker = hardwareMap.get(Servo.class, OUTTAKE_BLOCKER);

        telemetry.addData("Status", "Initialized - servos move at START");
        showControls();
    }

    @Override
    public void start() {
        updateServos();
    }

    @Override
    public void loop() {
        drive();
        runIntakeAndTransfer();
        handleButtons();
        runOuttake();
        updateServos();
        showTelemetry();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
        outtake.setVelocity(0);
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
        transfer.setPower(transferPower);
    }

    private void handleButtons() {
        boolean a = gamepad1.a;
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean right = gamepad1.dpad_right;
        boolean left = gamepad1.dpad_left;
        boolean RT  = gamepad1.right_bumper;

        if(RT && !lastRT){
            outtakeOn = !outtakeOn;
        }

        if (a && !lastA) {
            outtakeOn = !outtakeOn;
        }
        if (x && !lastX) {
            flowerIndex = (flowerIndex + 1) % FLOWER_POSITIONS.length;
        }
        if (b && !lastB) {
            blockerOpen = !blockerOpen;
        }
        if (up && !lastUp) {
            outtakeVelocity += OUTTAKE_SMALL_STEP;
        }
        if (down && !lastDown) {
            outtakeVelocity -= OUTTAKE_SMALL_STEP;
        }
        if (right && !lastRight) {
            outtakeVelocity += OUTTAKE_BIG_STEP;
        }
        if (left && !lastLeft) {
            outtakeVelocity -= OUTTAKE_BIG_STEP;
        }
        outtakeVelocity = Range.clip(outtakeVelocity, OUTTAKE_MIN_VELOCITY, OUTTAKE_MAX_VELOCITY);

        lastA = a;
        lastX = x;
        lastB = b;
        lastUp = up;
        lastDown = down;
        lastRight = right;
        lastLeft = left;
    }

    private void runOuttake() {
        outtake.setVelocity(outtakeOn ? outtakeVelocity : 0.0);
    }

    private void updateServos() {
        flowerIntake.setPosition(FLOWER_POSITIONS[flowerIndex]);
        blocker.setPosition(blockerOpen ? BLOCKER_OPEN_POSITION : BLOCKER_CLOSED_POSITION);
    }

    private void showTelemetry() {
        telemetry.addData("Outtake", "%s  target %.0f  measured %.0f ticks/s",
                outtakeOn ? "ON" : "OFF", outtakeVelocity, outtake.getVelocity());
        telemetry.addData("Intake / Transfer", "%.2f / %.2f", intake.getPower(), transfer.getPower());
        telemetry.addData("Flower", "position %d / %d  pos %.2f",
                flowerIndex + 1, FLOWER_POSITIONS.length, flowerIntake.getPosition());
        telemetry.addData("Blocker", "%s  pos %.2f", blockerOpen ? "OPEN" : "CLOSED", blocker.getPosition());
        showControls();
    }

    private void showControls() {
        telemetry.addLine("Sticks: mecanum drive");
        telemetry.addLine("LT intake  RT transfer  LB/RB reverse them");
        telemetry.addLine("A outtake on/off  dpad up/down +-100  right/left +-500");
        telemetry.addLine("X flower next position  B blocker");
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

    private DcMotorEx initOuttake(String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    private static double deadzone(double value, double zone) {
        return Math.abs(value) < zone ? 0.0 : value;
    }
}
