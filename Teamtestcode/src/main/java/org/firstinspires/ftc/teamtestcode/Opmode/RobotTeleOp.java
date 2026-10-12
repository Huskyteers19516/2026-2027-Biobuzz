package org.firstinspires.ftc.teamtestcode.Opmode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamtestcode.pedropathing.Constants;

/**
 * Match TeleOp: 4 mecanum drive motors + intake + transfer + outtake (7 motors) and 1 servo.
 *
 * Drive (gamepad1 only):
 *   left stick = move, right stick X = turn, left trigger = slow mode
 * Mechanisms (gamepad1 OR gamepad2, same buttons on both):
 *   right bumper  hold  = intake in
 *   left bumper   hold  = eject (intake + transfer backwards)
 *   Y             press = outtake on / off
 *   B             press = outtake off
 *   dpad up/down  press = outtake speed +/-
 *   right trigger hold  = FIRE (transfer feeds only while the outtake is at speed)
 * There is no way to feed a ball into a slow flywheel. To clear a jam, eject.
 *
 * Everything marked UNCONFIRMED below is a guess until it is checked on the real robot.
 * The drive motor names and directions live in pedropathing/Constants.java.
 */
@TeleOp(name = "Robot TeleOp", group = "Match")
public class RobotTeleOp extends OpMode {

    // ---------- Config names (must match the Driver Station configuration) ----------
    public static final String INTAKE_NAME = "intake";                  // UNCONFIRMED
    public static final String TRANSFER_NAME = "transfer";              // UNCONFIRMED
    public static final String OUTTAKE_NAME = "outtake";                // UNCONFIRMED
    public static final String LAUNCHER_SERVO_NAME = "launcher_servo";  // UNCONFIRMED

    // ---------- Directions: positive power must pull a ball IN / feed it UP / shoot it OUT ----------
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;   // UNCONFIRMED
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD; // UNCONFIRMED
    public static final DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;  // UNCONFIRMED

    // ---------- Powers (-1.0 to 1.0) ----------
    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSFER_FEED_POWER = 1.0;
    public static final double TRANSFER_POWER_WHILE_INTAKING = 0.0;     // UNCONFIRMED: raise if balls stall before the transfer
    public static final double EJECT_POWER = -1.0;

    // ---------- Outtake flywheel (encoder ticks per second) ----------
    public static final double OUTTAKE_TARGET_VELOCITY = 1500.0;        // UNCONFIRMED: tune by shooting
    public static final double OUTTAKE_READY_TOLERANCE = 100.0;         // UNCONFIRMED
    public static final double OUTTAKE_VELOCITY_STEP = 50.0;
    public static final double OUTTAKE_MIN_VELOCITY = 500.0;            // UNCONFIRMED
    public static final double OUTTAKE_MAX_VELOCITY = 2600.0;           // UNCONFIRMED: depends on motor + gearing

    // ---------- Launcher servo (0.0 to 1.0) ----------
    // The servo is NOT commanded at all until these two positions are measured on the
    // robot and SERVO_POSITIONS_CONFIRMED is set to true. A wrong position can jam or
    // break the mechanism.
    public static final boolean SERVO_POSITIONS_CONFIRMED = false;
    public static final double LAUNCHER_SERVO_REST = 0.5;               // UNCONFIRMED placeholder
    public static final double LAUNCHER_SERVO_FIRE = 0.5;               // UNCONFIRMED placeholder

    // ---------- Driving ----------
    private static final double STICK_DEADZONE = 0.05;
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;
    private static final double TRIGGER_THRESHOLD = 0.5;

    private Drivetrain drivetrain;
    private DcMotor[] wheels = new DcMotor[4];
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotorEx outtake;
    private Servo launcherServo;

    private boolean outtakeOn = false;
    private double outtakeTarget = OUTTAKE_TARGET_VELOCITY;

    @Override
    public void init() {
        try {
            setUpHardware();
        } catch (RuntimeException e) {
            stopAll();
            throw e;
        }
    }

    private void setUpHardware() {
        wheels = driveMotors(hardwareMap);
        drivetrain = Constants.createDrivetrain(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER_NAME);
        outtake = hardwareMap.get(DcMotorEx.class, OUTTAKE_NAME);
        launcherServo = hardwareMap.get(Servo.class, LAUNCHER_SERVO_NAME);

        intake.setDirection(INTAKE_DIRECTION);
        transfer.setDirection(TRANSFER_DIRECTION);
        outtake.setDirection(OUTTAKE_DIRECTION);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAll();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Robot TeleOp");
        telemetry.addLine("Drive: left stick move | right stick X turn | LT slow");
        telemetry.addLine("RB intake | LB eject | Y outtake on/off | B outtake off");
        telemetry.addLine("RT fire (needs outtake at speed) | dpad outtake speed");
        addServoWarning();
        telemetry.addLine("Nothing is commanded until START");
        telemetry.update();
    }

    @Override
    public void start() {
        outtakeOn = false;
        outtakeTarget = OUTTAKE_TARGET_VELOCITY;
        if (SERVO_POSITIONS_CONFIRMED) {
            launcherServo.setPosition(LAUNCHER_SERVO_REST);
        }
    }

    @Override
    public void loop() {
        try {
            runLoop();
        } catch (RuntimeException e) {
            stopAll();
            throw e;
        }
    }

    private void runLoop() {
        // ---- Drive ----
        double speed = MechanismLogic.driveScale(gamepad1.left_trigger, NORMAL_SPEED, SLOW_SPEED);
        double forward = MechanismLogic.deadzone(-gamepad1.left_stick_y, STICK_DEADZONE) * speed;
        double strafe = MechanismLogic.deadzone(-gamepad1.left_stick_x, STICK_DEADZONE) * speed;
        double turn = MechanismLogic.deadzone(-gamepad1.right_stick_x, STICK_DEADZONE) * speed;
        drivetrain.drive(new DrivePowers(forward, strafe, turn), true);

        // ---- Outtake flywheel ----
        if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) {
            outtakeOn = !outtakeOn;
        }
        if (gamepad1.bWasPressed() | gamepad2.bWasPressed()) {
            outtakeOn = false;
        }
        if (gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed()) {
            outtakeTarget += OUTTAKE_VELOCITY_STEP;
        }
        if (gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed()) {
            outtakeTarget -= OUTTAKE_VELOCITY_STEP;
        }
        outtakeTarget = MechanismLogic.clip(outtakeTarget, OUTTAKE_MIN_VELOCITY, OUTTAKE_MAX_VELOCITY);

        if (outtakeOn) {
            outtake.setVelocity(outtakeTarget);
        } else {
            outtake.setPower(0.0);
        }

        double outtakeVelocity = outtake.getVelocity();
        boolean outtakeReady = MechanismLogic.outtakeReady(
                outtakeOn, outtakeVelocity, outtakeTarget, OUTTAKE_READY_TOLERANCE);

        // ---- Intake / transfer / servo ----
        boolean intakeHeld = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean ejectHeld = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean fireHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD
                || gamepad2.right_trigger > TRIGGER_THRESHOLD;

        MechanismLogic.Command command = MechanismLogic.decide(
                intakeHeld, ejectHeld, fireHeld, outtakeReady);

        intake.setPower(command.intakePower);
        transfer.setPower(command.transferPower);
        if (SERVO_POSITIONS_CONFIRMED) {
            launcherServo.setPosition(command.servoFire ? LAUNCHER_SERVO_FIRE : LAUNCHER_SERVO_REST);
        }

        // ---- Telemetry ----
        telemetry.addData("State", command.state);
        telemetry.addData("Drive", "fwd %.2f  left %.2f  turn %.2f  (x%.2f)", forward, strafe, turn, speed);
        telemetry.addData("Intake / Transfer", "%.2f / %.2f", command.intakePower, command.transferPower);
        telemetry.addData("Outtake", outtakeOn ? (outtakeReady ? "ON - READY" : "ON - spinning up") : "OFF");
        telemetry.addData("Outtake speed", "%.0f / %.0f ticks/s", outtakeVelocity, outtakeTarget);
        telemetry.addData("Servo", SERVO_POSITIONS_CONFIRMED ? (command.servoFire ? "FIRE" : "rest") : "DISABLED");
        addServoWarning();
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAll();
    }

    private void addServoWarning() {
        if (!SERVO_POSITIONS_CONFIRMED) {
            telemetry.addLine("!! SERVO POSITIONS NOT CONFIRMED - servo is disabled");
        }
    }

    // Safe to call even if init() failed half way. Each of the 7 motors is zeroed on its
    // own, so one failing device does not stop the others from being zeroed.
    private void stopAll() {
        outtakeOn = false;
        RuntimeException failure = MechanismLogic.runAll(
                () -> zero(wheels[0]),
                () -> zero(wheels[1]),
                () -> zero(wheels[2]),
                () -> zero(wheels[3]),
                () -> zero(intake),
                () -> zero(transfer),
                () -> zero(outtake),
                () -> { if (drivetrain != null) drivetrain.stop(); });
        reportStopFailure("RobotTeleOp", failure, telemetry);
    }

    // ---------- Shared with MatchAuto ----------

    /** The 4 drive motors themselves, so each one can be stopped without going through Pedro. */
    static DcMotor[] driveMotors(HardwareMap hardwareMap) {
        return new DcMotor[]{
                hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_MOTOR_NAME)};
    }

    static void zero(DcMotor motor) {
        if (motor != null) {
            motor.setPower(0.0);
        }
    }

    static void reportStopFailure(String tag, RuntimeException failure,
                                  org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        if (failure != null) {
            RobotLog.ee(tag, failure, "a device failed while stopping");
            telemetry.addData("!! STOP FAILED", failure.toString());
        }
    }
}
