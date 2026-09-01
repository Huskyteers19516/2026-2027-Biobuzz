package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
//@Disabled
public class testclass extends OpMode {

    // --- Tunables ---
    final double FEED_TIME_SECONDS = 0.20;   // how long the feeders run for one shot
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Realistic launcher velocity targets (ticks per second). Adjust ±200–300 in testing.
    final double LAUNCHER_TARGET_VELOCITY = 6200;
    final double LAUNCHER_MIN_VELOCITY    = 5900;

    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // State machine for launching
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState;

    // Drive telemetry
    double leftPower;
    double rightPower;

    // Timing for feeder pulse
    ElapsedTime feederTimer = new ElapsedTime();

    // Edge-detect for right bumper
    private boolean lastRB = false;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        // Map hardware (names must match Robot Configuration)
        leftDrive  = hardwareMap.get(DcMotor.class,   "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class,   "right_drive");
        launcher   = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class,   "87");
        rightFeeder= hardwareMap.get(CRServo.class,   "right_feeder");

        // Drive directions (adjust if your drivetrain is different)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Launcher encoder setup
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brakes for crisp stopping
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Initialize feeders stopped
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // Starter PIDF (tune as needed for your motor/ratio)
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 2, 16)
        );

        // Make both feeders move game piece the same way (flip if your mechanism needs it)
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightFeeder keeps default Direction.FORWARD

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void loop() {
        // --- Drive (arcade) ---
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // --- Manual flywheel control ---
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }

        // --- One-shot on Right Bumper (edge detect) ---
        boolean shotRequested = gamepad1.right_bumper && !lastRB;
        lastRB = gamepad1.right_bumper;
        launch(shotRequested);

        // --- Telemetry ---
        telemetry.addData("State", launchState);
        telemetry.addData("Drive", "L: %.2f  R: %.2f", leftPower, rightPower);
        telemetry.addData("Launcher tps", launcher.getVelocity());
    }

    @Override
    public void stop() { }

    // Arcade drive helper
    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    // Launcher state machine: spin up -> feed -> stop
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();                // start pulse timing
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launcher.setVelocity(0);        // stop flywheel after shot
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}