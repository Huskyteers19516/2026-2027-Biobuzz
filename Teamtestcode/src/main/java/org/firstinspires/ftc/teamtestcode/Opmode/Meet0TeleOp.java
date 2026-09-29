package org.firstinspires.ftc.teamtestcode.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Meet0 TeleOp", group = "Meet0")
public class Meet0TeleOp extends OpMode {


    private static final double LAUNCHER_TARGET_VELOCITY = 1200.0;
    private static final double LAUNCHER_READY_TOLERANCE = 100.0;
    private static final double INTAKE_POWER = 1.0;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private CRServo leftIntake;
    private CRServo rightIntake;

    private DcMotorEx launcher;

    private final ElapsedTime runtime = new ElapsedTime();


    private boolean launcherOn = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftIntake = hardwareMap.get(CRServo.class, "left_intake");
        rightIntake = hardwareMap.get(CRServo.class, "right_intake");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAll();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);


        if (gamepad1.a) {
            launcherOn = true;
        }
        if (gamepad1.b) {
            launcherOn = false;
        }

        if (launcherOn) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setPower(0.0);
        }

        double launcherVelocity = launcher.getVelocity();
        boolean launcherReady = launcherOn
                && Math.abs(launcherVelocity - LAUNCHER_TARGET_VELOCITY) < LAUNCHER_READY_TOLERANCE;

        //
        double intakePower = 0.0;
        if (gamepad1.right_bumper) {
            intakePower = INTAKE_POWER;
        } else if (gamepad1.left_bumper) {
            intakePower = -INTAKE_POWER;
        } else if (gamepad1.right_trigger > 0.5 && launcherReady) {
            intakePower = INTAKE_POWER;
        }

        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        // ---- Telemetry ----
        telemetry.addData("Status", "Running");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("Drive", "L %.2f  R %.2f", leftPower, rightPower);
        telemetry.addData("Intake", "%.2f", intakePower);
        telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
        telemetry.addData("Launcher speed", "%.0f / %.0f ticks/s", launcherVelocity, LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Launcher ready", launcherReady);
    }

    @Override
    public void stop() {
        stopAll();
    }

    private void stopAll() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);
        launcher.setPower(0.0);
    }
}
