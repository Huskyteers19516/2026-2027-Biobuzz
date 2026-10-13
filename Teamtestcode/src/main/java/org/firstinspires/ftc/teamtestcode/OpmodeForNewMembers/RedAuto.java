package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Auto", group = "Meet0")
public class RedAuto extends LinearOpMode {

    private static final int RED_TURN_90 = 500;
    private static final int RED_TURN_180 = 1000;

    private static final int RED_DRIVE_ACROSS = 3400;
    private static final int RED_DRIVE_PARK = 1600;

    private static final boolean RED_PASSAGE_CLEAR = false;
    private static final boolean RED_EXTRA_SHOT = true;

    private static final double RED_TEAMMATE_WAIT_SECONDS = 5.0;

    private static final double DRIVE_POWER = 0.35;
    private static final double TURN_POWER = 0.25;
    private static final double DRIVE_TIMEOUT_SECONDS = 8.0;
    private static final double TURN_TIMEOUT_SECONDS = 4.0;

    private static final double LAUNCHER_POWER = 0.8;
    private static final double LAUNCHER_SPINUP_SECONDS = 1.5;
    private static final double FEED_POWER = 0.8;
    private static final double FEED_SECONDS = 2.0;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private CRServo leftIntake;
    private CRServo rightIntake;
    private DcMotorEx launcher;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
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

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAll();

        while (opModeInInit()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Passage clear", RED_PASSAGE_CLEAR ? "yes" : "NO - route disabled");
            telemetry.addData("Extra shot", RED_EXTRA_SHOT ? "yes" : "no");
            telemetry.addData("Teammate wait", "%.1f s", RED_TEAMMATE_WAIT_SECONDS);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        runtime.reset();

        try {
            redFig2();
        } finally {
            stopAll();
            setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Status", "Finished");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.update();
    }

    private void redFig2() {
        if (!RED_PASSAGE_CLEAR) {
            telemetry.addData("Status", "Red passage not confirmed");
            telemetry.addLine("Set RED_PASSAGE_CLEAR = true once the route is measured.");
            telemetry.update();
            return;
        }

        telemetry.addData("Step", "Driving across");
        telemetry.update();
        if (!redDrive(RED_DRIVE_ACROSS)) return;

        if (RED_EXTRA_SHOT) {
            telemetry.addData("Step", "Spinning 180 to face the hive");
            telemetry.update();
            if (!redTurn(-RED_TURN_180)) return;

            telemetry.addData("Step", "Waiting for teammate");
            telemetry.update();
            if (!redWait(RED_TEAMMATE_WAIT_SECONDS)) return;

            telemetry.addData("Step", "Shooting pollen");
            telemetry.update();
            if (!redShoot()) return;

            telemetry.addData("Step", "Turning toward the red zone");
            telemetry.update();
            if (!redTurn(-RED_TURN_90)) return;
        } else {
            telemetry.addData("Step", "Turning toward the red zone");
            telemetry.update();
            if (!redTurn(RED_TURN_90)) return;
        }

        telemetry.addData("Step", "Parking in the red zone");
        telemetry.update();
        redDrive(RED_DRIVE_PARK);
    }

    private boolean redDrive(int ticks) {
        return redMove(DRIVE_POWER, ticks, ticks, DRIVE_TIMEOUT_SECONDS);
    }

    private boolean redTurn(int ticks) {
        return redMove(TURN_POWER, ticks, -ticks, TURN_TIMEOUT_SECONDS);
    }

    private boolean redDriveBusy() {
        return leftFront.isBusy() || leftBack.isBusy()
                || rightFront.isBusy() || rightBack.isBusy();
    }

    private boolean redMove(double power, int leftTicks, int rightTicks, double timeoutSeconds) {
        if (!opModeIsActive()) return false;
        if (leftTicks == 0 && rightTicks == 0) return true;

        try {
            setMotorPower(0);
            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setTargetPosition(leftTicks);
            leftBack.setTargetPosition(leftTicks);
            rightFront.setTargetPosition(rightTicks);
            rightBack.setTargetPosition(rightTicks);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            ElapsedTime moveTimer = new ElapsedTime();
            setMotorPower(Math.abs(power));

            while (opModeIsActive()
                    && moveTimer.seconds() < timeoutSeconds
                    && redDriveBusy()) {
                idle();
            }

            boolean reached = opModeIsActive() && !redDriveBusy();

            if (!reached) {
                telemetry.addData("Status", "Route stopped: move incomplete");
                telemetry.update();
            }

            return reached;
        } finally {
            setMotorPower(0);
            setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private boolean redWait(double seconds) {
        ElapsedTime waitTimer = new ElapsedTime();

        while (opModeIsActive() && waitTimer.seconds() < seconds) {
            telemetry.addData("Step", "Waiting for teammate");
            telemetry.addData("Elapsed", "%.1f / %.1f s", waitTimer.seconds(), seconds);
            telemetry.update();
            idle();
        }

        return opModeIsActive();
    }

    private boolean redShoot() {
        if (!opModeIsActive()) return false;

        try {
            launcher.setPower(LAUNCHER_POWER);
            sleep((long) (LAUNCHER_SPINUP_SECONDS * 1000));

            if (!opModeIsActive()) return false;

            leftIntake.setPower(FEED_POWER);
            rightIntake.setPower(FEED_POWER);
            sleep((long) (FEED_SECONDS * 1000));

            return opModeIsActive();
        } finally {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            launcher.setPower(0);
        }
    }

    private void stopAll() {
        setMotorPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        launcher.setPower(0);
    }

    private void setMotorPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }
}
