package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Meet0 Auto", group = "Meet0")
public class Meet0Auto extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private CRServo leftIntake;
    private CRServo rightIntake;

    private DcMotorEx launcher;

    private final ElapsedTime runtime = new ElapsedTime();

    private void setMotorMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setMotorPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }


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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Auto Path", "Driving out of the starting zone");
            telemetry.update();
            driveStraight(0.5,1200);
            sleep(500);

            // this code should make it leave the starting area .


            telemetry.addData("Auto Path", "Parking robot");
            telemetry.update();
            driveStraight(-0.3,400);
            // back up and return to parking
        }

        if (isStopRequested()) {
            return;
        }

        runtime.reset();

        stopAll();

        telemetry.addData("Status", "Finished");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.update();
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

    public void driveStraight(double power, int ticks) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(power);

        while (opModeIsActive() && leftFront.isBusy()) {

        }
        setMotorPower(0);
    }

    // rotate in the original position, if tick is positive, turn right, or turn left
    public void turn(double power, int ticks) {
        if (!opModeIsActive()) {
            return;
        }

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // rotate left and right
        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightFront.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime turnTimer = new ElapsedTime();

        try {
            setMotorPower(Math.abs(power));

            // wating for at most 3 sec
            while (opModeIsActive()
                    && turnTimer.seconds() < 3.0
                    && (leftFront.isBusy() || leftBack.isBusy()
                    || rightFront.isBusy() || rightBack.isBusy())) {
                idle();
            }
        } finally {
            setMotorPower(0);
            setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    //rotating variable needs more test
    private static final int BLUE_TURN_90 = 500;
    private static final int BLUE_TURN_180 = 1000;

    // the arrow in graph two need to go across and turn true afterwards
    private static final boolean BLUE_FIG2_PASSAGE_CLEAR = false;

    private static final boolean BLUE_FIG2_EXTRA_SHOT = true;


    // graph 1 blue
    private void blueFig1() {
        // if shoot close to mid point, turn 400 to 0
        if (!blueDrive(400)) return;
        if (!blueShoot()) return;

        // turn left, upward in the graph
        if (!blueTurn(-BLUE_TURN_90)) return;
        if (!blueDrive(1100)) return;

        // turn right, across the structure in middle.
        if (!blueTurn(BLUE_TURN_90)) return;
        if (!blueDrive(2400)) return;

        // turn left, facing the blue frame.
        if (!blueTurn(-BLUE_TURN_90)) return;
        blueDrive(700);
    }


    // graph 2 blue
    private void blueFig2() {
        if (!BLUE_FIG2_PASSAGE_CLEAR) {
            telemetry.addData("Status", "Fig 2 passage not confirmed");
            telemetry.update();
            return;
        }

        // follow the straight line until to the left side of midddle structure.
        if (!blueDrive(3400)) return;

        if (BLUE_FIG2_EXTRA_SHOT) {
            // turn half a round and face left
            if (!blueTurn(BLUE_TURN_180)) return;

            // waiting for teammate to shoot, wait for 5 seconds
            sleep(5000);
            if (!blueShoot()) return;

            if (!blueTurn(BLUE_TURN_90)) return;
        } else {
            if (!blueTurn(-BLUE_TURN_90)) return;
        }

        blueDrive(1600);
    }

    private boolean blueDrive(int ticks) {
        return blueMove(0.35, ticks, ticks, 8.0);
    }


    private boolean blueTurn(int ticks) {
        return blueMove(0.25, ticks, -ticks, 4.0);
    }


    private boolean blueDriveBusy() {
        return leftFront.isBusy() || leftBack.isBusy()
                || rightFront.isBusy() || rightBack.isBusy();
    }


    private boolean blueMove(double power, int leftTicks,
                             int rightTicks, double timeoutSeconds) {
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
                    && blueDriveBusy()) {
                idle();
            }

            boolean reached = opModeIsActive() && !blueDriveBusy();

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


    private boolean blueShoot() {
        if (!opModeIsActive()) return false;

        try {
            launcher.setPower(0.8);
            sleep(1500);

            if (!opModeIsActive()) return false;

            leftIntake.setPower(0.8);
            rightIntake.setPower(0.8);
            sleep(2000);

            return opModeIsActive();
        } finally {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            launcher.setPower(0);
        }
    }

}
