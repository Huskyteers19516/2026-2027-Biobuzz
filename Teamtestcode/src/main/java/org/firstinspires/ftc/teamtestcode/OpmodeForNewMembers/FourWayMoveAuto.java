package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Four Way Move Auto", group = "Testing")
public class FourWayMoveAuto extends LinearOpMode {

    DcMotor lf, rf, lb, rb;
    ElapsedTime timer = new ElapsedTime();

    double ticksPerInch = 537.7 / (3.78 * Math.PI);
    double strafeFix = 1.15;
    double speed = 0.4;
    double slowSpeed = 0.12;
    double moveInches = 24;
    double pause = 0.5;
    boolean stopped = false;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "left_front");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        lb = hardwareMap.get(DcMotor.class, "left_back");
        rb = hardwareMap.get(DcMotor.class, "right_back");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ready");
        telemetry.addLine("forward, back, right, left, then forward a little");
        telemetry.update();

        waitForStart();

        forward(moveInches);
        waitTime(pause);
        back(moveInches);
        waitTime(pause);
        right(moveInches);
        waitTime(pause);
        left(moveInches);
        waitTime(pause);
        forward(6);

        stopMotors();

        if (!stopped) {
            telemetry.addLine("done");
            telemetry.update();
        }
        while (opModeIsActive()) {
            idle();
        }
    }

    void forward(double inches) {
        drive(1, 1, 1, 1, inches * ticksPerInch, inches);
    }

    void back(double inches) {
        drive(-1, -1, -1, -1, inches * ticksPerInch, inches);
    }

    void right(double inches) {
        drive(1, -1, -1, 1, inches * ticksPerInch * strafeFix, inches);
    }

    void left(double inches) {
        drive(-1, 1, 1, -1, inches * ticksPerInch * strafeFix, inches);
    }

    void drive(int a, int b, int c, int d, double target, double inches) {
        int lfStart = lf.getCurrentPosition();
        int rfStart = rf.getCurrentPosition();
        int lbStart = lb.getCurrentPosition();
        int rbStart = rb.getCurrentPosition();

        if (stopped) {
            return;
        }

        double maxTime = 2 + inches * 0.25;
        timer.reset();

        while (opModeIsActive() && timer.seconds() < maxTime) {
            int lfMoved = Math.abs(lf.getCurrentPosition() - lfStart);
            int rfMoved = Math.abs(rf.getCurrentPosition() - rfStart);
            int lbMoved = Math.abs(lb.getCurrentPosition() - lbStart);
            int rbMoved = Math.abs(rb.getCurrentPosition() - rbStart);
            double moved = (lfMoved + rfMoved + lbMoved + rbMoved) / 4.0;

            // a wheel whose encoder has not counted after 1 s means a loose encoder cable
            // or a stuck wheel; without this the robot keeps driving until maxTime
            double minTicks = 0.5 * ticksPerInch;
            if (timer.seconds() > 1.0 && (lfMoved < minTicks || rfMoved < minTicks
                    || lbMoved < minTicks || rbMoved < minTicks)) {
                stopped = true;
                stopMotors();
                telemetry.addLine("STOPPED: an encoder is not counting");
                telemetry.addData("ticks", "lf %d  rf %d  lb %d  rb %d", lfMoved, rfMoved, lbMoved, rbMoved);
                telemetry.update();
                return;
            }

            double togo = target - moved;
            if (togo <= 0) {
                break;
            }

            double power = speed;
            if (togo < 6 * ticksPerInch) {
                power = slowSpeed;
            }

            lf.setPower(a * power);
            rf.setPower(b * power);
            lb.setPower(c * power);
            rb.setPower(d * power);

            telemetry.addData("moved", "%.1f / %.1f in", moved / ticksPerInch, target / ticksPerInch);
            telemetry.addData("time", "%.1f", timer.seconds());
            telemetry.update();
        }

        stopMotors();
    }

    void waitTime(double seconds) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            idle();
        }
    }

    void stopMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}
