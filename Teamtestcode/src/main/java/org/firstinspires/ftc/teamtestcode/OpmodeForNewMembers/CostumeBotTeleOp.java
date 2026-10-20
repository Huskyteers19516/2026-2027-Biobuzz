package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Costume Bot Drive", group = "Testing")
public class CostumeBotTeleOp extends OpMode {

    // Initial limit only; motor directions and speed have not been tested on the costume bot.
    private static final double DRIVE_POWER = 0.4;
    private static final double STICK_DEADZONE = 0.05;

    private DcMotor lf, lb, rf, rb;

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "left_front");
        lb = hardwareMap.get(DcMotor.class, "left_back");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        rb = hardwareMap.get(DcMotor.class, "right_back");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopDrive();
        telemetry.addLine("Four-motor drive ready; test with wheels off the ground first.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone(gamepad1.left_stick_x);
        double turn = deadzone(gamepad1.right_stick_x);

        double a = forward + strafe + turn;
        double b = forward - strafe + turn;
        double c = forward - strafe - turn;
        double d = forward + strafe - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(a), Math.abs(b)),
                Math.max(Math.abs(c), Math.abs(d))));

        lf.setPower(a / max * DRIVE_POWER);
        lb.setPower(b / max * DRIVE_POWER);
        rf.setPower(c / max * DRIVE_POWER);
        rb.setPower(d / max * DRIVE_POWER);

        telemetry.addData("Drive", "forward %.2f strafe %.2f turn %.2f", forward, strafe, turn);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
    }

    private void stopDrive() {
        lf.setPower(0.0);
        lb.setPower(0.0);
        rf.setPower(0.0);
        rb.setPower(0.0);
    }

    private double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
