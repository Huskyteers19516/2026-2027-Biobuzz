package org.firstinspires.ftc.teamtestcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmode.teleOp.RobotTeleOp;

/** Driver-controlled intake and transport using controller 1. */
@TeleOp(name = "Transport Intake Task", group = "Testing")
public class TransportIntakeTask extends OpMode {

    private static final double TRIGGER_DEADZONE = 0.05;

    private DcMotor intake;
    private DcMotor transfer;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, RobotTeleOp.INTAKE_NAME);
        transfer = hardwareMap.get(DcMotor.class, RobotTeleOp.TRANSFER_NAME);

        intake.setPower(0.0);
        transfer.setPower(0.0);
        intake.setDirection(RobotTeleOp.INTAKE_DIRECTION);
        transfer.setDirection(RobotTeleOp.TRANSFER_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        showControls();
    }

    @Override
    public void init_loop() {
        showControls();
    }

    @Override
    public void loop() {
        // Reverse takes priority if both controls are held.
        double power = 0.0;
        if (gamepad1.left_bumper) {
            power = -1.0;
        } else if (gamepad1.right_trigger > TRIGGER_DEADZONE) {
            power = gamepad1.right_trigger;
        }

        intake.setPower(power * RobotTeleOp.INTAKE_POWER);
        transfer.setPower(power * RobotTeleOp.TRANSFER_POWER);

        showControls();
        telemetry.addData("Status", power > 0.0 ? "Intaking" : power < 0.0 ? "Reversing" : "Stopped");
        telemetry.addData("Intake power", "%.2f", intake.getPower());
        telemetry.addData("Transfer power", "%.2f", transfer.getPower());
    }

    @Override
    public void stop() {
        if (intake != null) {
            intake.setPower(0.0);
        }
        if (transfer != null) {
            transfer.setPower(0.0);
        }
    }

    private void showControls() {
        telemetry.addLine("Controller 1: hold right trigger to intake and transport");
        telemetry.addLine("Trigger pressure controls speed; release to stop");
        telemetry.addLine("Hold left bumper to reverse both motors");
    }
}
