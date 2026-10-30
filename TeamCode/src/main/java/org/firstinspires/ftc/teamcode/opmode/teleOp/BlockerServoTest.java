package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.OUTTAKE_BLOCKER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Blocker Servo Test", group = "Test")
public class BlockerServoTest extends OpMode {

    private static final double SMALL_STEP = 0.01;
    private static final double BIG_STEP = 0.05;

    private Servo blocker;
    private double position = RobotTeleOp.BLOCKER_CLOSED_POSITION;

    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, OUTTAKE_BLOCKER);
        telemetry.addLine("The servo moves only after START.");
        telemetry.update();
    }

    @Override
    public void start() {
        blocker.setPosition(position);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) position += BIG_STEP;
        if (gamepad1.dpadDownWasPressed()) position -= BIG_STEP;
        if (gamepad1.dpadRightWasPressed()) position += SMALL_STEP;
        if (gamepad1.dpadLeftWasPressed()) position -= SMALL_STEP;
        if (gamepad1.aWasPressed()) position = RobotTeleOp.BLOCKER_CLOSED_POSITION;
        if (gamepad1.bWasPressed()) position = RobotTeleOp.BLOCKER_OPEN_POSITION;
        if (gamepad1.xWasPressed()) position = 0.0;
        if (gamepad1.yWasPressed()) position = 1.0;
        position = Range.clip(position, 0.0, 1.0);

        blocker.setPosition(position);

        telemetry.addData("Servo", "\"%s\"  position %.2f", OUTTAKE_BLOCKER, position);
        telemetry.addData("TeleOp values", "closed %.2f  open %.2f",
                RobotTeleOp.BLOCKER_CLOSED_POSITION, RobotTeleOp.BLOCKER_OPEN_POSITION);
        telemetry.addLine("dpad up/down +-0.05   right/left +-0.01");
        telemetry.addLine("A = TeleOp closed   B = TeleOp open   X = 0.0   Y = 1.0");
        telemetry.addLine("Find the real closed and open positions and tell them to me.");
    }
}
