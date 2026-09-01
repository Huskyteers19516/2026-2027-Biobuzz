package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class HardwareTest extends OpMode {
    CRServo rightServo;

    @Override
    public void init() {
        rightServo = hardwareMap.get(CRServo.class, "left_feeder");
    }

    @Override
    public void loop() {
        rightServo.setPower(1);
    }
}
