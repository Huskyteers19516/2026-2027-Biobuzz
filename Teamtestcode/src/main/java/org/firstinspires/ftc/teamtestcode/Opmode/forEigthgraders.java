package org.firstinspires.ftc.teamtestcode.Opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class forEigthgraders {

    private Servo servo;

    public void Opmode(){
        servo = hardwareMap.get(Servo.class,"servo");
        servo.setPosition(1.0);
    }
}
