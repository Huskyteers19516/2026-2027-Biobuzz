package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class LogiPadEdges {

    private boolean a;
    private boolean b;
    private boolean x;
    private boolean y;
    private boolean dpadUp;
    private boolean dpadDown;
    private boolean dpadLeft;
    private boolean dpadRight;
    private boolean leftBumper;
    private boolean rightBumper;
    private boolean back;
    private boolean start;
    private boolean leftStickButton;
    private boolean rightStickButton;
    private boolean leftTrigger;
    private boolean rightTrigger;
    private boolean leftBumperReleased;
    private boolean leftBumperTap;
    private boolean leftBumperCombo;

    public void read(Gamepad gamepad) {
        if (gamepad == null) {
            clear();
            return;
        }
        a = gamepad.aWasPressed();
        b = gamepad.bWasPressed();
        x = gamepad.xWasPressed();
        y = gamepad.yWasPressed();
        if (gamepad.start) {
            a = false;
            b = false;
            x = false;
            y = false;
        }
        dpadUp = gamepad.dpadUpWasPressed();
        dpadDown = gamepad.dpadDownWasPressed();
        dpadLeft = gamepad.dpadLeftWasPressed();
        dpadRight = gamepad.dpadRightWasPressed();
        leftBumper = gamepad.leftBumperWasPressed();
        rightBumper = gamepad.rightBumperWasPressed();
        back = gamepad.backWasPressed();
        start = gamepad.startWasPressed();
        leftStickButton = gamepad.leftStickButtonWasPressed();
        rightStickButton = gamepad.rightStickButtonWasPressed();
        leftTrigger = gamepad.leftTriggerWasPressed();
        rightTrigger = gamepad.rightTriggerWasPressed();
        leftBumperReleased = gamepad.leftBumperWasReleased();
        if (leftBumper) {
            leftBumperCombo = false;
        }
        boolean otherPressed = a || b || x || y || dpadUp || dpadDown || dpadLeft || dpadRight
                || rightBumper || back || start || leftStickButton || rightStickButton || leftTrigger || rightTrigger;
        if (otherPressed && (gamepad.left_bumper || leftBumper || leftBumperReleased)) {
            leftBumperCombo = true;
        }
        leftBumperTap = leftBumperReleased && !leftBumperCombo;
        if (leftBumperReleased) {
            leftBumperCombo = false;
        }
    }

    public void clear() {
        a = false;
        b = false;
        x = false;
        y = false;
        dpadUp = false;
        dpadDown = false;
        dpadLeft = false;
        dpadRight = false;
        leftBumper = false;
        rightBumper = false;
        back = false;
        start = false;
        leftStickButton = false;
        rightStickButton = false;
        leftTrigger = false;
        rightTrigger = false;
        leftBumperReleased = false;
        leftBumperTap = false;
        leftBumperCombo = false;
    }

    public boolean a() {
        return a;
    }

    public boolean b() {
        return b;
    }

    public boolean x() {
        return x;
    }

    public boolean y() {
        return y;
    }

    public boolean dpadUp() {
        return dpadUp;
    }

    public boolean dpadDown() {
        return dpadDown;
    }

    public boolean dpadLeft() {
        return dpadLeft;
    }

    public boolean dpadRight() {
        return dpadRight;
    }

    public boolean leftBumper() {
        return leftBumper;
    }

    public boolean rightBumper() {
        return rightBumper;
    }

    public boolean back() {
        return back;
    }

    public boolean start() {
        return start;
    }

    public boolean leftStickButton() {
        return leftStickButton;
    }

    public boolean rightStickButton() {
        return rightStickButton;
    }

    public boolean leftTrigger() {
        return leftTrigger;
    }

    public boolean rightTrigger() {
        return rightTrigger;
    }

    public boolean leftBumperReleased() {
        return leftBumperReleased;
    }

    public boolean leftBumperTap() {
        return leftBumperTap;
    }
}
