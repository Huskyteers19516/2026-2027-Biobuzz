package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

// TODO: PLACEHOLDER values so MatchAuto compiles. The real RobotTeleOp was never
// committed. Replace every value below with the real robot's numbers, then remove
// @Disabled from MatchAuto.
public class RobotTeleOp {

    public static final String INTAKE_NAME = "intake";
    public static final String TRANSFER_NAME = "transfer";
    public static final String OUTTAKE_NAME = "outtake";
    public static final String LAUNCHER_SERVO_NAME = "launcher_servo";

    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final double OUTTAKE_VELOCITY = 0.0;
    public static final double OUTTAKE_TOLERANCE = 0.0;
    public static final double SPINUP_TIMEOUT_SECONDS = 0.0;
    public static final boolean KEEP_OUTTAKE_SPINNING = false;

    public static final double FEED_POWER = 0.0;
    public static final double FEED_SECONDS = 0.0;

    public static final double INTAKE_POWER = 0.0;

    public static final double AIM_START_POSITION = 0.5;
    public static final double AIM_MIN_POSITION = 0.0;
    public static final double AIM_MAX_POSITION = 1.0;
}
