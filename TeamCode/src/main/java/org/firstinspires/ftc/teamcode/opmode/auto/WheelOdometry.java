package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.IMU_NAME;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.teleOp.RobotTeleOp;

public class WheelOdometry {

    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;
    public static final double FORWARD_SCALE = 1.0;
    public static final double STRAFE_SCALE = 1.0;

    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static final double INCHES_PER_TICK = Math.PI * WHEEL_DIAMETER_INCHES / DRIVE_TICKS_PER_REV;

    public static final String[] WHEEL_NAMES = {
            FRONT_LEFT_MOTOR_NAME, BACK_LEFT_MOTOR_NAME, FRONT_RIGHT_MOTOR_NAME, BACK_RIGHT_MOTOR_NAME
    };

    public final DcMotor leftFront;
    public final DcMotor leftBack;
    public final DcMotor rightFront;
    public final DcMotor rightBack;

    private final IMU imu;
    private final boolean imuReady;
    private final int[] lastTicks = new int[4];
    private final int[] zeroTicks = new int[4];

    private Pose start = new Pose(0.0, 0.0, 0.0);
    private double x;
    private double y;
    private double lastHeading;
    private double rawForward;
    private double rawStrafe;

    public WheelOdometry(HardwareMap hardwareMap) {
        leftFront = initDrive(hardwareMap, FRONT_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION);
        leftBack = initDrive(hardwareMap, BACK_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION);
        rightFront = initDrive(hardwareMap, FRONT_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION);
        rightBack = initDrive(hardwareMap, BACK_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION);

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imuReady = imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION)));
    }

    public void reset(Pose startPose) {
        start = startPose;
        imu.resetYaw();
        int[] ticks = ticks();
        System.arraycopy(ticks, 0, lastTicks, 0, 4);
        System.arraycopy(ticks, 0, zeroTicks, 0, 4);
        x = start.x();
        y = start.y();
        lastHeading = start.heading();
        rawForward = 0.0;
        rawStrafe = 0.0;
    }

    public void update() {
        int[] ticks = ticks();
        double dLeftFront = (ticks[0] - lastTicks[0]) * INCHES_PER_TICK;
        double dLeftBack = (ticks[1] - lastTicks[1]) * INCHES_PER_TICK;
        double dRightFront = (ticks[2] - lastTicks[2]) * INCHES_PER_TICK;
        double dRightBack = (ticks[3] - lastTicks[3]) * INCHES_PER_TICK;
        System.arraycopy(ticks, 0, lastTicks, 0, 4);

        double forwardRaw = (dLeftFront + dLeftBack + dRightFront + dRightBack) / 4.0;
        double strafeRaw = (dLeftFront - dLeftBack - dRightFront + dRightBack) / 4.0;
        rawForward += forwardRaw;
        rawStrafe += strafeRaw;

        double forward = forwardRaw * FORWARD_SCALE;
        double strafe = strafeRaw * STRAFE_SCALE;

        double heading = heading();
        double middle = lastHeading + normalizeRadians(heading - lastHeading) / 2.0;

        x += forward * Math.cos(middle) + strafe * Math.sin(middle);
        y += forward * Math.sin(middle) - strafe * Math.cos(middle);
        lastHeading = heading;
    }

    public boolean imuReady() {
        return imuReady;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double heading() {
        return normalizeRadians(start.heading() + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public double yawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double rawForwardInches() {
        return rawForward;
    }

    public double rawStrafeInches() {
        return rawStrafe;
    }

    public double forwardInches() {
        return rawForward * FORWARD_SCALE;
    }

    public double strafeInches() {
        return rawStrafe * STRAFE_SCALE;
    }

    public int[] ticks() {
        return new int[] {
                leftFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                rightBack.getCurrentPosition()
        };
    }

    public int[] ticksSinceReset() {
        int[] ticks = ticks();
        for (int i = 0; i < 4; i++) {
            ticks[i] -= zeroTicks[i];
        }
        return ticks;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    public String poseText() {
        return String.format("x %.1f  y %.1f  h %.0f", x, y, Math.toDegrees(heading()));
    }

    public static double normalizeRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static DcMotor initDrive(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
        return motor;
    }
}
