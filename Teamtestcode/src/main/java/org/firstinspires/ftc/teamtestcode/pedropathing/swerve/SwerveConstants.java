package org.firstinspires.ftc.teamtestcode.pedropathing.swerve;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamtestcode.pedropathing.Constants;

public class SwerveConstants {

    public static final String LEFT_POD_NAME = "left_pod";
    public static final String LEFT_MOTOR_NAME = "left_swerve_motor";
    public static final String LEFT_SERVO_NAME = "left_swerve_servo";
    public static final String LEFT_ENCODER_NAME = "left_swerve_encoder";

    public static final String RIGHT_POD_NAME = "right_pod";
    public static final String RIGHT_MOTOR_NAME = "right_swerve_motor";
    public static final String RIGHT_SERVO_NAME = "right_swerve_servo";
    public static final String RIGHT_ENCODER_NAME = "right_swerve_encoder";

    private static final double TRACK_WIDTH_IN = 13.0;
    private static final double POD_FORWARD_OFFSET_IN = 0.0;
    private static final double SWERVE_OFFSET_Y_SIGN = -1.0;

    private static final double LEFT_POD_ANGLE_OFFSET_RAD = 0.0;
    private static final boolean LEFT_POD_ENCODER_REVERSED = false;
    private static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction LEFT_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double RIGHT_POD_ANGLE_OFFSET_RAD = 0.0;
    private static final boolean RIGHT_POD_ENCODER_REVERSED = false;
    private static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction RIGHT_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double LEFT_ANALOG_MIN_VOLTAGE = 0.0;
    private static final double LEFT_ANALOG_MAX_VOLTAGE = 3.3;
    private static final double RIGHT_ANALOG_MIN_VOLTAGE = 0.0;
    private static final double RIGHT_ANALOG_MAX_VOLTAGE = 3.3;

    private static final double TURN_KP = 0.69;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.034;
    private static final double TURN_KS = 0.05;

    private static final boolean VOLTAGE_COMPENSATION = false;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final boolean MANUAL_BRAKE_MODE = true;
    private static final SwerveConfig.ZeroPowerBehavior SWERVE_ZERO_POWER_BEHAVIOR =
            SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES;

    public static CoaxialPodConfig leftPodConfig() {
        return new CoaxialPodConfig(c -> {
            c.name.set(LEFT_POD_NAME);
            c.motorName.set(LEFT_MOTOR_NAME);
            c.servoName.set(LEFT_SERVO_NAME);
            c.servoEncoderName.set(LEFT_ENCODER_NAME);
            c.driveDirection.set(LEFT_DRIVE_DIRECTION);
            c.servoDirection.set(LEFT_SERVO_DIRECTION);
            c.angleOffsetRad.set(LEFT_POD_ANGLE_OFFSET_RAD);
            c.encoderReversed.set(LEFT_POD_ENCODER_REVERSED);
            c.podOffset.set(Vector2D.cartesian(POD_FORWARD_OFFSET_IN, SWERVE_OFFSET_Y_SIGN * (TRACK_WIDTH_IN / 2.0)));
            c.analogMinVoltage.set(LEFT_ANALOG_MIN_VOLTAGE);
            c.analogMaxVoltage.set(LEFT_ANALOG_MAX_VOLTAGE);
            c.turnController.set(turnController());
        });
    }

    public static CoaxialPodConfig rightPodConfig() {
        return new CoaxialPodConfig(c -> {
            c.name.set(RIGHT_POD_NAME);
            c.motorName.set(RIGHT_MOTOR_NAME);
            c.servoName.set(RIGHT_SERVO_NAME);
            c.servoEncoderName.set(RIGHT_ENCODER_NAME);
            c.driveDirection.set(RIGHT_DRIVE_DIRECTION);
            c.servoDirection.set(RIGHT_SERVO_DIRECTION);
            c.angleOffsetRad.set(RIGHT_POD_ANGLE_OFFSET_RAD);
            c.encoderReversed.set(RIGHT_POD_ENCODER_REVERSED);
            c.podOffset.set(Vector2D.cartesian(POD_FORWARD_OFFSET_IN, -SWERVE_OFFSET_Y_SIGN * (TRACK_WIDTH_IN / 2.0)));
            c.analogMinVoltage.set(RIGHT_ANALOG_MIN_VOLTAGE);
            c.analogMaxVoltage.set(RIGHT_ANALOG_MAX_VOLTAGE);
            c.turnController.set(turnController());
        });
    }

    public static SwerveConfig swerveConfig() {
        return new SwerveConfig(c -> {
            c.voltageCompensation.set(VOLTAGE_COMPENSATION);
            c.nominalVoltage.set(NOMINAL_VOLTAGE);
            c.manualBrakeMode.set(MANUAL_BRAKE_MODE);
            c.zeroPowerBehavior.set(SWERVE_ZERO_POWER_BEHAVIOR);
        });
    }

    public static Controller turnController() {
        return Controller.sum(
                Controller.pid(TURN_KP, TURN_KI, TURN_KD),
                Controller.staticTargetFeedforward(TURN_KS)
        );
    }

    public static CoaxialPod createLeftPod(HardwareMap hardwareMap) {
        return new CoaxialPod(hardwareMap, leftPodConfig());
    }

    public static CoaxialPod createRightPod(HardwareMap hardwareMap) {
        return new CoaxialPod(hardwareMap, rightPodConfig());
    }

    public static Swerve createDrivetrain(HardwareMap hardwareMap) {
        return new Swerve(hardwareMap, swerveConfig(), createLeftPod(hardwareMap), createRightPod(hardwareMap));
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        Localizer localizer = Constants.createLocalizer(hardwareMap);
        Drivetrain drivetrain = createDrivetrain(hardwareMap);
        return new Follower(localizer, drivetrain, Constants.createAlgorithm());
    }
}
