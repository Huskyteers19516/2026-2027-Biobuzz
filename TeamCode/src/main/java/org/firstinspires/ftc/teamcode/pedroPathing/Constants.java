package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {

    public static final String FRONT_LEFT_MOTOR_NAME = "left_front";
    public static final String BACK_LEFT_MOTOR_NAME = "left_back";
    public static final String FRONT_RIGHT_MOTOR_NAME = "right_front";
    public static final String BACK_RIGHT_MOTOR_NAME = "right_back";
    public static final String INTAKE = "intake";
    public static final String TRANSFER = "transfer";
    public static final String OUTTAKE = "outtake";
    public static final String COUNTER_WHEEL = "counter_wheel";
    public static final String FLOWER_INTAKE = "flower_intake";
    public static final String OUTTAKE_BLOCKER = "outtake_blocker";
    public static final String WEBCAM = "Webcam 1";
    public static final String IMU_NAME = "imu";

    public static final DcMotorSimple.Direction LEFT_SIDE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_SIDE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction COUNTER_WHEEL_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;

    public static double[] FLOWER_POSITIONS = {0.2, 0.5, 0.8};
    public static final int FLOWER_START_INDEX = 0;

    private static final double SERVO_RANGE_DEGREES = 270.0;
    private static final double BLOCKER_CENTER_POSITION = 0.5;
    private static final double BLOCKER_START_TURN_DEGREES = -135.0;
    private static final double BLOCKER_TURN_DEGREES = 50.0;
    public static double BLOCKER_CLOSED_POSITION =
            Range.clip(BLOCKER_CENTER_POSITION + BLOCKER_START_TURN_DEGREES / SERVO_RANGE_DEGREES, 0.0, 1.0);
    public static double BLOCKER_OPEN_POSITION =
            Range.clip(BLOCKER_CLOSED_POSITION + BLOCKER_TURN_DEGREES / SERVO_RANGE_DEGREES, 0.0, 1.0);

    public static double SHOOTER_HEADING_OFFSET_DEGREES = 0.0;
    public static final double MAX_GOAL_ENTRY_ANGLE_DEGREES = 90.0;
    public static double AIM_KP = 1.5;
    public static double AIM_MIN_TURN_POWER = 0.06;
    public static double AIM_MAX_TURN_POWER = 0.6;
    public static double AIM_TOLERANCE_DEGREES = 2.0;
    public static double AIM_SETTLE_SECONDS = 0.15;

    public static final boolean GOAL_CHECK_CALIBRATED = true;
    public static double CAMERA_ROLL_OFFSET_DEGREES = 0.0;
    public static double GOAL_UP_MAX_ABS_ROLL_DEGREES = 90.0;
    public static final boolean BLOCK_SHOT_WHEN_GOAL_NOT_SEEN = false;
    public static double GOAL_OPEN_WAIT_SECONDS = 1.5;

    private static final boolean MANUAL_BRAKE_MODE = true;
    private static final double POWER_THRESHOLD = 0.01;

    public static final String PINPOINT_NAME = "pinpoint_left";

    private static final double PINPOINT_X_POD_OFFSET_MM = -84.0;
    private static final double PINPOINT_Y_POD_OFFSET_MM = -168.0;
    private static final DistanceUnit PINPOINT_OFFSET_UNIT = DistanceUnit.MM;
    private static final DistanceUnit PINPOINT_GLOBAL_UNIT = DistanceUnit.INCH;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    private static final double MAX_FORWARD_VELOCITY = 50.0;
    private static final double MAX_STRAFE_VELOCITY = 50.0;
    private static final double NATURAL_FORWARD_DECELERATION = 60.0;
    private static final double NATURAL_STRAFE_DECELERATION = 60.0;

    private static final double FORWARD_BRAKE_LINEAR = 0.05;
    private static final double FORWARD_BRAKE_QUADRATIC = 0.001;
    private static final double STRAFE_BRAKE_LINEAR = 0.05;
    private static final double STRAFE_BRAKE_QUADRATIC = 0.001;
    private static final double HEADING_BRAKE_LINEAR = 0.1;
    private static final double HEADING_BRAKE_QUADRATIC = 0.01;

    private static final double HEADING_KP = 1.0;
    private static final double FORWARD_TRANSLATIONAL_PRIMARY_KP = 0.1;
    private static final double FORWARD_TRANSLATIONAL_SECONDARY_KP = 0.05;
    private static final double STRAFE_TRANSLATIONAL_PRIMARY_KP = 0.1;
    private static final double STRAFE_TRANSLATIONAL_SECONDARY_KP = 0.05;
    private static final double TRANSLATIONAL_SWITCH_DISTANCE_IN = 2.5;
    private static final double COAST_KV = 0.02;
    private static final double BRAKE_KV = 0.02;

    public static MecanumConfig mecanumConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(FRONT_LEFT_MOTOR_NAME);
            c.backLeftName.set(BACK_LEFT_MOTOR_NAME);
            c.frontRightName.set(FRONT_RIGHT_MOTOR_NAME);
            c.backRightName.set(BACK_RIGHT_MOTOR_NAME);
            c.frontLeftDirection.set(LEFT_SIDE_DIRECTION);
            c.backLeftDirection.set(LEFT_SIDE_DIRECTION);
            c.frontRightDirection.set(RIGHT_SIDE_DIRECTION);
            c.backRightDirection.set(RIGHT_SIDE_DIRECTION);
            c.manualBrakeMode.set(MANUAL_BRAKE_MODE);
            c.powerThreshold.set(POWER_THRESHOLD);
        });
    }

    public static PinpointConfig pinpointConfig() {
        return new PinpointConfig(c -> {
            c.name.set(PINPOINT_NAME);
            c.xPodOffset.set(PINPOINT_X_POD_OFFSET_MM);
            c.yPodOffset.set(PINPOINT_Y_POD_OFFSET_MM);
            c.offsetUnits.set(PINPOINT_OFFSET_UNIT);
            c.globalDistanceUnit.set(PINPOINT_GLOBAL_UNIT);
            c.xPodDirection.set(PINPOINT_X_POD_DIRECTION);
            c.yPodDirection.set(PINPOINT_Y_POD_DIRECTION);
            c.podType.set(PINPOINT_POD_TYPE);
        });
    }

    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.piecewise(Controller.proportional(FORWARD_TRANSLATIONAL_SECONDARY_KP))
                    .put(TRANSLATIONAL_SWITCH_DISTANCE_IN, Controller.proportional(FORWARD_TRANSLATIONAL_PRIMARY_KP)));
            c.strafeTranslational.set(Controller.piecewise(Controller.proportional(STRAFE_TRANSLATIONAL_SECONDARY_KP))
                    .put(TRANSLATIONAL_SWITCH_DISTANCE_IN, Controller.proportional(STRAFE_TRANSLATIONAL_PRIMARY_KP)));
            c.coast.set(Controller.proportionalFeedforward(COAST_KV));
            c.brake.set(Controller.proportionalFeedforward(BRAKE_KV));
            c.headingFeedback.set(Controller.proportional(HEADING_KP));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(HEADING_BRAKE_LINEAR, HEADING_BRAKE_QUADRATIC));
            c.linearBrakeCoefficients.set(Matrix.diag(FORWARD_BRAKE_LINEAR, STRAFE_BRAKE_LINEAR));
            c.quadraticBrakeCoefficients.set(Matrix.diag(FORWARD_BRAKE_QUADRATIC, STRAFE_BRAKE_QUADRATIC));
            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(NATURAL_FORWARD_DECELERATION);
            c.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECELERATION);
        });
    }

    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, mecanumConfig());
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, pinpointConfig());
    }

    public static Algorithm createAlgorithm() {
        return new Foresight(foresightConfig());
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        Localizer localizer = createLocalizer(hardwareMap);
        Drivetrain drivetrain = createDrivetrain(hardwareMap);
        return new Follower(localizer, drivetrain, createAlgorithm());
    }
}
