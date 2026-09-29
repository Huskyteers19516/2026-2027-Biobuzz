package org.firstinspires.ftc.teamtestcode.pedropathing;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static final String FRONT_LEFT_MOTOR_NAME = "left_front";
    public static final String BACK_LEFT_MOTOR_NAME = "left_back";
    public static final String FRONT_RIGHT_MOTOR_NAME = "right_front";
    public static final String BACK_RIGHT_MOTOR_NAME = "right_back";

    private static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private static final boolean MANUAL_BRAKE_MODE = true;
    private static final double POWER_THRESHOLD = 0.01;

    public static final String PINPOINT_NAME = "pinpoint";

    private static final double PINPOINT_X_POD_OFFSET_IN = 0.0;
    private static final double PINPOINT_Y_POD_OFFSET_IN = 0.0;
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
            c.frontLeftDirection.set(FRONT_LEFT_DIRECTION);
            c.backLeftDirection.set(BACK_LEFT_DIRECTION);
            c.frontRightDirection.set(FRONT_RIGHT_DIRECTION);
            c.backRightDirection.set(BACK_RIGHT_DIRECTION);
            c.manualBrakeMode.set(MANUAL_BRAKE_MODE);
            c.powerThreshold.set(POWER_THRESHOLD);
        });
    }

    public static PinpointConfig pinpointConfig() {
        return new PinpointConfig(c -> {
            c.name.set(PINPOINT_NAME);
            c.xPodOffset.set(PINPOINT_X_POD_OFFSET_IN);
            c.yPodOffset.set(PINPOINT_Y_POD_OFFSET_IN);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
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

    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap) {
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

    public static Follower create(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }
}
