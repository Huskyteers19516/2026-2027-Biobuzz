package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.LAUNCHER;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.SERVO_ONE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.SERVO_THREE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.SERVO_TWO;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.TRANSFER;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmode.FieldPoints;


@TeleOp(name = "Drive + Pinpoint TeleOp", group = "Testing")
public class DrivePinpointTeleOp extends OpMode {

    private static final String LEFT_FRONT_NAME = "left_front";
    private static final String LEFT_BACK_NAME = "left_back";
    private static final String RIGHT_FRONT_NAME = "right_front";
    private static final String RIGHT_BACK_NAME = "right_back";
    private static final String PINPOINT_LEFT_NAME = "pinpoint_left";
    private static final String PINPOINT_RIGHT_NAME = "pinpoint_right";

    private static final DcMotorSimple.Direction LEFT_SIDE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_SIDE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private static final double SLOW_SCALE = 0.35;
    private static final double STICK_DEADZONE = 0.05;

    private static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    private static final double PINPOINT_LEFT_X_OFFSET_MM = -84.0;
    private static final double PINPOINT_LEFT_Y_OFFSET_MM = -168.0;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_LEFT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_LEFT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final double PINPOINT_RIGHT_X_OFFSET_MM = -84.0;
    private static final double PINPOINT_RIGHT_Y_OFFSET_MM = 168.0;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_RIGHT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection PINPOINT_RIGHT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;


    private static final double FLOWER_1_X = 0.0;
    private static final double FLOWER_1_Y = 0.0;

    private static final double FLOWER_2_X = 0.0;
    private static final double FLOWER_2_Y = 0.0;

    private static final double FLOWER_3_X = 0.0;
    private static final double FLOWER_3_Y = 0.0;

    private static final double FLOWER_4_X = 0.0;
    private static final double FLOWER_4_Y = 0.0;


    private static final double FLOWER_RADIUS_IN = 18.0;

    private static final double ZONE_EXIT_MARGIN_IN = 2.0;

    private static final double INTAKE_SERVO_FLOWER_POSITION = 0.8;
    private static final double INTAKE_SERVO_DEFAULT_POSITION = 0.2;

    private enum RobotMode { DRIVE, FLOWER }

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private GoBildaPinpointDriver pinpointLeft;
    private GoBildaPinpointDriver pinpointRight;

    private DcMotor launcher;
    private DcMotor transfer;
    private DcMotor intake;
    private Servo intakeServo;
    private Servo outtakeServo;
    private Servo blocker;

    private boolean lastBack = false;
    private boolean lastA2 = false;
    private boolean lastY2 = false;

    private RobotMode mode = RobotMode.DRIVE;

    private boolean manualMode = false;
    private int currentZone = -1;

    @Override
    public void init() {
        leftFront = initMotor(FRONT_LEFT_MOTOR_NAME, LEFT_SIDE_DIRECTION);
        leftBack = initMotor(BACK_LEFT_MOTOR_NAME, LEFT_SIDE_DIRECTION);
        rightFront = initMotor(FRONT_RIGHT_MOTOR_NAME, RIGHT_SIDE_DIRECTION);
        rightBack = initMotor(BACK_RIGHT_MOTOR_NAME, RIGHT_SIDE_DIRECTION);
        launcher = hardwareMap.get(DcMotor.class,LAUNCHER);
        transfer = hardwareMap.get(DcMotor.class,TRANSFER);
        intake = hardwareMap.get(DcMotor.class,INTAKE);
        intakeServo=hardwareMap.get(Servo.class,SERVO_ONE);
        outtakeServo = hardwareMap.get(Servo.class,SERVO_TWO);
        blocker = hardwareMap.get(Servo.class,SERVO_THREE);
        pinpointLeft = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_LEFT_NAME);
        pinpointRight = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_RIGHT_NAME);

        configurePinpoint(pinpointLeft, PINPOINT_LEFT_X_OFFSET_MM, PINPOINT_LEFT_Y_OFFSET_MM,
                PINPOINT_LEFT_X_DIRECTION, PINPOINT_LEFT_Y_DIRECTION);
        configurePinpoint(pinpointRight, PINPOINT_RIGHT_X_OFFSET_MM, PINPOINT_RIGHT_Y_OFFSET_MM,
                PINPOINT_RIGHT_X_DIRECTION, PINPOINT_RIGHT_Y_DIRECTION);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        pinpointLeft.update();
        pinpointRight.update();
        handleResetButton();

        telemetry.addData("Status", "Init - waiting for START");
        addPoseTelemetry();
    }

    @Override
    public void start() {
        setStartPose();
    }

    @Override
    public void loop() {
        pinpointLeft.update();
        pinpointRight.update();
        handleResetButton();
        updateMode();

        double drive = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone(gamepad1.left_stick_x);
        double turn = deadzone(gamepad1.right_stick_x);
        double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;

        double lf = drive + strafe + turn;
        double lb = drive - strafe + turn;
        double rf = drive - strafe - turn;
        double rb = drive + strafe - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));

        leftFront.setPower(lf / max * scale);
        leftBack.setPower(lb / max * scale);
        rightFront.setPower(rf / max * scale);
        rightBack.setPower(rb / max * scale);

        if (mode == RobotMode.FLOWER) {
            intakeServo.setPosition(INTAKE_SERVO_FLOWER_POSITION);
        } else {
            intakeServo.setPosition(INTAKE_SERVO_DEFAULT_POSITION);
        }

        telemetry.addData("Mode", "%s  (%s)", mode, manualMode ? "MANUAL - gp2 Y = back to auto" : "AUTO");
        telemetry.addData("Flower zone", currentZone >= 0 ? "flower " + (currentZone + 1) : "none");
        telemetry.addData("Intake servo", "%.2f", intakeServo.getPosition());
        telemetry.addData("Slow mode", gamepad1.right_bumper ? "ON" : "OFF");
        addPoseTelemetry();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void updateMode() {
        boolean a2 = gamepad2.a;
        boolean y2 = gamepad2.y;
        if (a2 && !lastA2) {
            manualMode = true;
            mode = mode == RobotMode.DRIVE ? RobotMode.FLOWER : RobotMode.DRIVE;
        }
        if (y2 && !lastY2) {
            manualMode = false;
        }
        lastA2 = a2;
        lastY2 = y2;

        Pose2D pose = robotPose();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double radius = FLOWER_RADIUS_IN;
        if (mode == RobotMode.FLOWER) {
            radius += ZONE_EXIT_MARGIN_IN;
        }

        if (Math.hypot(x - FLOWER_1_X, y - FLOWER_1_Y) <= radius) {
            currentZone = 0;
        } else if (Math.hypot(x - FLOWER_2_X, y - FLOWER_2_Y) <= radius) {
            currentZone = 1;
        } else if (Math.hypot(x - FLOWER_3_X, y - FLOWER_3_Y) <= radius) {
            currentZone = 2;
        } else if (Math.hypot(x - FLOWER_4_X, y - FLOWER_4_Y) <= radius) {
            currentZone = 3;
        } else {
            currentZone = -1;
        }

        if (!manualMode) {
            if (currentZone >= 0) {
                mode = RobotMode.FLOWER;
            } else {
                mode = RobotMode.DRIVE;
            }
        }
    }


    private Pose2D robotPose() {
        boolean leftOk = pinpointLeft.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        boolean rightOk = pinpointRight.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        if (leftOk && rightOk) {
            return new Pose2D(DistanceUnit.INCH,
                    (pinpointLeft.getPosX(DistanceUnit.INCH) + pinpointRight.getPosX(DistanceUnit.INCH)) / 2.0,
                    (pinpointLeft.getPosY(DistanceUnit.INCH) + pinpointRight.getPosY(DistanceUnit.INCH)) / 2.0,
                    AngleUnit.RADIANS, pinpointLeft.getHeading(AngleUnit.RADIANS));
        }
        return rightOk ? pinpointRight.getPosition() : pinpointLeft.getPosition();
    }

    private void setStartPose() {
        Pose2D start = new Pose2D(DistanceUnit.INCH,
                FieldPoints.START_POSE.x(), FieldPoints.START_POSE.y(),
                AngleUnit.RADIANS, FieldPoints.START_POSE.heading());
        pinpointLeft.setPosition(start);
        pinpointRight.setPosition(start);
    }

    private void handleResetButton() {
        boolean back = gamepad1.back;
        if (back && !lastBack) {
            setStartPose();
        }
        lastBack = back;
    }

    private void addPoseTelemetry() {
        telemetry.addData("Pinpoint L", "x %.1f in  y %.1f in  h %.1f deg  %s",
                pinpointLeft.getPosX(DistanceUnit.INCH),
                pinpointLeft.getPosY(DistanceUnit.INCH),
                pinpointLeft.getHeading(AngleUnit.DEGREES),
                pinpointLeft.getDeviceStatus());
        telemetry.addData("Pinpoint R", "x %.1f in  y %.1f in  h %.1f deg  %s",
                pinpointRight.getPosX(DistanceUnit.INCH),
                pinpointRight.getPosY(DistanceUnit.INCH),
                pinpointRight.getHeading(AngleUnit.DEGREES),
                pinpointRight.getDeviceStatus());
        telemetry.addData("Freq", "L %.1f Hz  R %.1f Hz",
                pinpointLeft.getFrequency(), pinpointRight.getFrequency());
        telemetry.addLine("gp1 back = reset to START_POSE   gp2 A = switch mode");
    }

    private DcMotor initMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void configurePinpoint(GoBildaPinpointDriver pinpoint, double xOffsetMm, double yOffsetMm,
                                   GoBildaPinpointDriver.EncoderDirection xDirection,
                                   GoBildaPinpointDriver.EncoderDirection yDirection) {
        pinpoint.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PINPOINT_POD_TYPE);
        pinpoint.setEncoderDirections(xDirection, yDirection);
        pinpoint.resetPosAndIMU();
    }

    private static double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
