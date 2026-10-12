package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.LAUNCHER;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.SERVO_ONE;
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

    private boolean lastBack = false;

    @Override
    public void init() {
        leftFront =hardwareMap.get(DcMotor.class,FRONT_LEFT_MOTOR_NAME);
        leftBack = hardwareMap.get(DcMotor.class,BACK_LEFT_MOTOR_NAME);
        rightFront= hardwareMap.get(DcMotor.class,FRONT_RIGHT_MOTOR_NAME);
        rightBack= hardwareMap.get(DcMotor.class,BACK_RIGHT_MOTOR_NAME);
        launcher = hardwareMap.get(DcMotor.class,LAUNCHER);
        transfer = hardwareMap.get(DcMotor.class,TRANSFER);
        intake = hardwareMap.get(DcMotor.class,INTAKE);
        intakeServo=hardwareMap.get(Servo.class,SERVO_ONE);
        outtakeServo = hardwareMap.get(Servo.class,SERVO_TWO);
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
    public void loop() {
        pinpointLeft.update();
        pinpointRight.update();
        handleResetButton();

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

    private void handleResetButton() {
        boolean back = gamepad1.back;
        if (back && !lastBack) {
            pinpointLeft.resetPosAndIMU();
            pinpointRight.resetPosAndIMU();
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
        telemetry.addLine("back = reset both Pinpoints");
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
