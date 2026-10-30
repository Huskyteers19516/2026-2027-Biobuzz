package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FRONT_RIGHT_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive Motor Test", group = "Test")
public class DriveMotorTest extends OpMode {

    private static final double TEST_POWER = 0.3;
    private static final double TRIGGER_THRESHOLD = 0.5;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    @Override
    public void init() {
        leftFront = initMotor(FRONT_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION);
        leftBack = initMotor(BACK_LEFT_MOTOR_NAME, RobotTeleOp.LEFT_SIDE_DIRECTION);
        rightFront = initMotor(FRONT_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION);
        rightBack = initMotor(BACK_RIGHT_MOTOR_NAME, RobotTeleOp.RIGHT_SIDE_DIRECTION);
        telemetry.addLine("Lift the robot so the wheels are off the floor, then press START.");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean testLeftFront = gamepad1.left_bumper;
        boolean testRightFront = gamepad1.right_bumper;
        boolean testLeftBack = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean testRightBack = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        leftFront.setPower(testLeftFront ? TEST_POWER : 0.0);
        rightFront.setPower(testRightFront ? TEST_POWER : 0.0);
        leftBack.setPower(testLeftBack ? TEST_POWER : 0.0);
        rightBack.setPower(testRightBack ? TEST_POWER : 0.0);

        telemetry.addLine("Hold ONE button. That wheel must spin, rolling the robot FORWARD.");
        telemetry.addLine("Encoder ticks must go UP while it spins.");
        telemetry.addLine();
        addMotor("LB", FRONT_LEFT_MOTOR_NAME, "front left wheel", leftFront, testLeftFront);
        addMotor("RB", FRONT_RIGHT_MOTOR_NAME, "front right wheel", rightFront, testRightFront);
        addMotor("LT", BACK_LEFT_MOTOR_NAME, "back left wheel", leftBack, testLeftBack);
        addMotor("RT", BACK_RIGHT_MOTOR_NAME, "back right wheel", rightBack, testRightBack);
        telemetry.addLine();
        telemetry.addData("Left side direction", RobotTeleOp.LEFT_SIDE_DIRECTION);
        telemetry.addData("Right side direction", RobotTeleOp.RIGHT_SIDE_DIRECTION);
    }

    @Override
    public void stop() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
    }

    private void addMotor(String button, String name, String expectedWheel, DcMotor motor, boolean active) {
        telemetry.addData(button + "  \"" + name + "\"", "%s  ticks %d  (should be the %s)",
                active ? "SPINNING" : "off", motor.getCurrentPosition(), expectedWheel);
    }

    private DcMotor initMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
        return motor;
    }
}
