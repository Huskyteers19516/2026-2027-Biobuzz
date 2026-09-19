package org.firstinspires.ftc.teamtestcode.pedropathing;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pedro Mecanum TeleOp", group = "Pedro")
public class PedroMecanumTeleOp extends OpMode {

    private static final double STICK_DEADZONE = 0.05;
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;
    private static final Pose START_POSE = new Pose(0.0, 0.0, 0.0);

    private Follower follower;
    private boolean fieldCentric = false;
    private boolean toggleWasPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);
    }

    @Override
    public void init_loop() {
        follower.localizer.update();
        telemetry.addLine("Pedro Mecanum TeleOp");
        telemetry.addLine("Left stick: translate | Right stick X: turn");
        telemetry.addLine("Y: toggle field/robot centric | Back: reset pose");
        telemetry.addLine("Left bumper: slow mode");
        addPoseTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
    }

    @Override
    public void loop() {
        boolean togglePressed = gamepad1.y;
        if (togglePressed && !toggleWasPressed) {
            fieldCentric = !fieldCentric;
        }
        toggleWasPressed = togglePressed;

        if (gamepad1.back) {
            follower.setPose(START_POSE);
        }

        double speed = gamepad1.left_bumper ? SLOW_SPEED : NORMAL_SPEED;
        double forward = deadzone(-gamepad1.left_stick_y) * speed;
        double strafe = deadzone(-gamepad1.left_stick_x) * speed;
        double turn = deadzone(-gamepad1.right_stick_x) * speed;

        DrivePowers powers = new DrivePowers(forward, strafe, turn);
        if (fieldCentric) {
            powers = ManualDrive.fieldCentric(powers, follower.pose().heading());
        }

        follower.manual(powers);
        follower.update();

        telemetry.addData("Mode", fieldCentric ? "FIELD centric" : "ROBOT centric");
        telemetry.addData("Command", "fwd %.2f  left %.2f  turn %.2f", powers.forward(), powers.strafe(), powers.turn());
        addPoseTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg, CCW +)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Velocity", "%s", follower.velocity());
    }

    private double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
