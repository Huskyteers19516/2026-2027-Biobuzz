package org.firstinspires.ftc.teamtestcode.pedropathing;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pedro Pinpoint Pose Test", group = "Pedro")
public class PedroPinpointPoseTest extends OpMode {

    private static final Pose START_POSE = new Pose(0.0, 0.0, 0.0);

    private PinpointLocalizer localizer;

    @Override
    public void init() {
        localizer = Constants.createLocalizer(hardwareMap);
        localizer.setPose(START_POSE);
    }

    @Override
    public void init_loop() {
        localizer.update();
        telemetry.addLine("Pedro Pinpoint Pose Test (no motors are used)");
        telemetry.addLine("Push the robot by hand. Back: reset pose.");
        addPoseTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        localizer.setPose(START_POSE);
    }

    @Override
    public void loop() {
        if (gamepad1.back) {
            localizer.setPose(START_POSE);
        }

        localizer.update();

        telemetry.addLine("Push FORWARD: X must go UP");
        telemetry.addLine("Push LEFT: Y must go UP");
        telemetry.addLine("Turn CCW (seen from above): heading must go UP");
        addPoseTelemetry();
        telemetry.update();
    }

    private void addPoseTelemetry() {
        Pose pose = localizer.pose();
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg, CCW +)", "%.1f", Math.toDegrees(pose.heading()));
    }
}
