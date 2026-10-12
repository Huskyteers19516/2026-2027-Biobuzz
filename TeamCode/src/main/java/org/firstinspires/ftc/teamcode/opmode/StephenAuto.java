package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Stephen's auto", group = "Testing")
public class StephenAuto extends LinearOpMode {

    private static final double MOVE_INCHES = 12.0;
    // Fraction of the achievable speed configured in Constants, not motor power.
    private static final double PATH_SPEED = 0.4;
    // Only aborts a stuck path; this does not set the duration of a move.
    private static final double PATH_TIMEOUT_SECONDS = 8.0;
    private static final Pose START_POSE = new Pose(0, 0, 0);

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);

        try {
            stopDrive();
            ((Foresight) follower.algorithm()).config.maxPathSpeed.set(PATH_SPEED);
            follower.setPose(START_POSE);

            // Coordinates are inches relative to the starting position.
            // At heading 0, +X is forward and +Y is left.
            Pose forwardEnd = new Pose(MOVE_INCHES, 0, 0);
            Pose leftEnd = new Pose(MOVE_INCHES, MOVE_INCHES, 0);
            Pose backwardEnd = new Pose(0, MOVE_INCHES, 0);

            telemetry.addLine("Ready: forward, left, backward, right with Pedro Pathing.");
            telemetry.addData("Inches per move", MOVE_INCHES);
            telemetry.addData("Localizer", Constants.PINPOINT_NAME);
            telemetry.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            // Treat the robot's position when START is pressed as the origin.
            follower.setPose(START_POSE);
            follower.update();

            if (!followPath("Forward", START_POSE, forwardEnd)) return;
            if (!followPath("Left", forwardEnd, leftEnd)) return;
            if (!followPath("Backward", leftEnd, backwardEnd)) return;
            if (!followPath("Right", backwardEnd, START_POSE)) return;

            telemetry.addLine("Done: square path complete.");
            telemetry.update();
        } finally {
            stopDrive();
        }
    }

    private boolean followPath(String movement, Pose start, Pose end) {
        if (!opModeIsActive()) {
            return false;
        }

        // Keep the original heading so left/right strafe and backward reverses.
        Path path = Paths.line(start, end).constant(START_POSE.heading());
        follower.follow(path);
        ElapsedTime timeout = new ElapsedTime();

        while (opModeIsActive() && (follower.following() || follower.isBusy())) {
            if (timeout.seconds() >= PATH_TIMEOUT_SECONDS) {
                stopDrive();
                telemetry.addData("Stopped", "%s path timed out", movement);
                telemetry.update();
                return false;
            }

            follower.update();
            Pose current = follower.pose();
            telemetry.addData("Movement", movement);
            telemetry.addData("Position", "x %.1f, y %.1f in", current.x(), current.y());
            telemetry.addData("Distance to target", "%.1f in", current.distance(end));
            telemetry.update();
            idle();
        }

        return opModeIsActive();
    }

    private void stopDrive() {
        follower.stop();
        // Pedro 3's stop() changes mode; stop the motors immediately as well.
        follower.drivetrain.stop(true);
    }
}
