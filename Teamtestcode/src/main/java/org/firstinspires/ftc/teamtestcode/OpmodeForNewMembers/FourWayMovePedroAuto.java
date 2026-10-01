package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamtestcode.pedropathing.Constants;

@Autonomous(name = "Four Way Move Pedro Auto", group = "Testing")
public class FourWayMovePedroAuto extends LinearOpMode {

    Follower follower;
    ElapsedTime timer = new ElapsedTime();

    // heading 0: +x is forward, +y is left
    Pose start = new Pose(0.0, 0.0, 0.0);
    Pose front = new Pose(24.0, 0.0, 0.0);
    Pose right = new Pose(0.0, -24.0, 0.0);
    Pose end = new Pose(6.0, 0.0, 0.0);

    double maxTime = 6;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(start);

        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addLine("ready");
            telemetry.addLine("forward, back, right, left, then forward a little");
            telemetry.addData("pose", poseText(follower.pose()));
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        follower.setPose(start);

        goTo(front);
        waitTime(0.5);
        goTo(start);
        waitTime(0.5);
        goTo(right);
        waitTime(0.5);
        goTo(start);
        waitTime(0.5);
        goTo(end);

        follower.stop();
        follower.update();

        telemetry.addLine("done");
        telemetry.addData("pose", poseText(follower.pose()));
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }

    void goTo(Pose target) {
        Pose here = follower.pose();
        Path path = Paths.line(here, target).linear(here, target);
        follower.follow(path);

        timer.reset();
        while (opModeIsActive() && timer.seconds() < maxTime
                && (follower.following() || follower.isBusy())) {
            follower.update();

            telemetry.addData("target", poseText(target));
            telemetry.addData("pose", poseText(follower.pose()));
            telemetry.addData("time", "%.1f", timer.seconds());
            telemetry.update();
        }

        follower.hold(timer.seconds() < maxTime ? target : follower.pose());
        follower.update();
    }

    void waitTime(double seconds) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
        }
    }

    String poseText(Pose pose) {
        return String.format("x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
