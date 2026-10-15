package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Frank Auto", group = "Auto")
public class FrankAuto extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;


    private static final double FORWARD_INCHES = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        follower follower = Constants.createFollower(hardwareMap);

        Pose start = new Pose(0.0, 0.0, Math.toRadians(0.0));
        follower.setPose(start);

        telemetry.addData("Forward distance (inches)", FORWARD_INCHES);
        telemetry.addLine("Zero distance means no movement.");
        telemetry.update();

        waitForStart();

        if (isStopRequested() || FORWARD_INCHES <= 0.0) {
            return;
        }

        Pose end = new Pose(
                FORWARD_INCHES, 0.0, Math.toRadians(0.0));

        try{
            follower.follow(Paths.line(start, end).constant(start));

            resetRuntime();

            while (opModeIsActive()
                    && follower.isBusy()
                    && getRuntime() < 10) {

                follower.update();
                idle();
            }
        } finally {
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
        }
    }
}