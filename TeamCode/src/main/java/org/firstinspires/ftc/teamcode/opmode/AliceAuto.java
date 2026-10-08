//copy import statements and variables
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

@Autonomous(name = "Alice Auto", group = "Robot")

public class AliceAuto extends LinearOpMode {
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;


    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        stopMotors();
        telemetry.addLine("Simple Drive Test");
        telemetry.addLine("Ready to move forward and backward.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Move forward for 1 second
            telemetry.addLine("Moving FORWARD");
            telemetry.update();

            setDrivePower(0.5);
            sleep(1000);

            // Stop
            stopMotors();
            sleep(500);

            // Move right for 1 second
            telemetry.addLine("Moving RIGHT");
            telemetry.update();

            setDrivePower(-0.5);
            sleep(1000);

            // Stop
            stopMotors();

            telemetry.addLine("Test complete.");
            telemetry.update();
        }
    }

    private void setDrivePower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void setTurnRight(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
    }

    private void setTurnLeft(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}