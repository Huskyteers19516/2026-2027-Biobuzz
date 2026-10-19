package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
//Orientation
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import com.qualcomm.robotcore.hardware.IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name = "Michael's Auto", group = "Auto")

public class MichaelAuto extends LinearOpMode {
    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    // private IMU imu;

    public void runOpMode(){
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        // imu = hardwareMap.get(IMU.class, "imu");
        //pasted IMU initialization
        // RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        // RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        // imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);  
        
        turnLeft();
        sleep(100);
        forwards();
        sleep(300);
        stop();


    }

    public void stop(){
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    public void forwards(){
        left_front.setPower(1);
        left_back.setPower(1);
        right_front.setPower(1);
        right_back.setPower(1);
    }

    public void turnLeft(){
        
        left_front.setPower(-1);
        left_back.setPower(-1);
        right_front.setPower(1);
        right_back.setPower(1);
    }

    public void turnRight(){
        left_front.setPower(1);
        left_back.setPower(1);
        right_front.setPower(-1);
        right_back.setPower(-1);
    }
    
    // public void turnDegrees(double deltaD){
    //     deltaD = -deltaD;
    //     //initial degrees
    //     double start = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    //     //wanted degrees
    //     double target = start + deltaD;

    //     //Simplifies target to between -180 and 180 degrees
    //     target = normalizeHeading(target);

    //     double error = deltaD;
    //     //Looping to remove error, threshold is max amt of error we want.
    //     while(opModeIsActive() && Math.abs(error) > HEADING_THRESHOLD){
    //         //renewing start
    //         double current = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    //         //how far off in degrees we are
    //         error = getAngleError(target, current);

    //         turnLeft();

    //     }
    //     stop();

    // }
}