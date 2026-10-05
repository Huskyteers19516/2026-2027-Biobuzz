package org.firstinspires.ftc.teamtestcode.Opmode;

import java.lang.annotation.Target;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Meet0 Auto", group = "Meet0")

    private enum LaunchState{
        IDLE,
        PREPARE,
        LAUNCH,
    }
    private enum AutonomousState {
        Launch,
        WAIT_FOR_LAUNCH,
        
    }
public class Meet0Auto extends LinearOpMode {
    private double TargetVelocity;
    private AutonomousState autonomousState;
    private int shotstofire = 0;
    private LaunchState launchState;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
//Feeder Variables
    private ElapsedTime feederTimer = new ElapsedTime();
    final double intakeTime = 0.25;
    private ElapsedTime betweenShotTimer = new ElapsedTime();
    final double timeBetweenShots = 0.25;

    private CRServo leftIntake;
    private CRServo rightIntake;

    private DcMotorEx launcher;

    private final ElapsedTime runtime = new ElapsedTime();
    

    @Override
    public void init() {
        launchState = launchState.IDLE;
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftIntake = hardwareMap.get(CRServo.class, "left_intake");
        rightIntake = hardwareMap.get(CRServo.class, "right_intake");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAll();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }
        
        runtime.reset();

        stopAll();

        telemetry.addData("Status", "Finished");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.update();
    }
    public void init_loop(){

    }
    @Override 
    public void start() {

    }

    public void loop() {
        switch (autonomousState){
        case LAUNCH:
            launch(true);
            autonomousState = autonomousState.WAIT_FOR_LAUNCH;
            break;
        case WAIT_FOR_LAUNCH:
            if(launch(false)){
                shotstofire--;
                if(shotstofire > 0){
                    autonomousState = AutonomousState.Launch;
                } else {
                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        launcher.setVelocity(0);

                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                }
            }
    }
}
    private void stopAll() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);
        launcher.setPower(0.0);
    }
    boolean launch(boolean shotRequested){
        switch(launchState){
            case IDLE:
                if(shotRequested){
                    launchState = LaunchState.PREPARE;
                }

            case PREPARE:
                launcher.setVelocity(TargetPower);
                if(launcher.getVelocity() == TargetPower){
                    launchState = LaunchState.LAUNCH;
                    leftIntake.setPower(1);
                    rightIntake.setPower(1);
                    feederTimer.reset();
                }
            case LAUNCH:
                if(feederTimer.seconds() > intakeTime){
                    leftIntake.setPower(0);
                    rightIntake.setower(0);
                    if(betweenShotTimer.seconds() > timeBetweenShots){
                        launchState = launchState.IDLE;
                        return true;    
                    } 
                }     
        } return false;
    }
}

