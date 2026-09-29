package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

public void runOpMode() {
    follower = Constants.createFollower(hardwareMap);

    intake = hardwareMap.get(DcMotor.class, RobotTeleOp.INTAKE_NAME);
    transfer = hardwareMap.get(DcMotor.class, RobotTeleOp.TRANSFER_NAME);
    outtake = hardwareMap.get(DcMotorEx.class, RobotTeleOp.OUTTAKE_NAME);
    launcherServo = hardwareMap.get(Servo.class, RobotTeleOp.LAUNCHER_SERVO_NAME);

    intake.setDirection(RobotTeleOp.INTAKE_DIRECTION);
    transfer.setDirection(RobotTeleOp.TRANSFER_DIRECTION);
    outtake.setDirection(RobotTeleOp.OUTTAKE_DIRECTION);

    intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    follower.setPose(FieldPoints.START_POSE);

    List<FieldPoints.Step> mission = FieldPoints.mission();

    while (!isStarted() && !isStopRequested()) {
        follower.localizer.update();
        telemetry.addLine("Match Auto");
        telemetry.addData("Steps", mission.size());
        telemetry.addData("Budget", "%.1f s", AUTONOMOUS_BUDGET_SECONDS);
        telemetry.addData("Start pose", formatPose(FieldPoints.START_POSE));
        telemetry.addData("Measured pose", formatPose(follower.pose()));
        telemetry.addData("Localizer", Constants.PINPOINT_NAME);
        telemetry.addData("Outtake ticks", outtake.getCurrentPosition());
        if (!FieldPoints.POINTS_CONFIRMED) {
            telemetry.addLine("!! FIELD POINTS NOT CONFIRMED - AUTO WILL NOT MOVE");
            telemetry.addLine("Measure the real coordinates in FieldPoints.java,");
            telemetry.addLine("then set POINTS_CONFIRMED = true.");
        }

        telemetry.addLine("Nothing is commanded until START");
        telemetry.update();
    }

    if (!FieldPoints.POINTS_CONFIRMED) {
        stopAll();

        while (opModeIsActive()) {
            telemetry.addLine("!! FIELD POINTS NOT CONFIRMED - REFUSING TO DRIVE");
            telemetry.addLine("The poses in FieldPoints.java are placeholders.");
            telemetry.addLine("Measure them, then set POINTS_CONFIRMED = true.");
            telemetry.update();
        }

        return;
    }

