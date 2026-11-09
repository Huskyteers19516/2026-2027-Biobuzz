package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.logilight.ColorTarget;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLight;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightSettings;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiLightTelemetry;
import org.firstinspires.ftc.teamcode.vision.logilight.LogiResult;

@TeleOp(name = "LogiLight Quick Start", group = "LogiLight")
public class LogiLightQuickStart extends OpMode {

    private LogiLight camera;
    private LogiMecanumDrive drive;

    @Override
    public void init() {
        camera = new LogiLight(hardwareMap);
        camera.pipelineSwitch("goals");
        drive = new LogiMecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {
        camera.update();
        LogiLightTelemetry.addStatus(telemetry, camera);
        LogiLightTelemetry.addResult(telemetry, camera.getLatestResult());
        telemetry.addLine("gp1 A = goals   Y = yellow pollen   B = red nectar   X = blue nectar");
        telemetry.addLine("gp1 hold RB = turn to face the target (only after START)");
    }

    @Override
    public void loop() {
        camera.update();

        if (gamepad1.aWasPressed()) {
            camera.pipelineSwitch("goals");
        }
        if (gamepad1.yWasPressed()) {
            camera.pipelineSwitch("pollen-yellow");
        }
        if (gamepad1.bWasPressed()) {
            camera.pipelineSwitch("nectar-red");
        }
        if (gamepad1.xWasPressed()) {
            camera.pipelineSwitch("nectar-blue");
        }

        LogiResult result = camera.getLatestResult();

        double forward = -gamepad1.left_stick_y;
        double strafeLeft = -gamepad1.left_stick_x;
        double turnLeft = -gamepad1.right_stick_x;

        if (gamepad1.right_bumper && result.isValid()) {
            turnLeft = LogiLightSettings.AIM.turnLeftPowerForTx(result.getTx());
        }
        drive.drive(forward, strafeLeft, turnLeft);

        if (result.isValid()) {
            telemetry.addData("Target", result.getPrimaryName());
            telemetry.addData("tx / ty / ta", "%.1f deg  %.1f deg  %.2f %%", result.getTx(), result.getTy(), result.getTa());
            ColorTarget ball = result.getPrimaryColor();
            if (ball != null && ball.hasGroundPosition()) {
                telemetry.addData("Ball on floor", "%.1f in ahead  %.1f in left", ball.groundForwardInches, ball.groundLeftInches);
            }
        } else {
            telemetry.addData("Target", "none");
        }
        telemetry.addData("Latency", "%.0f ms", result.getStaleness());
        LogiLightTelemetry.addCompact(telemetry, result);
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (camera != null) {
            camera.close();
        }
    }
}
