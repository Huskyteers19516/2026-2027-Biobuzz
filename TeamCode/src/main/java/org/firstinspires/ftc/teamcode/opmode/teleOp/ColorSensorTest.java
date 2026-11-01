package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.COLOR_SENSOR;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends OpMode {

    private static final float START_GAIN = 10.0f;
    private static final float GAIN_STEP = 1.0f;
    private static final float MIN_GAIN = 1.0f;
    private static final double START_THRESHOLD_MM = 50.0;
    private static final double THRESHOLD_STEP_MM = 5.0;
    private static final double RELEASE_MARGIN_MM = 10.0;

    private final float[] hsv = new float[3];
    private ColorRangeSensor sensor;
    private float gain = START_GAIN;
    private double thresholdMm = START_THRESHOLD_MM;
    private boolean ballDetected = false;
    private int ballCount = 0;
    private double noBallMm = Double.NaN;
    private double ballMm = Double.NaN;
    private String noBallReading = "not saved";
    private String ballReading = "not saved";

    @Override
    public void init() {
        sensor = hardwareMap.get(ColorRangeSensor.class, COLOR_SENSOR);
        sensor.setGain(gain);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) setGain(gain + GAIN_STEP);
        if (gamepad1.dpadDownWasPressed()) setGain(gain - GAIN_STEP);
        if (gamepad1.dpadRightWasPressed()) thresholdMm += THRESHOLD_STEP_MM;
        if (gamepad1.dpadLeftWasPressed()) thresholdMm = Math.max(THRESHOLD_STEP_MM, thresholdMm - THRESHOLD_STEP_MM);
        if (gamepad1.yWasPressed()) ballCount = 0;

        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);
        double distanceMm = sensor.getDistance(DistanceUnit.MM);
        String reading = String.format("%.0f mm   hue %.0f  sat %.2f  val %.2f", distanceMm, hsv[0], hsv[1], hsv[2]);

        if (gamepad1.aWasPressed()) {
            noBallMm = distanceMm;
            noBallReading = reading;
            moveThresholdHalfway();
        }
        if (gamepad1.bWasPressed()) {
            ballMm = distanceMm;
            ballReading = reading;
            moveThresholdHalfway();
        }

        if (distanceMm < thresholdMm) {
            if (!ballDetected) {
                ballCount++;
            }
            ballDetected = true;
        } else if (distanceMm > thresholdMm + RELEASE_MARGIN_MM) {
            ballDetected = false;
        }

        boolean maxedOut = colors.red >= 1.0f || colors.green >= 1.0f || colors.blue >= 1.0f;

        telemetry.addLine(ballDetected ? ">>> BALL <<<" : "no ball");
        telemetry.addData("Distance", "%.1f mm   (BALL when under %.0f mm)", distanceMm, thresholdMm);
        telemetry.addData("Hue / Sat / Val", "%.0f   %.2f   %.2f", hsv[0], hsv[1], hsv[2]);
        telemetry.addData("R / G / B / Alpha", "%.3f   %.3f   %.3f   %.3f",
                colors.red, colors.green, colors.blue, colors.alpha);
        telemetry.addData("Gain", "%.0f%s", gain,
                maxedOut ? "   TOO HIGH - a color is maxed at 1.000, press dpad down" : "");
        telemetry.addData("Balls counted", "%d   (Y = reset)", ballCount);
        telemetry.addLine();
        telemetry.addData("A  no ball", noBallReading);
        telemetry.addData("B  ball", ballReading);
        telemetry.addLine();
        telemetry.addData("Sensor", "%s on %s", sensor.getDeviceName(), sensor.getConnectionInfo());
        telemetry.addLine("A = save with NO ball   B = save with your ball where it will sit");
        telemetry.addLine("After A and B, the BALL distance moves halfway between them.");
        telemetry.addLine("dpad up/down gain +-1   right/left BALL distance +-5 mm");
        telemetry.addLine("Nothing moves in this OpMode. Tell me the A and B lines (press B again for each ball color).");
    }

    @Override
    public void loop() {
        init_loop();
    }

    private void setGain(float newGain) {
        gain = Math.max(MIN_GAIN, newGain);
        sensor.setGain(gain);
    }

    private void moveThresholdHalfway() {
        if (ballMm < noBallMm) {
            thresholdMm = (noBallMm + ballMm) / 2.0;
        }
    }
}
