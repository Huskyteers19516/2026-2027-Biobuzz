package org.firstinspires.ftc.teamtestcode.apriltagtesting;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "AprilTag Test", group = "Testing")
public class AprilTagTest extends LinearOpMode {

    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final float[] DECIMATION_STEPS = {1.0f, 2.0f, 3.0f};
    private static final int DEFAULT_DECIMATION_INDEX = 1;

    private static final int DEFAULT_EXPOSURE_MS = 6;
    private static final int DEFAULT_GAIN = 250;
    private static final int GAIN_STEP = 10;

    private static final ExposureControl.Mode[] AUTO_EXPOSURE_MODES = {
            ExposureControl.Mode.ContinuousAuto,
            ExposureControl.Mode.AperturePriority,
            ExposureControl.Mode.Auto
    };

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagLibrary tagLibrary;

    private boolean liveViewOn = true;
    private int decimationIndex = DEFAULT_DECIMATION_INDEX;

    private boolean cameraControlsReady = false;
    private boolean manualExposure = false;
    private int exposureMs = DEFAULT_EXPOSURE_MS;
    private int gain = DEFAULT_GAIN;
    private int minExposureMs;
    private int maxExposureMs;
    private int minGain;
    private int maxGain;

    @Override
    public void runOpMode() {
        tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .build();
        aprilTag.setDecimation(DECIMATION_STEPS[decimationIndex]);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .addProcessor(aprilTag)
                .build();

        while (opModeInInit()) {
            readCameraControlsIfReady();
            telemetryLibrary();
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            readCameraControlsIfReady();
            handleControls();
            telemetryStatus();
            telemetryDetections();
            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private void readCameraControlsIfReady() {
        if (cameraControlsReady || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        if (exposureControl == null || gainControl == null) {
            return;
        }

        minExposureMs = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposureMs = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();

        exposureMs = Range.clip(DEFAULT_EXPOSURE_MS, minExposureMs, maxExposureMs);
        gain = Range.clip(DEFAULT_GAIN, minGain, maxGain);

        cameraControlsReady = true;
    }

    private void handleControls() {
        if (gamepad1.aWasPressed()) {
            liveViewOn = !liveViewOn;
            if (liveViewOn) {
                visionPortal.resumeLiveView();
            } else {
                visionPortal.stopLiveView();
            }
        }

        if (gamepad1.xWasPressed()) {
            decimationIndex = (decimationIndex + 1) % DECIMATION_STEPS.length;
            aprilTag.setDecimation(DECIMATION_STEPS[decimationIndex]);
        }

        boolean exposureModeToggled = gamepad1.bWasPressed();
        boolean exposureUp = gamepad1.dpadUpWasPressed();
        boolean exposureDown = gamepad1.dpadDownWasPressed();
        boolean gainUp = gamepad1.dpadRightWasPressed();
        boolean gainDown = gamepad1.dpadLeftWasPressed();

        if (!cameraControlsReady) {
            return;
        }

        boolean settingsChanged = false;

        if (exposureModeToggled) {
            manualExposure = !manualExposure;
            settingsChanged = true;
        }
        if (exposureUp) {
            exposureMs = Range.clip(exposureMs + 1, minExposureMs, maxExposureMs);
            settingsChanged = true;
        }
        if (exposureDown) {
            exposureMs = Range.clip(exposureMs - 1, minExposureMs, maxExposureMs);
            settingsChanged = true;
        }
        if (gainUp) {
            gain = Range.clip(gain + GAIN_STEP, minGain, maxGain);
            settingsChanged = true;
        }
        if (gainDown) {
            gain = Range.clip(gain - GAIN_STEP, minGain, maxGain);
            settingsChanged = true;
        }

        if (settingsChanged) {
            applyCameraSettings();
        }
    }

    private void applyCameraSettings() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        if (manualExposure) {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            sleep(20);
            gainControl.setGain(gain);
            sleep(20);
            return;
        }

        for (ExposureControl.Mode mode : AUTO_EXPOSURE_MODES) {
            if (exposureControl.isModeSupported(mode)) {
                exposureControl.setMode(mode);
                sleep(50);
                return;
            }
        }
    }

    private void telemetryLibrary() {
        telemetry.addLine("AprilTag Test");
        telemetry.addData("Camera", visionPortal.getCameraState());
        telemetry.addLine("Press START to begin detecting.");

        AprilTagClusterMetadata[] clusters = tagLibrary.getAllClusters();
        AprilTagMetadata[] tags = tagLibrary.getAllTags();

        telemetry.addLine();
        telemetry.addData("Clusters in library", clusters == null ? 0 : clusters.length);
        if (clusters != null) {
            for (AprilTagClusterMetadata cluster : clusters) {
                telemetry.addLine(String.format("  %s  (%s)", cluster.shortName, cluster.name));
            }
        }

        telemetry.addLine();
        telemetry.addData("Tags in library", tags == null ? 0 : tags.length);
        if (tags != null) {
            for (AprilTagMetadata tag : tags) {
                telemetry.addLine(String.format("  ID %d  %s  (%.2f %s)",
                        tag.id, tag.name, tag.tagsize, tag.distanceUnit));
            }
        }
    }

    private void telemetryStatus() {
        telemetry.addData("Camera", visionPortal.getCameraState());
        telemetry.addData("FPS", "%.1f", visionPortal.getFps());
        telemetry.addData("Decimation", "%.0f", DECIMATION_STEPS[decimationIndex]);
        telemetry.addData("Live view", liveViewOn ? "on" : "off");

        if (!cameraControlsReady) {
            telemetry.addData("Exposure / gain", "waiting for camera");
        } else if (manualExposure) {
            telemetry.addData("Exposure", "manual %d ms  (%d-%d)", exposureMs, minExposureMs, maxExposureMs);
            telemetry.addData("Gain", "%d  (%d-%d)", gain, minGain, maxGain);
        } else {
            telemetry.addData("Exposure", "auto");
            telemetry.addData("Manual preset", "%d ms, gain %d", exposureMs, gain);
        }

        telemetry.addLine("A live view | B auto/manual | X decimation");
        telemetry.addLine("Dpad up/down exposure | left/right gain");
    }

    private void telemetryDetections() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addLine();
        telemetry.addData("Detections", detections.size());

        AprilTagClusterDetection closestCluster = null;
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection && detection.ftcPose != null) {
                if (closestCluster == null || detection.ftcPose.range < closestCluster.ftcPose.range) {
                    closestCluster = (AprilTagClusterDetection) detection;
                }
            }
        }

        if (closestCluster != null) {
            telemetry.addLine(String.format("AIM > %s   range %.1f in   bearing %.1f   yaw %.1f",
                    closestCluster.metadata.shortName,
                    closestCluster.ftcPose.range,
                    closestCluster.ftcPose.bearing,
                    closestCluster.ftcPose.yaw));
        }

        for (AprilTagDetection detection : detections) {
            long ageMs = (System.nanoTime() - detection.frameAcquisitionNanoTime) / 1_000_000L;

            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n[CLUSTER] %s  (%s)", cluster.metadata.shortName, cluster.metadata.name));
                telemetry.addLine(String.format("Tags found %d%%   age %d ms", cluster.percentClusterFound, ageMs));
                telemetryPose(detection);
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.metadata != null) {
                    telemetry.addLine(String.format("\n[TAG %d] %s", single.id, single.metadata.name));
                    telemetry.addLine(String.format("Margin %.1f   hamming %d   age %d ms",
                            single.decisionMargin, single.hamming, ageMs));
                    telemetryPose(detection);
                } else {
                    telemetry.addLine(String.format("\n[TAG %d] not in library", single.id));
                    telemetry.addLine(String.format("Center %.0f, %.0f px   margin %.1f",
                            single.center.x, single.center.y, single.decisionMargin));
                }
            }
        }

        telemetry.addLine();
        telemetry.addLine("XYZ = right, forward, up (in)");
        telemetry.addLine("RBE = range (in), bearing, elevation (deg)");
    }

    private void telemetryPose(AprilTagDetection detection) {
        if (detection.ftcPose == null) {
            telemetry.addLine("No pose");
            return;
        }

        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f",
                detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f",
                detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f",
                detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
    }
}
