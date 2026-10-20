package org.firstinspires.ftc.teamtestcode.apriltagtesting;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "AprilTag Tester (Logitech)", group = "Testing")
public class AprilTagTester extends LinearOpMode {

    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private static final float DECIMATION = 2.0f;

    private static final double CENTERED_BEARING_DEGREES = 3.0;
    private static final double INCH_TO_CM = 2.54;
    private static final int MAX_TAG_ID = 586;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagLibrary tagLibrary;
    private final Map<String, List<Integer>> clusterMemberIds = new LinkedHashMap<>();

    private final ElapsedTime sinceLastSeen = new ElapsedTime();
    private boolean everSeen = false;
    private String lastSeenName = "";

    @Override
    public void runOpMode() {
        tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        buildClusterMemberIds();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .build();
        aprilTag.setDecimation(DECIMATION);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        while (opModeInInit()) {
            telemetry.addLine("=== APRILTAG TESTER ===");
            telemetry.addData("Camera", visionPortal.getCameraState());
            telemetry.addData("Resolution", "%d x %d", CAMERA_WIDTH, CAMERA_HEIGHT);
            telemetry.addLine();
            telemetry.addLine("Wait for Camera = STREAMING, then press START.");
            telemetry.addLine();
            telemetryLibraryIds();
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection target = pickTarget(detections);

            if (!detections.isEmpty()) {
                everSeen = true;
                sinceLastSeen.reset();
                lastSeenName = describe(detections.get(0));
            }

            telemetry.addLine("=== APRILTAG TESTER ===");
            telemetry.addData("Camera", "%s   %.1f FPS", visionPortal.getCameraState(), visionPortal.getFps());
            telemetry.addLine();

            if (detections.isEmpty()) {
                telemetry.addLine("DETECTED:  NO");
                if (everSeen) {
                    telemetry.addData("Last seen", "%.1f s ago  (%s)", sinceLastSeen.seconds(), lastSeenName);
                } else {
                    telemetry.addData("Last seen", "never");
                }
            } else {
                telemetry.addLine(String.format("DETECTED:  YES  (%d visible)", detections.size()));
                telemetry.addData("IDs", visibleIds(detections));
                telemetry.addLine();
                telemetryTarget(target);
                telemetry.addLine();
                telemetryAllVisible(detections);
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private AprilTagDetection pickTarget(List<AprilTagDetection> detections) {
        AprilTagDetection bestCluster = null;
        AprilTagDetection bestTag = null;
        AprilTagDetection anything = null;

        for (AprilTagDetection detection : detections) {
            if (anything == null) {
                anything = detection;
            }
            if (detection.ftcPose == null) {
                continue;
            }
            if (detection instanceof AprilTagClusterDetection) {
                if (bestCluster == null || detection.ftcPose.range < bestCluster.ftcPose.range) {
                    bestCluster = detection;
                }
            } else if (bestTag == null || detection.ftcPose.range < bestTag.ftcPose.range) {
                bestTag = detection;
            }
        }

        if (bestCluster != null) {
            return bestCluster;
        }
        if (bestTag != null) {
            return bestTag;
        }
        return anything;
    }

    private void telemetryTarget(AprilTagDetection target) {
        telemetry.addData("Target", describe(target));

        if (target instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) target;
            List<Integer> ids = memberIdsOf(cluster);
            int seen = (int) Math.round(cluster.percentClusterFound / 100.0 * ids.size());
            telemetry.addData("Tag IDs in cluster", joinIds(ids));
            telemetry.addData("Tags seen", "%d of %d  (%d %%)", seen, ids.size(), cluster.percentClusterFound);
        } else if (target instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) target;
            telemetry.addData("Tag ID", single.id);
            telemetry.addData("Confidence (margin)", "%.1f", single.decisionMargin);
        }

        if (target.ftcPose == null) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) target;
            telemetry.addLine("Not in the tag library, no position available.");
            telemetry.addData("Pixel center", "%.0f, %.0f", single.center.x, single.center.y);
            return;
        }

        double x = target.ftcPose.x;
        double y = target.ftcPose.y;
        double z = target.ftcPose.z;
        double bearing = target.ftcPose.bearing;

        telemetry.addLine();
        telemetry.addLine("--- POSITION (from camera) ---");
        telemetry.addData("Distance", "%.1f in  (%.1f cm)", target.ftcPose.range, target.ftcPose.range * INCH_TO_CM);
        telemetry.addData("Left / Right", "%.1f in %s", Math.abs(x), x >= 0 ? "RIGHT" : "LEFT");
        telemetry.addData("Forward", "%.1f in", y);
        telemetry.addData("Up / Down", "%.1f in %s", Math.abs(z), z >= 0 ? "UP" : "DOWN");

        telemetry.addLine();
        telemetry.addLine("--- ANGLE ---");
        telemetry.addData("Bearing", "%.1f deg  %s", bearing, bearingHint(bearing));
        telemetry.addData("Elevation", "%.1f deg", target.ftcPose.elevation);
        telemetry.addData("Tag yaw", "%.1f deg", target.ftcPose.yaw);
        telemetry.addData("Tag pitch / roll", "%.1f / %.1f deg", target.ftcPose.pitch, target.ftcPose.roll);

        long ageMs = (System.nanoTime() - target.frameAcquisitionNanoTime) / 1_000_000L;
        telemetry.addData("Frame age", "%d ms", ageMs);
    }

    private void telemetryAllVisible(List<AprilTagDetection> detections) {
        telemetry.addLine("--- ALL VISIBLE ---");
        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose != null) {
                telemetry.addLine(String.format("%s   %.1f in   bearing %.1f",
                        describe(detection), detection.ftcPose.range, detection.ftcPose.bearing));
            } else {
                telemetry.addLine(describe(detection) + "   no position");
            }
        }
    }

    private String bearingHint(double bearing) {
        if (Math.abs(bearing) <= CENTERED_BEARING_DEGREES) {
            return "(CENTERED)";
        }
        return bearing > 0 ? "(tag is LEFT, turn left)" : "(tag is RIGHT, turn right)";
    }

    private void buildClusterMemberIds() {
        for (int id = 0; id <= MAX_TAG_ID; id++) {
            AprilTagClusterMetadata cluster = tagLibrary.lookupCluster(id);
            if (cluster == null) {
                continue;
            }
            List<Integer> ids = clusterMemberIds.get(cluster.name);
            if (ids == null) {
                ids = new ArrayList<>();
                clusterMemberIds.put(cluster.name, ids);
            }
            ids.add(id);
        }
    }

    private void telemetryLibraryIds() {
        telemetry.addLine("--- TAG IDs IN LIBRARY ---");

        AprilTagClusterMetadata[] clusters = tagLibrary.getAllClusters();
        if (clusters != null) {
            for (AprilTagClusterMetadata cluster : clusters) {
                telemetry.addLine(String.format("Cluster %s:  IDs %s",
                        cluster.shortName, joinIds(clusterMemberIds.get(cluster.name))));
            }
        }

        AprilTagMetadata[] tags = tagLibrary.getAllTags();
        if (tags != null) {
            for (AprilTagMetadata tag : tags) {
                telemetry.addLine(String.format("Tag:  ID %d  %s", tag.id, tag.name));
            }
        }
    }

    private List<Integer> memberIdsOf(AprilTagClusterDetection cluster) {
        List<Integer> ids = clusterMemberIds.get(cluster.metadata.name);
        return ids == null ? new ArrayList<Integer>() : ids;
    }

    private String visibleIds(List<AprilTagDetection> detections) {
        StringBuilder text = new StringBuilder();
        for (AprilTagDetection detection : detections) {
            if (text.length() > 0) {
                text.append("  |  ");
            }
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                text.append(cluster.metadata.shortName).append(" ").append(joinIds(memberIdsOf(cluster)));
            } else {
                text.append(((AprilTagSingleDetection) detection).id);
            }
        }
        return text.toString();
    }

    private String joinIds(List<Integer> ids) {
        if (ids == null || ids.isEmpty()) {
            return "none";
        }
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < ids.size(); i++) {
            if (i > 0) {
                text.append(", ");
            }
            text.append(ids.get(i));
        }
        return text.toString();
    }

    private String describe(AprilTagDetection detection) {
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return "Cluster " + cluster.metadata.shortName + " (IDs " + joinIds(memberIdsOf(cluster)) + ")";
        }

        AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
        if (single.metadata != null) {
            return "Tag ID " + single.id + " " + single.metadata.name;
        }
        return "Tag ID " + single.id + " (unknown)";
    }
}
