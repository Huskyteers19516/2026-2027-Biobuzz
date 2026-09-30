package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class GoalScanner {

    public static final String WEBCAM_NAME = "Webcam 1";
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final float DECIMATION = 2.0f;
    public static final boolean LIVE_VIEW = false;

    public static final long SIGHTING_FRESHNESS_MS = 250L;
    public static final long RECENCY_PREFERENCE_MS = 50L;
    public static final int MIN_PERCENT_CLUSTER_FOUND = 50;

    public static final String CLUSTER_RED_SCORING = "RED SCORING";
    public static final String CLUSTER_RED_AUDIENCE = "RED AUDIENCE";
    public static final String CLUSTER_BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String CLUSTER_BLUE_SCORING = "BLUE SCORING";

    public enum Alliance {

        RED(CLUSTER_RED_SCORING, CLUSTER_RED_AUDIENCE),
        BLUE(CLUSTER_BLUE_SCORING, CLUSTER_BLUE_AUDIENCE);

        public final String firstCluster;
        public final String secondCluster;

        Alliance(String firstCluster, String secondCluster) {
            this.firstCluster = firstCluster;
            this.secondCluster = secondCluster;
        }
    }

    public static class Sighting {

        public final String shortName;
        public final int percentClusterFound;
        public final double range;
        public final double bearing;
        public final double elevation;
        public final long frameNanoTime;
        public final long observedNanoTime;

        Sighting(String shortName, int percentClusterFound, double range, double bearing,
                 double elevation, long frameNanoTime, long observedNanoTime) {
            this.shortName = shortName;
            this.percentClusterFound = percentClusterFound;
            this.range = range;
            this.bearing = bearing;
            this.elevation = elevation;
            this.frameNanoTime = frameNanoTime;
            this.observedNanoTime = observedNanoTime;
        }

        public long ageMs() {
            return (System.nanoTime() - observedNanoTime) / 1_000_000L;
        }

        public boolean isFresh() {
            return ageMs() <= SIGHTING_FRESHNESS_MS;
        }

        public boolean isStrongEnough() {
            return percentClusterFound >= MIN_PERCENT_CLUSTER_FOUND;
        }

        public boolean isUsable() {
            return isFresh() && isStrongEnough();
        }

        public String describe() {
            return String.format("%s  %d%%  range %.1f in  bearing %.1f deg  age %d ms",
                    shortName, percentClusterFound, range, bearing, ageMs());
        }
    }

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private final Map<String, Sighting> latestByCluster = new LinkedHashMap<String, Sighting>();

    private String fault = "";
    private long updateCount = 0L;
    private long clusterFrameCount = 0L;

    public GoalScanner(HardwareMap hardwareMap) {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setDrawTagOutline(LIVE_VIEW)
                    .setDrawTagID(LIVE_VIEW)
                    .setDrawAxes(false)
                    .build();
            aprilTag.setDecimation(DECIMATION);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                    .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                    .enableLiveView(LIVE_VIEW)
                    .setAutoStopLiveView(true)
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            aprilTag = null;
            visionPortal = null;
            fault = "camera '" + WEBCAM_NAME + "' unavailable: " + e.getClass().getSimpleName();
        }
    }

    public boolean isAvailable() {
        return aprilTag != null;
    }

    public String fault() {
        return fault;
    }

    public void update() {
        if (aprilTag == null) {
            return;
        }

        updateCount++;

        List<AprilTagDetection> detections;
        try {
            detections = aprilTag.getDetections();
        } catch (Exception e) {
            fault = "getDetections failed: " + e.getClass().getSimpleName();
            return;
        }

        if (detections == null || detections.isEmpty()) {
            return;
        }

        long now = System.nanoTime();
        boolean sawNewCluster = false;

        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }
            if (detection.ftcPose == null) {
                continue;
            }

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata == null || cluster.metadata.shortName == null) {
                continue;
            }

            String shortName = cluster.metadata.shortName;
            Sighting previous = latestByCluster.get(shortName);
            boolean frameClockUsable = detection.frameAcquisitionNanoTime > 0L;
            if (frameClockUsable && previous != null
                    && detection.frameAcquisitionNanoTime <= previous.frameNanoTime) {
                continue;
            }

            sawNewCluster = true;

            latestByCluster.put(shortName, new Sighting(
                    shortName,
                    cluster.percentClusterFound,
                    detection.ftcPose.range,
                    detection.ftcPose.bearing,
                    detection.ftcPose.elevation,
                    detection.frameAcquisitionNanoTime,
                    now));
        }

        if (sawNewCluster) {
            clusterFrameCount++;
        }
    }

    public Sighting bestTargetFor(Alliance alliance) {
        Sighting first = usableSighting(alliance.firstCluster);
        Sighting second = usableSighting(alliance.secondCluster);

        if (first == null) {
            return second;
        }
        if (second == null) {
            return first;
        }

        long deltaMs = (first.observedNanoTime - second.observedNanoTime) / 1_000_000L;
        if (deltaMs > RECENCY_PREFERENCE_MS) {
            return first;
        }
        if (-deltaMs > RECENCY_PREFERENCE_MS) {
            return second;
        }
        if (first.percentClusterFound != second.percentClusterFound) {
            return first.percentClusterFound > second.percentClusterFound ? first : second;
        }
        return first.range <= second.range ? first : second;
    }

    public Sighting usableSightingSince(String shortName, long sinceNanoTime) {
        Sighting sighting = usableSighting(shortName);
        if (sighting == null || sighting.observedNanoTime <= sinceNanoTime) {
            return null;
        }
        return sighting;
    }

    public String candidateSummary(Alliance alliance) {
        return describeCandidate(alliance.firstCluster) + "   |   " + describeCandidate(alliance.secondCluster);
    }

    public String cameraState() {
        if (visionPortal == null) {
            return "NO CAMERA";
        }
        try {
            return String.format("%s  %.1f FPS", visionPortal.getCameraState(), visionPortal.getFps());
        } catch (Exception e) {
            return "state unavailable";
        }
    }

    public long updateCount() {
        return updateCount;
    }

    public long clusterFrameCount() {
        return clusterFrameCount;
    }

    public void close() {
        if (visionPortal != null) {
            try {
                visionPortal.close();
            } catch (Exception e) {
                fault = "close failed: " + e.getClass().getSimpleName();
            }
            visionPortal = null;
        }
        aprilTag = null;
    }

    private Sighting usableSighting(String shortName) {
        Sighting sighting = latestByCluster.get(shortName);
        if (sighting == null || !sighting.isUsable()) {
            return null;
        }
        return sighting;
    }

    private String describeCandidate(String shortName) {
        Sighting sighting = latestByCluster.get(shortName);
        if (sighting == null) {
            return shortName + " never seen";
        }
        String flag = "";
        if (!sighting.isFresh()) {
            flag = " STALE";
        } else if (!sighting.isStrongEnough()) {
            flag = String.format(" WEAK<%d%%", MIN_PERCENT_CLUSTER_FOUND);
        }
        return String.format("%s %d%% %.0fin %dms%s", shortName, sighting.percentClusterFound,
                sighting.range, sighting.ageMs(), flag);
    }
}
