package org.firstinspires.ftc.teamcode.vision.logilight.math;

import java.util.Locale;

public final class CameraModel {
    public static final double FALLBACK_FOCAL_PIXELS_AT_640 = 578.272;
    public static final double MAX_GROUND_DISTANCE_INCHES = 240.0;
    public static final int DEFAULT_WIDTH = 640;
    public static final int DEFAULT_HEIGHT = 480;

    private static final double MIN_DEPTH = 1e-6;
    private static final double MIN_RAY_VERTICAL = 1e-9;

    private static final Mat3 OPTICAL_TO_BODY = new Mat3(
            0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0);

    public final int width;
    public final int height;
    public final double fx;
    public final double fy;
    public final double cx;
    public final double cy;
    public final double forwardInches;
    public final double leftInches;
    public final double upInches;
    public final double yawLeftDegrees;
    public final double tiltUpDegrees;
    public final double rollClockwiseDegrees;
    public final double crosshairX;
    public final double crosshairY;

    private final Mat3 opticalToRobot;
    private final Mat3 robotToOptical;
    private final Vec3 mount;
    private final double rollCos;
    private final double rollSin;
    private final double[] crosshairAnglesNoCrosshair;

    public CameraModel(int width, int height,
                       double fx, double fy, double cx, double cy,
                       double forwardInches, double leftInches, double upInches,
                       double yawLeftDegrees, double tiltUpDegrees, double rollClockwiseDegrees,
                       double crosshairX, double crosshairY) {
        this.width = width > 0 ? width : DEFAULT_WIDTH;
        this.height = height > 0 ? height : DEFAULT_HEIGHT;
        double[] fallback = fallbackIntrinsics(this.width, this.height);
        this.fx = positiveOr(fx, fallback[0]);
        this.fy = positiveOr(fy, fallback[1]);
        this.cx = finiteOr(cx, fallback[2]);
        this.cy = finiteOr(cy, fallback[3]);
        this.forwardInches = finiteOr(forwardInches, 0.0);
        this.leftInches = finiteOr(leftInches, 0.0);
        this.upInches = finiteOr(upInches, 0.0);
        this.yawLeftDegrees = finiteOr(yawLeftDegrees, 0.0);
        this.tiltUpDegrees = finiteOr(tiltUpDegrees, 0.0);
        this.rollClockwiseDegrees = finiteOr(rollClockwiseDegrees, 0.0);
        this.crosshairX = Mat3.clamp(finiteOr(crosshairX, 0.0), -1.0, 1.0);
        this.crosshairY = Mat3.clamp(finiteOr(crosshairY, 0.0), -1.0, 1.0);

        double rollRad = Math.toRadians(this.rollClockwiseDegrees);
        this.rollCos = Math.cos(rollRad);
        this.rollSin = Math.sin(rollRad);
        this.opticalToRobot = Mat3.rotZ(Math.toRadians(this.yawLeftDegrees))
                .times(Mat3.rotY(-Math.toRadians(this.tiltUpDegrees)))
                .times(Mat3.rotX(rollRad))
                .times(OPTICAL_TO_BODY);
        this.robotToOptical = opticalToRobot.transpose();
        this.mount = new Vec3(this.forwardInches, this.leftInches, this.upInches);
        double[] crosshair = crosshairPixel();
        this.crosshairAnglesNoCrosshair = anglesNoCrosshair(crosshair[0], crosshair[1]);
    }

    public static CameraModel uncalibrated(int width, int height) {
        double[] intrinsics = fallbackIntrinsics(width > 0 ? width : DEFAULT_WIDTH, height > 0 ? height : DEFAULT_HEIGHT);
        return new CameraModel(width, height, intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public static Mat3 opticalToBodyRotation() {
        return OPTICAL_TO_BODY;
    }

    public Mat3 opticalToRobotRotation() {
        return opticalToRobot;
    }

    public Mat3 robotToOpticalRotation() {
        return robotToOptical;
    }

    public Vec3 mountPosition() {
        return mount;
    }

    public Vec3 opticalAxisInRobot() {
        return opticalToRobot.column(2);
    }

    public Vec3 opticalPointToRobot(Vec3 pOpt) {
        if (pOpt == null) {
            return null;
        }
        return mount.plus(opticalToRobot.times(pOpt));
    }

    public Vec3 opticalDirectionToRobot(Vec3 dOpt) {
        if (dOpt == null) {
            return null;
        }
        return opticalToRobot.times(dOpt);
    }

    public Vec3 robotPointToOptical(Vec3 pRobot) {
        if (pRobot == null) {
            return null;
        }
        return robotToOptical.times(pRobot.minus(mount));
    }

    public Vec3 robotDirectionToOptical(Vec3 dRobot) {
        if (dRobot == null) {
            return null;
        }
        return robotToOptical.times(dRobot);
    }

    public Mat3 opticalRotationToRobot(Mat3 rOpt) {
        if (rOpt == null) {
            return null;
        }
        return opticalToRobot.times(rOpt);
    }

    public Mat3 robotRotationToOptical(Mat3 rRobot) {
        if (rRobot == null) {
            return null;
        }
        return robotToOptical.times(rRobot);
    }

    public double[] project(Vec3 pOpt) {
        if (pOpt == null || !pOpt.isFinite() || pOpt.z < MIN_DEPTH) {
            return null;
        }
        return new double[]{fx * pOpt.x / pOpt.z + cx, fy * pOpt.y / pOpt.z + cy};
    }

    public double[] projectRobotPoint(Vec3 pRobot) {
        return project(robotPointToOptical(pRobot));
    }

    public Vec3 pixelRay(double u, double v) {
        return new Vec3((u - cx) / fx, (v - cy) / fy, 1.0);
    }

    public Vec3 robotRay(double u, double v) {
        return opticalToRobot.times(pixelRay(u, v));
    }

    public double[] anglesNoCrosshair(double u, double v) {
        double nx = (u - cx) / fx;
        double ny = (v - cy) / fy;
        double nxLevel = nx * rollCos - ny * rollSin;
        double nyLevel = nx * rollSin + ny * rollCos;
        return new double[]{Math.toDegrees(Math.atan(nxLevel)), Math.toDegrees(Math.atan(-nyLevel))};
    }

    public double[] crosshairPixel() {
        return new double[]{cx + crosshairX * width / 2.0, cy - crosshairY * height / 2.0};
    }

    public double[] crosshairFromPixel(double u, double v) {
        return new double[]{
                Mat3.clamp((u - cx) / (width / 2.0), -1.0, 1.0),
                Mat3.clamp((cy - v) / (height / 2.0), -1.0, 1.0)};
    }

    public double[] targetAngles(double u, double v) {
        double[] noCrosshair = anglesNoCrosshair(u, v);
        return new double[]{
                noCrosshair[0] - crosshairAnglesNoCrosshair[0],
                noCrosshair[1] - crosshairAnglesNoCrosshair[1],
                noCrosshair[0],
                noCrosshair[1]};
    }

    public Vec3 groundIntersection(double u, double v, double targetHeightInches) {
        if (!Vec3.isFiniteValue(u) || !Vec3.isFiniteValue(v) || !Vec3.isFiniteValue(targetHeightInches)) {
            return null;
        }
        Vec3 direction = robotRay(u, v);
        if (Math.abs(direction.z) < MIN_RAY_VERTICAL) {
            return null;
        }
        double distanceAlongRay = (targetHeightInches - mount.z) / direction.z;
        if (!(distanceAlongRay > 0.0) || Double.isInfinite(distanceAlongRay)) {
            return null;
        }
        Vec3 hit = mount.plus(direction.times(distanceAlongRay));
        if (!hit.isFinite() || hit.horizontalNorm() > MAX_GROUND_DISTANCE_INCHES) {
            return null;
        }
        return new Vec3(hit.x, hit.y, targetHeightInches);
    }

    public boolean isInImage(double u, double v) {
        return u >= 0.0 && v >= 0.0 && u <= width && v <= height;
    }

    public double horizontalFovDegrees() {
        return Math.toDegrees(Math.atan(cx / fx) + Math.atan((width - cx) / fx));
    }

    public double verticalFovDegrees() {
        return Math.toDegrees(Math.atan(cy / fy) + Math.atan((height - cy) / fy));
    }

    public CameraModel withCrosshair(double x, double y) {
        return new CameraModel(width, height, fx, fy, cx, cy,
                forwardInches, leftInches, upInches, yawLeftDegrees, tiltUpDegrees, rollClockwiseDegrees,
                x, y);
    }

    public CameraModel withMount(double fwd, double left, double up, double yawLeft, double tiltUp, double rollCw) {
        return new CameraModel(width, height, fx, fy, cx, cy,
                fwd, left, up, yawLeft, tiltUp, rollCw,
                crosshairX, crosshairY);
    }

    public CameraModel withIntrinsics(int newWidth, int newHeight, double newFx, double newFy, double newCx, double newCy) {
        return new CameraModel(newWidth, newHeight, newFx, newFy, newCx, newCy,
                forwardInches, leftInches, upInches, yawLeftDegrees, tiltUpDegrees, rollClockwiseDegrees,
                crosshairX, crosshairY);
    }

    public static double[] fallbackIntrinsics(int width, int height) {
        return new double[]{FALLBACK_FOCAL_PIXELS_AT_640, FALLBACK_FOCAL_PIXELS_AT_640, width / 2, height / 2};
    }

    public static double[] scaleIntrinsics(double fx, double fy, double cx, double cy,
                                           int fromW, int fromH, int toW, int toH) {
        if (fromW <= 0 || fromH <= 0 || toW <= 0 || toH <= 0) {
            return new double[]{fx, fy, cx, cy};
        }
        double scaleX = toW / (double) fromW;
        double scaleY = toH / (double) fromH;
        return new double[]{fx * scaleX, fy * scaleY, cx * scaleX, cy * scaleY};
    }

    private static double finiteOr(double value, double fallback) {
        return Vec3.isFiniteValue(value) ? value : fallback;
    }

    private static double positiveOr(double value, double fallback) {
        return Vec3.isFiniteValue(value) && value > 0.0 ? value : fallback;
    }

    @Override
    public String toString() {
        return String.format(Locale.US,
                "%dx%d  f %.1f/%.1f  c %.1f/%.1f  mount fwd %.1f  left %.1f  up %.1f in  yawL %.1f  tiltUp %.1f  rollCw %.1f deg  crosshair %.2f/%.2f",
                width, height, fx, fy, cx, cy, forwardInches, leftInches, upInches,
                yawLeftDegrees, tiltUpDegrees, rollClockwiseDegrees, crosshairX, crosshairY);
    }
}
