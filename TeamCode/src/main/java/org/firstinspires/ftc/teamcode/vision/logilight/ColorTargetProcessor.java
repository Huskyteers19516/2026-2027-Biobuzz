package org.firstinspires.ftc.teamcode.vision.logilight;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.logilight.math.CameraModel;
import org.firstinspires.ftc.teamcode.vision.logilight.math.Vec3;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public final class ColorTargetProcessor implements VisionProcessor {

    public static final double DEFAULT_PROCESS_SCALE = 0.5;
    public static final double MIN_PROCESS_SCALE = 0.1;
    public static final double MAX_PROCESS_SCALE = 1.0;
    public static final int MIN_PROCESS_DIMENSION = 16;
    public static final int MAX_KERNEL_SIZE = 31;
    public static final int MAX_TARGETS_PER_PROFILE = 64;
    public static final int MAX_CANDIDATES_PER_PROFILE = 256;
    public static final int MAX_CACHED_IMAGES = 6;
    public static final double BORDER_ASPECT_FACTOR = 2.0;

    static final int METRIC_MIN_X = 0;
    static final int METRIC_MIN_Y = 1;
    static final int METRIC_MAX_X = 2;
    static final int METRIC_MAX_Y = 3;
    static final int METRIC_PERIMETER = 4;
    static final int METRIC_AREA = 5;
    static final int METRIC_CENTROID_X = 6;
    static final int METRIC_CENTROID_Y = 7;
    static final int METRIC_COUNT = 8;

    private static final double MIN_CENTROID_AREA = 1e-6;
    private static final double NANOS_PER_MS = 1.0e6;
    private static final double FULL_CHANNEL = 255.0;

    private final LogiFrameContext context;
    private volatile ColorFrame latest = ColorFrame.EMPTY;
    private volatile long thresholdCaptureNanos;
    private volatile boolean thresholdValid;

    private Mat smallRgba;
    private Mat rgb;
    private Mat mask;
    private Mat wrapMask;
    private Mat combinedMask;
    private Mat hierarchy;
    private MatOfInt hullIndices;
    private MatOfPoint2f contourFloat;

    private final List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    private final List<CachedMat> blurredImages = new ArrayList<CachedMat>();
    private final List<CachedMat> convertedImages = new ArrayList<CachedMat>();
    private final List<CachedMat> morphKernels = new ArrayList<CachedMat>();
    private final List<Candidate> candidatePool = new ArrayList<Candidate>();
    private final List<Candidate> candidates = new ArrayList<Candidate>();
    private final Scalar lowerBound = new Scalar(0.0, 0.0, 0.0, 0.0);
    private final Scalar upperBound = new Scalar(0.0, 0.0, 0.0, 0.0);
    private final Scalar zero = new Scalar(0.0);
    private int[] pointBuffer = new int[512];
    private int[] hullBuffer = new int[256];
    private double[] contourAreas = new double[512];
    private final double[] metrics = new double[METRIC_COUNT];
    private int candidatePoolIndex;
    private long frameStamp;

    public ColorTargetProcessor(LogiFrameContext context) {
        this.context = context == null ? new LogiFrameContext(null) : context;
    }

    public LogiFrameContext context() {
        return context;
    }

    public ColorFrame latest() {
        ColorFrame frame = latest;
        return frame == null ? ColorFrame.EMPTY : frame;
    }

    public Mat latestThreshold() {
        Mat combined = combinedMask;
        if (!thresholdValid || combined == null || combined.empty()) {
            return null;
        }
        return combined;
    }

    public Mat latestThreshold(long captureNanos) {
        if (thresholdCaptureNanos != captureNanos) {
            return null;
        }
        return latestThreshold();
    }

    public long thresholdCaptureNanos() {
        return thresholdCaptureNanos;
    }

    void release() {
        latest = ColorFrame.EMPTY;
        thresholdValid = false;
        releaseContours();
        releaseQuietly(smallRgba);
        smallRgba = null;
        releaseQuietly(rgb);
        rgb = null;
        releaseQuietly(mask);
        mask = null;
        releaseQuietly(wrapMask);
        wrapMask = null;
        releaseQuietly(combinedMask);
        combinedMask = null;
        releaseQuietly(hierarchy);
        hierarchy = null;
        releaseQuietly(hullIndices);
        hullIndices = null;
        releaseQuietly(contourFloat);
        contourFloat = null;
        releaseCache(blurredImages);
        releaseCache(convertedImages);
        releaseCache(morphKernels);
        candidates.clear();
    }

    private static void releaseCache(List<CachedMat> cache) {
        for (int i = 0; i < cache.size(); i++) {
            CachedMat entry = cache.get(i);
            if (entry != null) {
                releaseQuietly(entry.mat);
            }
        }
        cache.clear();
    }

    private static void releaseQuietly(Mat mat) {
        if (mat == null) {
            return;
        }
        try {
            mat.release();
        } catch (Throwable ignored) {
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        try {
            context.onInit(width, height, calibration);
        } catch (Throwable t) {
            context.reportError("color init", t);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long startNanos = System.nanoTime();
        try {
            context.markFrame(captureTimeNanos);
            LogiPipeline pipeline = context.activePipeline();
            if (pipeline == null || !pipeline.typeOrDefault().usesColor() || frame == null || frame.empty()) {
                thresholdValid = false;
                return null;
            }
            ColorFrame result = process(frame, captureTimeNanos, startNanos);
            latest = result;
            return result;
        } catch (Throwable t) {
            thresholdValid = false;
            releaseContours();
            context.reportError("color processor", t);
            return null;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    private ColorFrame process(Mat frame, long captureNanos, long startNanos) {
        frameStamp++;
        int width = frame.cols();
        int height = frame.rows();
        CameraModel model = context.ensureModel(width, height);
        LogiCameraConfig camera = context.camera();
        double scale = processScale(camera.colorProcessScale);
        int processWidth = clampDimension((int) Math.round(width * scale), width);
        int processHeight = clampDimension((int) Math.round(height * scale), height);
        Mat source = frame;
        if (processWidth != width || processHeight != height) {
            if (smallRgba == null) {
                smallRgba = new Mat();
            }
            Imgproc.resize(frame, smallRgba, new Size(processWidth, processHeight), 0.0, 0.0, Imgproc.INTER_AREA);
            source = smallRgba;
        }
        ensureWorkMats();
        toRgb(source, rgb);
        combinedMask.create(processHeight, processWidth, CvType.CV_8UC1);
        combinedMask.setTo(zero);

        double scaleX = width / (double) processWidth;
        double scaleY = height / (double) processHeight;
        double processPixels = (double) processWidth * (double) processHeight;

        ColorProfile[] profiles = context.profilesForFrame();
        String[] names = new String[profiles.length];
        int[] counts = new int[profiles.length];
        int used = 0;
        TargetSort primarySort = null;
        List<ColorTarget> merged = new ArrayList<ColorTarget>();
        for (ColorProfile profile : profiles) {
            if (profile == null || !profile.enabled) {
                continue;
            }
            if (primarySort == null) {
                primarySort = profile.sort == null ? TargetSort.LARGEST : profile.sort;
            }
            List<ColorTarget> found = detect(profile, model, processPixels, scaleX, scaleY);
            names[used] = profile.name == null ? "" : profile.name;
            counts[used] = found.size();
            used++;
            merged.addAll(found);
        }
        if (merged.size() > 1) {
            Collections.sort(merged, ColorTarget.comparator(primarySort));
        }
        String[] usedNames = new String[used];
        int[] usedCounts = new int[used];
        System.arraycopy(names, 0, usedNames, 0, used);
        System.arraycopy(counts, 0, usedCounts, 0, used);
        thresholdCaptureNanos = captureNanos;
        thresholdValid = true;
        double processingMs = (System.nanoTime() - startNanos) / NANOS_PER_MS;
        return new ColorFrame(captureNanos, processingMs, merged, usedNames, usedCounts,
                context.activePipelineIndex(), width, height);
    }

    private List<ColorTarget> detect(ColorProfile profile, CameraModel model, double processPixels,
                                     double scaleX, double scaleY) {
        ColorSpaceMode space = profile.colorSpace == null ? ColorSpaceMode.HSV : profile.colorSpace;
        Mat image = converted(space, oddKernel(profile.blurSize));
        threshold(image, profile, space);
        morph(profile.erodeSize, true);
        morph(profile.dilateSize, false);
        Core.bitwise_or(combinedMask, mask, combinedMask);
        collectCandidates(profile, processPixels);
        rejectSpeckles(profile.speckleRejectPercent);
        if (candidates.isEmpty()) {
            return new ArrayList<ColorTarget>();
        }
        String name = profile.name == null ? "" : profile.name;
        double centerHeight = profile.targetCenterHeightInches;
        List<ColorTarget> targets = new ArrayList<ColorTarget>(candidates.size());
        for (Candidate candidate : candidates) {
            targets.add(buildTarget(name, candidate, model, centerHeight, processPixels, scaleX, scaleY));
        }
        candidates.clear();
        if (targets.size() > 1) {
            Collections.sort(targets, ColorTarget.comparator(profile.sort));
        }
        int limit = targetLimit(profile.maxTargets);
        while (targets.size() > limit) {
            targets.remove(targets.size() - 1);
        }
        return targets;
    }

    private ColorTarget buildTarget(String name, Candidate c, CameraModel model, double centerHeight,
                                    double processPixels, double scaleX, double scaleY) {
        double u = toFull(c.centroidX, scaleX);
        double v = toFull(c.centroidY, scaleY);
        double[] angles = model.targetAngles(u, v);
        double groundForward = Double.NaN;
        double groundLeft = Double.NaN;
        double groundDistance = Double.NaN;
        double groundBearing = Double.NaN;
        Vec3 ground = model.groundIntersection(u, v, centerHeight);
        if (ground != null && ground.isFinite()) {
            groundForward = ground.x;
            groundLeft = ground.y;
            groundDistance = Math.hypot(ground.x, ground.y);
            groundBearing = Math.toDegrees(Math.atan2(ground.y, ground.x));
        }
        double[] corners = rectCorners(c, scaleX, scaleY);
        double ta = c.area / processPixels * 100.0;
        return new ColorTarget(name, u, v,
                angles[0], angles[1], angles[2], angles[3],
                ta, c.area * scaleX * scaleY,
                c.boxWidth * scaleX, c.boxHeight * scaleY,
                c.fullness, c.aspectRatio, c.circularity,
                corners,
                groundForward, groundLeft, groundDistance, groundBearing,
                c.boxLeft * scaleX - 0.5, c.boxTop * scaleY - 0.5,
                c.rectWidth * scaleX, c.rectHeight * scaleY, c.rectAngleDegrees);
    }

    private void threshold(Mat image, ColorProfile profile, ColorSpaceMode space) {
        double min0 = finiteOr(profile.min0, 0.0);
        double min1 = finiteOr(profile.min1, 0.0);
        double min2 = finiteOr(profile.min2, 0.0);
        double max0 = finiteOr(profile.max0, FULL_CHANNEL);
        double max1 = finiteOr(profile.max1, FULL_CHANNEL);
        double max2 = finiteOr(profile.max2, FULL_CHANNEL);
        if (space == ColorSpaceMode.HSV && min0 > max0) {
            setBounds(min0, min1, min2, ColorSpaceMode.HUE_MAX, max1, max2);
            Core.inRange(image, lowerBound, upperBound, mask);
            setBounds(0.0, min1, min2, max0, max1, max2);
            Core.inRange(image, lowerBound, upperBound, wrapMask);
            Core.bitwise_or(mask, wrapMask, mask);
        } else {
            setBounds(min0, min1, min2, max0, max1, max2);
            Core.inRange(image, lowerBound, upperBound, mask);
        }
    }

    private void setBounds(double min0, double min1, double min2, double max0, double max1, double max2) {
        lowerBound.val[0] = min0;
        lowerBound.val[1] = min1;
        lowerBound.val[2] = min2;
        lowerBound.val[3] = 0.0;
        upperBound.val[0] = max0;
        upperBound.val[1] = max1;
        upperBound.val[2] = max2;
        upperBound.val[3] = FULL_CHANNEL;
    }

    private void morph(int size, boolean erode) {
        int kernelSize = Math.min(size, MAX_KERNEL_SIZE);
        if (kernelSize <= 1) {
            return;
        }
        Mat kernel = morphKernel(kernelSize);
        if (erode) {
            Imgproc.erode(mask, mask, kernel);
        } else {
            Imgproc.dilate(mask, mask, kernel);
        }
    }

    private void collectCandidates(ColorProfile profile, double processPixels) {
        candidates.clear();
        candidatePoolIndex = 0;
        double minAreaPercent = Math.max(0.0, finiteOr(profile.minAreaPercent, 0.0));
        double maxAreaPercent = positiveOr(profile.maxAreaPercent, 100.0);
        double minAreaPixels = minAreaPercent / 100.0 * processPixels;
        double maxAreaPixels = maxAreaPercent / 100.0 * processPixels;
        double minFullness = finiteOr(profile.minFullness, 0.0);
        double minAspect = Math.max(0.0, finiteOr(profile.minAspectRatio, 0.0));
        double maxAspect = positiveOr(profile.maxAspectRatio, Double.POSITIVE_INFINITY);
        double minCircularity = finiteOr(profile.minCircularity, 0.0);
        int maskCols = mask.cols();
        int maskRows = mask.rows();
        try {
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            int total = contours.size();
            if (contourAreas.length < total) {
                contourAreas = new double[Math.max(total, contourAreas.length * 2)];
            }
            final double[] areas = contourAreas;
            int passing = 0;
            for (int i = 0; i < total; i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (!(area > 0.0) || area < minAreaPixels || area > maxAreaPixels) {
                    areas[i] = -1.0;
                } else {
                    areas[i] = area;
                    passing++;
                }
            }
            if (passing <= MAX_CANDIDATES_PER_PROFILE) {
                for (int i = 0; i < total; i++) {
                    if (areas[i] > 0.0) {
                        addCandidate(contours.get(i), areas[i], minAspect, maxAspect, minFullness, minCircularity,
                                maskCols, maskRows);
                    }
                }
            } else {
                Integer[] order = new Integer[passing];
                int n = 0;
                for (int i = 0; i < total; i++) {
                    if (areas[i] > 0.0) {
                        order[n++] = Integer.valueOf(i);
                    }
                }
                Arrays.sort(order, 0, n, new Comparator<Integer>() {
                    @Override
                    public int compare(Integer a, Integer b) {
                        return Double.compare(areas[b.intValue()], areas[a.intValue()]);
                    }
                });
                for (int k = 0; k < n && candidates.size() < MAX_CANDIDATES_PER_PROFILE; k++) {
                    int i = order[k].intValue();
                    addCandidate(contours.get(i), areas[i], minAspect, maxAspect, minFullness, minCircularity,
                            maskCols, maskRows);
                }
            }
        } finally {
            releaseContours();
        }
    }

    private void addCandidate(MatOfPoint contour, double area, double minAspect, double maxAspect,
                              double minFullness, double minCircularity, int maskCols, int maskRows) {
        Candidate candidate = measure(contour, area, minAspect, maxAspect, minFullness, minCircularity, maskCols, maskRows);
        if (candidate != null) {
            candidates.add(candidate);
        }
    }

    private Candidate measure(MatOfPoint contour, double area, double minAspect, double maxAspect,
                              double minFullness, double minCircularity, int maskCols, int maskRows) {
        int count = (int) contour.total();
        if (count < 3) {
            return null;
        }
        if (pointBuffer.length < count * 2) {
            pointBuffer = new int[count * 4];
        }
        contour.get(0, 0, pointBuffer);
        polygonMetrics(pointBuffer, count, metrics);
        double minX = metrics[METRIC_MIN_X];
        double minY = metrics[METRIC_MIN_Y];
        double maxX = metrics[METRIC_MAX_X];
        double maxY = metrics[METRIC_MAX_Y];
        double perimeter = metrics[METRIC_PERIMETER];
        double boxWidth = maxX - minX + 1.0;
        double boxHeight = maxY - minY + 1.0;
        double aspect = boxWidth / boxHeight;
        boolean touchesBorder = minX <= 0.0 || minY <= 0.0 || maxX >= maskCols - 1 || maxY >= maskRows - 1;
        double aspectLow = touchesBorder ? minAspect / BORDER_ASPECT_FACTOR : minAspect;
        double aspectHigh = touchesBorder ? maxAspect * BORDER_ASPECT_FACTOR : maxAspect;
        if (aspect < aspectLow || aspect > aspectHigh) {
            return null;
        }
        Imgproc.convexHull(contour, hullIndices);
        int hullCount = (int) hullIndices.total();
        if (hullBuffer.length < hullCount) {
            hullBuffer = new int[hullCount * 2];
        }
        double hullArea = 0.0;
        if (hullCount >= 3) {
            hullIndices.get(0, 0, hullBuffer);
            hullArea = hullArea(pointBuffer, count, hullBuffer, hullCount);
        }
        double fullness = hullArea > 0.0 ? Math.min(1.0, area / hullArea) : 0.0;
        if (fullness < minFullness) {
            return null;
        }
        double circularity = circularity(area, perimeter);
        if (circularity < minCircularity) {
            return null;
        }
        double centroidX = metrics[METRIC_CENTROID_X];
        double centroidY = metrics[METRIC_CENTROID_Y];
        contour.convertTo(contourFloat, CvType.CV_32FC2);
        RotatedRect rect = Imgproc.minAreaRect(contourFloat);
        Candidate candidate = obtainCandidate();
        candidate.area = area;
        candidate.boxLeft = minX;
        candidate.boxTop = minY;
        candidate.boxWidth = boxWidth;
        candidate.boxHeight = boxHeight;
        candidate.aspectRatio = aspect;
        candidate.fullness = fullness;
        candidate.circularity = circularity;
        candidate.centroidX = centroidX;
        candidate.centroidY = centroidY;
        if (rect != null && rect.center != null && rect.size != null) {
            candidate.rectCenterX = rect.center.x;
            candidate.rectCenterY = rect.center.y;
            candidate.rectWidth = rect.size.width;
            candidate.rectHeight = rect.size.height;
            candidate.rectAngleDegrees = rect.angle;
        } else {
            candidate.rectCenterX = (minX + maxX) / 2.0;
            candidate.rectCenterY = (minY + maxY) / 2.0;
            candidate.rectWidth = boxWidth;
            candidate.rectHeight = boxHeight;
            candidate.rectAngleDegrees = 0.0;
        }
        return candidate;
    }

    private void rejectSpeckles(double speckleRejectPercent) {
        if (!(speckleRejectPercent > 0.0) || candidates.size() < 2) {
            return;
        }
        double largest = 0.0;
        for (Candidate candidate : candidates) {
            largest = Math.max(largest, candidate.area);
        }
        double minimum = largest * speckleRejectPercent / 100.0;
        for (int i = candidates.size() - 1; i >= 0; i--) {
            if (candidates.get(i).area < minimum) {
                candidates.remove(i);
            }
        }
    }

    static void polygonMetrics(int[] points, int count, double[] out) {
        int minX = Integer.MAX_VALUE;
        int minY = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE;
        int maxY = Integer.MIN_VALUE;
        double perimeter = 0.0;
        double doubledArea = 0.0;
        double centroidXSum = 0.0;
        double centroidYSum = 0.0;
        for (int i = 0; i < count; i++) {
            int x = points[2 * i];
            int y = points[2 * i + 1];
            int next = i + 1 == count ? 0 : i + 1;
            int nx = points[2 * next];
            int ny = points[2 * next + 1];
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
            perimeter += Math.hypot(nx - x, ny - y);
            double cross = (double) x * ny - (double) nx * y;
            doubledArea += cross;
            centroidXSum += (x + nx) * cross;
            centroidYSum += (y + ny) * cross;
        }
        double signedArea = doubledArea / 2.0;
        out[METRIC_MIN_X] = minX;
        out[METRIC_MIN_Y] = minY;
        out[METRIC_MAX_X] = maxX;
        out[METRIC_MAX_Y] = maxY;
        out[METRIC_PERIMETER] = perimeter;
        out[METRIC_AREA] = Math.abs(signedArea);
        if (Math.abs(signedArea) > MIN_CENTROID_AREA) {
            out[METRIC_CENTROID_X] = centroidXSum / (6.0 * signedArea);
            out[METRIC_CENTROID_Y] = centroidYSum / (6.0 * signedArea);
        } else {
            out[METRIC_CENTROID_X] = (minX + maxX) / 2.0;
            out[METRIC_CENTROID_Y] = (minY + maxY) / 2.0;
        }
    }

    static double hullArea(int[] points, int count, int[] hull, int hullCount) {
        if (hullCount < 3) {
            return 0.0;
        }
        double doubledHull = 0.0;
        for (int i = 0; i < hullCount; i++) {
            int a = clampIndex(hull[i], count);
            int b = clampIndex(hull[i + 1 == hullCount ? 0 : i + 1], count);
            doubledHull += (double) points[2 * a] * points[2 * b + 1] - (double) points[2 * b] * points[2 * a + 1];
        }
        return Math.abs(doubledHull) / 2.0;
    }

    static double circularity(double area, double perimeter) {
        if (!(perimeter > 0.0)) {
            return 0.0;
        }
        return Math.min(1.0, 4.0 * Math.PI * area / (perimeter * perimeter));
    }

    private static double[] rectCorners(Candidate c, double scaleX, double scaleY) {
        return rectCorners(c.rectCenterX, c.rectCenterY, c.rectWidth, c.rectHeight, c.rectAngleDegrees, scaleX, scaleY);
    }

    static double[] rectCorners(double centerX, double centerY, double width, double height,
                                double angleDegrees, double scaleX, double scaleY) {
        double angle = Math.toRadians(angleDegrees);
        double halfCos = Math.cos(angle) * 0.5;
        double halfSin = Math.sin(angle) * 0.5;
        double p0x = centerX - halfSin * height - halfCos * width;
        double p0y = centerY + halfCos * height - halfSin * width;
        double p1x = centerX + halfSin * height - halfCos * width;
        double p1y = centerY - halfCos * height - halfSin * width;
        double p2x = 2.0 * centerX - p0x;
        double p2y = 2.0 * centerY - p0y;
        double p3x = 2.0 * centerX - p1x;
        double p3y = 2.0 * centerY - p1y;
        return new double[]{
                toFull(p0x, scaleX), toFull(p0y, scaleY),
                toFull(p1x, scaleX), toFull(p1y, scaleY),
                toFull(p2x, scaleX), toFull(p2y, scaleY),
                toFull(p3x, scaleX), toFull(p3y, scaleY)};
    }

    private Mat converted(ColorSpaceMode space, int blurKernel) {
        Mat blurredRgb = blurred(blurKernel);
        if (space == ColorSpaceMode.RGB) {
            return blurredRgb;
        }
        int key = space.ordinal() * 1000 + blurKernel;
        CachedMat entry = cachedEntry(convertedImages, key);
        if (entry.stamp != frameStamp) {
            int code = space == ColorSpaceMode.YCRCB ? Imgproc.COLOR_RGB2YCrCb : Imgproc.COLOR_RGB2HSV;
            Imgproc.cvtColor(blurredRgb, entry.mat, code);
            entry.stamp = frameStamp;
        }
        return entry.mat;
    }

    private Mat blurred(int blurKernel) {
        if (blurKernel <= 1) {
            return rgb;
        }
        CachedMat entry = cachedEntry(blurredImages, blurKernel);
        if (entry.stamp != frameStamp) {
            Imgproc.GaussianBlur(rgb, entry.mat, new Size(blurKernel, blurKernel), 0.0);
            entry.stamp = frameStamp;
        }
        return entry.mat;
    }

    private Mat morphKernel(int size) {
        CachedMat entry = findEntry(morphKernels, size);
        if (entry == null) {
            evictIfFull(morphKernels);
            entry = new CachedMat(size, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(size, size)));
            morphKernels.add(entry);
        }
        entry.stamp = frameStamp;
        return entry.mat;
    }

    private CachedMat cachedEntry(List<CachedMat> cache, int key) {
        CachedMat entry = findEntry(cache, key);
        if (entry == null) {
            evictIfFull(cache);
            entry = new CachedMat(key, new Mat());
            entry.stamp = -1L;
            cache.add(entry);
        }
        return entry;
    }

    private static CachedMat findEntry(List<CachedMat> cache, int key) {
        for (int i = 0; i < cache.size(); i++) {
            CachedMat entry = cache.get(i);
            if (entry.key == key) {
                return entry;
            }
        }
        return null;
    }

    private void evictIfFull(List<CachedMat> cache) {
        if (cache.size() < MAX_CACHED_IMAGES) {
            return;
        }
        int oldest = 0;
        for (int i = 1; i < cache.size(); i++) {
            if (cache.get(i).stamp < cache.get(oldest).stamp) {
                oldest = i;
            }
        }
        CachedMat removed = cache.remove(oldest);
        if (removed.stamp != frameStamp) {
            removed.mat.release();
        }
    }

    private void ensureWorkMats() {
        if (rgb == null) {
            rgb = new Mat();
        }
        if (mask == null) {
            mask = new Mat();
        }
        if (wrapMask == null) {
            wrapMask = new Mat();
        }
        if (combinedMask == null) {
            combinedMask = new Mat();
        }
        if (hierarchy == null) {
            hierarchy = new Mat();
        }
        if (hullIndices == null) {
            hullIndices = new MatOfInt();
        }
        if (contourFloat == null) {
            contourFloat = new MatOfPoint2f();
        }
    }

    private static void toRgb(Mat source, Mat destination) {
        int channels = source.channels();
        if (channels == 4) {
            Imgproc.cvtColor(source, destination, Imgproc.COLOR_RGBA2RGB);
        } else if (channels == 3) {
            source.copyTo(destination);
        } else if (channels == 1) {
            Imgproc.cvtColor(source, destination, Imgproc.COLOR_GRAY2RGB);
        } else {
            throw new IllegalArgumentException("unsupported frame channel count " + channels);
        }
    }

    private Candidate obtainCandidate() {
        if (candidatePoolIndex < candidatePool.size()) {
            return candidatePool.get(candidatePoolIndex++);
        }
        Candidate candidate = new Candidate();
        candidatePool.add(candidate);
        candidatePoolIndex++;
        return candidate;
    }

    private void releaseContours() {
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            if (contour != null) {
                contour.release();
            }
        }
        contours.clear();
    }

    private static double toFull(double processCoordinate, double scale) {
        return (processCoordinate + 0.5) * scale - 0.5;
    }

    private static int clampIndex(int index, int count) {
        if (index < 0) {
            return 0;
        }
        return index >= count ? count - 1 : index;
    }

    private static int oddKernel(int blurSize) {
        if (blurSize <= 1) {
            return 0;
        }
        int kernel = Math.min(blurSize, MAX_KERNEL_SIZE) | 1;
        return Math.min(kernel, MAX_KERNEL_SIZE);
    }

    private static int targetLimit(int maxTargets) {
        if (maxTargets <= 0) {
            return MAX_TARGETS_PER_PROFILE;
        }
        return Math.min(maxTargets, MAX_TARGETS_PER_PROFILE);
    }

    static double processScale(double configured) {
        if (Double.isNaN(configured) || Double.isInfinite(configured) || configured <= 0.0) {
            return DEFAULT_PROCESS_SCALE;
        }
        return Math.max(MIN_PROCESS_SCALE, Math.min(MAX_PROCESS_SCALE, configured));
    }

    private static int clampDimension(int value, int full) {
        int minimum = Math.min(full, MIN_PROCESS_DIMENSION);
        return Math.max(minimum, Math.min(full, value));
    }

    private static double finiteOr(double value, double fallback) {
        return Double.isNaN(value) || Double.isInfinite(value) ? fallback : value;
    }

    private static double positiveOr(double value, double fallback) {
        return value > 0.0 && !Double.isInfinite(value) ? value : fallback;
    }

    private static final class CachedMat {
        final int key;
        final Mat mat;
        long stamp;

        CachedMat(int key, Mat mat) {
            this.key = key;
            this.mat = mat;
        }
    }

    private static final class Candidate {
        double area;
        double boxLeft;
        double boxTop;
        double boxWidth;
        double boxHeight;
        double aspectRatio;
        double fullness;
        double circularity;
        double centroidX;
        double centroidY;
        double rectCenterX;
        double rectCenterY;
        double rectWidth;
        double rectHeight;
        double rectAngleDegrees;
    }
}
