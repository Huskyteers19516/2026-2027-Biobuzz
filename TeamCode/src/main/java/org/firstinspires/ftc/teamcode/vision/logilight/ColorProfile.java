package org.firstinspires.ftc.teamcode.vision.logilight;

import java.util.Locale;

public class ColorProfile {

    public String name = "yellow";
    public boolean enabled = true;
    public ColorSpaceMode colorSpace = ColorSpaceMode.HSV;

    public double min0 = 18.0;
    public double min1 = 110.0;
    public double min2 = 100.0;
    public double max0 = 35.0;
    public double max1 = 255.0;
    public double max2 = 255.0;

    public int blurSize = 3;
    public int erodeSize = 1;
    public int dilateSize = 2;

    public double minAreaPercent = 0.05;
    public double maxAreaPercent = 40.0;
    public double minFullness = 0.5;
    public double minAspectRatio = 0.6;
    public double maxAspectRatio = 1.6;
    public double minCircularity = 0.0;
    public double speckleRejectPercent = 0.0;

    public TargetSort sort = TargetSort.LARGEST;
    public double targetCenterHeightInches = 1.4;
    public int maxTargets = 8;

    public ColorProfile() {
    }

    public ColorProfile copy() {
        ColorProfile copy = new ColorProfile();
        copy.name = name;
        copy.enabled = enabled;
        copy.colorSpace = colorSpace;
        copy.min0 = min0;
        copy.min1 = min1;
        copy.min2 = min2;
        copy.max0 = max0;
        copy.max1 = max1;
        copy.max2 = max2;
        copy.blurSize = blurSize;
        copy.erodeSize = erodeSize;
        copy.dilateSize = dilateSize;
        copy.minAreaPercent = minAreaPercent;
        copy.maxAreaPercent = maxAreaPercent;
        copy.minFullness = minFullness;
        copy.minAspectRatio = minAspectRatio;
        copy.maxAspectRatio = maxAspectRatio;
        copy.minCircularity = minCircularity;
        copy.speckleRejectPercent = speckleRejectPercent;
        copy.sort = sort;
        copy.targetCenterHeightInches = targetCenterHeightInches;
        copy.maxTargets = maxTargets;
        return copy;
    }

    public static ColorProfile pollenYellow() {
        ColorProfile profile = new ColorProfile();
        profile.name = "yellow";
        profile.colorSpace = ColorSpaceMode.HSV;
        profile.setRange(18.0, 110.0, 100.0, 35.0, 255.0, 255.0);
        profile.minAspectRatio = 0.45;
        profile.maxAspectRatio = 2.3;
        profile.targetCenterHeightInches = 1.4;
        return profile;
    }

    public static ColorProfile nectarRed() {
        ColorProfile profile = new ColorProfile();
        profile.name = "red";
        profile.colorSpace = ColorSpaceMode.HSV;
        profile.setRange(170.0, 120.0, 80.0, 8.0, 255.0, 255.0);
        profile.targetCenterHeightInches = 1.8;
        return profile;
    }

    public static ColorProfile nectarBlue() {
        ColorProfile profile = new ColorProfile();
        profile.name = "blue";
        profile.colorSpace = ColorSpaceMode.HSV;
        profile.setRange(100.0, 120.0, 60.0, 125.0, 255.0, 255.0);
        profile.targetCenterHeightInches = 1.8;
        return profile;
    }

    public void setRange(double min0, double min1, double min2, double max0, double max1, double max2) {
        this.min0 = min0;
        this.min1 = min1;
        this.min2 = min2;
        this.max0 = max0;
        this.max1 = max1;
        this.max2 = max2;
    }

    public boolean hueWraps() {
        return colorSpace == ColorSpaceMode.HSV && min0 > max0;
    }

    public double channelMin(int channel) {
        if (channel <= 0) {
            return min0;
        }
        return channel == 1 ? min1 : min2;
    }

    public double channelMax(int channel) {
        if (channel <= 0) {
            return max0;
        }
        return channel == 1 ? max1 : max2;
    }

    public void setChannelMin(int channel, double value) {
        if (channel <= 0) {
            min0 = value;
        } else if (channel == 1) {
            min1 = value;
        } else {
            min2 = value;
        }
    }

    public void setChannelMax(int channel, double value) {
        if (channel <= 0) {
            max0 = value;
        } else if (channel == 1) {
            max1 = value;
        } else {
            max2 = value;
        }
    }

    public boolean matches(double channel0, double channel1, double channel2) {
        boolean firstMatches = hueWraps()
                ? channel0 >= min0 || channel0 <= max0
                : channel0 >= min0 && channel0 <= max0;
        return firstMatches
                && channel1 >= min1 && channel1 <= max1
                && channel2 >= min2 && channel2 <= max2;
    }

    public String rangeSummary() {
        ColorSpaceMode space = colorSpace == null ? ColorSpaceMode.HSV : colorSpace;
        return String.format(Locale.US, "%s  %s %.0f-%.0f%s  %s %.0f-%.0f  %s %.0f-%.0f",
                space.name(),
                space.channelName(0), min0, max0, hueWraps() ? " wrap" : "",
                space.channelName(1), min1, max1,
                space.channelName(2), min2, max2);
    }
}
