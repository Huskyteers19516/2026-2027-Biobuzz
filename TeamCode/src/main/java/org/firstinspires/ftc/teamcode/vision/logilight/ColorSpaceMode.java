package org.firstinspires.ftc.teamcode.vision.logilight;

public enum ColorSpaceMode {
    HSV,
    YCRCB,
    RGB;

    public static final double HUE_MAX = 180.0;
    public static final double CHANNEL_MAX = 255.0;

    public String channelName(int channel) {
        int index = Math.max(0, Math.min(2, channel));
        if (this == HSV) {
            return index == 0 ? "H" : index == 1 ? "S" : "V";
        }
        if (this == YCRCB) {
            return index == 0 ? "Y" : index == 1 ? "Cr" : "Cb";
        }
        return index == 0 ? "R" : index == 1 ? "G" : "B";
    }

    public double channelMax(int channel) {
        return this == HSV && channel == 0 ? HUE_MAX : CHANNEL_MAX;
    }
}
