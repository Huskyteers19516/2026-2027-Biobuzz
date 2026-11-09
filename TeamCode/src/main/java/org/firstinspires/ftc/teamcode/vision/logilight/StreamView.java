package org.firstinspires.ftc.teamcode.vision.logilight;

public enum StreamView {
    ANNOTATED,
    THRESHOLD,
    RAW;

    public StreamView next() {
        StreamView[] views = values();
        return views[(ordinal() + 1) % views.length];
    }
}
