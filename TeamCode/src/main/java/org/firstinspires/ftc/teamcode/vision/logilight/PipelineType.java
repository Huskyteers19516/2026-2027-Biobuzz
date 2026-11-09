package org.firstinspires.ftc.teamcode.vision.logilight;

public enum PipelineType {
    APRILTAG,
    COLOR,
    APRILTAG_AND_COLOR,
    SCRIPT,
    DRIVER;

    public boolean usesAprilTag() {
        return this == APRILTAG || this == APRILTAG_AND_COLOR;
    }

    public boolean usesColor() {
        return this == COLOR || this == APRILTAG_AND_COLOR;
    }

    public boolean usesScript() {
        return this == SCRIPT;
    }
}
