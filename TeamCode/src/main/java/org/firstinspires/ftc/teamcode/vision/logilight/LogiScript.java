package org.firstinspires.ftc.teamcode.vision.logilight;

import org.opencv.core.Mat;

public interface LogiScript {

    String name();

    void init(int width, int height);

    LogiScriptResult run(Mat rgba, long captureNanos, double[] inputs);
}
