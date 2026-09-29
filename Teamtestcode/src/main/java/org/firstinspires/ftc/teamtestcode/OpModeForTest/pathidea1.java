package org.firstinspires.ftc.teamtestcode.OpModeForTest;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class pathidea1 {

    public Path MainChain;
    public static final Pose startPose = new Pose(61.75, 8.0, Math.toRadians(90.0));

    public pathidea1(Follower follower) {
        MainChain = path(
                curve(
                        new Pose(61.75, 8.0),
                        new Pose(18.048, 42.858),
                        new Pose(8.256, 102.476)
                )
        );

        MainChain.linear(Math.toRadians(90.0), Math.toRadians(90.0));
    }
}