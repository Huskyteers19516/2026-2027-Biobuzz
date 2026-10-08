public static class Paths {
    // Start pose: x 61.75 in, y 8.0 in, heading 90.0 deg (first path in the sequence).
    public static final Pose startPose = new Pose(61.75, 8.0, Math.toRadians(90.0));

    public PathChain MainChain;

    public Paths(Follower follower) {
        MainChain = follower.pathBuilder()
                // Path 1
                .addPath(
                        new BezierCurve(
                                new Pose(61.75, 8.0),
                                new Pose(39.533, 1.222),
                                new Pose(8.052, 102.201)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
                .build();
    }
}
