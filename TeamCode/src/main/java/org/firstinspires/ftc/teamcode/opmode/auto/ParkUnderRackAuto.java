package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamcode.opmode.GoalAim;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;

@Autonomous(name = "Park - Under Rack", group = "Park")
public class ParkUnderRackAuto extends ParkAuto {

    @Override
    protected String routeName() {
        return "Park - Under Rack (straight through underneath)";
    }

    @Override
    protected Pose[] route(GoalScanner.Alliance alliance) {
        if (GoalAim.isBlue(alliance)) {
            return new Pose[] {
                    FieldPointsBlue.UNDER_RACK_CONTROL_POINT_1,
                    FieldPointsBlue.UNDER_RACK_CONTROL_POINT_2,
                    FieldPointsBlue.PARK_POSE
            };
        }
        return new Pose[] {
                FieldPoints.UNDER_RACK_CONTROL_POINT_2,
                FieldPoints.UNDER_RACK_CONTROL_POINT_1,
                FieldPoints.PARK_POSE
        };
    }

    @Override
    protected double[] segmentTimeouts(GoalScanner.Alliance alliance) {
        double leg = FieldPoints.PARK_SEGMENT_TIMEOUT_SECONDS;
        return new double[] {leg, FieldPoints.RACK_UNDER_TIMEOUT_SECONDS, leg};
    }

    @Override
    protected boolean routeConfirmed(GoalScanner.Alliance alliance) {
        return GoalAim.isBlue(alliance) ? FieldPointsBlue.PARK_ROUTE_CONFIRMED : FieldPoints.PARK_ROUTE_CONFIRMED;
    }

    @Override
    protected String pointsFile(GoalScanner.Alliance alliance) {
        return GoalAim.isBlue(alliance) ? "FieldPointsBlue.java" : "FieldPoints.java";
    }
}
