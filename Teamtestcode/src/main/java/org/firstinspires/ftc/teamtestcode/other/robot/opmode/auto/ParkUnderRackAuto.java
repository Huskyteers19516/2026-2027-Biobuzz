package org.firstinspires.ftc.teamtestcode.other.robot.opmode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamtestcode.other.robot.opmode.FieldPoints;
import org.firstinspires.ftc.teamtestcode.other.robot.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamtestcode.other.robot.opmode.GoalAim;
import org.firstinspires.ftc.teamtestcode.other.robot.opmode.GoalScanner;

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
                    FieldPointsBlue.RACK_UNDER_ENTRY,
                    FieldPointsBlue.RACK_UNDER_EXIT,
                    FieldPointsBlue.PARK_POSE
            };
        }
        return new Pose[] {
                FieldPoints.RACK_UNDER_ENTRY,
                FieldPoints.RACK_UNDER_EXIT,
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
