package org.firstinspires.ftc.teamtestcode.other.robot.opmode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamtestcode.other.robot.opmode.FieldPoints;
import org.firstinspires.ftc.teamtestcode.other.robot.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamtestcode.other.robot.opmode.GoalAim;
import org.firstinspires.ftc.teamtestcode.other.robot.opmode.GoalScanner;

@Autonomous(name = "Park - Around Rack", group = "Park")
public class ParkAroundRackAuto extends ParkAuto {

    @Override
    protected String routeName() {
        return "Park - Around Rack (outside the rack)";
    }

    @Override
    protected Pose[] route(GoalScanner.Alliance alliance) {
        if (GoalAim.isBlue(alliance)) {
            return new Pose[] {
                    FieldPointsBlue.RACK_AROUND_1,
                    FieldPointsBlue.RACK_AROUND_2,
                    FieldPointsBlue.PARK_POSE
            };
        }
        return new Pose[] {
                FieldPoints.RACK_AROUND_1,
                FieldPoints.RACK_AROUND_2,
                FieldPoints.PARK_POSE
        };
    }

    @Override
    protected double[] segmentTimeouts(GoalScanner.Alliance alliance) {
        double leg = FieldPoints.PARK_SEGMENT_TIMEOUT_SECONDS;
        return new double[] {leg, leg, leg};
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
