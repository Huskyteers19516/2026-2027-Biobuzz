package org.firstinspires.ftc.teamcode.opmode.autolib;

import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.ANGLE_1;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.ANGLE_2;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.ANGLE_3;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.ANGLE_4;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_1_END;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_1_START;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_2_END;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_2_START;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_3_END;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_3_START;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_4_END;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.FLOWER_4_START;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.PARK;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.RACK_AROUND_1;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.RACK_AROUND_2;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.SHOOT_1;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.SHOOT_2;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.START;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.UNDER_RACK_ENTRY;
import static org.firstinspires.ftc.teamcode.opmode.autolib.Spot.UNDER_RACK_EXIT;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.GoalScanner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public final class AutoLibrary {




    private static AutoLibrary instance;

    private final Map<Integer, Leg> byNumber = new TreeMap<>();
    private final List<Leg> ordered = new ArrayList<>();
    private String section = "";

    //This is the path for Red team 4 (Brandon): 1, 31, 53, 34, 48, 73

    private AutoLibrary() {
        startPaths();
        shootToShootPaths();
        shootToFlowerPaths();
        flowerSweepPaths();
        flowerToShootPaths();
        flowerToFlowerPaths();
        parkPaths();
        ringPaths();
        rackPaths();
    }

    private void startPaths() {
        section("Start");
        path(1, START, SHOOT_1);
        path(2, START, SHOOT_2);
        path(3, START, FLOWER_1_START);
        path(4, START, FLOWER_2_START);
        path(5, START, FLOWER_3_START);
        path(6, START, FLOWER_4_START);
        path(7, START, ANGLE_1);
        path(8, START, ANGLE_4);
    }

    private void shootToShootPaths() {
        section("Shoot to shoot (under the rack)");
        path(11, SHOOT_1, SHOOT_2);
        path(12, SHOOT_2, SHOOT_1);
    }

    private void shootToFlowerPaths() {
        section("Shoot to flower");
        path(21, SHOOT_1, FLOWER_1_START);
        path(22, SHOOT_1, FLOWER_2_START);
        path(23, SHOOT_1, FLOWER_3_START);
        path(24, SHOOT_1, FLOWER_4_START);
        path(25, SHOOT_2, FLOWER_1_START);
        path(26, SHOOT_2, FLOWER_2_START);
        path(27, SHOOT_2, FLOWER_3_START);
        path(28, SHOOT_2, FLOWER_4_START);
    }

    private void flowerSweepPaths() {
        section("Flower sweep");
        path(31, FLOWER_1_START, FLOWER_1_END);
        path(32, FLOWER_2_START, FLOWER_2_END);
        path(33, FLOWER_3_START, FLOWER_3_END);
        path(34, FLOWER_4_START, FLOWER_4_END);
    }

    private void flowerToShootPaths() {
        section("Flower to shoot");
        path(41, FLOWER_1_END, SHOOT_1);
        path(42, FLOWER_1_END, SHOOT_2);
        path(43, FLOWER_2_END, SHOOT_1);
        path(44, FLOWER_2_END, SHOOT_2);
        path(45, FLOWER_3_END, SHOOT_1);
        path(46, FLOWER_3_END, SHOOT_2);
        path(47, FLOWER_4_END, SHOOT_1);
        path(48, FLOWER_4_END, SHOOT_2);
    }

    private void flowerToFlowerPaths() {
        section("Flower to flower");
        path(51, FLOWER_1_END, FLOWER_2_START);
        path(52, FLOWER_1_END, FLOWER_3_START);
        path(53, FLOWER_1_END, FLOWER_4_START);
        path(54, FLOWER_2_END, FLOWER_1_START);
        path(55, FLOWER_2_END, FLOWER_3_START);
        path(56, FLOWER_2_END, FLOWER_4_START);
        path(57, FLOWER_3_END, FLOWER_1_START);
        path(58, FLOWER_3_END, FLOWER_2_START);
        path(59, FLOWER_3_END, FLOWER_4_START);
        path(60, FLOWER_4_END, FLOWER_1_START);
        path(61, FLOWER_4_END, FLOWER_2_START);
        path(62, FLOWER_4_END, FLOWER_3_START);
    }

    private void parkPaths() {
        section("Park");
        path(71, START, PARK).via(RACK_AROUND_1, RACK_AROUND_2);
        path(72, SHOOT_1, PARK).via(ANGLE_1);
        path(73, SHOOT_2, PARK);
        path(74, FLOWER_1_END, PARK);
        path(75, FLOWER_2_END, PARK);
        path(76, FLOWER_3_END, PARK);
        path(77, FLOWER_4_END, PARK);
        path(78, ANGLE_1, PARK);
        path(79, ANGLE_2, PARK);
    }

    private void ringPaths() {
        section("Ring: corner to corner");
        path(81, ANGLE_1, ANGLE_2);
        path(82, ANGLE_2, ANGLE_1);
        path(83, ANGLE_2, ANGLE_3);
        path(84, ANGLE_3, ANGLE_2);
        path(85, ANGLE_3, ANGLE_4);
        path(86, ANGLE_4, ANGLE_3);
        path(87, ANGLE_4, ANGLE_1);
        path(88, ANGLE_1, ANGLE_4);

        section("Ring: shoot and corner");
        path(91, SHOOT_1, ANGLE_1);
        path(92, SHOOT_1, ANGLE_4);
        path(93, ANGLE_1, SHOOT_1);
        path(94, ANGLE_4, SHOOT_1);
        path(95, SHOOT_2, ANGLE_2);
        path(96, SHOOT_2, ANGLE_3);
        path(97, ANGLE_2, SHOOT_2);
        path(98, ANGLE_3, SHOOT_2);

        section("Ring: flower end to corner");
        path(101, FLOWER_1_END, ANGLE_1);
        path(102, FLOWER_1_END, ANGLE_2);
        path(103, FLOWER_2_END, ANGLE_2);
        path(104, FLOWER_2_END, ANGLE_3);
        path(105, FLOWER_3_END, ANGLE_3);
        path(106, FLOWER_3_END, ANGLE_4);
        path(107, FLOWER_4_END, ANGLE_4);
        path(108, FLOWER_4_END, ANGLE_1);

        section("Ring: corner to flower start");
        path(111, ANGLE_1, FLOWER_1_START);
        path(112, ANGLE_2, FLOWER_1_START);
        path(113, ANGLE_2, FLOWER_2_START);
        path(114, ANGLE_3, FLOWER_2_START);
        path(115, ANGLE_3, FLOWER_3_START);
        path(116, ANGLE_4, FLOWER_3_START);
        path(117, ANGLE_4, FLOWER_4_START);
        path(118, ANGLE_1, FLOWER_4_START);
    }

    private void rackPaths() {
        section("Rack: around and under");
        path(121, START, RACK_AROUND_1);
        path(122, RACK_AROUND_1, RACK_AROUND_2);
        path(123, RACK_AROUND_2, PARK);
        path(124, START, UNDER_RACK_ENTRY);
        path(125, UNDER_RACK_ENTRY, UNDER_RACK_EXIT).timeout(FieldPoints.RACK_UNDER_TIMEOUT_SECONDS);
        path(126, UNDER_RACK_EXIT, PARK);
    }

    public static synchronized AutoLibrary get() {
        if (instance == null) {
            instance = new AutoLibrary();
        }
        return instance;
    }

    public static Leg number(int number) {
        Leg leg = get().byNumber.get(number);
        if (leg == null) {
            throw new IllegalStateException("AutoLibrary has no path #" + number);
        }
        return leg;
    }

    public static boolean has(int number) {
        return get().byNumber.containsKey(number);
    }

    public List<Leg> all() {
        return Collections.unmodifiableList(ordered);
    }

    private void section(String name) {
        section = name;
    }

    private Leg path(int number, Spot from, Spot to) {
        if (from == to) {
            throw new IllegalStateException("AutoLibrary path #" + number + " " + from + " -> " + to + " goes nowhere");
        }
        if (byNumber.containsKey(number)) {
            throw new IllegalStateException("AutoLibrary uses #" + number + " twice");
        }
        for (Leg other : ordered) {
            if (other.from == from && other.to == to) {
                throw new IllegalStateException("AutoLibrary #" + other.number + " and #" + number
                        + " are both " + from + " -> " + to);
            }
        }
        Leg leg = new Leg(number, section, from, to);
        byNumber.put(number, leg);
        ordered.add(leg);
        return leg;
    }

    public static final class Leg {
        public final int number;
        public final String section;
        public final Spot from;
        public final Spot to;
        private final List<Spot> via = new ArrayList<>();
        private double timeoutSeconds = -1.0;

        private Leg(int number, String section, Spot from, Spot to) {
            this.number = number;
            this.section = section;
            this.from = from;
            this.to = to;
        }

        private Leg via(Spot... spots) {
            via.addAll(Arrays.asList(spots));
            return this;
        }

        private Leg timeout(double seconds) {
            timeoutSeconds = seconds;
            return this;
        }

        public List<Spot> spots() {
            List<Spot> spots = new ArrayList<>();
            spots.add(from);
            spots.addAll(via);
            spots.add(to);
            return spots;
        }

        public Pose[] poses(GoalScanner.Alliance alliance) {
            List<Spot> spots = spots();
            Pose[] poses = new Pose[spots.size()];
            for (int i = 0; i < poses.length; i++) {
                poses[i] = spots.get(i).pose(alliance);
            }
            return poses;
        }

        public Pose start(GoalScanner.Alliance alliance) {
            return from.pose(alliance);
        }

        public Pose end(GoalScanner.Alliance alliance) {
            return to.pose(alliance);
        }

        public double length(GoalScanner.Alliance alliance) {
            Pose[] poses = poses(alliance);
            double total = 0.0;
            for (int i = 1; i < poses.length; i++) {
                total += poses[i - 1].distance(poses[i]);
            }
            return total;
        }

        public Path build(GoalScanner.Alliance alliance) {
            Pose[] poses = poses(alliance);
            Pose first = poses[0];
            Pose last = poses[poses.length - 1];
            if (poses.length == 2) {
                return Paths.line(first, last).linear(first, last);
            }
            Path[] pieces = new Path[poses.length - 1];
            for (int i = 0; i < pieces.length; i++) {
                pieces[i] = Paths.line(poses[i], poses[i + 1]);
            }
            return Paths.path(pieces).linear(first, last);
        }

        public boolean confirmed(GoalScanner.Alliance alliance) {
            for (Spot spot : spots()) {
                if (!spot.confirmed(alliance)) {
                    return false;
                }
            }
            return true;
        }

        public double timeoutSeconds() {
            if (timeoutSeconds > 0) {
                return timeoutSeconds;
            }
            return (via.size() + 1) * FieldPoints.DEFAULT_PATH_TIMEOUT_SECONDS;
        }

        public String describe() {
            StringBuilder text = new StringBuilder("#").append(number).append(' ').append(from.name());
            for (Spot spot : via) {
                text.append(" ~ ").append(spot.name());
            }
            return text.append(" -> ").append(to.name()).toString();
        }
    }
}
