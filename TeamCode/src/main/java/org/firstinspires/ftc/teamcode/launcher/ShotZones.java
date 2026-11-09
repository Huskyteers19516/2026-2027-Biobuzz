package org.firstinspires.ftc.teamcode.launcher;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

public final class ShotZones {

    public static final class Zone {
        public final String name;
        public final double nearInches;
        public final double farInches;
        public final double velocity;

        public Zone(String name, double nearInches, double farInches, double velocity) {
            this.name = name;
            this.nearInches = nearInches;
            this.farInches = farInches;
            this.velocity = velocity;
        }

        public boolean contains(double inches) {
            return inches >= nearInches && inches <= farInches;
        }

        public String describe() {
            return String.format(Locale.US, "%s %.1f-%.1f in -> %.0f", name, nearInches, farInches, velocity);
        }
    }

    public static final Zone[] ZONES = {
            new Zone("3FT", 38.0, 46.5, 1500.0),
            new Zone("WALL", 48.0, 64.0, 1450.0),
    };

    public static final class Result {
        public final double distance;
        public final double velocity;
        public final String zone;
        public final boolean outside;

        Result(double distance, double velocity, String zone, boolean outside) {
            this.distance = distance;
            this.velocity = velocity;
            this.zone = zone;
            this.outside = outside;
        }

        public String describe() {
            return String.format(Locale.US, "%s  %.1f in -> %.0f%s",
                    zone, distance, velocity, outside ? "  (no data here, add a zone)" : "");
        }
    }

    private static final List<Zone> SORTED;
    private static final String PROBLEM;

    static {
        List<Zone> sorted = new ArrayList<>();
        String problem = null;
        for (int i = 0; i < ZONES.length && problem == null; i++) {
            problem = check(i, ZONES[i]);
            if (problem == null) {
                sorted.add(ZONES[i]);
            }
        }
        if (problem == null && sorted.isEmpty()) {
            problem = "ZONES is empty - add at least one zone";
        }
        if (problem == null) {
            Collections.sort(sorted, Comparator.comparingDouble(z -> z.nearInches));
            for (int i = 1; i < sorted.size() && problem == null; i++) {
                Zone a = sorted.get(i - 1);
                Zone b = sorted.get(i);
                if (b.nearInches < a.farInches) {
                    problem = "zones " + a.name + " and " + b.name + " overlap";
                }
            }
        }
        PROBLEM = problem;
        SORTED = problem == null ? Collections.unmodifiableList(sorted) : Collections.<Zone>emptyList();
    }

    private ShotZones() {
    }

    public static String problem() {
        return PROBLEM;
    }

    public static List<Zone> zones() {
        return SORTED;
    }

    public static double velocityForDistance(double inches) {
        return lookup(inches).velocity;
    }

    public static Result lookup(double inches) {
        if (PROBLEM != null) {
            return new Result(inches, Launcher.DEFAULT_TARGET_VELOCITY, "BROKEN ZONES", true);
        }
        if (Double.isNaN(inches) || Double.isInfinite(inches)) {
            return new Result(inches, Launcher.DEFAULT_TARGET_VELOCITY, "NO DISTANCE", true);
        }
        Zone first = SORTED.get(0);
        if (inches < first.nearInches) {
            return new Result(inches, first.velocity, "closer than " + first.name, true);
        }
        for (int i = 0; i < SORTED.size(); i++) {
            Zone zone = SORTED.get(i);
            if (zone.contains(inches)) {
                return new Result(inches, zone.velocity, zone.name, false);
            }
            if (i + 1 < SORTED.size()) {
                Zone next = SORTED.get(i + 1);
                if (inches > zone.farInches && inches < next.nearInches) {
                    double t = (inches - zone.farInches) / (next.nearInches - zone.farInches);
                    double velocity = zone.velocity + t * (next.velocity - zone.velocity);
                    return new Result(inches, velocity, zone.name + ">" + next.name, false);
                }
            }
        }
        Zone last = SORTED.get(SORTED.size() - 1);
        return new Result(inches, last.velocity, "past " + last.name, true);
    }

    public static String summary() {
        if (PROBLEM != null) {
            return "BROKEN: " + PROBLEM;
        }
        StringBuilder text = new StringBuilder();
        for (Zone zone : SORTED) {
            if (text.length() > 0) {
                text.append("  |  ");
            }
            text.append(String.format(Locale.US, "%s %.0f-%.0f in %.0f",
                    zone.name, zone.nearInches, zone.farInches, zone.velocity));
        }
        return text.toString();
    }

    private static String check(int index, Zone zone) {
        String label = "ZONES row " + (index + 1);
        if (zone == null) {
            return label + " is null";
        }
        if (zone.name == null || zone.name.trim().isEmpty()) {
            return label + " needs a name";
        }
        if (!(zone.nearInches >= 0.0) || Double.isInfinite(zone.farInches) || !(zone.farInches >= zone.nearInches)) {
            return label + " (" + zone.name + ") needs 0 <= near <= far inches";
        }
        if (!(zone.velocity >= Launcher.MIN_TARGET_VELOCITY && zone.velocity <= Launcher.MAX_TARGET_VELOCITY)) {
            return String.format(Locale.US, "%s (%s) velocity must be %.0f-%.0f",
                    label, zone.name, Launcher.MIN_TARGET_VELOCITY, Launcher.MAX_TARGET_VELOCITY);
        }
        return null;
    }
}
