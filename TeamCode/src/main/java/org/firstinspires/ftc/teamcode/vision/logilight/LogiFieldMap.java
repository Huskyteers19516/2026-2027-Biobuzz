package org.firstinspires.ftc.teamcode.vision.logilight;

import org.firstinspires.ftc.teamcode.opmode.FieldPoints;
import org.firstinspires.ftc.teamcode.opmode.FieldPointsBlue;
import org.firstinspires.ftc.teamcode.vision.logilight.math.FieldTargetState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public final class LogiFieldMap {

    public static final String RED_SCORING = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING = "BLUE SCORING";
    public static final String DEFAULT_STATE_LABEL = "up";
    public static final double PLACEHOLDER_TARGET_HEIGHT_INCHES = 38.0;
    public static final String DEFAULTS_SOURCE = "code defaults (placeholders)";

    public static final class Target {

        public final String name;
        public final boolean surveyed;
        public final List<FieldTargetState> states;

        public Target(String name, boolean surveyed, List<FieldTargetState> states) {
            this.name = name == null ? "" : name.trim();
            this.surveyed = surveyed;
            List<FieldTargetState> copy = new ArrayList<FieldTargetState>();
            if (states != null) {
                for (FieldTargetState state : states) {
                    if (state != null) {
                        copy.add(state);
                    }
                }
            }
            this.states = Collections.unmodifiableList(copy);
        }

        public boolean hasStates() {
            return !states.isEmpty();
        }

        public FieldTargetState firstState() {
            return states.isEmpty() ? null : states.get(0);
        }

        public FieldTargetState state(String label) {
            if (label == null) {
                return null;
            }
            String wanted = label.trim();
            for (FieldTargetState state : states) {
                if (state.label.trim().equalsIgnoreCase(wanted)) {
                    return state;
                }
            }
            return null;
        }

        public boolean matches(String otherName) {
            return otherName != null && name.equalsIgnoreCase(otherName.trim());
        }

        public Target withSurveyedState(FieldTargetState newState) {
            if (newState == null) {
                return this;
            }
            List<FieldTargetState> next = new ArrayList<FieldTargetState>();
            if (surveyed) {
                boolean replaced = false;
                String wanted = newState.label.trim();
                for (FieldTargetState state : states) {
                    if (state.label.trim().equalsIgnoreCase(wanted)) {
                        if (!replaced) {
                            next.add(newState);
                            replaced = true;
                        }
                    } else {
                        next.add(state);
                    }
                }
                if (!replaced) {
                    next.add(newState);
                }
            } else {
                next.add(newState);
            }
            return new Target(name, true, next);
        }

        public Target withoutState(String label) {
            if (label == null) {
                return this;
            }
            String wanted = label.trim();
            List<FieldTargetState> next = new ArrayList<FieldTargetState>();
            for (FieldTargetState state : states) {
                if (!state.label.trim().equalsIgnoreCase(wanted)) {
                    next.add(state);
                }
            }
            return new Target(name, surveyed, next);
        }

        public List<String> stateLabels() {
            List<String> labels = new ArrayList<String>();
            for (FieldTargetState state : states) {
                labels.add(state.label);
            }
            return labels;
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "%s  %s  states %d", name, surveyed ? "surveyed" : "UNSURVEYED", states.size());
        }
    }

    private final List<Target> targets;
    private final String source;

    public LogiFieldMap(List<Target> targets, String source) {
        List<Target> copy = new ArrayList<Target>();
        if (targets != null) {
            for (Target target : targets) {
                if (target == null || target.name.isEmpty()) {
                    continue;
                }
                int existing = indexOf(copy, target.name);
                if (existing >= 0) {
                    copy.set(existing, target);
                } else {
                    copy.add(target);
                }
            }
        }
        this.targets = Collections.unmodifiableList(copy);
        this.source = source == null ? "" : source;
    }

    public List<Target> targets() {
        return targets;
    }

    public Target find(String name) {
        if (name == null) {
            return null;
        }
        int index = indexOf(targets, name);
        return index >= 0 ? targets.get(index) : null;
    }

    public String source() {
        return source;
    }

    public int size() {
        return targets.size();
    }

    public int surveyedCount() {
        int count = 0;
        for (Target target : targets) {
            if (target.surveyed) {
                count++;
            }
        }
        return count;
    }

    public LogiFieldMap withTarget(Target target) {
        if (target == null || target.name.isEmpty()) {
            return this;
        }
        List<Target> next = new ArrayList<Target>(targets);
        int index = indexOf(next, target.name);
        if (index >= 0) {
            next.set(index, target);
        } else {
            next.add(target);
        }
        return new LogiFieldMap(next, source);
    }

    public LogiFieldMap withoutTarget(String name) {
        if (name == null) {
            return this;
        }
        List<Target> next = new ArrayList<Target>(targets);
        int index = indexOf(next, name);
        if (index < 0) {
            return this;
        }
        next.remove(index);
        return new LogiFieldMap(next, source);
    }

    public LogiFieldMap withDefaultTarget(String name) {
        Target placeholder = bioBuzzDefaults().find(name);
        return placeholder == null ? withoutTarget(name) : withTarget(placeholder);
    }

    public LogiFieldMap withSurveyedState(String targetName, FieldTargetState state) {
        if (targetName == null || targetName.trim().isEmpty() || state == null) {
            return this;
        }
        Target existing = find(targetName);
        Target base = existing == null ? new Target(targetName, false, null) : existing;
        return withTarget(base.withSurveyedState(state));
    }

    public LogiFieldMap withSource(String newSource) {
        return new LogiFieldMap(targets, newSource);
    }

    public String summary() {
        return String.format(Locale.US, "%d targets  %d surveyed  (%s)", targets.size(), surveyedCount(), source);
    }

    @Override
    public String toString() {
        return summary();
    }

    public static Target placeholderTarget(String name, double xInches, double yInches) {
        List<FieldTargetState> states = new ArrayList<FieldTargetState>();
        states.add(FieldTargetState.fromDegrees(DEFAULT_STATE_LABEL, xInches, yInches,
                PLACEHOLDER_TARGET_HEIGHT_INCHES, 0.0, 0.0, 0.0));
        return new Target(name, false, states);
    }

    public static LogiFieldMap bioBuzzDefaults() {
        List<Target> list = new ArrayList<Target>();
        list.add(placeholderTarget(RED_SCORING, FieldPoints.GOAL_SCORING.x(), FieldPoints.GOAL_SCORING.y()));
        list.add(placeholderTarget(RED_AUDIENCE, FieldPoints.GOAL_AUDIENCE.x(), FieldPoints.GOAL_AUDIENCE.y()));
        list.add(placeholderTarget(BLUE_AUDIENCE, FieldPointsBlue.GOAL_AUDIENCE.x(), FieldPointsBlue.GOAL_AUDIENCE.y()));
        list.add(placeholderTarget(BLUE_SCORING, FieldPointsBlue.GOAL_SCORING.x(), FieldPointsBlue.GOAL_SCORING.y()));
        return new LogiFieldMap(list, DEFAULTS_SOURCE);
    }

    private static int indexOf(List<Target> list, String name) {
        String wanted = name.trim();
        for (int i = 0; i < list.size(); i++) {
            if (list.get(i).name.equalsIgnoreCase(wanted)) {
                return i;
            }
        }
        return -1;
    }
}
