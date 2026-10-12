package org.firstinspires.ftc.teamtestcode.OpmodeForNewMembers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Mecanum auto that drives forward, backward, left and right by encoder distance.
 *
 * Every direction is just a sign pattern for the four wheels (LF, RF, LB, RB), so one
 * move() method handles all of them. The route is the ROUTE table below: edit that
 * table to change the path, no other code needs to change.
 *
 * BIOBUZZ: LEAVE only scores if the robot is NOT touching the perimeter wall at the end
 * of AUTO, so the route finishes with a short move away from the wall.
 */
@Autonomous(name = "Four Way Move Auto", group = "Testing")
public class FourWayMoveAuto extends LinearOpMode {

    // ---------- Robot geometry (goBILDA 312 rpm + 96 mm mecanum) ----------
    private static final double TICKS_PER_REV = 537.7;
    private static final double GEAR_RATIO = 1.0;
    private static final double WHEEL_DIAMETER_IN = 3.78;
    private static final double TICKS_PER_INCH =
            TICKS_PER_REV * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_IN);

    // Mecanum wheels slip when strafing, so sideways moves need extra ticks.
    // Tune: strafe 24 in, measure the real distance, set this to 24 / measured.
    private static final double STRAFE_CORRECTION = 1.15;

    // ---------- Motion tuning ----------
    private static final double MAX_POWER = 0.4;
    private static final double MIN_POWER = 0.12;      // enough to keep the robot moving
    private static final double SLOWDOWN_INCHES = 6.0; // start slowing this far from the target
    private static final double ARRIVE_TOLERANCE_IN = 0.3;
    private static final double PAUSE_BETWEEN_MOVES_S = 0.5;

    // Safety timeout per move = base + inches * rate / power
    private static final double TIMEOUT_BASE_S = 2.0;
    private static final double TIMEOUT_RATE = 0.1;

    // ---------- Optional intake (off by default, same as the old test auto) ----------
    private static final boolean RUN_INTAKE_ON_FORWARD = false;
    private static final double INTAKE_POWER = 0.0;

    // Wheel order everywhere in this file: LF, RF, LB, RB
    private static final String[] MOTOR_NAMES = {"left_front", "right_front", "left_back", "right_back"};
    private static final int LF = 0, RF = 1, LB = 2, RB = 3;

    private enum Direction {
        FORWARD(+1, +1, +1, +1, false),
        BACKWARD(-1, -1, -1, -1, false),
        LEFT(-1, +1, +1, -1, true),
        RIGHT(+1, -1, -1, +1, true);

        final int[] wheelSign;
        final boolean sideways;

        Direction(int lf, int rf, int lb, int rb, boolean sideways) {
            this.wheelSign = new int[]{lf, rf, lb, rb};
            this.sideways = sideways;
        }
    }

    private static final class Move {
        final Direction direction;
        final double inches;

        Move(Direction direction, double inches) {
            this.direction = direction;
            this.inches = inches;
        }

        @Override
        public String toString() {
            return String.format("%s %.1f in", direction, inches);
        }
    }

    // ---------- The route. Edit here. ----------
    private static final Move[] ROUTE = {
            new Move(Direction.FORWARD, 24.0),
            new Move(Direction.BACKWARD, 24.0),
            new Move(Direction.RIGHT, 24.0),
            new Move(Direction.LEFT, 24.0),
            new Move(Direction.FORWARD, 6.0),   // end off the wall so LEAVE counts
    };

    private final DcMotor[] wheels = new DcMotor[4];
    private DcMotor intake; // null if the robot has no "intake" motor configured

    private final ElapsedTime clock = new ElapsedTime();
    private final StringBuilder log = new StringBuilder();

    @Override
    public void runOpMode() {
        setUpHardware();

        while (opModeInInit()) {
            telemetry.addLine("Four Way Move Auto");
            for (int i = 0; i < ROUTE.length; i++) {
                telemetry.addData("Move " + (i + 1), ROUTE[i]);
            }
            telemetry.addData("Max power", "%.2f", MAX_POWER);
            telemetry.addData("Intake", intake == null ? "not found (skipped)" : "found");
            telemetry.addLine("Check: encoder cables plugged in, clear space on all 4 sides.");
            telemetry.update();
        }

        try {
            for (int i = 0; i < ROUTE.length && opModeIsActive(); i++) {
                Move m = ROUTE[i];
                boolean intakeOn = RUN_INTAKE_ON_FORWARD && m.direction == Direction.FORWARD;

                setIntake(intakeOn ? INTAKE_POWER : 0.0);
                String result = move(m, i + 1);
                setIntake(0.0);

                log.append(i + 1).append(": ").append(result).append('\n');

                if (i < ROUTE.length - 1) {
                    pause(PAUSE_BETWEEN_MOVES_S);
                }
            }
        } finally {
            stopEverything();
        }

        telemetry.addLine("Route finished.");
        telemetry.addLine(log.toString());
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }

    private void setUpHardware() {
        for (int i = 0; i < 4; i++) {
            wheels[i] = hardwareMap.get(DcMotor.class, MOTOR_NAMES[i]);
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Right side motors are mounted mirrored, so flip them.
        wheels[LF].setDirection(DcMotorSimple.Direction.FORWARD);
        wheels[LB].setDirection(DcMotorSimple.Direction.FORWARD);
        wheels[RF].setDirection(DcMotorSimple.Direction.REVERSE);
        wheels[RB].setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor w : wheels) {
            w.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            w.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        intake = hardwareMap.tryGet(DcMotor.class, "intake");
        if (intake != null) {
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(0.0);
        }
    }

    /**
     * Drives one move with our own speed loop (RUN_USING_ENCODER), slowing down near the
     * end instead of relying on RUN_TO_POSITION. Returns a one-line result for the log.
     */
    private String move(Move m, int number) {
        double targetTicks = Math.abs(m.inches) * TICKS_PER_INCH
                * (m.direction.sideways ? STRAFE_CORRECTION : 1.0);
        double toleranceTicks = ARRIVE_TOLERANCE_IN * TICKS_PER_INCH;
        double timeout = TIMEOUT_BASE_S + Math.abs(m.inches) * TIMEOUT_RATE / MAX_POWER;

        int[] start = new int[4];
        for (int i = 0; i < 4; i++) {
            start[i] = wheels[i].getCurrentPosition();
        }

        double travelled = 0.0;
        clock.reset();

        while (opModeIsActive() && clock.seconds() < timeout) {
            travelled = averageTravel(start);
            double remaining = targetTicks - travelled;
            if (remaining <= toleranceTicks) {
                break;
            }

            double remainingInches = remaining / TICKS_PER_INCH;
            double power = Range.clip(MAX_POWER * remainingInches / SLOWDOWN_INCHES,
                    MIN_POWER, MAX_POWER);
            applyPower(m.direction, power);

            telemetry.addData("Move", "%d / %d: %s", number, ROUTE.length, m);
            telemetry.addData("Progress", "%.1f / %.1f in",
                    travelled / TICKS_PER_INCH, targetTicks / TICKS_PER_INCH);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Time", "%.1f / %.1f s", clock.seconds(), timeout);
            telemetry.addData("Ticks", "LF %d  RF %d  LB %d  RB %d",
                    wheels[LF].getCurrentPosition() - start[LF],
                    wheels[RF].getCurrentPosition() - start[RF],
                    wheels[LB].getCurrentPosition() - start[LB],
                    wheels[RB].getCurrentPosition() - start[RB]);
            telemetry.update();
        }

        applyPower(m.direction, 0.0);

        if (!opModeIsActive()) {
            return m + " -> stopped";
        }
        double doneInches = travelled / TICKS_PER_INCH;
        if (clock.seconds() >= timeout) {
            return String.format("%s -> TIMEOUT after %.1f in", m, doneInches);
        }
        return String.format("%s -> ok in %.1f s", m, clock.seconds());
    }

    // Average distance the wheels have turned in the commanded direction.
    private double averageTravel(int[] start) {
        double sum = 0.0;
        for (int i = 0; i < 4; i++) {
            sum += Math.abs(wheels[i].getCurrentPosition() - start[i]);
        }
        return sum / 4.0;
    }

    private void applyPower(Direction d, double power) {
        for (int i = 0; i < 4; i++) {
            wheels[i].setPower(d.wheelSign[i] * power);
        }
    }

    private void pause(double seconds) {
        clock.reset();
        while (opModeIsActive() && clock.seconds() < seconds) {
            telemetry.addData("Move", "pause %.1f / %.1f s", clock.seconds(), seconds);
            telemetry.update();
        }
    }

    private void setIntake(double power) {
        if (intake != null) {
            intake.setPower(power);
        }
    }

    private void stopEverything() {
        for (DcMotor w : wheels) {
            w.setPower(0.0);
        }
        setIntake(0.0);
    }
}
