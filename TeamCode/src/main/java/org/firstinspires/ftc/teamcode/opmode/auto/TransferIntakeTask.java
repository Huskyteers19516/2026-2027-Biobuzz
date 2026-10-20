package org.firstinspires.ftc.teamcode.opmode.auto;

public final class TransferIntakeTask {


    public interface PowerOutput {
        void setPower(double power);
    }

    public enum State {
        IDLE,
        INTAKING,
        TRANSFERRING,
        FINISHED,
        CANCELLED
    }

    private final PowerOutput intake;
    private final PowerOutput transfer;

    private State state = State.IDLE;

    private double transferPower;
    private double intakeSeconds;
    private double transferSeconds;
    private long phaseStartNanos;

    public TransferIntakeTask(
            PowerOutput intake,
            PowerOutput transfer) {

        if (intake == null || transfer == null) {
            throw new IllegalArgumentException(
                    "Intake and transfer outputs cannot be null");
        }

        this.intake = intake;
        this.transfer = transfer;
    }

    public boolean start(
            double intakePower,
            double transferPower,
            double intakeSeconds,
            double transferSeconds) {

        if (isBusy()) {
            return false;
        }

        checkPower(intakePower);
        checkPower(transferPower);
        checkDuration(intakeSeconds);
        checkDuration(transferSeconds);

        this.transferPower = transferPower;
        this.intakeSeconds = intakeSeconds;
        this.transferSeconds = transferSeconds;

        stopOutputs();

        if (intakeSeconds > 0.0) {
            intake.setPower(intakePower);
            phaseStartNanos = System.nanoTime();
            state = State.INTAKING;
        } else {
            beginTransfer();
        }

        return true;
    }

    // 在 TeleOp 或自动程序的每次循环中调用。
    public void update() {
        switch (state) {
            case INTAKING:
                if (elapsedSeconds() >= intakeSeconds) {
                    intake.setPower(0.0);
                    beginTransfer();
                }
                break;

            case TRANSFERRING:
                if (elapsedSeconds() >= transferSeconds) {
                    finish();
                }
                break;

            default:
                break;
        }
    }

    private void beginTransfer() {
        if (transferSeconds <= 0.0) {
            finish();
            return;
        }

        transfer.setPower(transferPower);
        phaseStartNanos = System.nanoTime();
        state = State.TRANSFERRING;
    }

    private double elapsedSeconds() {
        return (System.nanoTime() - phaseStartNanos)
                / 1_000_000_000.0;
    }

    private void finish() {
        state = State.FINISHED;
        stopOutputs();
    }

    public void cancel() {
        if (isBusy()) {
            state = State.CANCELLED;
        }

        stopOutputs();
    }

    private void stopOutputs() {
        try {
            intake.setPower(0.0);
        } finally {
            transfer.setPower(0.0);
        }
    }

    public boolean isBusy() {
        return state == State.INTAKING
                || state == State.TRANSFERRING;
    }

    public boolean isFinished() {
        return state == State.FINISHED;
    }

    public State getState() {
        return state;
    }

    private static void checkPower(double value) {
        if (Double.isNaN(value)
                || Double.isInfinite(value)
                || value < -1.0
                || value > 1.0) {

            throw new IllegalArgumentException(
                    "Power must be between -1.0 and 1.0");
        }
    }

    private static void checkDuration(double value) {
        if (Double.isNaN(value)
                || Double.isInfinite(value)
                || value < 0.0) {

            throw new IllegalArgumentException(
                    "Duration must be finite and nonnegative");
        }
    }
}