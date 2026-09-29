package org.firstinspires.ftc.teamtestcode.pedropathing.tuning;

import android.content.Context;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TunerRegistrar;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.teamtestcode.pedropathing.Constants;

public class Tuning {

    private static final String MECANUM_TUNER_NAME = "Pedro Mecanum Tuner";
    private static final String PINPOINT_TUNER_NAME = "Pedro Pinpoint Tuner";
    private static final String FORESIGHT_TUNER_NAME = "Pedro Foresight Tuner";
    private static final String TESTS_NAME = "Pedro Tests";

    @OnCreate
    public static void registerTuners(Context context) {
        register(MECANUM_TUNER_NAME, mecanumTuner());
        register(PINPOINT_TUNER_NAME, pinpointTuner());
        register(FORESIGHT_TUNER_NAME, foresightTuner());
        register(TESTS_NAME, tests());
    }

    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }

    private static synchronized void register(String name, Procedure procedure) {
        for (TunerRegistrar.RegisteredProcedure registered : TunerRegistrar.getProcedures()) {
            if (registered.name.equals(name)) {
                return;
            }
        }
        TunerRegistrar.register(name, procedure);
    }
}
