package org.firstinspires.ftc.teamcode.vision.logilight;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class PollenSeekSettings {

    public static PollenSeekConfig SEEK = new PollenSeekConfig();

    public static final boolean REQUIRE_PUSH_TEST = true;

    private PollenSeekSettings() {
    }

    public static PollenSeekConfig current() {
        PollenSeekConfig live = SEEK;
        return live == null ? new PollenSeekConfig() : live;
    }
}
