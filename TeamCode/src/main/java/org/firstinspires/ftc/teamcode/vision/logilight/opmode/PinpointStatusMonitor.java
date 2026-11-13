package org.firstinspires.ftc.teamcode.vision.logilight.opmode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeekEngine;
import org.firstinspires.ftc.teamcode.vision.logilight.PollenSeeker;

public final class PinpointStatusMonitor {

    private final GoBildaPinpointDriver driver;
    private final String fault;
    private String lastName = "not read yet";

    private PinpointStatusMonitor(GoBildaPinpointDriver driver, String fault) {
        this.driver = driver;
        this.fault = fault == null ? "" : fault;
    }

    public static PinpointStatusMonitor create(HardwareMap hardwareMap, boolean pinpointCreated) {
        if (!pinpointCreated || hardwareMap == null) {
            return new PinpointStatusMonitor(null, "no Pinpoint");
        }
        try {
            GoBildaPinpointDriver found = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT_NAME);
            return new PinpointStatusMonitor(found, found == null ? "Pinpoint driver not found" : "");
        } catch (Throwable t) {
            return new PinpointStatusMonitor(null, LogiOpModeTools.describe(t));
        }
    }

    public boolean isAvailable() {
        return driver != null;
    }

    public void report(PollenSeeker seeker) {
        if (driver == null || seeker == null) {
            return;
        }
        GoBildaPinpointDriver.DeviceStatus status;
        try {
            status = driver.getDeviceStatus();
        } catch (Throwable t) {
            status = null;
        }
        lastName = status == null ? "status read failed" : status.name();
        seeker.engine().reportLocalizerStatus(classify(status), lastName);
    }

    public static PollenSeekEngine.LocalizerHealth classify(GoBildaPinpointDriver.DeviceStatus status) {
        if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
            return PollenSeekEngine.LocalizerHealth.READY;
        }
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY) {
            return PollenSeekEngine.LocalizerHealth.FAULT;
        }
        return PollenSeekEngine.LocalizerHealth.TRANSIENT;
    }

    public String line() {
        if (driver == null) {
            return "!! Pinpoint status NOT MONITORED - " + fault;
        }
        return lastName;
    }
}
