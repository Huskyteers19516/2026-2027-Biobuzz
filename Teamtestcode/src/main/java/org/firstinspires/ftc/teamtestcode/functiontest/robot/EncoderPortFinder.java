package org.firstinspires.ftc.teamtestcode.functiontest.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamtestcode.other.robot.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Encoder Port Finder", group = "Test")
public class EncoderPortFinder extends OpMode {

    private static final int PORTS = 4;
    private static final int MOVED_TICKS = 20;

    private List<LynxModule> modules;
    private int[][] startTicks;
    private String outtakeInfo = "not found";

    @Override
    public void init() {
        modules = hardwareMap.getAll(LynxModule.class);
        startTicks = new int[modules.size()][PORTS];
        for (LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        try {
            DcMotor outtake = hardwareMap.get(DcMotor.class, Constants.OUTTAKE);
            outtake.setPower(0.0);
            outtakeInfo = outtake.getConnectionInfo();
        } catch (RuntimeException e) {
            outtakeInfo = "no motor named \"" + Constants.OUTTAKE + "\" in the configuration";
        }
        readAll(true);
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            readAll(true);
        }
        int[][] ticks = readAll(false);

        telemetry.addLine("Spin the LAUNCHER wheel by hand. The port whose ticks change is where its encoder is plugged in.");
        telemetry.addData("\"" + Constants.OUTTAKE + "\" is configured at", outtakeInfo);
        telemetry.addLine("Nothing moves in this OpMode. Press A to zero the counts.");
        telemetry.addLine();
        for (int m = 0; m < modules.size(); m++) {
            LynxModule module = modules.get(m);
            telemetry.addLine((module.isParent() ? "Control Hub" : "Expansion Hub") + "  (address " + module.getModuleAddress() + ")");
            for (int port = 0; port < PORTS; port++) {
                int moved = ticks[m][port] - startTicks[m][port];
                telemetry.addData(String.format("  encoder port %d", port), "moved %d ticks%s",
                        moved, Math.abs(moved) >= MOVED_TICKS ? "   <-- MOVING" : "");
            }
        }
        telemetry.addLine();
        telemetry.addLine("If no port moves while you spin it: the cable is unplugged or broken, or the motor has no working encoder.");
        telemetry.update();
    }

    @Override
    public void loop() {
        init_loop();
    }

    private int[][] readAll(boolean zero) {
        int[][] ticks = new int[modules.size()][PORTS];
        for (int m = 0; m < modules.size(); m++) {
            LynxModule module = modules.get(m);
            module.clearBulkCache();
            LynxModule.BulkData data = module.getBulkData();
            for (int port = 0; port < PORTS; port++) {
                ticks[m][port] = data.getMotorCurrentPosition(port);
                if (zero) {
                    startTicks[m][port] = ticks[m][port];
                }
            }
        }
        return ticks;
    }
}
