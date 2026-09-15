# Teamtestcode — Test OpModes

This module is a separate test app for trying out mechanisms in the pit or on the
bench. Nothing in here is competition code.

## Installing the test app

1. Open `2026-2027-Biobuzz` in Android Studio. **If you just pulled, run
   File → Sync Project with Gradle Files first.** The project uses FTC SDK 12.0;
   without a sync, Android Studio still uses the old SDK and the AprilTag files
   show red errors.
2. Connect to the Control Hub with USB-C, or over Wi-Fi:
   ```
   adb connect 192.168.43.1:5555
   ```
3. In the run configuration dropdown (left of the green Run arrow), pick
   **Teamtestcode**, then press Run.
4. On the Driver Station, open the TeleOp or Autonomous list. Every tester is in
   the **Testing** group.

To go back to competition code, pick **TeamCode** in step 3 and press Run again.
Only one of the two apps can be on the robot at a time — installing either one
replaces the other.

## The testers

| Name on the Driver Station | Type | What it tests | Guide |
| --- | --- | --- | --- |
| Single Swerve Module | TeleOp | One swerve module on its own | [swervedrivetesting](swervedrivetesting/SingleSwerveTeleOp_README.md) |
| Swerve Drive Testing (1 module) | TeleOp | Swerve module mounted on the tank test robot | [swervedrivetesting](swervedrivetesting/README.md) |
| Tank Drive Auto (Square) | Autonomous | Tank robot drives a square and returns to start | [tankdriveauto](tankdriveauto/README.md) |
| Intake Motor Test | TeleOp | Intake motor speed and current | [intaketesting](intaketesting/README.md) |
| AprilTag Tester (Logitech) | TeleOp | Is a tag visible, which ID, where is it | [apriltagtesting](apriltagtesting/README.md) |
| AprilTag Test | TeleOp | Camera tuning: exposure, gain, decimation | [apriltagtesting](apriltagtesting/README.md) |

## Robot configuration names

Names must match exactly, including underscores and capital letters.

| Name | Device type | Used by |
| --- | --- | --- |
| `swerve_motor` | DC Motor | both swerve testers |
| `swerve_servo` | Continuous Rotation Servo | both swerve testers |
| `swerve_encoder` | Analog Input | both swerve testers |
| `left_front` | DC Motor | Swerve Drive Testing, Tank Drive Auto |
| `left_back` | DC Motor | Swerve Drive Testing, Tank Drive Auto |
| `right_front` | DC Motor | Swerve Drive Testing, Tank Drive Auto |
| `right_back` | DC Motor | Swerve Drive Testing, Tank Drive Auto |
| `imu` | IMU (built into the Control Hub) | Tank Drive Auto |
| `intake_motor` | DC Motor — set the real motor model | Intake Motor Test |
| `Webcam 1` | Webcam | both AprilTag testers |

If a name is wrong, INIT fails with `Could not find a hardware device with the name ...`.
Fix the name in the Driver Station configuration, not in the code.

## Rules for every tester

- **First run of anything that drives: wheels off the ground**, and keep a hand on STOP.
- Settings you are allowed to change are the constants at the top of each file
  (the lines starting with `private static final`). Leave the rest alone.
- After changing a constant, press Run in Android Studio again to reinstall.
