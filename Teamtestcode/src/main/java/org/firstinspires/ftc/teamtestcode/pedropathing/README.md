# Pedro Pathing v3.0.1: mecanum + goBILDA Pinpoint

Everything for Pedro Pathing lives in this folder, inside the **Teamtestcode** app.
TeamCode is untouched. Gradle changes are only in `Teamtestcode/build.gradle`:

```groovy
repositories {
    mavenCentral()
    maven { url 'https://repo.dairy.foundation/releases/' }
}
dependencies {
    implementation 'com.pedropathing:revhub:3.0.1'
    implementation 'com.pedropathing:tuning:1.0.1'
}
```

The main setup is a **4-motor mecanum drivetrain** with a **goBILDA Pinpoint** for odometry.
The old 2-pod swerve setup is kept in the `swerve/` subpackage (see the last section).

## Files

| File | What it is |
|---|---|
| `Constants.java` | **Every robot number goes here.** Builds the `Mecanum` drivetrain, the `PinpointLocalizer`, the `Foresight` algorithm and the `Follower` (`Constants.createFollower(hardwareMap)`). |
| `PedroPinpointPoseTest.java` | TeleOp **"Pedro Pinpoint Pose Test"**. Pinpoint only, no motors. Push the robot by hand and watch the pose. |
| `PedroMecanumTeleOp.java` | TeleOp **"Pedro Mecanum TeleOp"**. Drives the mecanum through the Pedro Follower (robot or field centric) and shows the Pinpoint pose. |
| `tuning/Tuning.java` | Registers the Pedro web tuners (see "Why `tuning/Tuning.java` registers the tuners by hand"). |
| `tuning/MecanumTuner.java` | Unchanged copy of the official Pedro v3.0.1 Quickstart "Mecanum Tuner" (only the package line differs). Spins one motor at a time so you can pick each motor direction. |
| `tuning/PinpointTuner.java`, `tuning/ForesightTuner.java`, `tuning/Tests.java` | Copied from the official Pedro v3.0.1 Quickstart. We removed the comments, changed `List.of` to `Arrays.asList`, made the path tests use the Distance input and set Localization as the default test. The OTOS, OctoQuad and dead-wheel tuners were left out. |
| `swerve/` | The 2-pod swerve setup, kept for later: `SwerveConstants.java`, `SwervePodCalibration.java`, `PedroSwerveTeleOp.java`. |

All Driver Station OpModes are in group **"Pedro"**.

## Driver Station configuration

The drive motor names are the same ones the team's `MecanumTeleOp` uses.

| Name | Type | Plugged into |
|---|---|---|
| `left_front` | Motor (e.g. goBILDA 5203) | a motor port |
| `left_back` | Motor | a motor port |
| `right_front` | Motor | a motor port |
| `right_back` | Motor | a motor port |
| `pinpoint` | **goBILDA Pinpoint Odometry Computer** | an I2C port |

**Motor ports:** the 4 drive motors plus the `launcher` from `MecanumTeleOp` are **5 motors**. One Control Hub only has **4 motor ports**, so the robot needs an Expansion Hub (or another motor controller) for the fifth motor. The Pedro code only uses the 4 drive motors and does not care which hub they are on, as long as the names match.

## Coordinate conventions (important)

- Pedro poses are in **inches and radians**. At a heading of 0, **+X is forward and +Y is left**. Heading grows **counter-clockwise** (seen from above).
- `DrivePowers(forward, strafe, turn)`: strafe + means left and turn + means CCW.
- **Mecanum motor directions**: with every direction set right, positive power on each motor must roll the robot **forward**. The starting values match `MecanumTeleOp` (left side `FORWARD`, right side `REVERSE`). The **"Pedro Mecanum Tuner"** checks each motor one at a time.
- **Pinpoint offsets** (goBILDA convention, inches): `PINPOINT_X_POD_OFFSET_IN` is how far **left** of the robot center the forward-measuring (X) pod is (left is +). `PINPOINT_Y_POD_OFFSET_IN` is how far **forward** of center the strafe-measuring (Y) pod is (forward is +).

## Constants to measure / fill in (`Constants.java`)

| Constant | How to get it |
|---|---|
| `FRONT_LEFT/BACK_LEFT/FRONT_RIGHT/BACK_RIGHT_MOTOR_NAME` | Must match the Driver Station config (`left_front`, `left_back`, `right_front`, `right_back`). |
| `FRONT_LEFT/BACK_LEFT/FRONT_RIGHT/BACK_RIGHT_DIRECTION` | Pedro web tuner "Pedro Mecanum Tuner". |
| `MANUAL_BRAKE_MODE`, `POWER_THRESHOLD` | Pedro defaults (`true`, `0.01`). BRAKE is used while driving by hand and when the Follower stops. |
| `PINPOINT_*` | Pedro web tuner "Pedro Pinpoint Tuner", or measure by hand. |
| `MAX_*_VELOCITY`, `NATURAL_*_DECELERATION`, `*_BRAKE_*`, `HEADING_KP`, `*_TRANSLATIONAL_*_KP`, `COAST_KV`, `BRAKE_KV` | Pedro web tuner "Pedro Foresight Tuner". It prints a `ForesightConfig` block. Copy each number into the matching constant. |

The Pinpoint and Foresight numbers in `Constants` right now are **placeholders**. The Foresight values are required by Pedro (`ConfigVar.required()`), so they have to be set to *something* for the Follower to be built. Do not trust path following until the Foresight Tuner has been run on the mecanum robot. Foresight values depend on the drivetrain, so tune them again after any drivetrain change.

## Step-by-step order

Keep a hand on STOP for every step that moves motors.

1. **Configuration.** Create the Driver Station config with the names above. Build and install the **Teamtestcode** app. It replaces the TeamCode app on the Control Hub because both use the same applicationId.
2. **Motor directions: "Pedro Mecanum Tuner"** (web tuner, robot on a stand with the wheels OFF the ground). Enter `left_front`, `right_front`, `left_back`, `right_back` as the names. Each motor spins by itself; answer whether it spun forward. Copy the resulting directions into the four `*_DIRECTION` constants and rebuild.
3. **Pinpoint check: "Pedro Pinpoint Pose Test"** (no motors). Push the robot forward: X should go up. Push it left: Y should go up. Turn it CCW: heading should go up. If an axis is wrong, flip `PINPOINT_X_POD_DIRECTION` / `PINPOINT_Y_POD_DIRECTION`.
4. **"Pedro Pinpoint Tuner"** (web tuner): pod type, directions and offsets. Copy the result into `PINPOINT_*` and rebuild.
5. **Driving test: "Pedro Mecanum TeleOp"** (robot on the floor, slow mode with the left bumper). Drive forward, strafe left and turn left. The robot must move that way and X, Y and heading must change like in step 3. If a direction is wrong, go back to step 2.
6. **"Pedro Foresight Tuner"** (web tuner, open space of at least about 2 m / 7 ft in every direction): velocities, decelerations, braking and controller gains. Copy the result into the Foresight constants and rebuild.
7. **"Pedro Tests"** (web tuner): Localization / Odometry / Driving / Pose tests for a sanity check, then Hold / Line / Curve tests to check path following.

Then drive with "Pedro Mecanum TeleOp" (Y toggles field or robot centric, Back resets the pose) and write autos with `Constants.createFollower(hardwareMap)`.

## Pedro web tuners

Connect the laptop to the Control Hub Wi-Fi and open **http://192.168.43.1:10158** (the Pedro tuning web UI served by the robot app, port 10158). Registered procedures: **Pedro Mecanum Tuner**, **Pedro Pinpoint Tuner**, **Pedro Foresight Tuner**, **Pedro Tests**. The Foresight Tuner and Tests use the mecanum drivetrain, the Pinpoint localizer and the Foresight algorithm from `Constants`.

## Why `tuning/Tuning.java` registers the tuners by hand

The Pedro tuning library (`com.pedropathing:tuning:1.0.1`) finds `@Tuner` methods with a Sinister/Sloth scanner. That scanner uses `NarrowSearch`, which only looks inside `org.firstinspires.ftc.teamcode`. Our package is `org.firstinspires.ftc.teamtestcode...`, so the scanner would never find our tuners. Instead, `Tuning.registerTuners` is an FTC SDK `@OnCreate` hook that calls `TunerRegistrar.register(...)` directly. If the tuners are ever missing from the web UI, this is the place to look.

Known edge case: the library's `TunerScanner.unload` calls `TunerRegistrar.deregisterAll()`, which also removes our hand-registered tuners. That only happens if classes are hot-reloaded (for example with the Sloth Gradle load plugin, which we do not use). `@OnCreate` runs once per app start, so if the tuners disappear from http://192.168.43.1:10158, **restart the robot app**.

## The `swerve/` subpackage (kept for later)

The 2-pod coaxial swerve work is kept in `swerve/` (package `org.firstinspires.ftc.teamtestcode.pedropathing.swerve`) so it still compiles and can be used again if the team rebuilds the swerve robot.

- `SwerveConstants.java`: pod names, pod calibration numbers, `SwerveConfig`, `createDrivetrain` (a `Swerve`) and `createFollower`. The swerve Follower **shares the Pinpoint and Foresight constants with `Constants.java`** (`Constants.createLocalizer` and `Constants.createAlgorithm`), so odometry and path-following numbers live in one place. Re-run the Foresight Tuner after switching drivetrains.
- `SwervePodCalibration.java`: TeleOp **"Swerve Pod Calibration"** (wheels off the ground) for each pod's angle offset, encoder direction, servo direction, drive direction and analog voltage range.
- `PedroSwerveTeleOp.java`: TeleOp **"Pedro Swerve TeleOp"**.

Swerve Driver Station config: `left_swerve_motor` / `right_swerve_motor` (Motor), `left_swerve_servo` / `right_swerve_servo` (Continuous Rotation Servo, Axon), `left_swerve_encoder` / `right_swerve_encoder` (Analog Input, the Axon feedback wire), plus `pinpoint`.

Swerve notes that still apply when using it:

- `Swerve.computePodPowers` only uses the direction of each pod offset. In revhub 3.0.1 the rotation only comes out right if `podOffset = (forward, RIGHT)`, which is why `SWERVE_OFFSET_Y_SIGN = -1`. If the turn stick spins the robot the wrong way while pods are calibrated, set it to `1.0`. Re-check after every Pedro version bump.
- `*_POD_ANGLE_OFFSET_RAD` is in radians: the raw angle with the wheel pointing forward, or raw + pi for a pod with `ENCODER_REVERSED = true`. The calibration OpMode shows both values.
- A robot on only two swerve pods also needs passive support (casters or skids).
- In revhub 3.0.1, `follower.stop()` on swerve leaves the drive motors floating (BRAKE only applies while driving with `follower.manual(...)`).
- To run the web tuners on the swerve robot, point `tuning/Tuning.java`'s `foresightTuner()` and `tests()` at `SwerveConstants::createDrivetrain` instead of `Constants::createDrivetrain`, and skip the Mecanum Tuner.
