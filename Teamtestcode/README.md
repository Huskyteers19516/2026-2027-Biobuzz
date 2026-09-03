# Teamtestcode

A standalone test app, separate from `TeamCode`. Use it to try things on the robot
without touching competition code.

`Teamtestcode` and `TeamCode` are both application modules, so each one builds its
own Robot Controller APK. You pick which of the two is on the robot by switching
the run configuration in Android Studio.

## Running the test app on the robot

1. **Connect to the Robot Controller.** USB-C from the laptop to the Control Hub,
   or over Wi-Fi once the hub is up:

   ```
   adb connect 192.168.43.1:5555
   ```

   Check it took with `adb devices` — the hub should be listed.

2. **Pick the module.** In the Android Studio toolbar, open the run configuration
   dropdown (it sits left of the green Run arrow) and choose **Teamtestcode**.

3. **Run.** Press the green arrow. Gradle builds the APK and installs it, replacing
   whichever Robot Controller app was there before.

4. **Restart the Robot Controller** if the Driver Station shows a lost connection.
   Installing the app stops it, and the Driver Station reconnects on its own after
   a few seconds.

5. **Pick the OpMode** on the Driver Station, from the TeleOp or Autonomous
   dropdown. Anything in this module annotated `@TeleOp` or `@Autonomous` shows up
   in the list.

## Going back to competition code

Same steps, but pick **TeamCode** in step 2. It reinstalls over the test app.

Only one of the two can be on the robot at a time. They share the applicationId
`com.qualcomm.ftcrobotcontroller`, so installing either one replaces the other —
this is deliberate, and it is why no uninstall step is needed.

## What is in the OpMode list

`Teamtestcode` depends on `:FtcRobotController` only, not on `:TeamCode`. So when
the test app is loaded, the Driver Station shows:

| Loaded app | OpModes available |
| --- | --- |
| Teamtestcode | this module's OpModes, plus the SDK samples |
| TeamCode | `opmode`, `pedroPathing`, `testclass`, plus the SDK samples |

The test app does **not** see competition OpModes, and competition code does not
see anything in here. If a test needs to call into `TeamCode`, either copy the
class over or add `implementation project(':TeamCode')` to this module's
`build.gradle`.

## Adding a test OpMode

Put the class under `src/main/java/org/firstinspires/ftc/teamtestcode/`, in a
subpackage per experiment (there is a `swervedrivetesting` package already). Give
it an `@TeleOp` or `@Autonomous` annotation, otherwise it never reaches the
Driver Station list:

```java
@TeleOp(name = "My Test", group = "Testing")
public class MyTest extends OpMode { ... }
```

The `name` is what appears on the Driver Station; the `group` is the heading it
gets sorted under.

## Telling the two apps apart on the robot

Both modules take their app name from `FtcRobotController`, so out of the box each
one installs as "FTC Robot Controller" and there is no way to tell from the device
which is loaded. To label this one differently, create
`src/main/res/values/strings.xml` here:

```xml
<?xml version="1.0" encoding="utf-8"?>
<resources>
    <string name="app_name">FTC RC (TEST)</string>
</resources>
```

Application module resources win over library ones, so this overrides the name for
the test APK only.

## Troubleshooting

**OpMode missing from the Driver Station list.** The annotation is absent, or the
wrong module is loaded. Check the run configuration.

**`Could not find a hardware device with the name ...`** at init. The name in
`hardwareMap.get(...)` does not match the robot configuration, or the port type is
wrong — a CRServo configured as a Servo, an analog encoder configured as a digital
device, and so on. Fix it in the Driver Station configuration, not in code.

**Build succeeds but the robot runs old code.** The install went to a different
device. Run `adb devices` and disconnect anything that is not the Control Hub.
