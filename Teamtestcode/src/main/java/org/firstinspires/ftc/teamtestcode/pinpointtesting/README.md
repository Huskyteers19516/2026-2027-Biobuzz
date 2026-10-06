# Pinpoint Tester

`PinpointTester.java` — TeleOp **"Pinpoint Tester"**, group **"Testing"**.

This OpMode checks whether the two goBILDA Pinpoint Odometry Computers on the robot
actually know where the robot is. It answers one question: *if I move the robot by
hand a known amount, does the Pinpoint report that amount?*

**The OpMode never commands a motor or a servo.** It does not even look them up in the
hardware map. Every test is performed by a human pushing or turning the robot by hand.
It is safe to run with the robot on a table, on the floor, or in your hands.

---

## Controls

| Control | What it does |
| --- | --- |
| Right bumper | next mode |
| Left bumper | previous mode |
| A | reset the active test (zeroes the test baseline, keeps the device pose) |
| B | `resetPosAndIMU()` on **both** devices — the robot must be completely still |
| X | switch which Pinpoint is "primary" for the single-device tests |
| Y | in DISTANCE TEST only: switch between the forward run and the strafe run |

Telemetry always shows, for both devices: found or not, device status, x, y, heading
and the device's internal loop frequency. The primary device gets the detailed verdict;
the other one gets a shorter line so you can still see it disagree.

B calls `resetPosAndIMU()`, which re-zeroes the gyro. The device takes about a quarter
of a second of samples to do that, and if the robot moves during those samples the gyro
zero is wrong and every heading reading afterwards drifts. Hold it still.

For a full second after B the OpMode **pauses all measuring** and says so on screen. That
is deliberate: during the device's calibration window the Pinpoint still reports the old
pose, and latching a baseline or a maximum from that stale data would poison every number
in the test. When the pause clears, the baseline is taken fresh and automatically. You do
not have to press A again after B, but it does no harm.

---

## Screen order vs. run order

The bumpers walk the modes in this fixed order:

```
MODE 1 LIVE   MODE 2 DIRECTION   MODE 3 DISTANCE   MODE 4 SPIN   MODE 5 OFFSET   MODE 6 COMPARE
```

The order you should **run** them in is *not* the same. The steps below are the run order,
and each heading says which screen mode it is. In particular, **DISTANCE is screen mode 3
but you run it fifth.**

---

## Run the modes in this order

The order is not a suggestion. Each test assumes the ones before it already pass,
because an error early in the chain corrupts every measurement after it.

### Step 1 — LIVE (screen MODE 1 of 6) — is anything alive at all

Put the robot down, leave it still, and look at the NOISE figure: the peak-to-peak
spread of x, y and heading over the last second.

* All three near zero while still → good.
* Position jumping around while the robot is still → a pod cable is loose, or the pod
  wheel is not touching the floor, or there is a bad I2C connection.
* Now push the robot by hand. If an axis stays at exactly `0.000` while you push it in
  that direction, that pod is **dead** — unplugged, wrong port, or a broken encoder cable.

This mode also shows each device's yaw scalar. If it reads `UNREADABLE`, the extra I2C
read for that register is failing; see the SPIN TEST note below.

Also check device status. `FAULT_X_POD_NOT_DETECTED`, `FAULT_Y_POD_NOT_DETECTED` and
`FAULT_NO_PODS_DETECTED` mean the Pinpoint itself cannot see the pod. `FAULT_IMU_RUNAWAY`
means the internal IMU is reporting nonsense. `CALIBRATING` is normal for a moment after B.

Fix every fault here before going on. There is no point measuring a pod that is not plugged in.

### Step 2 — DIRECTION CHECK (screen MODE 2 of 6) — is any pod backwards

Press B (robot still), then A. Push the robot **forward**, then **left**, then turn it
**counter-clockwise**. The OpMode latches how far each value swung in each direction and
reports a verdict per axis.

This mode measures travel in the **robot frame**: it takes each frame's field-frame
position change and rotates it back by the current heading before accumulating. That is
why the order of the three steps does not matter and why a robot that is already turned
does not produce a false verdict. (The other modes work in the field frame, which is why
they all tell you to press B first.)

The convention the Pinpoint uses:

* X (forward) **increases** when the robot moves forward.
* Y (strafe) **increases** when the robot moves to its **left**.
* Heading **increases** counter-clockwise, seen from above.

If an axis says WRONG DIRECTION the telemetry prints the exact remedy:

* X backwards → flip the **first** argument of
  `setEncoderDirections(xEncoder, yEncoder)` to `EncoderDirection.REVERSED`.
* Y backwards → flip the **second** argument.
* Heading backwards → **this is not fixable in software.** There is no heading-direction
  setter on the driver. The Pinpoint board is mounted upside down. Flip the board over
  and re-run everything.

**Why this must be first.** A reversed pod makes the distance test report a negative
distance. A reversed heading makes the spin test count backwards and makes the offset
test's circle point the wrong way. Fix direction before you measure anything.

### Step 3 — SPIN TEST (screen MODE 4 of 6) — is the heading scale right

Mark the direction the robot faces. Press B (still), then A. Turn the robot in place
exactly `SPIN_TURNS` full turns — 10 by default — slowly and smoothly, and stop facing
exactly the way it started.

The OpMode tracks **unwrapped** heading: it accumulates the wrapped frame-to-frame deltas,
so 10 turns reads as 3600 degrees instead of wrapping back to 0 every lap. It reports
total degrees turned, the error in degrees and percent, and a correction factor
`expected / measured`.

The driver exposes `setYawScalar(double)` and `getYawScalar()`, so the OpMode reads the
device's current yaw scalar and prints the **new** value to set:
`newYawScalar = currentYawScalar * (expected / measured)`.

`getYawScalar()` is not part of the driver's bulk read — it issues its own I2C read, and
the driver's backing field starts at `0`, so a rejected packet makes it return `0.0`. The
OpMode therefore validates the value (it must be between 0.5 and 2.0) and retries once a
second. If it still cannot read it, the screen says so and prints only the **factor**:
multiply whatever `setYawScalar` value your own code already uses by that factor. Never
write a yaw scalar of `0` to a Pinpoint — it would report a heading of zero forever.

Call `setYawScalar` once, right after you get the driver from the hardware map and
**before** `resetPosAndIMU()`. It is not persisted on the device, so it has to be set
every time your code starts.

goBILDA tunes each board at the factory, so the factor should already be very close to 1.
If you need a scalar below 0.95 or above 1.05, the board is faulty — contact goBILDA
rather than papering over it with a scalar.

**Why this comes before the offset and distance tests.** The Pinpoint subtracts a
rotation term from both pod readings, using heading, to turn pod travel into robot
travel. If heading is scaled wrong, that subtraction is wrong, and both the offset test
and any straight push that drifts slightly will give you a wrong answer.

### Step 4 — OFFSET TEST (screen MODE 5 of 6) — are the pod offsets right

Put the robot's intended tracking point (usually the center of the robot) over a mark on
the floor. Press B (still), then A. Turn the robot in place, on the spot, two or three
turns, keeping that point over the mark.

If the offsets are right, x and y stay near zero the whole time. If an offset is wrong,
the reported position traces a **circle**.

Here is the geometry, because the answer is counter-intuitive. Say the X (forward) pod's
configured sideways offset is wrong by `d`. During an in-place rotation the device is left
with a residual *robot-frame forward* velocity of `d * omega`. Integrating that in the
field frame from heading 0 gives

```
X(theta) = d * sin(theta)
Y(theta) = d * (1 - cos(theta))
```

which is a circle of radius `|d|` **centred at `(0, d)`** — the centre sits on the **Y**
axis, and the *Y* reading is the one that swings furthest (0 to `2d`, versus `-d` to `+d`
for X). A wrong Y (strafe) pod forward offset by `e` is the mirror image: the circle is
centred at `(e, 0)`, on the **X** axis.

So **which axis swings more is the opposite of which pod is at fault**, and the OpMode
decides on the **circle centre**, not on the swings:

* **Centre on the Y axis** → the **X (forward) pod's sideways offset** is wrong. That is
  `xOffset`, the **first** argument of `setOffsets(xOffset, yOffset, unit)`. Left of the
  tracking point is positive, right is negative.
* **Centre on the X axis** → the **Y (strafe) pod's forward offset** is wrong. That is
  `yOffset`, the **second** argument. Forward of the tracking point is positive, backward
  is negative.

The centre also gives the signed size of the error, so the screen prints the suggested
**new absolute value** in the same unit as the configured offset:
`newXOffset = currentXOffset - centreY`, `newYOffset = currentYOffset - centreX`.
The sign comes from the derivation above, not from a measurement of the device's internal
convention, so verify it once: apply the change and re-run. If `MAX WANDER` got **bigger**,
the sign was wrong — go back and move the offset the other way by **twice** the amount
shown, because you have to cross back over the original value and out the far side.

The verdict needs at least `OFFSET_MIN_TURN_DEG` (300 degrees) of rotation, which is
enough for both axis extremes to be reached so the centre is meaningful.

**Why before the distance test.** A wrong offset only shows up when the robot rotates.
If you push the robot "straight" and it rotates two degrees on the way, a wrong offset
adds a few tenths of an inch of pure error to the distance reading, and you would blame
the pod resolution for it.

### Step 5 — DISTANCE TEST (screen MODE 3 of 6) — is the pod resolution right

Line the robot up on a tape measure or the tile seams. Press B (still), then A. Push the
robot exactly `TEST_DISTANCE_INCHES` — 48 by default, three tile widths — straight
forward, without letting it rotate. Press Y and repeat pushing exactly sideways to the
left for the strafe run.

The OpMode reports measured distance on the axis under test, the cross-axis reading
(should be near zero), heading drift while you pushed, the error in inches and percent,
and two numbers:

```
SUGGESTED SCALE CORRECTION = commanded / measured
TICKS-PER-MM MULTIPLIER    = measured / commanded
```

**They are reciprocals, and you need the second one for the config.** The reported
distance is `ticks / ticksPerMm`, so distance and ticks-per-mm move in *opposite*
directions: if the device under-reports, you must make ticks-per-mm **smaller**, not
bigger. Multiplying by the scale correction would double the error instead of removing it.

> **Multiply the pod's ticks-per-mm by the TICKS-PER-MM MULTIPLIER** (equivalently, divide
> it by the scale correction) and apply the result with
> `setEncoderResolution(ticksPerMm, DistanceUnit.MM)` instead of the
> `setEncoderResolution(GoBildaOdometryPods)` preset.

The screen does this arithmetic for you against the configured preset and prints the
resulting absolute ticks-per-mm. The presets are 13.26291192 ticks/mm for a goBILDA
Swingarm pod and 19.89436789 ticks/mm for a 4-Bar pod (they are `private` inside
`GoBildaPinpointDriver`, so `PinpointTester` carries its own copies of the same numbers —
if goBILDA ever changes them, update both).

If the measured distance comes back **negative**, the OpMode refuses to print a correction
at all and sends you back to DIRECTION CHECK. A negative reading is a reversed pod, not a
scale error, and a negative ticks-per-mm is not a calibration.

The forward run blames the **X (forward) pod**; the strafe run blames the **Y (strafe)
pod**. The telemetry says which one, and says which device it is on, so you never apply
the left Pinpoint's correction to the right one.

If the correction comes out more than a couple of percent from 1.0 on a genuine goBILDA
pod, suspect a slipping or dirty pod wheel or a wheel riding on a seam, not the preset.

### Step 6 — COMPARE (screen MODE 6 of 6) — which of the two is lying

Park the robot on a marked spot. Press B (still), then A. Push it around a long loop by
hand and bring it back onto the exact same spot, same heading.

For each device the OpMode reports the **closure error**: how far from 0, 0 it thinks it
is now that it is physically back where it started. It also reports the running maximum
disagreement between the two devices in inches and degrees.

* Both close well → odometry is good, go path-tune.
* One closes well, the other does not → that one is the bad one. Re-run steps 2 to 5 on it.
* Both close badly in the same way → something common: the pod wheels, the floor, or the
  tracking-point geometry.

Run this last, because it only makes sense once both devices individually pass.

---

## Driver Station configuration

Each Pinpoint is added in the robot configuration as

```
I2C Bus ... -> goBILDA(R) Pinpoint Odometry Computer
```

(XML tag `goBILDAPinpoint`). Create two of them, named exactly:

* `pinpoint_left`
* `pinpoint_right`

**They must be on two different I2C buses.** The I2C address `0x31` is hard-coded inside
`GoBildaPinpointDriver` — it is set in the driver's constructor and there is no setter
for it, and the device firmware does not expose an address-change register. Two devices
on the same bus would both answer at `0x31` and the data would be garbage. A Control Hub
has four I2C buses (0 through 3); bus 0 already holds the built-in IMU, so put the two
Pinpoints on, for example, bus 1 and bus 2 — or one on the Control Hub and one on the
Expansion Hub.

If a device is missing or misnamed, the OpMode does **not** crash. It fetches both with
`hardwareMap.tryGet(...)` and prints `FOUND: NO` with a hint, so a bad config shows up on
screen at INIT instead of as an exception.

---

## Where each correction goes

`PinpointTester` applies its own constants at INIT so you can test a change quickly, but
it is a tester — it does not write anything back. Once a number is good, put it in the
real configs:

**Inside this OpMode (for the next test run):**
the constants at the top of `PinpointTester.java` —
`LEFT_DEVICE_NAME`, `RIGHT_DEVICE_NAME`, `POD_TYPE`,
`LEFT_X_POD_OFFSET` / `LEFT_Y_POD_OFFSET` / `LEFT_OFFSET_UNIT`,
`RIGHT_X_POD_OFFSET` / `RIGHT_Y_POD_OFFSET` / `RIGHT_OFFSET_UNIT`,
`LEFT_X_POD_DIRECTION` / `LEFT_Y_POD_DIRECTION`,
`RIGHT_X_POD_DIRECTION` / `RIGHT_Y_POD_DIRECTION`,
`TEST_DISTANCE_INCHES`, `SPIN_TURNS`, `NOISE_WINDOW_SECONDS`, and the tolerance block.

**In the competition code — the Pedro configs:**

* `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java`
* `Teamtestcode/src/main/java/org/firstinspires/ftc/teamtestcode/pedropathing/Constants.java`

Both build a `PinpointConfig` in `pinpointConfig()`. Against `com.pedropathing:revhub:3.0.1`,
which is what both modules pin, the mapping is:

| Tester result | Pedro field in `pinpointConfig()` | Raw driver call |
| --- | --- | --- |
| device name | `c.name` | — |
| pod offsets | `c.xPodOffset`, `c.yPodOffset`, `c.offsetUnits` | `setOffsets(x, y, unit)` |
| encoder directions | `c.xPodDirection`, `c.yPodDirection` | `setEncoderDirections(x, y)` |
| pod type | `c.podType` | `setEncoderResolution(GoBildaOdometryPods)` |
| scale correction | `c.ticksPerUnit` = `OptionalDouble.of(v)` **and** `c.encoderResolutionUnit` = `DistanceUnit.MM` | `setEncoderResolution(ticksPerMm, DistanceUnit.MM)` |

`PinpointLocalizer` reads `ticksPerUnit` and, when it is present, calls
`setEncoderResolution(ticksPerUnit, encoderResolutionUnit)` instead of the `podType`
preset. **`encoderResolutionUnit` defaults to `DistanceUnit.INCH`**, so if your value is
in ticks per millimetre you must set the unit as well, or the resolution will be wrong by
a factor of 25.4.

One thing has **no** Pedro config field:

* **Yaw scalar.** `PinpointConfig` has no field for it, and `PinpointLocalizer` keeps its
  `GoBildaPinpointDriver` in a `private final` field with no getter, so you cannot reach
  the localizer's instance. Fetch the device yourself —
  `hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint_left")` returns the same cached
  object the localizer holds — and call `setYawScalar(value)` on it before the localizer
  resets the IMU.

Note that the two Constants files currently disagree — the TeamCode one already names
`pinpoint_left` / `pinpoint_right` with offsets in millimetres, the Teamtestcode one still
says `"pinpoint"` with zero offsets. Make them match once this tester gives you the real
numbers.

---

## Tolerances to expect

These are the defaults in the file. Loosen them only when you understand why.

| Thing | Constant | Default | Meaning |
| --- | --- | --- | --- |
| Noise, position | `NOISE_POSITION_TOLERANCE_IN` | 0.08 in | peak-to-peak over 1 s while still |
| Noise, heading | `NOISE_HEADING_TOLERANCE_DEG` | 0.40 deg | same |
| Direction, min travel | `DIRECTION_MIN_TRAVEL_IN` | 4.0 in | below this it says "not moved yet" |
| Direction, min turn | `DIRECTION_MIN_TURN_DEG` | 20 deg | same, for heading |
| Distance scale | `DISTANCE_TOLERANCE_PERCENT` | 2.0 % | over a 48 in push, about 1 in |
| Cross axis | `CROSS_AXIS_TOLERANCE_IN` | 1.0 in | sideways creep during a straight push |
| Heading drift | `HEADING_DRIFT_TOLERANCE_DEG` | 2.0 deg | rotation during a straight push |
| Spin | `SPIN_TOLERANCE_PERCENT` | 1.0 % | over 10 turns, about 36 deg |
| Spin, min turn | `SPIN_MIN_TURN_DEG` | 180 deg | below this no correction is printed |
| Offset wander | `OFFSET_WANDER_TOLERANCE_IN` | 1.0 in | max distance from origin while spinning |
| Offset, min turn | `OFFSET_MIN_TURN_DEG` | 300 deg | enough rotation for the circle centre to mean something |
| Loop closure | `CLOSURE_TOLERANCE_IN` | 2.0 in | after a hand-pushed loop |
| Device agreement | `DEVICE_AGREEMENT_TOLERANCE_IN` | 1.0 in | left vs right |
| Device agreement | `DEVICE_AGREEMENT_TOLERANCE_DEG` | 2.0 deg | left vs right |
| Reset pause | `CALIBRATION_SETTLE_SECONDS` | 1.0 s | measuring is frozen this long after B |

A healthy Pinpoint reports a loop frequency in the hundreds of hertz. If `Hz` reads very
low or the loop time is huge, the I2C bus is overloaded or the cable is marginal.

---

## Troubleshooting

**FOUND: NO.** The name in the robot configuration does not match
`LEFT_DEVICE_NAME` / `RIGHT_DEVICE_NAME`, or the device was added as the wrong type.
Names are case-sensitive and must have no trailing spaces.

**Only one device is ever found, or the two give identical garbage.** Both are on the same
I2C bus. They share the fixed address `0x31`. Move one to another bus.

**Status is `FAULT_X_POD_NOT_DETECTED` / `FAULT_Y_POD_NOT_DETECTED`.** The pod is not
plugged into that port on the Pinpoint, or the encoder cable is broken. Swap the cable
with the other pod's cable to tell a bad cable from a bad pod.

**Status is `FAULT_IMU_RUNAWAY`.** The internal IMU is reporting an impossible rate. Power
cycle; if it persists the board is faulty.

**Status stays `CALIBRATING` or `NOT_READY`.** Give it a second after B. If it never leaves,
check 3.3 V power and the I2C cable.

**Status is `FAULT_BAD_READ`.** The driver's error detection rejected the packet. Usually a
long or poorly seated I2C cable.

**yawScalar reads `UNREADABLE`.** The extra I2C read for the `YAW_SCALAR` register is being
rejected — same cable and bus causes as `FAULT_BAD_READ`. The SPIN TEST still gives you a
valid correction *factor*; it just cannot show the resulting absolute value.

**Heading drifts while the robot sits still.** The gyro zero is bad: the robot moved during
`resetPosAndIMU()`. Press B again and genuinely do not touch it.

**Distance reads about right but position wanders during turns.** Pod offsets. Go to
OFFSET TEST.

**Numbers look wild for the first second after you press B.** They should not — measuring
is frozen for `CALIBRATION_SETTLE_SECONDS` and the screen says so. If you still see a jump
afterwards, raise that constant: your device is taking longer than a second to settle.

**Everything passes by hand but paths still overshoot.** Odometry is not your problem any
more — that is Pedro tuning, not calibration.

**One device passes everything and the other fails everything.** Swap the pods between the
two Pinpoints and re-run. If the fault follows the pods, it is the pods; if it stays with
the board, it is the board.
