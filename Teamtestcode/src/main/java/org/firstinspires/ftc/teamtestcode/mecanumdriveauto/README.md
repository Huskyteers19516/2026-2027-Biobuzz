# Mecanum Drive Auto (Square)

An Autonomous for the 4-motor **mecanum** test robot. It drives a
`SIDE_LENGTH_INCHES` square (24 in by default) and ends back where it started.

There are two modes, picked by one boolean at the top of the file:

```java
private static final boolean USE_TURNS = false;
```

Distances always come from the drive motor encoders (`RUN_TO_POSITION`).
Heading always comes from the Control Hub IMU, which is far more accurate than
encoders for angles.

The `launcher` motor is **not** used by this OpMode. Only the four drive motors
and the IMU are touched.

## The two modes

### `USE_TURNS = false` (default, the mecanum showcase)

Forward, strafe right, backward, strafe left. The robot **never turns** — the
IMU holds the starting heading for the whole square, so the front of the robot
points the same way at every corner.

```
              2. strafe RIGHT
        +--------------------->+
        ^                      |
        |                      |
     1. |                      | 3. backward
   forward                     |
        |                      v
        +<---------------------+
      START     4. strafe LEFT

   robot nose points UP the whole time
```

The square is traced **clockwise**: up, right, down, left. Because the nose never
turns, "up" is forward, "right" is a strafe right, "down" is backward and "left"
is a strafe left.

### `USE_TURNS = true`

Exactly the tank behaviour: drive forward one side, turn right 90 deg, settle,
repeat four times. Useful as a back-to-back comparison with `TankDriveAuto`.

```
        +----------+        after each side the whole
        |          |        robot rotates 90 deg CW
        |          v
        ^          |
        |          |
        +<---------+
       START
```

## Hardware (Driver Station configuration names)

| Config name | Device | Notes |
| --- | --- | --- |
| `left_front` | DC Motor | direction FORWARD |
| `left_back` | DC Motor | direction FORWARD |
| `right_front` | DC Motor | direction REVERSE |
| `right_back` | DC Motor | direction REVERSE |
| `imu` | IMU | built into the Control Hub |

Same names and directions as `MecanumTeleOp`, so a config that runs the TeleOp
runs this without any change.

> **The encoder cable on all 4 drive motors must be plugged in.**
> `RUN_TO_POSITION` needs them. A leg only ends when **all four** wheels report
> they have arrived, so a single unplugged encoder makes that leg run until its
> timeout expires or you press STOP. **First run: wheels off the ground.**

The rollers must form an **X** when you look down at the robot from above. If
your wheels form an O, swap the left pair with the right pair, or the strafe
directions in this file are wrong.

## Set these before the first run

| Constant | Default | What to put |
| --- | --- | --- |
| `COUNTS_PER_MOTOR_REV` | 537.7 | From the motor spec sheet. 537.7 is goBILDA 5203 312 RPM |
| `DRIVE_GEAR_REDUCTION` | 1.0 | Extra gearing between motor and wheel, if any |
| `WHEEL_DIAMETER_INCHES` | 3.78 | Measure the wheel. 3.78 in = 96 mm |
| `STRAFE_MULTIPLIER` | 1.15 | See below. Measure it, do not guess |
| `HUB_LOGO_DIRECTION` | UP | Which way the REV logo on the Control Hub faces |
| `HUB_USB_DIRECTION` | FORWARD | Which way the Control Hub's USB ports face |
| `SIDE_LENGTH_INCHES` | 24.0 | Size of the square |
| `USE_TURNS` | false | `true` = tank-style square with 90 deg turns |
| `STRAFE_CALIBRATION` | false | `true` = stop after the strafe leg, for measuring `STRAFE_MULTIPLIER` |
| `DRIVE_SPEED` / `STRAFE_SPEED` / `TURN_SPEED` | 0.4 / 0.4 / 0.3 | Lower these for the first run |

`COUNTS_PER_INCH` is computed from the first three:

```
COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                  / (WHEEL_DIAMETER_INCHES * PI)
```

**The two hub directions must match how the Control Hub is actually mounted.**
If they are wrong, the heading correction pushes the wrong way and the robot
curves instead of holding straight.

### Measuring `STRAFE_MULTIPLIER`

Mecanum wheels slip sideways. One encoder tick of strafe moves the robot less
than one tick of forward driving, so a strafe needs *more* ticks for the same
inches. `STRAFE_MULTIPLIER` is applied to the strafe tick target only.

1. Set `STRAFE_MULTIPLIER = 1.0`, `USE_TURNS = false` and
   `STRAFE_CALIBRATION = true`. The last one makes the OpMode **stop by itself
   after the strafe leg**, so you do not have to press STOP at the right moment.
2. Temporarily set `SIDE_LENGTH_INCHES = 48.0` so the error is easy to see.
3. Mark the floor at a wheel and run the OpMode. It drives forward, strafes
   right, and stops. Measure the real sideways distance from the mark.
4. Check the `Last leg` telemetry line before you trust the number. It must say
   `strafing: arrived`. If it says `TIMEOUT`, the leg was cut short and the
   measurement is worthless — raise `LEG_TIMEOUT_SECONDS_PER_INCH` and re-run.
5. `STRAFE_MULTIPLIER = commanded / measured`, i.e. `48 / measured`.
   Example: it only went 42 in, so `48 / 42 = 1.14`.
6. Put that number in, restore `SIDE_LENGTH_INCHES`, set
   `STRAFE_CALIBRATION = false`, and re-run the full square to confirm.

Expect something between 1.0 and 1.3. Carpet, wheel wear and robot weight all
change it, so re-measure when the robot changes.

## Running it

1. Driver Station -> Autonomous list -> **Mecanum Drive Auto (Square)** -> INIT.
2. Place the robot with a clear 24 in square in front of it and to its right.
3. Press START. The IMU yaw is zeroed at that moment, so whichever way the robot
   faces when you press START is "0 deg".

## Step order

`USE_TURNS = false`:

| # | Step | Method |
| --- | --- | --- |
| 1 | forward 24 in, heading held at 0 | `driveStraight(+)` |
| 2 | settle for `SETTLE_SECONDS` | `holdHeading` |
| 3 | strafe right 24 in, heading held at 0 | `strafe(+)` |
| 4 | settle for `SETTLE_SECONDS` (stops here if `STRAFE_CALIBRATION`) | `holdHeading` |
| 5 | backward 24 in, heading held at 0 | `driveStraight(-)` |
| 6 | settle for `SETTLE_SECONDS` | `holdHeading` |
| 7 | strafe left 24 in, heading held at 0 | `strafe(-)` |
| 8 | settle for `SETTLE_SECONDS` | `holdHeading` |

Every leg is followed by a settle, the same way the turn mode settles after each
side. The settle squares up the heading before the next leg and keeps the robot
from slamming straight from a full-speed strafe into a full-speed reverse.

`USE_TURNS = true`, repeated 4 times:

| # | Step | Method |
| --- | --- | --- |
| 1 | forward 24 in | `driveStraight(+)` |
| 2 | turn right 90 deg until error < `HEADING_THRESHOLD` | `turnToHeading` |
| 3 | hold the new heading for `SETTLE_SECONDS` | `holdHeading` |

Every step is guarded by `opModeIsActive()` and by a timeout, and the motors are
set to zero power at the end of every step, at the end of the OpMode, and if the
OpMode is stopped part-way through.

A turn is bounded by `TURN_TIMEOUT_SECONDS` (4 s). A driving or strafing leg
gets a timeout sized to that leg, so that making the square bigger or the robot
slower does not silently cut the leg short:

```
leg timeout = LEG_TIMEOUT_BASE_SECONDS
              + |inches| * LEG_TIMEOUT_SECONDS_PER_INCH
                * (LEG_TIMEOUT_REFERENCE_SPEED / speed)
```

With the defaults, a 24 in leg at speed 0.4 gets 8 s, a 48 in leg at 0.4 gets
14 s, and a 24 in leg at 0.2 also gets 14 s. The number in use is on the
`Leg timeout` telemetry line, and `Last leg` says whether the leg finished
because it arrived or because the timeout fired.

## Telemetry

| Line | Meaning |
| --- | --- |
| Step | Which leg of the square, in words |
| Mode | `driving`, `strafing`, `turning`, `holding` |
| Last leg | How the previous leg ended: `arrived in x.x s`, `TIMEOUT ...` or `stopped` |
| Leg timeout | The timeout computed for the leg that is running now |
| Target heading | The heading it should hold, in degrees |
| Heading | What the IMU reads now |
| Error | Target minus actual |
| Targets / Actual | Encoder target and current position for each wheel |
| Final heading | Shown at the end. Should be close to 0 |

## Tuning table

| Symptom | Gain / constant to change |
| --- | --- |
| Square is the wrong size in the **forward** direction | `COUNTS_PER_MOTOR_REV`, `WHEEL_DIAMETER_INCHES`, `DRIVE_GEAR_REDUCTION` |
| Forward legs are right but **sideways** legs are short | Raise `STRAFE_MULTIPLIER` |
| Sideways legs overshoot | Lower `STRAFE_MULTIPLIER` |
| Robot rotates slowly while driving or strafing | Raise `P_DRIVE_GAIN` |
| Robot shakes / snakes while driving or strafing | Lower `P_DRIVE_GAIN` |
| Wobbles at the end of a turn (`USE_TURNS = true`) | Lower `P_TURN_GAIN` |
| Stops turning before 90 deg | Raise `P_TURN_GAIN`, or raise `TURN_TIMEOUT_SECONDS` |
| Heading correction fights the drive and stalls a wheel | Lower `MAX_CORRECTION_SCALE` (default 0.5 = correction may never exceed half the leg speed) |
| Leg ends early and `Last leg` says `TIMEOUT` | Raise `LEG_TIMEOUT_SECONDS_PER_INCH` or `LEG_TIMEOUT_BASE_SECONDS` |

**`P_DRIVE_GAIN` cannot fix a rotated finish.** Under `RUN_TO_POSITION` each
wheel turns exactly as far as its tick target says, whatever the power is
(`setPower` only sets a speed cap in that mode). The correction changes *when*
each wheel gets there, not *how far* it goes, so the only heading it can buy
back during a leg comes from wheel slip. If the robot still ends a leg rotated,
look at the mechanical side — even weight, roller X pattern, a wheel with no
grip — instead of raising the gain. Raising it also widens the split between the
fast pair and the slow pair of wheels, which makes the legs less even, not more.

## Troubleshooting

| Problem | Fix |
| --- | --- |
| A leg keeps going until `Last leg` says `TIMEOUT` | An encoder cable is unplugged. In the `Actual` telemetry, the wheel whose count stays at 0 is the one. Press STOP |
| Robot drives and never stops | Same cause, and check that the timeout is not set absurdly high. Press STOP |
| Robot drives backward on leg 1 | Motor directions are wrong (left FORWARD, right REVERSE) |
| Robot **spins** instead of strafing on leg 2 | One motor's direction is wrong, or the front/back motors are swapped in the configuration |
| Robot strafes **left** when it should go right | The left pair and right pair of wheels are swapped, or the rollers form an O instead of an X |
| Robot strafes diagonally | One wheel has no grip (weight not even) or one encoder is unplugged |
| Heading drifts the wrong way and gets worse | `HUB_LOGO_DIRECTION` / `HUB_USB_DIRECTION` do not match how the hub is mounted |
| Square does not close | Check `Final heading` first. Off angle -> heading problem. Angle fine -> distance problem (`COUNTS_PER_INCH` or `STRAFE_MULTIPLIER`) |
| Robot barely moves | `DRIVE_SPEED` too low for the carpet, or brake is fighting a stale `RUN_TO_POSITION` target |

## Difference from Tank Drive Auto

| | `TankDriveAuto` | `MecanumDriveAuto` |
| --- | --- | --- |
| Sideways motion | impossible | `strafe()` leg, no rotation |
| Square without turning | impossible | default mode |
| Power mixing | left pair / right pair | per wheel `setWheelPowers(fl, fr, bl, br)` with proportional normalization |
| Heading correction | added to the left/right split | added as a **turn term** on all four wheels, so it works while strafing too |
| Extra constant | none | `STRAFE_MULTIPLIER`, `MAX_CORRECTION_SCALE`, the leg timeout constants |
| When a leg ends | as soon as one wheel arrives | only when **all four** wheels arrive, or on the leg timeout |
| Shape of a leg | tick target is the same for all 4 wheels | tick target has a **per-wheel sign** (see below) |

### Why the per-wheel signs look like that

With left FORWARD / right REVERSE, a positive power on any wheel drives the
robot forward. Using the same mixing Pedro uses (`strafe +` = LEFT,
`turn +` = counter-clockwise):

```
fl = forward - strafe - turn
fr = forward + strafe + turn
bl = forward + strafe - turn
br = forward - strafe + turn
```

Strafing **right** is `strafe` negative, so the signs are
`fl +, fr -, bl -, br +` — which is what the code writes into the encoder
targets. Strafing left is the mirror of that.

### Why a leg waits for all four wheels

The heading correction is subtracted from the left wheels and added to the right
ones, so on every leg two wheels are capped faster than the other two even
though all four carry tick targets of the same size. The slow pair therefore
arrives last. If the leg stopped at the *first* arrival — the way the tank
version does, where it does not matter much — the slow pair would be abandoned
part-way and the robot would end the leg both short and skewed. That error would
then be measured into `STRAFE_MULTIPLIER` and the calibration would never
repeat. Hence the leg waits for all four, with the leg timeout as the backstop.

### Why the per-wheel signs work in both directions

`RUN_TO_POSITION` ignores the **sign** of the power and only uses its size, so
the direction of each wheel comes from the sign of its tick target and the speed
comes from `|power|`. That is why the heading correction is added to a base
power that already carries the target's sign: it automatically flips for
backward and for strafe-left legs, the same way the tank version flips its
correction when `inches < 0`.
