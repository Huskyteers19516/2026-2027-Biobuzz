# Swerve Module Testers

Both caster OpModes make the swerve module point where the left stick points and
then drive. Push the stick further to go faster.

| Name on the Driver Station | Steering hardware | Use it when |
| --- | --- | --- |
| **Single Swerve Module** | **CR servo *or* position servo**, encoder optional | Testing the module by itself. It detects which servo you have at INIT |
| **Swerve Drive Testing (1 module)** | **CR servo + analog encoder**, both required | The module is mounted on the tank test robot. Adds the 4 tank motors |
| **Swerve Servo Bench Test** | **CR servo + analog encoder**, both required | Poking the servo open loop with no caster logic at all |

Step-by-step guide for Single Swerve Module:
[SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md)

> **Before you build:** `:Teamtestcode:assembleDebug` currently fails with
> `class Meet0Auto is public, should be declared in a file named Meet0Auto.java`
> in `OpmodeForNewMembers/Meet0TeleOp.java`. That is the **only** error and it has
> nothing to do with the swerve code — move `Meet0Auto` into its own
> `Meet0Auto.java` and the build goes through.

---

## The important difference

**Single Swerve Module adapts to your configuration at runtime. The other two do
not.**

Single Swerve Module looks `swerve_servo` up with `hardwareMap.tryGet`, which
returns `null` instead of throwing. It tries `CRServo` first, then `Servo`, and
picks a control scheme to match. The encoder and the drive motor are looked up
the same way, so a missing device is a line of telemetry rather than a failed
INIT.

| `swerve_servo` is | `swerve_encoder` | Single Swerve Module does |
| --- | --- | --- |
| Continuous Rotation Servo | present | Closed-loop caster: PD + friction loop on the measured angle |
| Continuous Rotation Servo | missing | Open-loop CR steer: stick x is raw servo power, stick y drives. Says on screen that heading cannot be held |
| Servo (position) | present | Open-loop caster: commanded angle to servo position, with the measured angle shown beside it as a check |
| Servo (position) | missing | Same, with nothing to check it against |
| neither | either | Commands nothing, and says what it looked for |

Swerve Drive Testing and Swerve Servo Bench Test both use `hardwareMap.get`, so
they **fail at INIT** with *Unable to find a hardware device with name
"swerve_servo" and type CRServo* if the servo is configured as a plain Servo.
That is not a bug, it is just the older behaviour - if you get it, run Single
Swerve Module instead and read its INIT screen to find out what you really have.

---

## Configuration

| Config name | Device | Single Swerve Module | Swerve Drive Testing | Bench Test |
| --- | --- | --- | --- | --- |
| `swerve_motor` | DC Motor | optional | required | required |
| `swerve_servo` | **CR Servo or Servo** | either, detected | **CR Servo only** | **CR Servo only** |
| `swerve_encoder` | **Analog Input** | optional | required | required |
| `left_front`, `left_back`, `right_front`, `right_back` | DC Motor | not used | required | not used |

One saved configuration serves all three, as long as it has the four tank motors
in it and `swerve_servo` is a **Continuous Rotation Servo**. Single Swerve Module
simply ignores the tank motors.

The steering servo is meant to be an **Axon MINI in continuous-rotation mode**,
with its three-wire cable in a servo port and its separate analog feedback wire
in an analog port. Both have to be plugged in for closed-loop steering.

What goes wrong:

| Mistake | Symptom |
| --- | --- |
| `swerve_servo` configured as a plain **Servo** | Single Swerve Module says `Detected swerve_servo: Servo (position mode)` at INIT and runs open loop. The other two fail INIT with *type CRServo* |
| No `swerve_encoder` configured | Single Swerve Module falls back to open loop and says so. The other two fail INIT |
| Feedback wire not plugged in, or in the wrong analog port | INIT looks fine and `Raw encoder` never changes. In closed-loop mode the stall watchdog usually fires a couple of seconds after you push the stick - but **not** if the frozen reading happens to sit within about 17 degrees of where you are pushing. Always do the by-hand check at INIT |
| Running Swerve Drive Testing with no tank motors configured | INIT fails: *Could not find a hardware device*. Use Single Swerve Module on a bare module instead |

---

## Per-file notes

### Single Swerve Module - `SingleSwerveTeleOp.java`

Dual-mode caster. The mode is decided at INIT from what is actually configured,
reported on the INIT screen, and never guessed.

**Closed-loop mode (CR servo + encoder)**

- A PD + static-friction loop drives the CR servo against the absolute analog
  encoder, so the module knows its real angle at power-on.
- The commanded angle is folded to the **shorter path from the measured angle**,
  and the drive motor reverses when folded.
- That flip is **sticky**: the error has to pass 105 degrees before the module
  swaps sides (`FOLD_HYSTERESIS_DEGREES`). Without it, noise at exactly 90
  degrees of error would make the module slam 180 degrees back and forth every
  loop.
- Drive power is scaled by the cosine of the steering error, clipped to [0, 1].
- The damping term is taken on the **measured** angle, not on the error, so
  moving the stick or flipping sides does not throw a one-loop full-power spike
  at the servo.
- Releasing the stick stops the motor and **holds** the last angle
  (`HOLD_ANGLE_ON_RELEASE`).
- A **stall watchdog** cuts steering and drive, and shows
  `!! STALLED / NO FEEDBACK`, when steering is commanded above
  `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without `Error` shrinking by
  `STALL_MOVE_DEGREES`. It watches progress rather than raw movement, so a
  jittering floating input on a broken feedback wire still trips it, and so does
  a runaway.

**Position mode (plain Servo)**

- The commanded module angle is mapped straight to a servo position with
  `SERVO_CENTER` and `SERVO_TRAVEL_DEGREES`. There is no loop.
- The target is folded to **+/-105 degrees of the mechanical zero** (90 plus
  `FOLD_HYSTERESIS_DEGREES`), so limited servo travel is enough to reach every
  direction; the drive motor reverses when folded. Budget **210 degrees** of
  module rotation, not 180, before narrowing `SERVO_MIN_POSITION` /
  `SERVO_MAX_POSITION`.
- Drive power is **ramped** - cut back proportionally on a one-loop
  commanded-angle jump over `SERVO_SETTLE_DEGREES` (10 degrees), zeroed outright
  on a fold flip, and slewed back up at `DRIVE_RAMP_PER_SECOND` - because a
  position servo takes real time to swing. An ordinary sweep of the stick does
  not interrupt it.
- With an encoder present the screen shows `Commanded vs measured` and
  `Implied travel`, the measured degrees per full position span, so
  `SERVO_TRAVEL_DEGREES` can be corrected from the robot instead of guessed.
- `Servo position` is marked `(CLIPPED - out of travel)` when the fold could not
  keep the angle inside the servo's range.

**CR servo with no encoder**

- Closed loop is impossible, so it falls back to direct steering - stick x is
  servo power, stick y is drive power - and says on screen that feedback is
  missing so heading cannot be held. It does not silently do nothing.

**In every mode**

- **Hold LB** for manual override: stick x steers directly with no feedback at
  all (CR: power; position: offset from centre), drive motor off. This is the
  test that proves the module can physically turn.
- **Y** homes the module to 0 degrees and stops there, and homing also runs once
  at START (`HOME_ON_START`). Closed loop homes against the encoder with a 4 s
  timeout that reports how far off it ended; position mode just commands centre
  and reports the measured error if there is an encoder.
- **Hold A** for calibration: drive motor off, live dpad trim (encoder zero in CR
  mode, servo centre in position mode) and raw encoder, volts, volts-seen min/max
  and a per-second `Noise p-p` readout so a real encoder fault can be told apart
  from the +/-180 wrap.
- Nothing is commanded during INIT, so the module can safely be turned by hand
  before START.
- Anything the detected configuration cannot do is stated on the Driver Station
  in one short line rather than silently skipped.

Full setup, decision tree and troubleshooting:
[SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md).

### Swerve Drive Testing (1 module) - `swerveDriveTesting.java`

Closed-loop steering, plus the 4 tank motors from the right stick. **CR servo and
encoder both required** - no runtime detection.

- Same fold to the shorter path from the measured angle, but **without**
  hysteresis - it flips at exactly 90 degrees of error, so it can chatter with
  the stick held near sideways.
- Same cosine drive scaling (`SCALE_DRIVE_BY_ERROR`).
- Releasing the stick cuts power to both the motor and the servo - the module
  goes limp and can be turned by hand. It does not hold its angle.
- No `ENCODER_REVERSED`, no configurable analog range (it assumes 0 V to the hub
  maximum), no stall watchdog, no homing, no manual override.
- Calibration (hold **A**) cuts all power including the tank motors; dpad
  left/right nudges the encoder zero by 0.5 degrees.
- Key constants: `ENCODER_OFFSET_DEGREES`, `STEER_REVERSED`, `STEER_KP`,
  `STEER_KD`, `STEER_KS`, `STEER_TOLERANCE_DEGREES`, `STEER_MAX_POWER`,
  `SCALE_DRIVE_BY_ERROR`, `TANK_POWER_SCALE`.

Note it has **no** `DRIVE_REVERSED` constant - flipping the drive direction there
is a code change in `init()`, unlike Single Swerve Module.

### Swerve Servo Bench Test - `SwerveServoBenchTest.java`

Pure open-loop diagnostic, no PID and no caster logic. **CR servo and encoder
both required.**

- **A** hand mode (all power off), **B** / **X** drive the servo at a stepped
  test power, dpad up/down changes that power, the left stick steers directly,
  and the right trigger runs the drive motor.
- Reports `Moved since press`, so you can see whether the encoder moves at all
  while the servo is powered.

Use Single Swerve Module's **LB** manual override for the same test when the
servo turns out to be in position mode.

The files keep **separate copies** of their constants. They mean the same things,
so a zero or a set of gains measured on one is worth carrying across by hand -
but nothing is shared in code, and Single Swerve Module has several constants the
others do not.

---

## `ENCODER_REVERSED` vs `STEER_REVERSED`

Two different faults that look similar for about one second, and only Single
Swerve Module can fix both.

| What you see | Cause | Fix |
| --- | --- | --- |
| The module settles neatly, but on the **mirrored** heading - stick right points the wheel left | The encoder counts the wrong way | `ENCODER_REVERSED = true` |
| The module **runs away** - `Error` grows and it keeps spinning | The servo pushes away from the target | `STEER_REVERSED = true` |

Flipping `STEER_REVERSED` on a reversed encoder makes the loop converge again, so
it looks like a fix, but it converges on the mirrored heading. Test the encoder
direction first, by hand, with power off: **hold A, turn the wheel right,
`Module angle` must rise.**

---

## Controls

| Input | Single Swerve Module | Swerve Drive Testing |
| --- | --- | --- |
| Left stick | Aim and drive the module | Aim and drive the module |
| Right stick up/down | - | Tank robot forward / backward |
| Release left stick | Motor off, the angle is **held** | Motor and servo both off, module goes limp |
| Hold **LB** | **Manual steer, no feedback.** Drive motor off | - |
| Press **Y** | Home the module to forward | - |
| Hold **A** | Calibration: drive off, trim live | Calibration: everything off (tank too) |
| Dpad left/right (while holding A) | Encoder zero by 0.5 degrees, or servo centre by 0.005 | Nudge the encoder zero by 0.5 degrees |

In Single Swerve Module's CR-without-encoder fallback the left stick is not a
caster: x is steering power and y is drive power. The screen says so.

## What the telemetry means

Angles: **0 = straight forward, positive = to the right.** `Module angle`,
`Target angle`, `Held angle` and `Error` run -180 to +180; `Raw encoder` and
`Offset` run 0 to 360.

| Line | Which OpMode | Meaning |
| --- | --- | --- |
| Detected swerve_servo | Single Swerve Module | `CRServo`, `Servo (position mode)` or `NOT FOUND`. Read this first |
| Mode | Single Swerve Module | Which of the four control schemes is running |
| Module angle | both | Where the module actually points right now |
| Target angle | both | Where it is trying to point. In position mode this is **commanded**, not measured |
| Held angle | Single Swerve Module | The angle being held while the stick is released |
| Error | closed loop | Target minus actual. Should shrink toward 0 |
| Steer power | CR modes | Power sent to the CR servo |
| Servo position | Single Swerve Module, position mode | The 0 to 1 position sent. May say `(CLIPPED - out of travel)` |
| Servo centre | Single Swerve Module, position mode | `SERVO_CENTER` plus dpad trim |
| Commanded vs measured | Single Swerve Module, position mode with encoder | The two angles side by side. This is how you see whether the servo got there |
| Implied travel | Single Swerve Module, position mode with encoder | Measured degrees per full position span - the number to put in `SERVO_TRAVEL_DEGREES`. Always positive; a warning line appears if the encoder counts backwards |
| Raw encoder | both | Encoder reading before the zero offset, in 0 to 360. Copy it into `ENCODER_OFFSET_DEGREES` |
| Offset | both | Current encoder zero offset, in the same 0 to 360 range |
| Encoder volts | Single Swerve Module | The raw analog voltage right now |
| Volts seen | Single Swerve Module | Lowest and highest voltage since the OpMode started |
| Noise p-p | Single Swerve Module | Peak-to-peak voltage over the last second, in volts and degrees. Tells a real encoder fault from the +/-180 wrap |
| Analog range | Single Swerve Module | The voltage range in use, and the hub's maximum for comparison |
| Encoder reversed | Single Swerve Module | The value of `ENCODER_REVERSED` |
| Homing | Single Swerve Module | Result of the last home, including `TIMEOUT, still N deg off` |
| `!! STALLED / NO FEEDBACK` | Single Swerve Module, closed loop | Steering was commanded but the error stopped shrinking. Power is cut |
| Drive power | both | Power sent to the wheel motor. `(reversed)` is normal |
| Tank power | Swerve Drive Testing | Tank drive output |

---

## First-time setup

**Lift the module off the ground first**, for any of these OpModes.

The order matters: each step assumes the ones above it are already right.

1. **Find out what you have.** Run Single Swerve Module, press INIT and read
   `Detected swerve_servo` and `Mode`. Do not argue with this line - it is what
   the Driver Station configuration actually says.
2. **Prove the module can turn.** START, hold **LB**, push the left stick fully
   left and right. The wheel must physically turn. If it does not, the fault is
   mechanical or electrical and no code change will help.
3. **Check the encoder is alive.** Turn the module by hand and watch
   `Raw encoder`. You can do this during INIT, before anything is powered. If the
   number never changes, check the feedback wire, the analog port, and that
   `swerve_encoder` is configured as Analog Input. Watch `Noise p-p` too: several
   tenths of a volt while the module is still means a floating input.
4. **Check the encoder direction.** Hold **A** (power off) and turn the wheel to
   the right by hand. `Module angle` must rise. If it falls, set
   `ENCODER_REVERSED = true` and reinstall. Do this before the zero - it changes
   what every reading means.
5. **Set the zero.** CR mode: point the wheel straight forward, hold **A**, read
   `Raw encoder`, put that number into `ENCODER_OFFSET_DEGREES`. Position mode:
   hold **A** and trim with dpad until the wheel is straight, then copy
   `Servo centre` into `SERVO_CENTER`. Trims only last until the OpMode stops -
   always copy the final value into the file.
6. **Check the steering direction.** *(closed loop only.)* Release A and LB, push
   the left stick a little. `Error` must get smaller. **If the module keeps
   spinning or the error grows, press STOP immediately** and set
   `STEER_REVERSED = true`.
7. **Fix the travel.** *(position mode only.)* Push the stick to one side, read
   `Implied travel`, put it into `SERVO_TRAVEL_DEGREES`, reinstall. The figure is
   always printed positive, so copy it as-is; never enter a negative or zero
   value. If the screen adds "Encoder counts the OTHER way from the servo", set
   `ENCODER_REVERSED = true` too.
8. **Check the drive direction.** Point the stick forward. If the wheel faces
   forward but rolls backward: on Single Swerve Module set
   `DRIVE_REVERSED = true`; on Swerve Drive Testing flip
   `driveMotor.setDirection` in `init()`.
9. **Tune the PD loop.** *(closed loop only.)* See below.
10. **Swerve Drive Testing only: check the tank drive.** Push the right stick
    forward. If the robot goes backward or spins, the tank motor directions need
    flipping in `init()`.

Single Swerve Module also lets you measure the real analog range while you are in
calibration mode - turn the module through a full turn and read `Volts seen`. Do
that between steps 5 and 6, and redo the zero afterwards if you change the
constants.

## Tuning the steering

Single Swerve Module's closed-loop mode and Swerve Drive Testing use the same
loop and the same gain names, so tune on one and copy the numbers to the other.
Position mode has no gains to tune - it has `SERVO_CENTER` and
`SERVO_TRAVEL_DEGREES` instead.

Start with `STEER_KD = 0` and `STEER_KS = 0`, and lower `STEER_MAX_POWER` while
you work.

| What you see | Change |
| --- | --- |
| Turns slowly or stops short of the target | Raise `STEER_KP` |
| Overshoots and wobbles back and forth | Lower `STEER_KP`, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Buzzes or twitches when it is already pointed right | Raise `STEER_TOLERANCE_DEGREES` |
| Too fast to watch while tuning | Lower `STEER_MAX_POWER` |

## Things that look wrong but are normal

- **The module drives backward sometimes.** Both caster OpModes point the wheel
  the opposite way and reverse the motor rather than make a turn of more than 90
  degrees. Telemetry shows `(reversed)`.
- **The wheel speeds up gradually after a big turn.** Closed loop fades power in
  with the cosine of the steering error; position mode ramps it after every
  swing. Both stop the wheel scrubbing sideways.
- **Single Swerve Module parks up to 15 degrees off near sideways.** That is the
  fold hysteresis, and it is deliberate - without it the module would flap
  between pointing left and pointing right. Swerve Drive Testing has no
  hysteresis, which is why it can chatter there.
- **The wheel does not spin while the module is still more than 90 degrees from
  the target.** The cosine scaling is clipped at 0.
- **Swerve Drive Testing goes limp when you let go of the stick.** Power is cut on
  purpose so nothing buzzes and you can turn it by hand.
- **Single Swerve Module does *not* go limp.** Closed loop keeps holding the last
  angle, and a position servo holds by itself - in position mode even holding
  **A** only cuts the drive motor, because a position servo cannot be made limp
  from the OpMode. The screen says so. Unplug it to turn the module by hand.
- **`Module angle` flickers between +179 and -180.** That is the +/-180 wrap
  sitting wherever the wheel happens to point because the zero has not been set.
  It is not an encoder fault.
- **Steer power reads 0.00 while the module is pointed correctly.** Inside
  `STEER_TOLERANCE_DEGREES` the servo is switched off on purpose.

## Warning for the tank test robot

If the swerve wheel is on the ground and pointing sideways, driving the tank
forward drags the swerve wheel sideways. That wears the wheel and loads the
steering. Either keep the module lifted, or do not drive the tank hard while the
module is turned.
