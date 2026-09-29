# Swerve Module Testers

Three OpModes for the same hardware. Two of them are casters: push the left stick
in a direction, the module points that way and drives. Push further to go faster.

| Name on the Driver Station | What it is for |
| --- | --- |
| **Single Swerve Module** | Testing one module by itself. Closed-loop caster, homing, manual override, calibration, ratio measurement, stall watchdog |
| **Swerve Drive Testing (1 module)** | The module mounted on the tank test robot. Same closed-loop steering plus the 4 tank motors |
| **Swerve Servo Bench Test** | Poking the servo open loop with no caster logic at all |

Step-by-step guide for Single Swerve Module:
[SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md)

---

## The servo is geared down to the module

The Axon MINI's analog feedback wire reports the **servo shaft** angle, and the
module is geared down from the servo: turning the module about 10 degrees moves
the raw reading about 80 degrees, so this module runs at roughly **8 servo
degrees per module degree**.

**Only Single Swerve Module knows about this.** It has a `STEERING_RATIO`
constant, it reports and controls everything in **module** degrees, and it
recovers absolute position by unwrapping the servo reading rather than trusting
one sample - with a ratio of 8 the servo wraps 8 times per module turn, so a
single analog reading cannot say which sector the module is in.

`swerveDriveTesting.java` and `SwerveServoBenchTest.java` still assume **1:1**.
Their angles are servo angles wearing a module label, and their gains and zeros
do **not** transfer to or from Single Swerve Module any more.

---

## Configuration

**All three OpModes need a continuous-rotation servo and an analog encoder.**
There is no runtime detection and no fallback in any of them.

| Config name | Device type | Single Swerve Module | Swerve Drive Testing | Bench Test |
| --- | --- | --- | --- | --- |
| `swerve_motor` | DC Motor | required | required | required |
| `swerve_servo` | **Continuous Rotation Servo** | required | required | required |
| `swerve_encoder` | **Analog Input** | required | required | required |
| `left_front`, `left_back`, `right_front`, `right_back` | DC Motor | not used | required | not used |

One saved configuration serves all three, as long as it has the four tank motors
in it. Single Swerve Module simply ignores them.

The steering servo is meant to be an **Axon MINI in continuous-rotation mode**,
with its three-wire cable in a servo port and its separate analog feedback wire
in an analog port. Both have to be plugged in.

Every device is looked up with `hardwareMap.get`, so a wrong name or a wrong
device type **fails at INIT** with the SDK's own message rather than misbehaving
later:

> `Unable to find a hardware device with name "swerve_servo" and type CRServo`

What goes wrong:

| Mistake | Symptom |
| --- | --- |
| `swerve_servo` configured as a plain **Servo** | INIT fails naming `CRServo`. Change the device type in the configuration |
| No `swerve_encoder` configured, or the wrong name | INIT fails naming `AnalogInput` |
| Feedback wire not plugged in, or in the wrong analog port | INIT looks fine and `Servo raw` never changes. In Single Swerve Module the stall watchdog usually fires a couple of seconds after you push the stick - but **not** if the frozen reading happens to sit within about 8 module degrees of where you are pushing. Always do the by-hand check at INIT |
| Running Swerve Drive Testing with no tank motors configured | INIT fails: *Could not find a hardware device*. Use Single Swerve Module on a bare module instead |

---

## Per-file notes

### Single Swerve Module - `SingleSwerveTeleOp.java`

Closed-loop caster on a CR servo and the analog feedback, **with gear-ratio
support**. One control path, no modes.

- `STEERING_RATIO` (default 8.0, measure your own) is servo degrees per module
  degree. Everything the driver sees and everything the loop controls is in
  **module** degrees; the encoder is the only thing that speaks servo degrees. A
  zero or negative ratio falls back to 1.00 and says so on every screen.
- **The angle is unwrapped, not absolute.** Each loop the code wraps the
  *difference* between this raw reading and the last one into -180…+180 and adds
  it to a running `Servo unwrapped` total, then
  `Module angle = normalize((Servo unwrapped - zero) / STEERING_RATIO)`. This
  survives the raw 0/360 seam in both directions and does not drift on a noisy
  encoder, because a sum of differences cancels its own noise. It assumes the
  servo moves **less than 180 servo degrees between two loops**. Two telemetry
  tests watch that assumption: `Slowest loop`, which warns when a loop ran long
  enough that a full-speed servo could have crossed 180 degrees inside it, and
  `Biggest wrapped servo step`, which warns above 90 but is measured **after**
  wrapping and so cannot see a step near a whole servo turn. Pressing **B**
  clears both counters.
- **The zero is taken, not stored.** There is no `ENCODER_OFFSET_DEGREES` any
  more - it was deleted, because a stored absolute zero is meaningless when the
  servo wraps 8 times per module turn. **Point the wheel straight forward by hand
  before START**; START takes that position as 0. **B re-takes the zero at any
  time**, in any mode, including INIT.
- `Module angle` is normalized to -180…+180 and is what the loop controls;
  `Module continuous` is the unnormalized running value and may read 500 degrees.
- **Calibration mode measures the ratio**: hold **A**, press **X** to start,
  hand-turn the module by exactly `RATIO_REFERENCE_DEGREES` (90) against a
  square, and read `Servo moved` and `Implied ratio`. The screen states in
  capitals that the number is only true if you turned exactly that much.
- A PD + static-friction loop drives the CR servo against the encoder, so the
  module knows its real angle relative to the zero it was given.
- The commanded angle is folded to the **shorter path from the measured angle**,
  and the drive motor reverses when folded.
- That flip is **sticky**: the error has to pass 105 degrees before the module
  swaps sides (`FOLD_HYSTERESIS_DEGREES`). Without it, noise at exactly 90
  degrees of error would make the module slam 180 degrees back and forth every
  loop.
- Drive power is scaled by the cosine of the steering error, clipped to [0, 1].
- The damping term is taken on the **measured** angle, not on the error, so
  moving the stick or flipping sides does not throw a one-loop full-power spike
  at the servo. It is now a **module** angular velocity, about 8 times smaller
  than the old servo-level number, so `STEER_KD` has to grow with the ratio. The
  shipped 0.002 is a deliberately soft starting point, not the full 8 times.
- Releasing the stick stops the motor and **holds** the last angle. Hold **A** to
  make everything go limp.
- **Y** homes the module to 0 degrees and stops there, with a 6 s timeout that
  reports how far off it ended **in module degrees** - longer than before because
  a geared module covers module degrees 8 times slower than the servo turns.
  Homing also runs at START, but since the zero is taken at that same instant the
  module is already at 0, so **nothing moves at START**. Homing is under the same
  stall watchdog as the caster, so a home that runs away on a wrong
  `STEER_REVERSED` is cut after 2 s and reported as `STALLED`. On a timeout or a
  stall the module holds where it stopped instead of fighting on.
- **Hold LB** for manual override: the **right** stick x is raw servo power with
  no loop and no watchdog, drive motor off. This is the test that proves the
  module can physically turn. It is on the right stick so that releasing **LB**
  with it still pushed over cannot hand the caster - which reads the left stick -
  a target nobody asked for.
- **Hold A** for calibration: everything off, dpad left/right trims the zero by
  0.5 **module** degrees, **X** restarts the ratio measurement, and the screen
  shows the ratio readout, raw and unwrapped servo angle, volts, volts-seen
  min/max and a per-second `Noise p-p` readout. `Noise p-p` is an encoder-level
  figure, so it is given in **servo** degrees with the module equivalent
  (servo / ratio) beside it.
- A **stall watchdog** cuts steering and drive, and shows
  `!! STALLED / NO FEEDBACK`, when steering is commanded above
  `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without `Error` shrinking by
  `STALL_MOVE_DEGREES` - now 1.0 **module** degree, which is 8 servo degrees of
  real travel. `STALL_POWER_THRESHOLD` is 0.45, so with the shipped gains it only
  watches while the error is about 8 module degrees or more; it has to be raised
  alongside `STEER_KP` or it starts cutting the drive motor for ordinary parking
  errors. It watches progress rather than raw movement, so a
  jittering floating input on a broken feedback wire still trips it, and so does
  a runaway. A new stick direction clears the latch, and so does holding **A**,
  pressing **Y** or pressing **B**.
- Nothing is commanded during INIT - the servo port is not even energised - so
  the module can safely be turned by hand before START, which is exactly what you
  are asked to do: **point the wheel forward by hand first**. Hold **A** across
  START on a first run, before the ratio and the direction constants are
  confirmed.
- **A** and **LB** win over **Y** and the dpad, and a press made while either is
  held is discarded rather than stored, so nothing fires when the button is
  released.
- Leaving calibration or manual override, and the end of homing, all leave the
  module holding the angle it is actually at, so no button release slams the
  servo. None of them touch the zero or the unwrapped total, which keep counting
  through every mode, so a hand-turn while the power is off is still tracked.

Full setup, first run, telemetry and troubleshooting:
[SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md).

### Swerve Drive Testing (1 module) - `swerveDriveTesting.java`

Closed-loop steering, plus the 4 tank motors from the right stick.

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

It also has **no gear-ratio support**: it still reads the analog encoder as an
absolute 1:1 module angle through `ENCODER_OFFSET_DEGREES`. On a geared module
its `Module angle` is really a servo angle, its target is reached 8 times too
early, and its `ENCODER_OFFSET_DEGREES` picks one of 8 possible headings.

### Swerve Servo Bench Test - `SwerveServoBenchTest.java`

Pure open-loop diagnostic, no PID and no caster logic.

- **A** hand mode (all power off), **B** / **X** drive the servo at a stepped
  test power, dpad up/down changes that power, the left stick steers directly,
  and the right trigger runs the drive motor.
- Reports `Moved since press`, so you can see whether the encoder moves at all
  while the servo is powered.

Single Swerve Module's **LB** manual override does the same job without leaving
the caster OpMode. The bench test's `Moved since press` is in **servo** degrees;
divide by the ratio for module degrees.

The files keep **separate copies** of their constants, and since the gear ratio
went into Single Swerve Module they no longer **mean** the same things. Its
angles, tolerances and gains are per **module** degree; the other two are per
**servo** degree. Do not copy `STEER_KP`, `STEER_KD`, `STEER_TOLERANCE_DEGREES`
or a zero between them without multiplying or dividing by `STEERING_RATIO`, and
`ENCODER_OFFSET_DEGREES` does not exist in Single Swerve Module at all.

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
| Right stick left/right | Raw servo power **while LB is held** | - |
| Release left stick | Motor off, the angle is **held** | Motor and servo both off, module goes limp |
| Hold **LB** | **Manual steer on the right stick x, raw servo power, no feedback.** Drive motor off | - |
| Press **Y** | Home the module to forward | - |
| Press **B** | **Re-zero: the wheel's current heading becomes 0.** Works in every mode, including INIT | - |
| Hold **A** | Calibration and ratio measurement: all power off, trim live | Calibration: everything off (tank too) |
| Press **X** (while holding A) | Restart the ratio measurement from here | - |
| Dpad left/right (while holding A) | Nudge the zero by 0.5 **module** degrees | Nudge the encoder zero by 0.5 servo degrees |

## What the telemetry means

Angles: **0 = straight forward, positive = to the right.**

In Single Swerve Module, `Module angle`, `Target angle`, `Held angle` and `Error`
are **module** degrees running -180 to +180; `Servo raw` is 0 to 360 and wraps
several times per module turn; `Servo unwrapped`, the zero and `Module
continuous` are unbounded running values and are meant to be large. In Swerve
Drive Testing every angle is a servo angle labelled as a module angle, because it
has no ratio.

| Line | Which OpMode | Meaning |
| --- | --- | --- |
| Steering ratio | Single Swerve Module | The `STEERING_RATIO` in use, or a warning if it is not positive |
| Module angle | both | Where the module actually points right now. Module degrees in Single Swerve Module |
| Module continuous | Single Swerve Module | The same angle before normalizing. 400 means once round and a bit |
| Target angle | both | Where it is trying to point |
| Held angle | Single Swerve Module | The angle being held while the stick is released |
| Error | both | Target minus actual. Should shrink toward 0. Single Swerve Module also shows the servo travel it implies |
| Module speed | Single Swerve Module | Measured angular velocity in module degrees per second. This is what `STEER_KD` multiplies |
| Steer power | both | Power sent to the CR servo |
| Servo raw | Single Swerve Module | The encoder reading itself, 0 to 360, wrapping. Nothing to copy into a constant any more |
| Servo unwrapped | Single Swerve Module | The running servo total, with the zero it is measured from |
| Raw encoder / Offset | Swerve Drive Testing | Encoder reading before the zero offset, and the offset, both 0 to 360. Copy into `ENCODER_OFFSET_DEGREES` |
| Servo moved / Implied ratio | Single Swerve Module | On the calibration screen: the ratio measurement. Only true if you just hand-turned the module by exactly `RATIO_REFERENCE_DEGREES` |
| Encoder volts | Single Swerve Module | The raw analog voltage right now |
| Volts seen | Single Swerve Module | Lowest and highest voltage since the OpMode started |
| Noise p-p | Single Swerve Module | Peak-to-peak voltage over about the last second, in volts and **servo** degrees, with the module equivalent beside it. It shows the spread so far while the first window is still filling, so it is honest from the first INIT loop |
| Biggest wrapped servo step | Single Swerve Module | Largest single-loop servo step **after wrapping**, since the last zero. It can never read above 180, so a true step near a whole servo turn hides here |
| Slowest loop | Single Swerve Module | Longest single loop since the last zero, next to the loop time above which a full-speed servo could alias the unwrapping. This is the reliable alias signal |
| `!! Unwrap may have aliased` | Single Swerve Module | A wrapped step passed 90 servo degrees, or a loop ran slow enough to alias. The angle may have skipped 360/ratio. Point forward and press **B**, which also clears both counters |
| Analog range in use | Single Swerve Module | The voltage range in use, for comparison with `Volts seen` |
| Encoder reversed | Single Swerve Module | The value of `ENCODER_REVERSED` |
| Homing / Status | Single Swerve Module | Result of the last home: `done in N s`, `TIMEOUT, still N module deg off`, `STALLED N module deg off - check STEER_REVERSED`, `cancelled`, or `cancelled by re-zero`. `Status` is the same string on the homing screen |
| Error to forward | Single Swerve Module | On the homing screen: how far the module still is from 0, in module degrees |
| Servo power | Single Swerve Module | On the LB screen: the raw power going to the CR servo |
| `!! STALLED / NO FEEDBACK` | Single Swerve Module | Steering was commanded but the error stopped shrinking. Power is cut |
| Drive power | both | Power sent to the wheel motor. `(reversed)` is normal |
| Tank power | Swerve Drive Testing | Tank drive output |

---

## First-time setup

**Lift the module off the ground first**, for any of these OpModes.

The order matters: each step assumes the ones above it are already right. This is
the order for **Single Swerve Module**; it changed when the gear ratio went in.

0. **Point the wheel straight forward by hand, then press START.** The zero is
   taken at START from wherever the wheel is pointing, so this is now part of
   starting the OpMode. On a first run **hold A while you press START** and keep
   holding it - that cuts all power while you do steps 1 to 4.
1. **Check the encoder is alive.** At INIT, or holding **A** after START, turn
   the module by hand. `Servo raw` must change. At a ratio of 8 it races round
   and wraps several times for a small module turn: that is the gearing, not a
   fault. If it never moves, check the feedback wire, the analog port, and that
   `swerve_encoder` is configured as Analog Input. Watch `Noise p-p` too: several
   tenths of a volt while the module is still means a floating input. While you
   are here, read `Volts seen` and fix `ANALOG_MIN_VOLTAGE` /
   `ANALOG_MAX_VOLTAGE` if the swing is not about 0.00 - 3.30 V.
2. **Measure `STEERING_RATIO`.** Still holding **A**: square the wheel up, press
   **X**, hand-turn the module by **exactly 90 degrees** against a square, and
   read `Implied ratio`. Repeat it a few times, put the number into
   `STEERING_RATIO` and reinstall. Get this wrong and every error, tolerance and
   gain is wrong by the same factor - that is what makes the module twitch,
   overshoot and get pulled back.
3. **Check the encoder direction.** Still holding **A**, turn the wheel to the
   right by hand: `Module angle` must rise. If it falls, set
   `ENCODER_REVERSED = true` and reinstall. Do this before the zero - it changes
   what every reading means.
4. **Set the zero.** Still holding **A**, point the wheel straight forward by
   hand and **press B** - **B** works in every mode, so there is no need to let
   go and let the loop fight your hand. `Module angle` must read about 0. Dpad
   left/right, still holding **A**, trims the last degree. There is nothing to copy into the file: the zero is taken
   fresh every run, and **B** re-takes it at any time.
5. **Check the steering direction.** Release A and LB, push the left stick a
   little. `Error` must get smaller. **If the module keeps spinning or the error
   grows, press STOP immediately** and set `STEER_REVERSED = true`. The stall
   watchdog cuts it after about 2 s by itself, but STOP is faster.
6. **Check the drive direction.** Point the stick forward. If the wheel faces
   forward but rolls backward: on Single Swerve Module set
   `DRIVE_REVERSED = true`; on Swerve Drive Testing flip
   `driveMotor.setDirection` in `init()`.
7. **Tune the PD loop.** See below. **The gains must be retuned** after the ratio
   is set - the shipped defaults are a starting point, not a setting.
8. **Swerve Drive Testing only: check the tank drive.** Push the right stick
   forward. If the robot goes backward or spins, the tank motor directions need
   flipping in `init()`. That OpMode has no ratio support, so skip steps 2 and 4
   there and use `ENCODER_OFFSET_DEGREES` as before.

**If the module never turns at all**, none of the above applies yet. Hold **LB**
in Single Swerve Module and push the right stick fully over - that is raw servo power
with no loop and no encoder in the path. If the wheel still does not move, the
fault is mechanical or electrical and no code change will help. If it does move
but `Module angle` stays put, the feedback path is dead. The full decision tree is
in [SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md).

Single Swerve Module also lets you measure the real analog range while you are in
calibration mode - turn the module slowly and read `Volts seen`. Do that during
step 1, before the ratio, because a wrong voltage span scales every angle
including the ratio you are about to measure.

## Tuning the steering

Single Swerve Module and Swerve Drive Testing use the same loop and the same gain
names, but **no longer the same units**: Single Swerve Module's error is in
module degrees and Swerve Drive Testing's is in servo degrees. Numbers do not
copy across without multiplying or dividing by `STEERING_RATIO`.

**Retune Single Swerve Module after setting `STEERING_RATIO`.** An error of 90
module degrees is 720 servo degrees of travel at a ratio of 8, so `STEER_KP`
means something completely different from the old 1:1 code. Its defaults were
rescaled for a ratio of 8 (`STEER_KP` 0.05, `STEER_KD` 0.002,
`STEER_TOLERANCE_DEGREES` 1.0, `STEER_MAX_POWER` 0.6) and are a starting point
only. Rules of thumb: `STEER_KP` and `STEER_KD` scale **with** the ratio,
`STEER_KS` does not scale at all, and `STEER_TOLERANCE_DEGREES` can get tighter
as the ratio grows.

Start with `STEER_KD = 0` and `STEER_KS = 0`, and lower `STEER_MAX_POWER` while
you work.

| What you see | Change |
| --- | --- |
| Turns slowly or stops short of the target | Raise `STEER_KP` |
| Overshoots and wobbles back and forth | Lower `STEER_KP`, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Buzzes or twitches when it is already pointed right | Raise `STEER_TOLERANCE_DEGREES` |
| Too fast to watch while tuning | Lower `STEER_MAX_POWER` |
| Wrong by a consistent *factor*, both ways | Not a gain. Re-measure `STEERING_RATIO` |

## Things that look wrong but are normal

- **`Servo raw` races round and wraps several times for a small module turn.**
  That is the gear ratio, and it is exactly why Single Swerve Module unwraps the
  reading instead of treating it as an absolute module position.
- **Nothing moves at START in Single Swerve Module.** The zero is taken at START
  from wherever the wheel points, so the home that follows is already finished.
  Press **Y** after turning the module away to watch a real home.
- **`Module continuous` keeps growing.** It is a running total and is allowed to.
  `Module angle` is the normalized value the loop controls.
- **The module drives backward sometimes.** Both caster OpModes point the wheel
  the opposite way and reverse the motor rather than make a turn of more than 90
  degrees. Telemetry shows `(reversed)`.
- **The wheel speeds up gradually after a big turn.** Power fades in with the
  cosine of the steering error, so the wheel does not scrub sideways.
- **Single Swerve Module parks up to 15 degrees off near sideways.** That is the
  fold hysteresis, and it is deliberate - without it the module would flap
  between pointing left and pointing right. Swerve Drive Testing has no
  hysteresis, which is why it can chatter there.
- **The wheel does not spin while the module is still more than 90 degrees from
  the target.** The cosine scaling is clipped at 0.
- **Swerve Drive Testing goes limp when you let go of the stick.** Power is cut on
  purpose so nothing buzzes and you can turn it by hand.
- **Single Swerve Module does *not* go limp.** It keeps holding the last angle.
  Hold **A** to cut all power and turn it by hand.
- **`Module angle` flickers between +179 and -180.** That is the +/-180 wrap
  sitting wherever the wheel happens to point because the zero has not been set.
  It is not an encoder fault. In Single Swerve Module, point the wheel forward
  and press **B**.
- **Steer power reads 0.00 while the module is pointed correctly.** Inside
  `STEER_TOLERANCE_DEGREES` the servo is switched off on purpose.

## Warning for the tank test robot

If the swerve wheel is on the ground and pointing sideways, driving the tank
forward drags the swerve wheel sideways. That wears the wheel and loads the
steering. Either keep the module lifted, or do not drive the tank hard while the
module is turned.
