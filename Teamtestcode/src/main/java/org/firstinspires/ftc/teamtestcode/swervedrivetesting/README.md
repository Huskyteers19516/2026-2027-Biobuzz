# Swerve Module Testers

Three OpModes for the same hardware. Two of them are casters: push the left stick
in a direction, the module points that way and drives. Push further to go faster.

| Name on the Driver Station | What it is for |
| --- | --- |
| **Single Swerve Module** | Testing one module by itself. Closed-loop caster, homing, manual override, calibration, stall watchdog |
| **Swerve Drive Testing (1 module)** | The module mounted on the tank test robot. Same closed-loop steering plus the 4 tank motors |
| **Swerve Servo Bench Test** | Poking the servo open loop with no caster logic at all |

Step-by-step guide for Single Swerve Module:
[SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md)

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
| Feedback wire not plugged in, or in the wrong analog port | INIT looks fine and `Raw encoder` never changes. In Single Swerve Module the stall watchdog usually fires a couple of seconds after you push the stick - but **not** if the frozen reading happens to sit within about 17 degrees of where you are pushing. Always do the by-hand check at INIT |
| Running Swerve Drive Testing with no tank motors configured | INIT fails: *Could not find a hardware device*. Use Single Swerve Module on a bare module instead |

---

## Per-file notes

### Single Swerve Module - `SingleSwerveTeleOp.java`

Closed-loop caster on a CR servo and the absolute analog encoder. One control
path, no modes.

- A PD + static-friction loop drives the CR servo against the encoder, so the
  module knows its real angle at power-on.
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
- Releasing the stick stops the motor and **holds** the last angle. Hold **A** to
  make everything go limp.
- **Y** homes the module to 0 degrees and stops there, and homing also runs once
  at START, with a 4 s timeout that reports how far off it ended. Homing is under
  the same stall watchdog as the caster, so a home that runs away on a wrong
  `STEER_REVERSED` is cut after 2 s and reported as `STALLED`. On a timeout or a
  stall the module holds where it stopped instead of fighting on.
- **Hold LB** for manual override: the **right** stick x is raw servo power with
  no loop and no watchdog, drive motor off. This is the test that proves the
  module can physically turn. It is on the right stick so that releasing **LB**
  with it still pushed over cannot hand the caster - which reads the left stick -
  a target nobody asked for.
- **Hold A** for calibration: everything off, dpad left/right trims the encoder
  zero by 0.5 degrees, and the screen shows raw encoder, volts, volts-seen
  min/max and a per-second `Noise p-p` readout so a real encoder fault can be
  told apart from the +/-180 wrap.
- A **stall watchdog** cuts steering and drive, and shows
  `!! STALLED / NO FEEDBACK`, when steering is commanded above
  `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without `Error` shrinking by
  `STALL_MOVE_DEGREES`. It watches progress rather than raw movement, so a
  jittering floating input on a broken feedback wire still trips it, and so does
  a runaway. A new stick direction clears the latch, and so does holding **A** or
  pressing **Y**.
- Nothing is commanded during INIT - the servo port is not even energised - so
  the module can safely be turned by hand before START. The first thing that
  moves is the home at START, which runs under closed-loop power. Hold **A**
  across START on a first run, before the direction constants are confirmed.
- **A** and **LB** win over **Y** and the dpad, and a press made while either is
  held is discarded rather than stored, so nothing fires when the button is
  released.
- Leaving calibration or manual override, and the end of homing, all leave the
  module holding the angle it is actually at, so no button release slams the
  servo.

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

### Swerve Servo Bench Test - `SwerveServoBenchTest.java`

Pure open-loop diagnostic, no PID and no caster logic.

- **A** hand mode (all power off), **B** / **X** drive the servo at a stepped
  test power, dpad up/down changes that power, the left stick steers directly,
  and the right trigger runs the drive motor.
- Reports `Moved since press`, so you can see whether the encoder moves at all
  while the servo is powered.

Single Swerve Module's **LB** manual override does the same job without leaving
the caster OpMode.

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
| Right stick left/right | Raw servo power **while LB is held** | - |
| Release left stick | Motor off, the angle is **held** | Motor and servo both off, module goes limp |
| Hold **LB** | **Manual steer on the right stick x, raw servo power, no feedback.** Drive motor off | - |
| Press **Y** | Home the module to forward | - |
| Hold **A** | Calibration: all power off, trim live | Calibration: everything off (tank too) |
| Dpad left/right (while holding A) | Nudge the encoder zero by 0.5 degrees | Nudge the encoder zero by 0.5 degrees |

## What the telemetry means

Angles: **0 = straight forward, positive = to the right.** `Module angle`,
`Target angle`, `Held angle` and `Error` run -180 to +180; `Raw encoder` and
`Offset` run 0 to 360.

| Line | Which OpMode | Meaning |
| --- | --- | --- |
| Module angle | both | Where the module actually points right now |
| Target angle | both | Where it is trying to point |
| Held angle | Single Swerve Module | The angle being held while the stick is released |
| Error | both | Target minus actual. Should shrink toward 0 |
| Steer power | both | Power sent to the CR servo |
| Raw encoder | both | Encoder reading before the zero offset, in 0 to 360. Copy it into `ENCODER_OFFSET_DEGREES` |
| Offset | both | Current encoder zero offset, in the same 0 to 360 range |
| Encoder volts | Single Swerve Module | The raw analog voltage right now |
| Volts seen | Single Swerve Module | Lowest and highest voltage since the OpMode started |
| Noise p-p | Single Swerve Module | Peak-to-peak voltage over about the last second, in volts and degrees. It shows the spread so far while the first window is still filling, so it is honest from the first INIT loop. Tells a real encoder fault from the +/-180 wrap |
| Analog range in use | Single Swerve Module | The voltage range in use, for comparison with `Volts seen` |
| Encoder reversed | Single Swerve Module | The value of `ENCODER_REVERSED` |
| Homing / Status | Single Swerve Module | Result of the last home: `done in N s`, `TIMEOUT, still N deg off`, `STALLED N deg off - check STEER_REVERSED`, or `cancelled`. `Status` is the same string on the homing screen |
| Error to forward | Single Swerve Module | On the homing screen: how far the module still is from 0 degrees |
| Servo power | Single Swerve Module | On the LB screen: the raw power going to the CR servo |
| `!! STALLED / NO FEEDBACK` | Single Swerve Module | Steering was commanded but the error stopped shrinking. Power is cut |
| Drive power | both | Power sent to the wheel motor. `(reversed)` is normal |
| Tank power | Swerve Drive Testing | Tank drive output |

---

## First-time setup

**Lift the module off the ground first**, for any of these OpModes.

The order matters: each step assumes the ones above it are already right.

1. **Check the encoder is alive.** Run Single Swerve Module, press INIT and turn
   the module by hand. Nothing is powered yet. `Raw encoder` must change. If it
   never does, check the feedback wire, the analog port, and that
   `swerve_encoder` is configured as Analog Input. Watch `Noise p-p` too: several
   tenths of a volt while the module is still means a floating input.
2. **Check the encoder direction.** Single Swerve Module homes itself under power
   at START, so on a first run - before steps 2 to 4 are done - **hold A while
   you press START** and keep holding it. That cuts all power. Now turn the wheel
   to the right by hand: `Module angle` must rise. If it falls, set
   `ENCODER_REVERSED = true` and reinstall. Do this before the zero - it changes
   what every reading means.
3. **Set the zero.** Still holding **A**, point the wheel straight forward, read
   `Raw encoder`, put that number into `ENCODER_OFFSET_DEGREES`. Trims only last
   until the OpMode stops - always copy the final value into the file.
4. **Check the steering direction.** Release A and LB, push the left stick a
   little. `Error` must get smaller. **If the module keeps spinning or the error
   grows, press STOP immediately** and set `STEER_REVERSED = true`. The stall
   watchdog cuts it after about 2 s by itself, but STOP is faster.
5. **Check the drive direction.** Point the stick forward. If the wheel faces
   forward but rolls backward: on Single Swerve Module set
   `DRIVE_REVERSED = true`; on Swerve Drive Testing flip
   `driveMotor.setDirection` in `init()`.
6. **Tune the PD loop.** See below.
7. **Swerve Drive Testing only: check the tank drive.** Push the right stick
   forward. If the robot goes backward or spins, the tank motor directions need
   flipping in `init()`.

**If the module never turns at all**, none of the above applies yet. Hold **LB**
in Single Swerve Module and push the right stick fully over - that is raw servo power
with no loop and no encoder in the path. If the wheel still does not move, the
fault is mechanical or electrical and no code change will help. If it does move
but `Module angle` stays put, the feedback path is dead. The full decision tree is
in [SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md).

Single Swerve Module also lets you measure the real analog range while you are in
calibration mode - turn the module through a full turn and read `Volts seen`. Do
that during step 2, and redo the zero afterwards if you change the constants.

## Tuning the steering

Single Swerve Module and Swerve Drive Testing use the same loop and the same gain
names, so tune on one and copy the numbers to the other.

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
  It is not an encoder fault.
- **Steer power reads 0.00 while the module is pointed correctly.** Inside
  `STEER_TOLERANCE_DEGREES` the servo is switched off on purpose.

## Warning for the tank test robot

If the swerve wheel is on the ground and pointing sideways, driving the tank
forward drags the swerve wheel sideways. That wears the wheel and loads the
steering. Either keep the module lifted, or do not drive the tank hard while the
module is turned.
