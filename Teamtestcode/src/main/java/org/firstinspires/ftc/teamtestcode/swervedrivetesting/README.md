# Swerve Module Testers

Both OpModes make the swerve module point where the left stick points and then
drive. Push the stick further to go faster.

**They use the same steering hardware and the same configuration.** The only
difference is that one of them also drives the four tank motors.

| Name on the Driver Station | Steering hardware | Use it when |
| --- | --- | --- |
| **Single Swerve Module** | **CR Servo + analog encoder** | Testing the module by itself |
| **Swerve Drive Testing (1 module)** | **CR Servo + analog encoder** | The module is mounted on the tank test robot. Adds the 4 tank motors |

Step-by-step guide for Single Swerve Module:
[SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md)

---

## One configuration for both

The steering servo is an **Axon MINI in continuous-rotation mode**. It needs two
things plugged in: the three-wire servo cable into a servo port, and the analog
feedback wire into an analog port. Both OpModes read that feedback and close a PD
loop around it.

| | Single Swerve Module | Swerve Drive Testing (1 module) |
| --- | --- | --- |
| `swerve_servo` device type | **Continuous Rotation Servo** | **Continuous Rotation Servo** |
| `swerve_encoder` needed? | **Yes** — Analog Input | **Yes** — Analog Input |
| Tank motors needed? | **No** | **Yes** — all four |

So one saved configuration serves both, as long as it also has the four tank
motors in it. Single Swerve Module simply ignores them.

What goes wrong:

| Mistake | Symptom |
| --- | --- |
| `swerve_servo` configured as a plain **Servo** | INIT fails: *Unable to find a hardware device with name "swerve_servo" and type CRServo*. The OpMode never starts |
| No `swerve_encoder` configured | INIT fails: *Could not find a hardware device* |
| Feedback wire not plugged in, or in the wrong analog port | INIT looks fine and `Raw encoder` never changes. Single Swerve Module usually shows `!! STALLED / NO FEEDBACK` and cuts power a couple of seconds after you push the stick — but **not** if the frozen reading happens to sit within about 17° of where you are pushing, in which case it drives a blind module and says nothing. Always do the by-hand check at INIT |
| Running Swerve Drive Testing with no tank motors configured | INIT fails: *Could not find a hardware device*. Use Single Swerve Module on a bare module instead |

---

## Hardware

| Config name | Device | Used by | Notes |
| --- | --- | --- | --- |
| `swerve_motor` | DC Motor | both | drives the wheel |
| `swerve_servo` | **Continuous Rotation Servo** | both | Axon MINI in CR mode, turns the module |
| `swerve_encoder` | **Analog Input** | both | the Axon's feedback wire, in an analog port |
| `left_front`, `left_back`, `right_front`, `right_back` | DC Motor | **Swerve Drive Testing only** | tank drive |

---

## Per-file notes

### Single Swerve Module — `SingleSwerveTeleOp.java`

Closed loop. A CR servo is driven by a PD + static-friction loop against the
absolute analog encoder, so the module knows its real angle at power-on with no
homing step.

- The commanded angle is folded to the **shorter path from the measured angle**,
  and the drive motor reverses when folded.
- That flip is **sticky**: the error has to pass 105° before the module swaps
  sides (`FOLD_HYSTERESIS_DEGREES`). Without it, noise at exactly 90° of error
  would make the module slam 180° back and forth every loop. The cost is up to
  15° of extra turn in a 30° window around sideways.
- Drive power is scaled by the cosine of the steering error, clipped at 0, so the
  wheel fades in as the module lines up and does not turn at all while the module
  is more than 90° away.
- The damping term is taken on the **measured** angle, not on the error, so moving
  the stick or flipping sides does not throw a one-loop full-power spike at the
  servo.
- Releasing the stick stops the motor and **holds** the last angle with the
  steering loop (`HOLD_ANGLE_ON_RELEASE`). Set that false to make the module go
  limp instead.
- A **stall watchdog** cuts steering and drive, and shows
  `!! STALLED / NO FEEDBACK`, when steering is commanded above
  `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without `Error` shrinking by
  `STALL_MOVE_DEGREES`. It watches progress rather than raw movement, so a
  jittering floating input on a broken feedback wire still trips it, and so does
  a runaway that is turning fast the wrong way. It stops the servo being driven
  at full power forever. It is a backstop, not a substitute for the by-hand
  encoder check at INIT — see the note in the table above.
- The analog range is configurable (`ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE`)
  rather than assumed to be 0 V to the hub maximum, and calibration telemetry
  shows the lowest and highest voltage actually seen so the real range can be
  measured.
- `ENCODER_REVERSED` fixes an encoder that counts the wrong way round. This is a
  different fault from `STEER_REVERSED` and the two are not interchangeable —
  see below.
- Calibration (hold **A**) cuts all power; dpad left/right nudges the encoder zero
  by 0.5°.
- INIT telemetry is read-only, so the module can safely be turned by hand before
  START to check the encoder is alive.
- Key constants: `ENCODER_OFFSET_DEGREES`, `ENCODER_REVERSED`, `STEER_REVERSED`,
  `DRIVE_REVERSED`, `ANALOG_MIN_VOLTAGE`, `ANALOG_MAX_VOLTAGE`, `STEER_KP`,
  `STEER_KD`, `STEER_KS`, `STEER_TOLERANCE_DEGREES`, `STEER_MAX_POWER`,
  `FOLD_HYSTERESIS_DEGREES`, `HOLD_ANGLE_ON_RELEASE`, `STALL_POWER_THRESHOLD`,
  `STALL_TIME_SECONDS`, `STALL_MOVE_DEGREES`.

Full setup, telemetry and troubleshooting: [SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md).

### Swerve Drive Testing (1 module) — `swerveDriveTesting.java`

The same closed-loop steering, plus the 4 tank motors from the right stick.

- Same fold to the shorter path from the measured angle, but **without**
  hysteresis — it flips at exactly 90° of error, so it can chatter with the stick
  held near sideways.
- Same cosine drive scaling (`SCALE_DRIVE_BY_ERROR`).
- Releasing the stick cuts power to both the motor and the servo — the module goes
  limp and can be turned by hand. It does not hold its angle.
- No `ENCODER_REVERSED`, no configurable analog range (it assumes 0 V to the hub
  maximum), and no stall watchdog.
- Calibration (hold **A**) cuts all power including the tank motors; dpad
  left/right nudges the encoder zero by 0.5°.
- Key constants: `ENCODER_OFFSET_DEGREES`, `STEER_REVERSED`, `STEER_KP`,
  `STEER_KD`, `STEER_KS`, `STEER_TOLERANCE_DEGREES`, `STEER_MAX_POWER`,
  `SCALE_DRIVE_BY_ERROR`, `TANK_POWER_SCALE`.

Note it has **no** `DRIVE_REVERSED` constant — flipping the drive direction there
is a code change in `init()`, unlike Single Swerve Module.

The two files keep **separate copies** of their constants. They mean the same
things, so a zero or a set of gains measured on one module is worth carrying
across by hand — but nothing is shared in code, and Single Swerve Module has
several constants the other one does not.

---

## `ENCODER_REVERSED` vs `STEER_REVERSED`

Two different faults that look similar for about one second, and only Single
Swerve Module can fix both.

| What you see | Cause | Fix |
| --- | --- | --- |
| The module settles neatly, but on the **mirrored** heading — stick right points the wheel left | The encoder counts the wrong way | `ENCODER_REVERSED = true` |
| The module **runs away** — `Error` grows and it keeps spinning | The servo pushes away from the target | `STEER_REVERSED = true` |

Flipping `STEER_REVERSED` on a reversed encoder makes the loop converge again, so
it looks like a fix, but it converges on the mirrored heading. Test the encoder
direction first, by hand, with power off: **hold A, turn the wheel right,
`Module angle` must rise.**

---

## Controls

| Input | Single Swerve Module | Swerve Drive Testing |
| --- | --- | --- |
| Left stick | Aim and drive the module | Aim and drive the module |
| Right stick up/down | — | Tank robot forward / backward |
| Release left stick | Motor off, the loop **holds** the angle | Motor and servo both off, module goes limp |
| Hold A | Calibration: everything off | Calibration: everything off (tank too) |
| Dpad left/right (while holding A) | Nudge the encoder zero by 0.5° | Nudge the encoder zero by 0.5° |

## What the telemetry means

Angles: **0° = straight forward, positive = to the right.** `Module angle`,
`Target angle`, `Held angle` and `Error` run −180°…+180°; `Raw encoder` and
`Offset` run 0°…360°.

| Line | Which OpMode | Meaning |
| --- | --- | --- |
| Module angle | both | Where the module actually points right now |
| Target angle | both | Where it is trying to point |
| Held angle | Single Swerve Module | The angle being held while the stick is released |
| Error | both | Target minus actual. Should shrink toward 0 |
| Steer power | both | Power sent to the CR servo |
| Raw encoder | both | Encoder reading before the zero offset, in 0°…360° rather than −180°…+180°. This is the number you copy into `ENCODER_OFFSET_DEGREES` |
| Offset | both | Current encoder zero offset, in the same 0°…360° range |
| Encoder volts | Single Swerve Module | The raw analog voltage right now |
| Volts seen | Single Swerve Module | Lowest and highest voltage since the OpMode started, for measuring `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` |
| Analog range | Single Swerve Module | The voltage range the code is using, and the hub's own maximum for comparison |
| Encoder reversed | Single Swerve Module | The value of `ENCODER_REVERSED` |
| `!! STALLED / NO FEEDBACK` | Single Swerve Module | Steering was commanded but the module did not move. Power is cut. Usually the feedback wire |
| Drive power | both | Power sent to the wheel motor. `(reversed)` is normal, see below |
| Tank power | Swerve Drive Testing | Tank drive output |

---

## First-time setup

**Lift the module off the ground first**, for either OpMode.

The order is the same for both, and it matters: each step assumes the ones above
it are already right.

1. **Check the encoder is alive.** Turn the module by hand and watch
   `Raw encoder`. On Single Swerve Module you can do this during INIT, before
   anything is powered. If the number never changes, check the feedback wire, the
   analog port, and that `swerve_encoder` is configured as Analog Input.
2. **Check the encoder direction.** Hold A (all power off) and turn the wheel to
   the right by hand. `Module angle` must rise. If it falls, set
   `ENCODER_REVERSED = true` on Single Swerve Module and reinstall. Do this
   before the zero — it changes what every reading means.
3. **Set the zero.** Point the wheel straight forward, hold A, read `Raw encoder`,
   put that number into `ENCODER_OFFSET_DEGREES`. Reinstall, then check
   `Module angle` reads about 0 when the wheel is straight. Dpad trims only last
   until the OpMode stops — always copy the final value into the file.
4. **Check the steering direction.** Release A and push the left stick a little.
   `Error` must get smaller. **If the module keeps spinning or the error grows,
   press STOP immediately** and set `STEER_REVERSED = true`.
5. **Check the drive direction.** Point the stick forward. If the wheel faces
   forward but rolls backward: on Single Swerve Module set
   `DRIVE_REVERSED = true`; on Swerve Drive Testing flip
   `driveMotor.setDirection` in `init()`.
6. **Tune the PD loop.** See below.
7. **Swerve Drive Testing only: check the tank drive.** Push the right stick
   forward. If the robot goes backward or spins, the tank motor directions need
   flipping in `init()`.

Single Swerve Module also lets you measure the real analog range while you are in
calibration mode — turn the module through a full turn and read `Volts seen`. Do
that between steps 3 and 4, and redo the zero afterwards if you change the
constants.

## Tuning the steering

Both OpModes use the same loop and the same gain names, so tune on one and copy
the numbers to the other.

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

- **The module drives backward sometimes.** Both OpModes point the wheel the
  opposite way and reverse the motor rather than make a turn of more than 90°.
  Telemetry shows `(reversed)`.
- **The wheel speeds up gradually after a big turn.** Both fade power in with the
  cosine of the steering error, so the wheel does not scrub sideways while the
  module is still turning.
- **Single Swerve Module parks up to 15° off near sideways.** That is the fold
  hysteresis, and it is deliberate — without it the module would flap between
  pointing left and pointing right. Swerve Drive Testing has no hysteresis, which
  is why it can chatter there.
- **The wheel does not spin while the module is still more than 90° from the
  target.** The cosine scaling is clipped at 0, so drive power is exactly 0 until
  the module comes round inside 90°. Near sideways the hysteresis can hold it
  just outside that, and the wheel waits.
- **Swerve Drive Testing goes limp when you let go of the stick.** Power is cut on
  purpose so nothing buzzes and you can turn it by hand.
- **Single Swerve Module does *not* go limp.** It keeps holding the last angle,
  because `HOLD_ANGLE_ON_RELEASE` is true. Set it false for the other behaviour.
- **Steer power reads 0.00 while the module is pointed correctly.** Inside
  `STEER_TOLERANCE_DEGREES` the servo is switched off on purpose.

## Warning for the tank test robot

If the swerve wheel is on the ground and pointing sideways, driving the tank
forward drags the swerve wheel sideways. That wears the wheel and loads the
steering. Either keep the module lifted, or do not drive the tank hard while the
module is turned.
