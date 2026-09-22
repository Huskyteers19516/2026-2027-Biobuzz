# Single Swerve Module

**Driver Station name:** `Single Swerve Module` (TeleOp, group *Testing*)
**Code:** `SingleSwerveTeleOp.java`

Tests one swerve module on its own. Push the left stick in a direction and the
module turns to point that way, then drives. The further you push the stick, the
faster it drives.

This tester is **closed loop**. The steering servo is an **Axon MINI running in
continuous-rotation mode**, and its analog feedback wire tells the code where the
module really points. A PD loop drives the servo until the measured angle matches
the target. Nothing here is guessed.

Use this tester for bench testing a module by itself. If the module is mounted on
the tank test robot, use **Swerve Drive Testing (1 module)** instead — see
`README.md` in this folder. **Both OpModes want the same hardware and the same
configuration**; the other one just adds the four tank motors.

---

## Quick start

1. Configure `swerve_motor`, `swerve_servo` (**CR Servo**) and `swerve_encoder`
   (**Analog Input**) — see the table below.
2. Lift the module off the table so the wheel spins freely.
3. INIT → turn the module by hand. `Raw encoder` on the screen must change. If it
   does not, stop and fix the feedback wire before doing anything else.
4. START → hold **A** → work through *First run* below, in order.
5. Release A, push the left stick gently in each direction.

INIT does not move anything, and holding **A** cuts all power. After START the
steering loop is **live even with the stick released** — it holds the last angle
(`HOLD_ANGLE_ON_RELEASE`), so the servo can drive the module back on its own if
you knock it off heading. Hold **A** whenever you want to turn the module by
hand.

---

## Hardware

| Part | Job |
| --- | --- |
| DC motor | Spins the wheel. Its encoder cable is **not** needed |
| Axon MINI, set to **continuous rotation** | Turns the module. Plugs into a servo port |
| The Axon's **analog feedback wire** | Plugs into an **analog port**. This is how the code knows the real angle |

The Axon MINI has three wires to a servo port plus a separate feedback wire. Both
have to be plugged in. Without the feedback wire the code is blind, and this
OpMode will refuse to keep steering — see *The stall warning*.

The feedback is **absolute**: it reads the same angle every time the robot powers
on, so there is no homing step.

## Robot configuration

| Name (exact) | Device type | Port |
| --- | --- | --- |
| `swerve_motor` | DC Motor | any motor port |
| `swerve_servo` | **Continuous Rotation Servo** | any servo port |
| `swerve_encoder` | **Analog Input** | any analog port (0–3 on a hub) |

If `swerve_servo` is configured as a plain **Servo**, INIT fails with *Unable to
find a hardware device with name "swerve_servo" and type CRServo*. Fix it in the
Driver Station configuration, not in the code.

A missing or misspelled name makes INIT fail with *Could not find a hardware
device*.

## Angles

Looking down at the module from above:

| Angle | Wheel points |
| --- | --- |
| 0° | Forward |
| +90° | Right |
| −90° | Left |

`Module angle`, `Target angle`, `Held angle` and `Error` are reported in
−180°…+180°. `Raw encoder` and `Offset` are reported in 0°…360° — a value like
250° is normal, and it is the number you copy into `ENCODER_OFFSET_DEGREES`.

Because the servo spins continuously, the module can reach **any** heading, and
it always takes the **shorter way round** from where it actually is. A target
more than 90° away is reached by pointing the wheel the opposite way and running
the drive motor backward instead — that is the same motion and it is half the
turn. Telemetry shows `(reversed)` when this happens.

`Module angle` is the measured angle after the zero offset. `Raw encoder` is the
same reading before the offset, which is the number you copy into
`ENCODER_OFFSET_DEGREES`.

## Controls (gamepad 1)

| Input | What happens |
| --- | --- |
| Left stick, any direction | Module steers to point that way and drives |
| Left stick, how far | Speed. Nothing happens until the stick passes 15% |
| Let go of the stick | Motor stops. The steering loop **holds** the last angle |
| Hold **A** | Calibration mode: motor off, servo off, nothing moves |
| Dpad left / right **while holding A** | Nudge the encoder zero by 0.5° |

Dpad trims are lost when the OpMode stops. Always copy the final number into the
file.

---

## The screens

### During INIT (before START)

Read-only. No hardware is commanded, so it is safe to turn the module by hand.

| Line | Meaning |
| --- | --- |
| Module angle | Measured angle after the zero offset |
| Raw encoder | Measured angle before the offset |
| Encoder volts | The raw analog voltage right now |
| Analog range | `ANALOG_MIN_VOLTAGE`–`ANALOG_MAX_VOLTAGE` in use, and the hub's own max for comparison |
| Encoder reversed | The value of `ENCODER_REVERSED` |

### Driving (stick pushed)

| Line | Meaning |
| --- | --- |
| Stick | Raw stick position, x (right +) and y (forward +) |
| Module angle | Where the module actually points right now |
| Target angle | Where it is trying to point |
| Error | Target minus actual. Should shrink toward 0 |
| Steer power | Power sent to the CR servo |
| Drive power | Power sent to the wheel motor. May show `(reversed)` — that is normal |
| Offset | The encoder zero in use |
| Encoder volts | The raw analog voltage right now |
| `!! STALLED / NO FEEDBACK` | The watchdog fired. See *The stall warning* |

### Stick released

Same lines, with `Stick: released`, `Held angle` in place of `Target angle`, and
`Drive power: 0.00`. Steer power is usually 0 as well, because the loop is inside
its 2° deadband — but the loop is still running. Push the module off its held
angle by hand and the servo pushes back. Hold **A** to make it go limp.

### Calibration mode (holding A)

| Line | Meaning |
| --- | --- |
| Raw encoder | The number to copy into `ENCODER_OFFSET_DEGREES` |
| Module angle | Should read about 0 when the wheel is straight forward |
| Offset | The encoder zero in use, including dpad trim |
| Encoder volts | The raw analog voltage right now |
| Volts seen | The lowest and highest voltage seen since the OpMode started. Turn the module all the way round and these become your real `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` |
| Analog range in use | What the constants are set to now, for comparison |
| Encoder reversed | The value of `ENCODER_REVERSED` |

---

## First run

Do these **in order**. Each step assumes the ones before it are done. Keep a hand
on STOP from step 4 on, and keep the module lifted off the table throughout.

### 1. Is the encoder alive?

Press INIT. Do not press START. Turn the module by hand through a full turn.

`Raw encoder` and `Encoder volts` must both change smoothly.

| What you see | Meaning |
| --- | --- |
| Numbers move as you turn | Good, go on |
| Numbers never move | The feedback wire is unplugged, in the wrong port, or broken. Or `swerve_encoder` is on the wrong analog port |
| Numbers jump around while you hold the module still | Bad connection or a noisy ground. Fix it now — the PD loop cannot work on a noisy signal |

Do not skip this step. Everything below assumes a live encoder.

### 2. Which way does the encoder count?

Still in calibration mode (START, then hold **A** — all power is off).

**The one-line test: turn the wheel to the RIGHT by hand. `Module angle` must go
UP.**

`Module angle` jumps once from +180° to −180° somewhere in the turn. That is the
wrap, not a direction error. If you land on it, turn the wheel a quarter turn
away and try again. `Encoder volts` is the tiebreaker: with
`ENCODER_REVERSED = false` the voltage must **rise** as the wheel turns right,
except across the one seam.

| What you see | Fix |
| --- | --- |
| Turning right makes `Module angle` rise | `ENCODER_REVERSED = false` — leave it |
| Turning right makes `Module angle` fall | `ENCODER_REVERSED = true` — set it and reinstall |

Do this **before** setting the zero. Flipping `ENCODER_REVERSED` changes what
every raw reading means, so a zero measured with the wrong value is wrong.

This matters more than it looks. If the encoder counts the wrong way, the
steering loop still settles — but on the **mirrored** heading, so pushing the
stick right points the wheel left. `STEER_REVERSED` does **not** fix that: it
flips the servo, not the measurement, and with it the loop would run away instead
of settling. One constant fixes the measurement, the other fixes the motor. Get
this one right first.

### 3. Set the zero

1. Still holding **A**, point the wheel straight forward by hand, as accurately
   as you can.
2. Read `Raw encoder`.
3. Put that number into `ENCODER_OFFSET_DEGREES` at the top of
   `SingleSwerveTeleOp.java` and reinstall.
4. Hold **A** again. With the wheel straight forward, `Module angle` must now
   read about 0.

For the last degree or two, use dpad left/right while holding **A** and watch
`Module angle`, then copy the `Offset` line into `ENCODER_OFFSET_DEGREES`.
`Offset` is kept in the same 0°…360° range as `Raw encoder`, so the two numbers
stay comparable.

While you are here: turn the module slowly through a full turn and look at
`Volts seen`. If the lowest and highest are not close to `ANALOG_MIN_VOLTAGE` and
`ANALOG_MAX_VOLTAGE`, fix those constants — see *Analog range* below — and then
redo this step, because changing the range changes every angle.

### 4. Which way does the servo turn?

Release **A**. Push the left stick a **little** to the right.

`Error` must get **smaller** every loop and the module must settle.

**If the module spins without stopping, or `Error` grows, press STOP
immediately.** Set `STEER_REVERSED = true` and reinstall.

If instead the module settles neatly but on the opposite side from where you
pushed the stick, the problem is step 2, not this one. Go back and fix
`ENCODER_REVERSED`.

### 5. Which way does the wheel roll?

Push the stick straight forward. The wheel should point forward and roll
**forward**.

If it points forward but rolls backward, set `DRIVE_REVERSED = true` and
reinstall.

### 6. Tune the PD loop

Only once steps 1–5 are all correct. See *Tuning* below.

---

## Constants

All of these are at the top of `SingleSwerveTeleOp.java`. Reinstall after any
change.

| Constant | Default | What it does |
| --- | --- | --- |
| `ENCODER_OFFSET_DEGREES` | 0.0 | Raw encoder reading when the wheel points straight forward. Set in first-run step 3 |
| `ENCODER_REVERSED` | false | Set true when the analog reading **falls** as the wheel turns right. Set in first-run step 2 |
| `STEER_REVERSED` | false | Set true when the servo drives the module **away** from the target. Set in first-run step 4 |
| `DRIVE_REVERSED` | false | Set true when the wheel rolls backward while pointing forward |
| `ANALOG_MIN_VOLTAGE` | 0.0 | Voltage the Axon reports at its lowest angle |
| `ANALOG_MAX_VOLTAGE` | 3.3 | Voltage the Axon reports at its highest angle |
| `STEER_KP` | 0.012 | Proportional gain, power per degree of error |
| `STEER_KD` | 0.0006 | Damping, works against overshoot |
| `STEER_KS` | 0.05 | Constant push that gets the module over its own friction |
| `STEER_TOLERANCE_DEGREES` | 2.0 | Inside this error the servo is given 0 power, so it stops buzzing |
| `STEER_MAX_POWER` | 1.0 | Power ceiling for the servo. Lower it while tuning |
| `STICK_DEADZONE` | 0.15 | How far the stick must move before anything happens |
| `DRIVE_POWER_SCALE` | 1.0 | Wheel speed limit. 0.5 = half speed |
| `TRIM_STEP_DEGREES` | 0.5 | How much one dpad press moves the zero |
| `FOLD_HYSTERESIS_DEGREES` | 15.0 | How far past 90° the error must go before the module flips to the other side and the motor reverses. See *The hysteresis trade-off* |
| `HOLD_ANGLE_ON_RELEASE` | true | true: the loop keeps holding the last angle after you let go. false: everything goes limp and the module can be turned by hand |
| `STALL_POWER_THRESHOLD` | 0.25 | Steer power above which the watchdog starts watching. With the default gains that is an error of about **16.7°** — `(0.25 − STEER_KS) / STEER_KP`. Lowering `STEER_KP` arms the watchdog at a smaller error |
| `STALL_TIME_SECONDS` | 2.0 | How long steering may be commanded above that power without the error shrinking before the warning fires |
| `STALL_MOVE_DEGREES` | 2.0 | How much closer to the target the module must get to count as progress |

### Analog range

The code turns a voltage into an angle with
`(volts − ANALOG_MIN_VOLTAGE) / (ANALOG_MAX_VOLTAGE − ANALOG_MIN_VOLTAGE) × 360°`.

It does **not** assume 0 V to the hub's maximum, because a real Axon MINI often
swings over a narrower band. If you assume a 3.3 V swing on a servo that really
delivers 0.1–3.2 V, every angle is scaled and offset slightly wrong: the zero can
be trimmed away, but a 90° command lands a few degrees short, and the error grows
the further you turn from zero.

To measure yours: hold **A**, turn the module slowly through a full turn, and read
`Volts seen`. Those two numbers are your constants. Redo the zero afterwards.

The voltage wraps sharply from the maximum back to the minimum at one point in
the turn. That jump is the seam in the sensor, not a fault.

---

## Tuning

Start with `STEER_KD = 0` and `STEER_KS = 0`, and lower `STEER_MAX_POWER` to
about 0.4 so mistakes are slow. Push the stick to a fixed direction and watch
`Error`.

| What you see | Change |
| --- | --- |
| Turns slowly, or stops short of the target | Raise `STEER_KP` |
| Overshoots and wobbles back and forth | Lower `STEER_KP`, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Twitches or hums when it is already pointed right | Raise `STEER_TOLERANCE_DEGREES` |
| Jerky and noisy at every small correction | Lower `STEER_KD` — D amplifies encoder noise |
| Full-power jolt the instant the stick moves | Not the D term. It is measured on the module angle, not on the error, so moving the stick does not spike it. Look at `STEER_KP` and `STEER_MAX_POWER` |
| Too fast to watch | Lower `STEER_MAX_POWER` |

Raise `STEER_MAX_POWER` back to 1.0 when you are happy, then check nothing
overshoots at full speed.

`STEER_KS` is a static push, not a gain: it is added at full size as soon as the
error leaves the tolerance band, so too much of it makes the module hunt around
the target instead of settling.

---

## The hysteresis trade-off

The module can drive a direction two ways: point the wheel at the target and run
the motor forward, or point it 180° the other way and run the motor backward. The
code picks whichever is the shorter turn from the **measured** angle.

Exactly at 90° of error the two are equally short, and that is a problem. Stick
noise and encoder noise are enough to keep crossing that line, and each crossing
asks the module for a 180° turn. Held near sideways, the module would slam back
and forth for as long as you held the stick.

`FOLD_HYSTERESIS_DEGREES` fixes this. The module only flips once the error passes
**105°** (90 + 15), not 90°. Once it has flipped, the error to its new target is
at most 75°, which is well inside the band — so the flip cannot immediately undo
itself, and it cannot get stuck either, because only one of the two choices can
ever be outside the band at a time.

**The cost:** in a 30° window around sideways, the module keeps the side it
already had, so it can be steering to an error of up to 105° instead of up to
90°. It gets there — it is just a slightly longer turn than strictly necessary.
While the error is over 90° the drive power is **exactly 0**, not just low,
because the cosine scaling is clipped at 0; a raw cosine would have quietly
driven the wheel backwards instead. The wheel starts pulling as the module comes
round inside 90°.

Raise the constant if the module still flips too eagerly. Lower it toward 0 if
you would rather always have the shortest possible turn and you do not mind the
flapping. Do not make it negative.

---

## The stall warning

The feedback wire is a single wire on a moving module. It comes loose. When it
does, the naive behaviour is the dangerous one: the code sees an error that never
shrinks, so it drives the servo at full power forever, and keeps driving the
wheel while it does.

The watchdog stops that. It watches **progress**, not movement: if steering power
stays above `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without `Error`
getting at least `STALL_MOVE_DEGREES` smaller than the best it has managed since
the target last changed, the screen shows:

```
!! STALLED / NO FEEDBACK - steering and drive cut
```

and **both** the servo and the drive motor are set to 0. The drive motor is cut
too, because a stall means the code does not know where the wheel is pointing, so
driving it is a guess.

It clears when the module really gets closer to the target — turn it by hand
towards where the stick is pointing — or when you hold **A**.

Progress is the right test rather than raw movement. A feedback wire that has
come loose leaves a floating analog input, which does not sit still: it jitters.
A watchdog that cleared on any 2° of movement would be reset by that jitter and
never fire, which is the one case it exists for. Watching the error shrink
instead also catches a **runaway** — a module turning fast in the wrong direction
moves plenty, but its error only grows.

**It is not a complete safety net.** With a dead wire the reported angle is stuck
at one number, and if the stick happens to ask for a direction within about 17°
of that number the error never gets big enough to arm the watchdog, so nothing is
reported while the wheel still drives. That 33°-wide window is why first-run step
1 — turn the module by hand at INIT and watch `Raw encoder` change — is the check
that actually matters. Do not skip it and rely on the warning.

| What fired it | How to tell |
| --- | --- |
| Feedback wire unplugged or broken | `Encoder volts` sits at a fixed number, often 0.000, no matter how you turn the module by hand |
| Wrong analog port | Same as above. Check the configuration against the port the wire is actually in |
| Module is mechanically jammed | `Encoder volts` changes when you free the module by hand |
| Servo horn stripped or loose | The servo is audibly turning but the module is not |
| Gains far too low | Steer power sits just over the threshold and the module creeps slower than about 1° per second. Raise `STEER_KP` / `STEER_KS` |

A false alarm is possible if the module is genuinely turning but very slowly —
slower than `STALL_MOVE_DEGREES` of error per `STALL_TIME_SECONDS`, i.e. about 1°
per second with the defaults. Raise `STALL_TIME_SECONDS` if that happens — but
check the mechanics and the gains first, because a module that slow is not going
to be useful on the robot either.

---

## Things that look wrong but are normal

**The wheel drives backward when you push the stick backward.**
Pointing the wheel forward and reversing the motor is the same motion as turning
180°, and it is a much shorter turn. Telemetry shows `(reversed)`.

**The wheel speeds up gradually as the module lines up.**
Drive power is scaled by the cosine of the steering error, so it fades in as the
error shrinks. It stops the wheel scrubbing sideways mid-turn.

**The module keeps its heading when you let go of the stick.**
`HOLD_ANGLE_ON_RELEASE` is true, so the loop keeps holding. You will not be able
to turn it by hand while the OpMode is running. Set the constant false if you
want it to go limp instead.

**The steer power reads 0.00 while the module is pointed correctly.**
That is `STEER_TOLERANCE_DEGREES`. Inside 2° the servo is deliberately switched
off so it does not hum.

**The module parks slightly off what the stick asks for near sideways.**
That is the fold hysteresis, up to 15° in a 30° window. See above.

**The wheel does not spin at all while the module is still more than 90° from the
target.**
The cosine drive scaling is clipped at 0, so there is no drive power until the
module is pointing within 90° of where it is going. It starts pulling as the
module comes round.

**Nothing happens with a tiny stick push.**
That is the 15% deadzone (`STICK_DEADZONE`).

**`Encoder volts` jumps from the top of the range to the bottom once per turn.**
That is the seam in the absolute sensor. Expected.

---

## Troubleshooting

| Problem | Fix |
| --- | --- |
| INIT fails: *Could not find a hardware device* | A configuration name is wrong, or `swerve_encoder` is missing |
| INIT fails: *Unable to find … type CRServo* | `swerve_servo` is configured as a plain Servo. Change it to Continuous Rotation Servo |
| `Raw encoder` never changes | Feedback wire, analog port, or the `swerve_encoder` configuration. First-run step 1 |
| Module spins and never stops, `Error` grows | STOP. Set `STEER_REVERSED = true` |
| Module settles, but stick right points the wheel left | `ENCODER_REVERSED` is wrong. First-run step 2. Do **not** reach for `STEER_REVERSED` — that makes it run away instead |
| Wheel faces forward but rolls backward | Set `DRIVE_REVERSED = true` |
| Every angle is off by the same amount | Redo the zero, first-run step 3 |
| Small angles are right, big angles drift further off | `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` are wrong. Measure them with `Volts seen` |
| `!! STALLED / NO FEEDBACK` | See *The stall warning* |
| Module hunts around the target | Lower `STEER_KS`, then `STEER_KP` |
| Module buzzes while pointed correctly | Raise `STEER_TOLERANCE_DEGREES` |
| Module slams 180° back and forth with the stick near sideways | Raise `FOLD_HYSTERESIS_DEGREES` |
| Module points correctly but the wheel does not spin | `swerve_motor` wiring, or `DRIVE_POWER_SCALE` is 0 |
| Module drifts off angle after you let go | `HOLD_ANGLE_ON_RELEASE` is false, or the tolerance band is too wide |
| Numbers are right on the bench and wrong on the robot | The horn slipped. Redo first-run step 3 after any mechanical work |
