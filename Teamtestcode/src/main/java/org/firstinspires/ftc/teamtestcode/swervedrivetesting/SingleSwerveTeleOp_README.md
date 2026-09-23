# Single Swerve Module

**Driver Station name:** `Single Swerve Module` (TeleOp, group *Testing*)
**Code:** `SingleSwerveTeleOp.java`

Push the left stick in a direction and the module turns to face that way and
drives, like the caster wheel under an office chair. Push the stick further to go
faster.

There is **one** control scheme: a closed-loop caster on a **continuous-rotation
servo** with an **analog absolute encoder**. Nothing is detected at runtime and
there are no fallback modes.

---

## Robot configuration

All three devices are required, with these exact names:

| Name (exact) | Device type in the configuration | Port |
| --- | --- | --- |
| `swerve_motor` | **DC Motor** | any motor port |
| `swerve_servo` | **Continuous Rotation Servo** | any servo port |
| `swerve_encoder` | **Analog Input** | any analog port (0–3 on a hub) |

The steering servo is an **Axon MINI in continuous-rotation mode**: its three-wire
servo cable goes into the servo port, and its **separate analog feedback wire**
goes into the analog port. Both have to be plugged in.

The OpMode looks all three up with `hardwareMap.get`, so a wrong name or a wrong
device type **fails at INIT** with the SDK's own message, for example:

> `Unable to find a hardware device with name "swerve_servo" and type CRServo`

That failure is deliberate. If you see it, `swerve_servo` is probably configured
as a plain **Servo** — change it to **Continuous Rotation Servo** in the Driver
Station configuration and save.

---

## Quick start

1. Configure the three devices above.
2. Lift the module off the table so the wheel spins freely.
3. **INIT.** Nothing is powered. Turn the wheel by hand — `Raw encoder` must
   change.
4. **On the very first run, hold A while you press START.** START homes the
   module to forward *under closed-loop power*, and until `ENCODER_REVERSED` and
   `STEER_REVERSED` are confirmed that home can run away. Holding **A** cuts all
   power, so nothing moves. Keep holding it and do *First run* steps 1–3 below.
   Once both constants are confirmed, START on its own is safe and the module
   just homes itself to forward.
5. Hold **LB** and push the **right** stick left and right. The wheel must
   physically turn. If it does not, stop here and fix the mechanics or the
   wiring.
6. Release **LB** and push the left stick gently in each direction.

Nothing is commanded during INIT. The first thing that moves is the home at
START.

---

## Controls (gamepad 1)

| Input | What happens |
| --- | --- |
| Left stick, any direction | The module steers to face that way and drives |
| Left stick, how far | Speed. Nothing happens until the stick passes 15% (`STICK_DEADZONE`) |
| Let go of the stick | Drive motor stops. The module **holds** the last angle |
| Hold **LB** | **Manual override.** **Right** stick x is raw servo power, no loop, no watchdog. Drive motor off |
| Press **Y** | Home to forward (0°) and stop there |
| Hold **A** | Calibration. Everything off, live dpad trim, raw encoder readout |
| Dpad left / right **while holding A** | Nudge the encoder zero by 0.5° (`TRIM_STEP_DEGREES`) |

Manual override is on the **right** stick on purpose. The caster reads the
**left** stick, so letting go of **LB** with the manual stick still pushed over
cannot hand the caster a target you did not ask for.

Trims are lost when the OpMode stops. Always copy the final number into the file.

Releasing **LB**, releasing **A**, and the end of homing all leave the module
holding **the angle it is actually at**, so a button release never slams the
servo anywhere.

**A** and **LB** win over **Y** and the dpad. A **Y** press made while you are
holding either one is discarded rather than stored, so nothing fires the instant
you let go. Dpad trim only counts while **A** is held.

---

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

---

## How the caster works

- The stick direction becomes a target angle with `atan2(x, y)`.
- The target is **folded to the shorter path from the measured angle**: a target
  more than 105° away is reached by pointing the wheel the opposite way and
  running the drive motor backwards instead. Telemetry shows `(reversed)`.
- The fold is **sticky**. It only flips once the error passes 105°
  (90 + `FOLD_HYSTERESIS_DEGREES`), so noise at exactly 90° cannot make the module
  slam 180° back and forth. The cost is up to 15° of extra turn in a 30° window
  around straight sideways.
- A **PD + static-friction loop** drives the CR servo until the *measured* angle
  matches the target. The damping term is taken on the **measured angle**, not on
  the error, so moving the stick does not throw a one-loop full-power spike at
  the servo.
- Drive power is scaled by `cos(error)` clipped to [0, 1], so the wheel fades in
  as the module lines up and does not turn at all while the module is more than
  90° away.
- Releasing the stick stops the wheel and **holds the last angle**. Push the
  module off heading by hand and the servo pushes back. Hold **A** to make
  everything go limp.
- At START, and again on **Y**, the module drives itself to 0° and stops there,
  with a `HOME_TIMEOUT_SECONDS` (4 s) timeout that reports how far off it ended.
  Homing is under the **same stall watchdog** as the caster, so a home that runs
  away on a wrong `STEER_REVERSED` is cut after `STALL_TIME_SECONDS` (2 s) and
  reports `STALLED`, rather than pushing at full power for the whole timeout.
- The **stall watchdog** cuts steering and drive when steering has been commanded
  above `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without the error
  shrinking by `STALL_MOVE_DEGREES`. It watches *progress*, not movement, so a
  dead feedback wire trips it and so does a runaway. The latch clears as soon as
  you push the stick in a **new** direction, and holding **A** or pressing **Y**
  clears it too.

---

## Telemetry

Caster screen:

| Line | Meaning |
| --- | --- |
| Stick | Raw stick position, or `released` |
| Module angle | Where the module actually points right now |
| Target angle / Held angle | Where it is trying to point |
| Error | Target minus actual. Should shrink toward 0 |
| Steer power | Power sent to the CR servo. 0.00 inside `STEER_TOLERANCE_DEGREES` |
| Drive power | Power sent to the wheel motor. `(reversed)` is normal |
| Encoder volts | The raw analog voltage right now |
| Offset | The encoder zero in use, including dpad trim |
| Homing | Result of the last home: `done in 0.8 s`, `TIMEOUT, still 43.2 deg off`, `STALLED 43.2 deg off - check STEER_REVERSED`, or `cancelled` |
| Noise p-p | Peak-to-peak voltage, in volts and degrees, over about the last `NOISE_WINDOW_SECONDS`. It shows the spread so far while the first window is still filling, so it is honest from the first INIT loop |
| `!! STALLED / NO FEEDBACK` | The watchdog fired. Steering and drive are cut |

Homing screen (at START and after **Y**):

| Line | Meaning |
| --- | --- |
| Status | The same string as `Homing`, live: `running` until it finishes |
| Error to forward | How far the module still is from 0°, signed |

Manual override screen (**LB**):

| Line | Meaning |
| --- | --- |
| Servo power | The raw power going to the CR servo, straight from the **right** stick x |

Extra lines in calibration mode (**A**):

| Line | Meaning |
| --- | --- |
| Raw encoder | The number to copy into `ENCODER_OFFSET_DEGREES` |
| Volts seen | Lowest and highest voltage since the OpMode started. Turn the module all the way round and these become your real `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` |
| Analog range in use | What the constants are set to now, for comparison. Also on the INIT screen, where it adds the hub's own maximum |
| Encoder reversed | The value of `ENCODER_REVERSED` |

---

## First run

Do these **in order**, with the module lifted off the table. Each step assumes
the ones above it are already right.

**Hold A through the whole of steps 1 to 3, including the moment you press
START.** START homes under power, and steps 2 and 3 are exactly the checks that
make that home safe. Holding **A** keeps everything off until you are ready.

### 1. Is the encoder alive?

INIT (or hold **A** after START — both cut all power) and turn the module by hand
through a full turn.

- `Raw encoder` and `Encoder volts` must both change smoothly.
- `Noise p-p` must stay small while you hold the module still.

If `Raw encoder` never moves, the feedback path is dead: check the feedback wire,
the analog port, and that `swerve_encoder` is configured as **Analog Input**.
Nothing below this step can work until it moves.

While you are here, turn the module slowly all the way round and read
`Volts seen`. If it is not close to 0.00 – 3.30 V, put the real numbers into
`ANALOG_MIN_VOLTAGE` and `ANALOG_MAX_VOLTAGE` and reinstall.

### 2. Which way does the encoder count?

Still holding **A**: **turn the wheel to the RIGHT by hand. `Module angle` must
go UP.**

If it falls, set `ENCODER_REVERSED = true` and reinstall. Do this **before**
setting the zero, because flipping it changes what every raw reading means.

This matters more than it looks. With the encoder counting backwards the loop
still settles — on the **mirrored** heading, so stick right points the wheel
left. `STEER_REVERSED` does **not** fix that; it flips the servo, not the
measurement, and with it the loop runs away instead.

### 3. Set the zero

Holding **A**, point the wheel straight forward by hand, read `Raw encoder`, put
that number into `ENCODER_OFFSET_DEGREES`, reinstall.

Check `Module angle` now reads about 0. Use dpad left/right for the last degree
or two and copy the `Offset` line into the file.

### 4. Which way does the servo turn?

Release **A** and **LB**, then push the left stick a little to the right.
`Error` must get **smaller** every loop and the module must settle.

**If the module spins without stopping, or `Error` grows, press STOP
immediately.** Set `STEER_REVERSED = true` and reinstall. The stall watchdog
also cuts it after about 2 s on its own, but STOP is faster and cannot be
argued with.

If it settles neatly but on the opposite side from the stick, that is step 2, not
this one.

### 5. Which way does the wheel roll?

Push the stick straight forward. The wheel should point forward and roll
**forward**. If it rolls backward, set `DRIVE_REVERSED = true`.

### 6. Tune the gains

Only once everything above is right. See *Tuning*.

---

## Constants

All at the top of `SingleSwerveTeleOp.java`. Reinstall after any change.

### Directions and feel

| Constant | Default | What it does |
| --- | --- | --- |
| `STEER_REVERSED` | false | Set true when the servo drives the module **away** from the target |
| `DRIVE_REVERSED` | false | Set true when the wheel rolls backward while pointing forward |
| `STICK_DEADZONE` | 0.15 | How far the stick must move before anything happens |
| `DRIVE_POWER_SCALE` | 1.0 | Wheel speed limit. 0.5 = half speed |
| `FOLD_HYSTERESIS_DEGREES` | 15.0 | How far past 90° the error must go before the module flips to the other side and the motor reverses |

### Encoder

| Constant | Default | What it does |
| --- | --- | --- |
| `ENCODER_OFFSET_DEGREES` | 0.0 | Raw encoder reading when the wheel points straight forward. First-run step 3 |
| `ENCODER_REVERSED` | false | Set true when the analog reading **falls** as the wheel turns right. First-run step 2 |
| `ANALOG_MIN_VOLTAGE` | 0.0 | Voltage the Axon reports at its lowest angle |
| `ANALOG_MAX_VOLTAGE` | 3.3 | Voltage the Axon reports at its highest angle |
| `TRIM_STEP_DEGREES` | 0.5 | How much one dpad press moves the encoder zero |
| `NOISE_WINDOW_SECONDS` | 1.0 | Window for the `Noise p-p` figure |

### Steering loop

| Constant | Default | What it does |
| --- | --- | --- |
| `STEER_KP` | 0.012 | Proportional gain, power per degree of error |
| `STEER_KD` | 0.0006 | Damping, taken on the **measured angle** |
| `STEER_KS` | 0.05 | Constant push that gets the module over its own friction |
| `STEER_TOLERANCE_DEGREES` | 2.0 | Inside this error the servo is given 0 power, so it stops buzzing |
| `STEER_MAX_POWER` | 1.0 | Power ceiling for the servo. Lower it while tuning |

### Watchdog and homing

| Constant | Default | What it does |
| --- | --- | --- |
| `STALL_POWER_THRESHOLD` | 0.25 | Steer power above which the watchdog starts watching. With the default gains that is an error of about **16.7°** — `(0.25 − STEER_KS) / STEER_KP` |
| `STALL_TIME_SECONDS` | 2.0 | How long steering may be commanded above that power without progress |
| `STALL_MOVE_DEGREES` | 2.0 | How much closer to the target counts as progress |
| `HOME_TOLERANCE_DEGREES` | 2.0 | Close enough to forward to call homing done |
| `HOME_TIMEOUT_SECONDS` | 4.0 | Give up homing after this long and report how far off |

### Analog range

The code turns a voltage into an angle with
`(volts − ANALOG_MIN_VOLTAGE) / (ANALOG_MAX_VOLTAGE − ANALOG_MIN_VOLTAGE) × 360°`.

It does **not** assume 0 V to the hub maximum, because a real Axon MINI often
swings over a narrower band. Assume a 3.3 V swing on a servo that really delivers
0.1–3.2 V and every angle is scaled and offset slightly wrong: the zero can be
trimmed away, but a 90° command lands a few degrees short.

Measure yours with `Volts seen` in calibration mode, then redo the zero.

---

## Tuning

Start with `STEER_KD = 0`, `STEER_KS = 0`, and `STEER_MAX_POWER` around 0.4 so
mistakes are slow. Push the stick to a fixed direction and watch `Error`.

| What you see | Change |
| --- | --- |
| Turns slowly, or stops short of the target | Raise `STEER_KP` |
| Overshoots and wobbles back and forth | Lower `STEER_KP`, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Twitches or hums when already pointed right | Raise `STEER_TOLERANCE_DEGREES` |
| Jerky and noisy at every small correction | Lower `STEER_KD` — D amplifies encoder noise |
| Full-power jolt the instant the stick moves | Not the D term. It is measured on the module angle, not on the error. Look at `STEER_KP` and `STEER_MAX_POWER` |
| Too fast to watch | Lower `STEER_MAX_POWER` |

Raise `STEER_MAX_POWER` back to 1.0 when you are happy.

`STEER_KS` is a static push, not a gain: it is added at full size as soon as the
error leaves the tolerance band, so too much of it makes the module hunt.

---

## Troubleshooting

### The module does not turn at all

Work down this list in order. It takes about a minute and only one cause
survives.

1. **Did INIT fail?** Read the Driver Station error.
   - *Unable to find a hardware device with name "swerve_servo" and type CRServo*
     → `swerve_servo` is configured as a plain **Servo**, or not configured.
     Change it to **Continuous Rotation Servo**.
   - The same message for `swerve_motor` (DcMotor) or `swerve_encoder`
     (AnalogInput) → that device is missing or has the wrong type.
2. **Hold LB and push the right stick fully over.** This is raw servo power
   with no loop, no watchdog and no encoder in the path.
   - **The wheel still does not move** → the fault is mechanical or electrical,
     and no code change will fix it. Check that the servo cable is in the port
     you configured, that the servo is powered, that the horn is not stripped,
     and that the module is not jammed. Stop looking at the code.
   - **The wheel moves** → the servo and the mechanism are fine. Go to 3.
3. **Does `Module angle` change while the wheel turns under LB?**
   - **No** → the feedback path is dead. Check the analog wire and the analog
     port. The loop was chasing a number that never changes, which is also what
     fires `!! STALLED / NO FEEDBACK`.
   - **Yes** → the hardware is all fine and the *reference* is wrong. Redo first
     run steps 2, 3 and 4 in that order.
4. **Does it turn under LB but creep under the left stick?** The gains are too
   low for the friction in the module. Raise `STEER_KS` first, then
   `STEER_KP`.

### Everything else

| Problem | Fix |
| --- | --- |
| INIT fails naming `swerve_servo` and `CRServo` | The device is configured as a plain Servo, or the name is wrong |
| `Raw encoder` never changes | Feedback wire, analog port, or the `swerve_encoder` configuration |
| The module creeps slowly while you hold **A**, or during homing once it says `done` | The servo's own CR centre is off. `setPower(0.0)` is the only stop a CR servo has, and it is a neutral pulse, not an off switch. Trim the centre on the servo itself. It cannot happen at INIT, where the port is never written at all |
| `Noise p-p` is several tenths of a volt while still | Floating analog input — the wire is unplugged or broken |
| `Module angle` flickers between +179 and −180 | The ±180 wrap sitting where the wheel happens to point. Set `ENCODER_OFFSET_DEGREES` |
| Module spins and never stops, `Error` grows | STOP. Set `STEER_REVERSED = true` |
| Module settles, but stick right points the wheel left | `ENCODER_REVERSED` is wrong. Do **not** reach for `STEER_REVERSED` |
| Wheel faces forward but rolls backward | Set `DRIVE_REVERSED = true` |
| Every angle is off by the same amount | Redo the zero |
| Small angles right, big angles drift further off | Measure `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` with `Volts seen` |
| `!! STALLED / NO FEEDBACK` | See *The module does not turn at all*, steps 2 and 3. Push the stick a new way, or hold **A**, to clear the latch |
| `Homing: TIMEOUT, still N deg off` | The module could not reach forward in 4 s. Gains, a jam, or dead feedback. The module then holds where it stopped |
| `Homing: STALLED N deg off - check STEER_REVERSED` | The home was pushing hard and getting no closer. Almost always `STEER_REVERSED`, otherwise a jam or dead feedback. Power was cut after `STALL_TIME_SECONDS` |
| Module hunts around the target | Lower `STEER_KS`, then `STEER_KP` |
| Module slams 180° back and forth near sideways | Raise `FOLD_HYSTERESIS_DEGREES` |
| Module points correctly but the wheel does not spin | `swerve_motor` wiring, or `DRIVE_POWER_SCALE` is 0 |
| Numbers right on the bench, wrong on the robot | The horn slipped. Redo the zero after any mechanical work |

---

## Things that look wrong but are normal

**The wheel drives backward when you push the stick backward.**
Pointing the wheel forward and reversing the motor is the same motion as turning
180°, and it is a much shorter turn. Telemetry shows `(reversed)`.

**The wheel speeds up gradually as the module lines up.**
Drive power is scaled by `cos(error)`, so the wheel does not scrub sideways
mid-turn.

**The module keeps its heading when you let go of the stick.**
That is deliberate. Hold **A** to cut all power and turn it by hand.

**`Steer power` reads 0.00 while the module is pointed correctly.**
That is `STEER_TOLERANCE_DEGREES`. Inside 2° the servo is deliberately off.

**The module parks slightly off what the stick asks for near sideways.**
That is the fold hysteresis, up to 15° in a 30° window.

**The wheel does not spin while the module is still more than 90° from the
target.** The cosine scaling is clipped at 0.

**`Encoder volts` jumps from the top of the range to the bottom once per turn.**
That is the seam in the absolute sensor. Real noise is a jump in the **voltage**
while the module is still; the wrap is a jump in the reported **angle** with a
smooth voltage behind it.

---

## See also

- `SwerveServoBenchTest.java` — **Swerve Servo Bench Test**, a pure open-loop
  diagnostic with stepped test powers and a moved-since-press readout. Use it when
  you want to poke the servo without any caster logic at all.
- `swerveDriveTesting.java` — **Swerve Drive Testing (1 module)**, the same
  closed-loop steering plus four tank motors.
- `README.md` in this folder — the three testers side by side.
