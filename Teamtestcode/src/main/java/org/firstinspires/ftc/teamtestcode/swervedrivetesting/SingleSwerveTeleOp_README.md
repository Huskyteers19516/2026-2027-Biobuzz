# Single Swerve Module

**Driver Station name:** `Single Swerve Module` (TeleOp, group *Testing*)
**Code:** `SingleSwerveTeleOp.java`

Push the left stick in a direction and the module turns to face that way and
drives, like the caster wheel under an office chair. Push the stick further to go
faster.

**This OpMode works with either kind of steering servo.** It looks at how
`swerve_servo` is actually configured when you press INIT and picks the control
scheme to match. You do not choose a mode, and you do not have to edit the code
to switch. The INIT screen tells you what it found.

---

## The four modes it can land in

| `swerve_servo` is | `swerve_encoder` | Mode on screen | What you get |
| --- | --- | --- | --- |
| **Continuous Rotation Servo** | present | `CLOSED LOOP CASTER (CR servo + encoder)` | The real thing. PD + friction loop on the measured angle, unlimited rotation, shortest-path folding, stall watchdog |
| **Continuous Rotation Servo** | missing | `OPEN LOOP CR STEER (no encoder)` | Stick x is raw servo power, stick y is drive power. No heading is held, because nothing measures the angle |
| **Servo** (position) | present | `OPEN LOOP CASTER (position servo)` | Caster feel, but the angle is *commanded*, not measured. The encoder is used only to show you whether the servo really got there |
| **Servo** (position) | missing | `OPEN LOOP CASTER (position servo)` | Same, with nothing to check it against |
| neither found | either | `NO STEERING SERVO` | Nothing is commanded. The screen tells you what it looked for |

Detection uses `hardwareMap.tryGet`, which returns `null` instead of throwing, so
**INIT never crashes** on a mis-configured or missing device. It tries `CRServo`
first, then `Servo`. The drive motor and the encoder are looked up the same way,
so a missing `swerve_motor` or `swerve_encoder` is reported on screen rather than
killing the OpMode.

---

## Read this first: which fault do you actually have?

These are the symptoms this rewrite exists for. Work down the tree — do not skip
to the bottom.

### Step 0. What does INIT say?

Press INIT and read `Detected swerve_servo`.

| It says | Meaning |
| --- | --- |
| `CRServo (Continuous Rotation Servo)` | The configuration really is CR |
| `Servo (position mode)` | The configuration is a **plain Servo**. This is the single most likely cause of "it moves a little and springs back" |
| `NOT FOUND` | The name is wrong, or the port is not configured at all |

This one line settles an argument that otherwise costs an afternoon. The
Driver Station configuration is the truth; what anyone remembers setting is not.

### Step 1. "It moves 1–2° and stops"

That is a **CR servo being given a position command**, or a position servo whose
travel constants are badly wrong.

- If INIT says `CRServo`, then old position-mode code is the cause. This OpMode
  does not do that any more.
- If INIT says `Servo`, check `Servo position` on screen. If it barely moves off
  `0.500`, `SERVO_TRAVEL_DEGREES` is far too large. See *Correcting
  SERVO_TRAVEL_DEGREES*.

### Step 2. "Push left, it moves slightly left, then is pulled straight back"

This is the headline symptom. It has three plausible causes in the hardware, plus
one in the calibration.

**Cause A — a position servo driven by a closed loop.** As the error shrinks the
loop shrinks its output, the pulse width returns to neutral, and a *position*
servo obediently walks back to centre. The module can never hold an angle. This
is what happens when position-mode hardware is driven by CR-mode code.

> **How to tell:** INIT says `Servo (position mode)`. That is conclusive. This
> OpMode then uses open-loop position commands instead, and the symptom goes away.

**Cause B — the module is mechanically jammed.** The servo pushes, the module
does not move, and when power drops the module relaxes back.

> **How to tell:** hold **LB** (manual override) and push the stick fully left.
> This sends full raw steering with no feedback of any kind. If the wheel still
> will not turn, nothing in software will fix it — free the mechanism.

**Cause C — dead or slipping feedback.** The loop chases a number that never
changes, so it never converges and the module twitches.

> **How to tell:** hold **A** (calibration, all power off) and turn the wheel by
> hand. `Raw encoder` and `Encoder volts` must change smoothly. If they do not
> move at all, the feedback path is dead. If they jump while you hold the module
> still, look at `Noise p-p` — see *Noise vs the wrap*.

**All three tests pass and it still will not hold.** INIT says `CRServo`, LB
turns the wheel, and `Raw encoder` moves smoothly by hand — then the loop is
healthy and the *reference* is wrong: the zero has never been set, or the encoder
counts backwards. That is exactly what the +179 / −180 flicker in step 3 is
telling you. Go to *First run*, steps **4**, **5** and **6**, in that order.

Run the LB test and the A test, in that order. Two tests, thirty seconds, and
only one of the four causes survives.

### Step 3. "The angle flickers between +179 and −180"

That is the **normalize wrap**, not a fault. Every angle is reported in
−180°…+180°, and the seam has to sit somewhere. It is sitting where the wheel
happens to point because `ENCODER_OFFSET_DEGREES` has never been set.

The screen says so itself whenever `|Module angle| > 150`. Set the zero (see
*First run*, step 3) and the seam moves round to the back of the module where you
will never touch it.

### Step 4. "STALLED / NO FEEDBACK fired"

The watchdog only exists in **closed-loop** mode. It fires when steering has been
commanded hard for 2 s without the error shrinking. That means one of: dead
feedback, a jam, a stripped horn, or gains so low the module creeps.

The latch clears itself as soon as you command a **new** direction — push the
stick somewhere else and the loop gets a fresh 2 s to prove itself. Holding
**A**, **LB** or pressing **Y** also clears it. If the module really is jammed it
simply fires again.

Go back to step 2 and run the LB test and the A test. The watchdog tells you
something is wrong; those two tests tell you *what*.

### Noise vs the wrap

`Noise p-p` is the peak-to-peak voltage seen in the last second, with the same
figure converted to degrees. Use it to tell a genuinely bad signal from the wrap:

| `Noise p-p` while the module is held still | Meaning |
| --- | --- |
| Under about 0.02 V | Healthy |
| Several tenths of a volt, drifting | Floating input. The feedback wire is unplugged or broken |
| Large only as the wheel crosses one point in a turn | That is the sensor seam. Normal |

The wrap is a jump in the *reported angle* with a smooth voltage behind it. Real
noise is a jump in the **voltage**. Watch `Encoder volts`, not `Module angle`.

---

## Quick start

1. Configure `swerve_motor`, `swerve_servo` and `swerve_encoder` — see
   *Robot configuration*. Either servo type is accepted.
2. Lift the module off the table so the wheel spins freely.
3. INIT → read `Detected swerve_servo` and `Mode`. Turn the module by hand;
   `Raw encoder` must change if an encoder is configured.
4. START → the module homes to forward by itself.
5. Hold **LB** and push the stick left and right. The wheel must physically turn.
   If it does not, stop here and fix the mechanics.
6. Release **LB** and push the left stick gently in each direction.

Nothing is powered during INIT, in any mode.

---

## Controls (gamepad 1)

| Input | What happens |
| --- | --- |
| Left stick, any direction | Module steers to face that way and drives |
| Left stick, how far | Speed. Nothing happens until the stick passes 15% |
| Let go of the stick | Motor stops. The module holds the last angle |
| Hold **LB** | **Manual override.** Stick x steers directly with no feedback at all. Drive motor off |
| Press **Y** | Home to forward (0°) and stop there |
| Hold **A** | Calibration. Drive motor off, live dpad trim, raw encoder readout |
| Dpad left / right **while holding A** | CR mode: nudge the encoder zero by 0.5°. Position mode: nudge the servo centre by 0.005 |

Trims are lost when the OpMode stops. Always copy the final number into the file.

In `OPEN LOOP CR STEER` mode (CR servo, no encoder) the stick does not work as a
caster: stick **x** is steering power and stick **y** is drive power. The screen
says so.

---

## Robot configuration

| Name (exact) | Device type | Port |
| --- | --- | --- |
| `swerve_motor` | DC Motor | any motor port |
| `swerve_servo` | **Continuous Rotation Servo** *or* **Servo** | any servo port |
| `swerve_encoder` | **Analog Input** | any analog port (0–3 on a hub) |

Nothing here is mandatory any more — a missing device is reported on the screen
instead of failing INIT. But the module only works properly with all three, and
only *steers properly* with a CR servo plus the encoder.

The Axon MINI has a three-wire servo cable plus a separate analog feedback wire.
Both have to be plugged in for closed-loop mode.

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

## Mode 1 — closed-loop caster (CR servo + encoder)

The intended setup. A PD + static-friction loop drives the CR servo until the
**measured** angle matches the target.

- The damping term is taken on the **measured angle**, not on the error, so
  moving the stick or flipping sides does not throw a one-loop full-power spike
  at the servo.
- The target is folded to the shorter path from the measured angle: a target more
  than 105° away is reached by pointing the wheel the opposite way and running the
  motor backwards instead. Telemetry shows `(reversed)`.
- The fold is sticky. It only flips once the error passes **105°**
  (90 + `FOLD_HYSTERESIS_DEGREES`), so noise at exactly 90° cannot make the module
  slam 180° back and forth. The cost is up to 15° of extra turn in a 30° window
  around sideways.
- Drive power is scaled by `cos(error)` clipped to [0, 1], so the wheel fades in
  as the module lines up and does not turn at all while the module is more than
  90° away.
- Releasing the stick holds the last angle (`HOLD_ANGLE_ON_RELEASE`). Push the
  module off heading by hand and the servo pushes back.
- The **stall watchdog** cuts steering and drive if steering is commanded above
  `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without the error shrinking by
  `STALL_MOVE_DEGREES`. It watches *progress*, not movement, so a jittering
  floating input still trips it, and so does a runaway.

### The screen

| Line | Meaning |
| --- | --- |
| Mode | The detected mode |
| Stick | Raw stick position, or `released` |
| Module angle | Where the module actually points right now |
| Target angle / Held angle | Where it is trying to point |
| Error | Target minus actual. Should shrink toward 0 |
| Steer power | Power sent to the CR servo |
| Drive power | Power sent to the wheel motor. `(reversed)` is normal |
| Offset | The encoder zero in use |
| Encoder volts | The raw analog voltage right now |
| Noise p-p | Peak-to-peak voltage over the last second, in volts and degrees |
| Homing | Result of the last home |
| `!! STALLED / NO FEEDBACK` | The watchdog fired |

---

## Mode 2 — open-loop caster (position servo)

If `swerve_servo` is a plain Servo, there is no loop. The commanded module angle
is turned straight into a servo position:

```
position = SERVO_CENTER + trim + (angle / SERVO_TRAVEL_DEGREES)
```

clipped to 0…1.

Because a position servo has limited travel, the target is folded to
**±105° of the mechanical zero** (90° plus `FOLD_HYSTERESIS_DEGREES`) before it
is converted — the same fold as closed-loop mode, but measured against the servo
centre instead of the encoder. A direction more than 105° from forward is reached
by pointing the wheel the opposite way and reversing the drive motor.

**Budget 210° of module rotation**, not 180°. The hysteresis band means the
commanded angle can sit anywhere in ±105°, so if you narrow `SERVO_MIN_POSITION`
and `SERVO_MAX_POSITION` to exactly ±90° you will get `(CLIPPED - out of travel)`
across a 30° band either side of straight sideways.

Drive power is **ramped** so the wheel does not scrub sideways while the servo
swings. An ordinary sweep of the stick does not interrupt it: only a jump in the
commanded angle larger than `SERVO_SETTLE_DEGREES` (10°) cuts the power back, and
it cuts *proportionally* — a 10° jump costs nothing, a 70° jump takes it to zero.
A fold flip (the moment the wheel turns round and the drive reverses) always
resets it to zero. Power then slews back up at `DRIVE_RAMP_PER_SECOND`.

### The screen

| Line | Meaning |
| --- | --- |
| Mode | `OPEN LOOP CASTER (position servo)` |
| Target angle / Held angle | The angle being **commanded**. Nothing guarantees the module got there |
| Servo position | The 0…1 position actually sent. `(CLIPPED - out of travel)` means the fold could not keep the angle inside the servo's range |
| Servo centre | `SERVO_CENTER` plus any dpad trim |
| Module angle | *Encoder only.* Where the module really points |
| Commanded vs measured | *Encoder only.* The two angles side by side plus the difference. This is how you see whether the servo actually got there |
| Implied travel | *Encoder only.* Measured degrees per full position span. See below |
| Drive power | Power sent to the wheel motor, after the ramp |

### Correcting `SERVO_TRAVEL_DEGREES`

`SERVO_TRAVEL_DEGREES` (default 270) is how many degrees of **module** rotation
one full 0→1 sweep of the servo produces. Get it wrong and every commanded angle
is scaled wrong — a 90° command lands short or long, and the error grows the
further you turn from centre.

With an encoder plugged in, the OpMode measures it for you. It takes a reference
at the end of homing (servo at centre) and then, once the servo has moved more
than 0.05 of position away from that, reports:

```
Implied travel: 210 deg per full span (set 270)
```

Push the stick to one side, read the number, put it into
`SERVO_TRAVEL_DEGREES`, reinstall. Repeat once to confirm it now matches.

**The number on screen is always positive, so you can copy it straight in.**
`SERVO_TRAVEL_DEGREES` must never be negative or zero — a negative value would
just mirror your steering, and zero pins the servo at centre forever. If the
screen adds *"Encoder counts the OTHER way from the servo"*, the measurement came
out negative: the printed figure is still the right magnitude, but the encoder is
reading backwards relative to the servo, so also set `ENCODER_REVERSED = true`
(or, if the wheel itself turns the wrong way when you push the stick,
`STEER_REVERSED = true`). Fix the direction flag, not the travel figure.

The measurement also works while you hold **LB** and sweep the servo by hand,
which is the easiest way to get a big, well-conditioned span.

Note this measures the **module**, not the servo. If the module is geared down
from the servo, the implied travel is the geared figure, which is exactly the
number this code wants.

### What this mode cannot do

Stated on the Driver Station too, one line:

> `POSITION MODE: no feedback loop, travel limited by SERVO_TRAVEL_DEGREES.`

- It cannot correct for the module being knocked off heading.
- It cannot go past the servo's mechanical travel.
- In calibration mode the servo **keeps holding**, at the trimmed centre. A
  position servo cannot be made limp from the OpMode, so to turn the module by
  hand you have to unplug it. The screen says this. The drive motor *is* off.

---

## Mode 3 — open-loop CR steer (CR servo, no encoder)

Closed loop is impossible without a measured angle, so the OpMode does **not**
silently do nothing and it does **not** pretend. It falls back to direct steering
and says why:

> `NO ENCODER: heading cannot be held, stick x is raw steer power.`

Stick **x** is servo power, stick **y** is drive power. Use it to prove the
module turns, then plug the feedback wire into an analog port configured as
`swerve_encoder` and get the real mode back.

---

## Homing to forward

At START, and again whenever **Y** is pressed, the module is driven to 0°
(forward) and stopped there.

| Mode | How it homes | Timeout |
| --- | --- | --- |
| Closed loop | PD loop to 0° against the encoder | `HOME_TIMEOUT_SECONDS` (4 s). On timeout the screen reports how far off it ended |
| Position | Commands servo centre and waits | `HOME_SETTLE_SECONDS` (0.7 s). With an encoder it then reports the measured error |
| CR, no encoder | Not possible. The screen reads `Homing: impossible, no encoder` and **Y** does nothing | — |

Nothing moves during INIT in any mode. Homing starts at START, not before.

The `Homing` line stays on screen afterwards, so a `TIMEOUT, still 43.2 deg off`
is still visible when you go looking for it.

---

## Manual override (hold LB)

Available in **every** mode. It is the test that settles arguments.

| Mode | What LB does |
| --- | --- |
| CR (either) | Stick x is sent straight to the servo as power |
| Position | Stick x moves the servo position up to ±0.5 either side of centre |

There is no feedback, no loop, no watchdog, and the drive motor is held at 0. If
the wheel will not turn with LB held and the stick pushed fully over, the problem
is mechanical or electrical — stop looking at the code.

With an encoder plugged in, watch `Module angle` while you do this. Wheel turns
but the angle does not → the feedback path is the fault, not the steering.

---

## Calibration mode (hold A)

All power off in every mode, live trim, and the numbers you need to tell a real
encoder fault from the ±180 wrap.

| Line | Meaning |
| --- | --- |
| Raw encoder | The number to copy into `ENCODER_OFFSET_DEGREES` |
| Module angle | Should read about 0 when the wheel is straight forward |
| Offset | The encoder zero in use, including dpad trim (CR mode) |
| Servo centre | The servo centre in use, including dpad trim (position mode) |
| Encoder volts | The raw analog voltage right now |
| Volts seen | Lowest and highest voltage since the OpMode started. Turn the module all the way round and these become your real `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` |
| Noise p-p | Peak-to-peak voltage over the last second, in volts and degrees |
| Analog range in use | What the constants are set to now, for comparison |
| Encoder reversed | The value of `ENCODER_REVERSED` |

In position mode the drive motor is off but the steering servo keeps holding. The
screen says so.

---

## First run

Do these **in order**, with the module lifted off the table.

### 1. What is it?

INIT. Read `Detected swerve_servo` and `Mode`. Everything below depends on the
answer.

### 2. Can the module physically turn?

START, hold **LB**, push the stick fully left, then fully right. The wheel must
turn. If it does not, free the mechanism or fix the servo wiring. Nothing else
matters until this works.

### 3. Is the encoder alive?

Hold **A** and turn the module by hand through a full turn. `Raw encoder` and
`Encoder volts` must both change smoothly, and `Noise p-p` must stay small while
you hold it still.

### 4. Which way does the encoder count? *(closed-loop mode only)*

Still holding **A**: **turn the wheel to the RIGHT by hand; `Module angle` must
go UP.**

If it falls, set `ENCODER_REVERSED = true` and reinstall. Do this **before**
setting the zero, because flipping it changes what every raw reading means.

This matters more than it looks. With the encoder counting backwards the loop
still settles — on the **mirrored** heading, so stick right points the wheel
left. `STEER_REVERSED` does **not** fix that; it flips the servo, not the
measurement, and with it the loop runs away instead.

### 5. Set the zero

**Closed-loop mode:** holding **A**, point the wheel straight forward by hand,
read `Raw encoder`, put that number into `ENCODER_OFFSET_DEGREES`, reinstall.
Check `Module angle` now reads about 0. Use dpad left/right for the last degree
or two and copy the `Offset` line.

**Position mode:** holding **A**, use dpad left/right until the wheel points
straight forward, then copy `Servo centre` into `SERVO_CENTER`. The servo is held
at the trimmed centre for as long as **A** is held, so the wheel moves as you
press the dpad — if it does not move at all, that is the same dead-servo evidence
the LB test gives you. (This is the one place calibration mode does *not* cut all
power: a position servo cannot be made limp from an OpMode. To turn the module by
hand in position mode you have to unplug the servo.)

### 6. Which way does the servo turn? *(closed-loop mode only)*

Release **A** and **LB**, push the left stick a little to the right. `Error` must
get **smaller** every loop and the module must settle.

**If the module spins without stopping, or `Error` grows, press STOP
immediately.** Set `STEER_REVERSED = true` and reinstall.

If it settles neatly but on the opposite side from the stick, that is step 4, not
this one.

### 7. Position mode only: fix the travel

Push the stick to one side (or hold **LB** and sweep it) and read
`Implied travel`. It is always printed positive — put that number into
`SERVO_TRAVEL_DEGREES` and reinstall. Never enter a negative or zero value. If
the screen also says *"Encoder counts the OTHER way from the servo"*, set
`ENCODER_REVERSED = true` as well.

### 8. Which way does the wheel roll?

Push the stick straight forward. The wheel should point forward and roll
**forward**. If it rolls backward, set `DRIVE_REVERSED = true`.

### 9. Tune the PD loop *(closed-loop mode only)*

Only once everything above is right. See *Tuning*.

---

## Constants

All at the top of `SingleSwerveTeleOp.java`. Reinstall after any change.

### Shared

| Constant | Default | What it does |
| --- | --- | --- |
| `STEER_REVERSED` | false | Flips the steering servo. CR mode: set true when the servo drives the module **away** from the target. Position mode: set true when a positive angle moves the wheel the wrong way |
| `DRIVE_REVERSED` | false | Set true when the wheel rolls backward while pointing forward |
| `STICK_DEADZONE` | 0.15 | How far the stick must move before anything happens |
| `DRIVE_POWER_SCALE` | 1.0 | Wheel speed limit. 0.5 = half speed |
| `FOLD_HYSTERESIS_DEGREES` | 15.0 | How far past 90° the error must go before the module flips to the other side and the motor reverses |
| `HOLD_ANGLE_ON_RELEASE` | true | Closed-loop mode: keep holding the last angle after you let go |
| `HOME_ON_START` | true | Home to forward automatically at START |
| `NOISE_WINDOW_SECONDS` | 1.0 | Window for the `Noise p-p` figure |

### Encoder

| Constant | Default | What it does |
| --- | --- | --- |
| `ENCODER_OFFSET_DEGREES` | 0.0 | Raw encoder reading when the wheel points straight forward. First-run step 5 |
| `ENCODER_REVERSED` | false | Set true when the analog reading **falls** as the wheel turns right. First-run step 4 |
| `ANALOG_MIN_VOLTAGE` | 0.0 | Voltage the Axon reports at its lowest angle |
| `ANALOG_MAX_VOLTAGE` | 3.3 | Voltage the Axon reports at its highest angle |
| `TRIM_STEP_DEGREES` | 0.5 | How much one dpad press moves the encoder zero |

### Closed loop only

| Constant | Default | What it does |
| --- | --- | --- |
| `STEER_KP` | 0.012 | Proportional gain, power per degree of error |
| `STEER_KD` | 0.0006 | Damping, taken on the **measured angle** |
| `STEER_KS` | 0.05 | Constant push that gets the module over its own friction |
| `STEER_TOLERANCE_DEGREES` | 2.0 | Inside this error the servo is given 0 power, so it stops buzzing |
| `STEER_MAX_POWER` | 1.0 | Power ceiling for the servo. Lower it while tuning |
| `STALL_POWER_THRESHOLD` | 0.25 | Steer power above which the watchdog starts watching. With the default gains that is an error of about **16.7°** — `(0.25 − STEER_KS) / STEER_KP` |
| `STALL_TIME_SECONDS` | 2.0 | How long steering may be commanded above that power without progress |
| `STALL_MOVE_DEGREES` | 2.0 | How much closer to the target counts as progress |
| `HOME_TOLERANCE_DEGREES` | 2.0 | Close enough to forward to call homing done |
| `HOME_TIMEOUT_SECONDS` | 4.0 | Give up homing after this long and report how far off |

### Position mode only

| Constant | Default | What it does |
| --- | --- | --- |
| `SERVO_CENTER` | 0.5 | Servo position at which the wheel points straight forward |
| `SERVO_TRAVEL_DEGREES` | 270.0 | Degrees of **module** rotation per full 0→1 servo sweep. Measure it with `Implied travel` |
| `SERVO_MIN_POSITION` / `SERVO_MAX_POSITION` | 0.0 / 1.0 | Position limits. Narrow them if the servo hits a hard stop |
| `SERVO_TRIM_STEP` | 0.005 | How much one dpad press moves the servo centre |
| `SERVO_MANUAL_SPAN` | 0.5 | How far either side of centre the LB override can reach |
| `SERVO_SETTLE_DEGREES` | 10.0 | A one-loop commanded-angle jump bigger than this cuts the drive ramp back, proportionally |
| `SERVO_JUMP_FULL_CUT_DEGREES` | 60.0 | A jump this far past `SERVO_SETTLE_DEGREES` takes the drive ramp all the way to zero |
| `DRIVE_RAMP_PER_SECOND` | 1.5 | How fast drive power slews back up after a swing |
| `HOME_SETTLE_SECONDS` | 0.7 | How long homing holds centre before reporting |

### Analog range

The code turns a voltage into an angle with
`(volts − ANALOG_MIN_VOLTAGE) / (ANALOG_MAX_VOLTAGE − ANALOG_MIN_VOLTAGE) × 360°`.

It does **not** assume 0 V to the hub maximum, because a real Axon MINI often
swings over a narrower band. Assume a 3.3 V swing on a servo that really delivers
0.1–3.2 V and every angle is scaled and offset slightly wrong: the zero can be
trimmed away, but a 90° command lands a few degrees short.

To measure yours: hold **A**, turn the module slowly through a full turn, read
`Volts seen`. Redo the zero afterwards.

---

## Tuning (closed loop)

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

## Things that look wrong but are normal

**The wheel drives backward when you push the stick backward.**
Pointing the wheel forward and reversing the motor is the same motion as turning
180°, and it is a much shorter turn. Telemetry shows `(reversed)`.

**The wheel speeds up gradually as the module lines up.**
Closed loop scales drive power by `cos(error)`; position mode ramps it after
every swing. Both stop the wheel scrubbing sideways mid-turn.

**The module keeps its heading when you let go of the stick.**
`HOLD_ANGLE_ON_RELEASE` is true in closed loop, and a position servo holds
anyway. Hold **A** to cut the drive motor.

**`Steer power` reads 0.00 while the module is pointed correctly.**
That is `STEER_TOLERANCE_DEGREES`. Inside 2° the servo is deliberately off.

**The module parks slightly off what the stick asks for near sideways.**
That is the fold hysteresis, up to 15° in a 30° window.

**The wheel does not spin while the module is still more than 90° from the
target.** The cosine scaling is clipped at 0.

**`Encoder volts` jumps from the top of the range to the bottom once per turn.**
That is the seam in the absolute sensor.

**`Module angle` flickers between +179 and −180.**
That is the ±180 wrap sitting where the wheel happens to point. Set the zero.

---

## Troubleshooting

| Problem | Fix |
| --- | --- |
| INIT shows `Detected swerve_servo: NOT FOUND` | The configuration name is wrong, or the servo port is not configured |
| INIT shows `Servo (position mode)` but you wanted CR | Change the device type in the Driver Station configuration. The OpMode will follow |
| Module moves slightly then springs back | Almost always a position servo. See *Read this first*, step 2 |
| Module moves 1–2° only | Position mode with `SERVO_TRAVEL_DEGREES` far too large, or a jam. LB test first |
| Wheel will not turn even with **LB** held | Mechanical or electrical. Not a code problem |
| `Raw encoder` never changes | Feedback wire, analog port, or the `swerve_encoder` configuration |
| `Noise p-p` is several tenths of a volt while still | Floating analog input — the wire is unplugged or broken |
| Angle flickers +179 / −180 | The ±180 wrap. Set `ENCODER_OFFSET_DEGREES` |
| Module spins and never stops, `Error` grows | STOP. Set `STEER_REVERSED = true` |
| Module settles, but stick right points the wheel left | `ENCODER_REVERSED` is wrong. Do **not** reach for `STEER_REVERSED` |
| Wheel faces forward but rolls backward | Set `DRIVE_REVERSED = true` |
| Every angle is off by the same amount | Redo the zero |
| Small angles right, big angles drift further off | CR: `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE`. Position: `SERVO_TRAVEL_DEGREES` |
| `Servo position` shows `(CLIPPED - out of travel)` | `SERVO_TRAVEL_DEGREES` is too small, or `SERVO_CENTER` is far off centre |
| `!! STALLED / NO FEEDBACK` | See *Read this first*, step 4 |
| `Homing: TIMEOUT, still N deg off` | The module could not reach forward in 4 s. Gains, a jam, or feedback |
| Module hunts around the target | Lower `STEER_KS`, then `STEER_KP` |
| Module slams 180° back and forth near sideways | Raise `FOLD_HYSTERESIS_DEGREES` |
| Module points correctly but the wheel does not spin | `swerve_motor` wiring, `Motor swerve_motor: NOT FOUND` on the INIT screen, or `DRIVE_POWER_SCALE` is 0 |
| Numbers right on the bench, wrong on the robot | The horn slipped. Redo the zero after any mechanical work |

---

## See also

- `SwerveServoBenchTest.java` — **Swerve Servo Bench Test**, a pure open-loop
  diagnostic with stepped test powers and a moved-since-press readout. Use it when
  you want to poke the servo without any caster logic at all. It requires a
  **CR servo** and an encoder.
- `README.md` in this folder — the two testers side by side.
- `swerveDriveTesting.java` — **Swerve Drive Testing (1 module)**, the same
  closed-loop steering plus four tank motors. It is **CR-only** and has no
  runtime detection.
