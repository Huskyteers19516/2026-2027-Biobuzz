# Single Swerve Module

**Driver Station name:** `Single Swerve Module` (TeleOp, group *Testing*)
**Code:** `SingleSwerveTeleOp.java`

Push the left stick in a direction and the module turns to face that way and
drives, like the caster wheel under an office chair. Push the stick further to go
faster.

There is **one** control scheme: a closed-loop caster on a **continuous-rotation
servo** with an **analog feedback** signal. Nothing is detected at runtime and
there are no fallback modes.

---

## Read this first: the servo is geared down to the module

The Axon MINI's analog feedback wire reports the **servo shaft** angle, not the
module angle, and the module is **geared down** from the servo. Turning the
module by about 10° moved the raw reading by about 80°, so this module runs at
roughly **8 servo degrees per module degree**.

That number lives in one constant:

```java
private static final double STEERING_RATIO = 8.0;
```

Everything the driver sees and everything the loop controls — target, error,
tolerances, homing, the watchdog, the gains — is in **module degrees**. The
encoder is the only thing that speaks servo degrees, and the code divides by
`STEERING_RATIO` on the way in.

**Measure your real ratio before anything else** (first run, step 2). The default
8.0 is a guess from one hand measurement. If `STEERING_RATIO` is set to zero or a
negative number the OpMode falls back to 1.00 and says so on every screen.

---

## The angle model

Three different angles, and it is worth keeping them apart:

| Name | Where it comes from | Range |
| --- | --- | --- |
| `Servo raw` | The analog voltage, scaled to 0…360° | 0…360, wraps |
| `Servo unwrapped` | Running total of `Servo raw` steps, never wraps | any value |
| `Module angle` | `(Servo unwrapped − zero) / STEERING_RATIO`, normalized | −180…+180 |

**Why the unwrapping is necessary.** With a ratio of 8 the servo turns 8 full
revolutions for one module revolution, so the analog reading passes through every
voltage 8 times per module turn. A single reading **cannot** tell you which of the
8 sectors the module is in. The absolute sensor is no longer absolute at the
module.

So the code keeps a running total instead. Every loop it takes the difference
between this reading and the previous one, wraps that **difference** into
−180…+180, and adds it to `Servo unwrapped`. Crossing the raw 0/360 seam in
either direction is just a normal small step, so the total keeps counting
correctly through as many turns as you like.

**The assumption this makes:** the servo must move **less than 180 servo degrees
between two loops**. Faster than that and the wrapped difference points the wrong
way, the total jumps a whole turn, and the module angle is wrong by
360/`STEERING_RATIO` = 45° until you re-zero. At a normal 50–100 Hz OpMode loop
and any speed a CR servo can actually reach, the real per-loop step is a few
degrees.

**How the alias warning actually works, and what it cannot see.** There are two
independent tests, because neither one alone is enough:

- **The loop-period test.** `Slowest loop` is the longest single loop this run.
  If a loop takes longer than 180 / `MAX_SERVO_DEGREES_PER_SECOND` (0.300 s at
  the default 600 °/s), a servo running flat out could have moved more than 180
  servo degrees inside it, so the screen warns. This is the test that catches the
  dangerous case, because it does not look at the encoder at all.
- **The wrapped-step test.** `Biggest wrapped servo step` is the largest
  *wrapped* difference seen, and it warns above 90. Read the label literally:
  the number is the difference **after** wrapping, so it can never exceed 180 no
  matter how fast the servo moved. A true step of 300° shows up as 60°, and a
  true step of 359° shows up as 1°. This test only notices true steps that land
  roughly between 90° and 270°; a step near a whole servo turn is **invisible**
  to it. That is why the loop-period test exists.

Both counters, and both worst-case figures, are **reset when you press B**, so
they always describe what has happened since the last zero.

What breaks the unwrapping in practice: a very slow loop (heavy telemetry, a
stalled Driver Station), or spinning the module by hand at speed while you are in
calibration mode. Neither is corrected automatically — the module angle simply
shifts by 45° until you point the wheel forward and press **B**.

**Drift:** the running total is a sum of differences, so the noise in each
reading cancels with the next one. A noisy encoder makes `Module angle` jitter,
but it does **not** walk away while the module sits still.

**Normalization:** `Module continuous` is the raw accumulated value and can read
540° after a few turns. Control uses **`Module angle`, normalized to −180…+180**.
Everything — the stick target, the fold, the error, homing, the watchdog — works
on that normalized value, exactly as it did before the ratio existed, so the fold
to the shorter path is unchanged. `Module continuous` is telemetry only.

---

## Zeroing: the reference is now relative

There is **no** `ENCODER_OFFSET_DEGREES` any more. It has been **deleted**, not
kept for later. A stored absolute zero cannot work with a ratio: the raw reading
that means "forward" also means seven other module headings.

Instead the zero is **taken, not stored**:

1. **Point the wheel straight forward by hand before you press START.**
2. At START the code sets `zero = Servo unwrapped`, so `Module angle` reads 0.
3. **Press B at any time** to take the zero again wherever the wheel is pointing
   right now. Do that after a hand-turn, after an alias warning, or any time
   `Module angle` disagrees with the wheel.

The zero lives only until the OpMode stops. There is nothing to copy into the
file, which also means there is nothing to get stale after mechanical work.

Dpad left/right while holding **A** nudges the same zero by
`TRIM_STEP_DEGREES` (0.5) **module** degrees — dpad right makes `Module angle`
read higher. It is for the last degree of squaring up, not for setting the zero.

Because the zero is taken at START, the home that runs at START now finishes
immediately — `Module angle` is already 0 at that instant. Nothing moves at
START. The first real motion is the stick, or a **Y** home after you have turned
the module away.

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
3. **INIT.** Nothing is powered. Turn the wheel by hand — `Servo raw` must
   change.
4. **Point the wheel straight forward by hand.** This is now part of starting the
   OpMode, not an optional nicety.
5. **On the very first run, hold A while you press START**, and keep holding it
   while you do *First run* steps 1–4. Holding **A** cuts all power.
6. Hold **LB** and push the **right** stick left and right. The wheel must
   physically turn. If it does not, stop here and fix the mechanics or the
   wiring.
7. Release **LB** and push the left stick gently in each direction.

Nothing is commanded during INIT.

---

## Controls (gamepad 1)

| Input | What happens |
| --- | --- |
| Left stick, any direction | The module steers to face that way and drives |
| Left stick, how far | Speed. Nothing happens until the stick passes 15% (`STICK_DEADZONE`) |
| Let go of the stick | Drive motor stops. The module **holds** the last angle |
| Hold **LB** | **Manual override.** **Right** stick x is raw servo power, no loop, no watchdog. Drive motor off |
| Press **Y** | Home to forward (0°) and stop there |
| Press **B** | **Re-zero.** The wheel's current heading becomes 0°. Works in every mode, including INIT |
| Hold **A** | Calibration and ratio measurement. Everything off, live dpad trim, raw readouts |
| Press **X** **while holding A** | Restart the ratio measurement from the current position |
| Dpad left / right **while holding A** | Nudge the zero by 0.5 module degrees (`TRIM_STEP_DEGREES`) |

Manual override is on the **right** stick on purpose. The caster reads the
**left** stick, so letting go of **LB** with the manual stick still pushed over
cannot hand the caster a target you did not ask for.

Releasing **LB**, releasing **A**, and the end of homing all leave the module
holding **the angle it is actually at**, so a button release never slams the
servo anywhere. None of them touch the zero or the unwrapped total, which keep
counting through every mode.

**B** re-zeros and then holds 0°, which is where the wheel already is, so it
never slams either. If a home was running, **B** cancels it (`cancelled by
re-zero`).

**A** and **LB** win over **Y** and the dpad. A **Y** press made while you are
holding either one is discarded rather than stored, so nothing fires the instant
you let go. Dpad trim and **X** only count while **A** is held.

---

## Angles

Looking down at the module from above:

| Angle | Wheel points |
| --- | --- |
| 0° | Forward |
| +90° | Right |
| −90° | Left |

`Module angle`, `Target angle`, `Held angle` and `Error` are **module** degrees
in −180°…+180°. `Error` also shows the servo travel it implies, because that is
what the servo actually has to do: 90 module degrees is 720 servo degrees at a
ratio of 8.

`Servo raw` is 0°…360° and wraps. `Servo unwrapped`, `zero` and
`Module continuous` are unbounded running values and are *expected* to be large.

---

## How the caster works

- The stick direction becomes a target angle with `atan2(x, y)`.
- The target is **folded to the shorter path from the measured angle**: a target
  more than 105° away is reached by pointing the wheel the opposite way and
  running the drive motor backwards instead. Telemetry shows `(reversed)`.
- The fold is **sticky**. It only flips once the error passes 105°
  (90 + `FOLD_HYSTERESIS_DEGREES`), so noise at exactly 90° cannot make the module
  slam 180° back and forth. All of this is in module degrees, on the normalized
  module angle, so the ratio does not change it.
- A **PD + static-friction loop** drives the CR servo until the *measured* angle
  matches the target. The damping term is taken on the **measured angle**, not on
  the error, so moving the stick does not throw a one-loop full-power spike at
  the servo. It is now a **module** angular velocity in module degrees per second
  — about 8× smaller than the old servo-level number, so `STEER_KD` has to grow
  with the ratio to mean the same damping. The shipped 0.002 is a deliberately
  soft starting point rather than the full 8× (which would be 0.0048); raise it
  during tuning if the module wobbles.
- Drive power is scaled by `cos(error)` clipped to [0, 1], so the wheel fades in
  as the module lines up and does not turn at all while the module is more than
  90° away.
- Releasing the stick stops the wheel and **holds the last angle**. Push the
  module off heading by hand and the servo pushes back. Hold **A** to make
  everything go limp.
- At START the zero is taken and the module homes to 0°, which it is already at,
  so nothing moves. On **Y** it really drives to 0°, with a
  `HOME_TIMEOUT_SECONDS` (6 s) timeout that reports how far off it ended **in
  module degrees**. The timeout is longer than it used to be because a geared
  module covers module degrees 8× slower than the servo turns. Homing is under
  the **same stall watchdog** as the caster, so a home that runs away on a wrong
  `STEER_REVERSED` is cut after `STALL_TIME_SECONDS` (2 s) and reports `STALLED`.
- The **stall watchdog** cuts steering and drive when steering has been commanded
  above `STALL_POWER_THRESHOLD` for `STALL_TIME_SECONDS` without the error
  shrinking by `STALL_MOVE_DEGREES` — now **1.0 module degree**, which is 8 servo
  degrees of real travel. It watches *progress*, not movement, so a dead feedback
  wire trips it and so does a runaway. The latch clears as soon as you push the
  stick in a **new** direction, and holding **A**, pressing **Y** or pressing
  **B** clears it too.

---

## Telemetry

Caster screen:

| Line | Meaning |
| --- | --- |
| Steering ratio | The `STEERING_RATIO` in use. Replaced by a warning if it is not positive |
| Stick | Raw stick position, or `released` |
| Module angle | Where the module points now, normalized to −180…+180. **This is what the loop controls** |
| Module continuous | The same angle without normalizing. 400° means the module has been round once and a bit |
| Target angle / Held angle | Where it is trying to point, in module degrees |
| Error | Target minus actual, in module degrees, with the servo travel that implies |
| Module speed | Measured angular velocity in module degrees per second. This is what `STEER_KD` multiplies |
| Steer power | Power sent to the CR servo. 0.00 inside `STEER_TOLERANCE_DEGREES` |
| Drive power | Power sent to the wheel motor. `(reversed)` is normal |
| Servo unwrapped | The running servo total and the zero it is measured from |
| Encoder volts | The raw analog voltage right now |
| Homing | Result of the last home: `done in 0.8 s`, `TIMEOUT, still 43.2 module deg off`, `STALLED 43.2 module deg off - check STEER_REVERSED`, `cancelled`, or `cancelled by re-zero` |
| Noise p-p | Peak-to-peak voltage over about the last `NOISE_WINDOW_SECONDS`. It is an **encoder-level** figure, so it is shown in servo degrees, with the module-level equivalent (servo ÷ ratio) beside it. 8 servo degrees of noise is only 1 module degree |
| Biggest wrapped servo step | The largest single-loop servo step **after wrapping**, since the last zero. It can never exceed 180, so a true step near a whole servo turn shows up as a small number here — see *The angle model* |
| Slowest loop | The longest single loop since the last zero, and the loop time above which a full-speed servo could alias the unwrapping (180 / `MAX_SERVO_DEGREES_PER_SECOND`) |
| `!! Unwrap may have aliased` | Either a wrapped step passed 90 servo degrees or a loop ran slow enough to alias. The unwrapped total may have skipped a servo turn, which is 360/ratio = 45 module degrees. Point the wheel forward and press **B**, which also clears both counters |
| `!! STALLED / NO FEEDBACK` | The watchdog fired. Steering and drive are cut |

Homing screen (at START and after **Y**):

| Line | Meaning |
| --- | --- |
| Status | The same string as `Homing`, live: `running` until it finishes |
| Error to forward | How far the module still is from 0°, signed, in module degrees |

Manual override screen (**LB**):

| Line | Meaning |
| --- | --- |
| Servo power | The raw power going to the CR servo, straight from the **right** stick x |

Calibration and ratio screen (**A**):

| Line | Meaning |
| --- | --- |
| Servo moved | How far `Servo unwrapped` has moved since you pressed **A** (or last pressed **X**). This is the ratio measurement |
| Implied ratio | `abs(Servo moved) / RATIO_REFERENCE_DEGREES`. **Only meaningful if you just turned the module by exactly that reference angle** — the screen says so in capitals, because the number is happy to be nonsense |
| Zero taken | How many times the zero has been taken this run |
| Trim | Reminder of the dpad step, in module degrees |
| Volts seen | Lowest and highest voltage since the OpMode started. Turn the module slowly and these become your real `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` |
| Analog range in use | What the constants are set to now, for comparison. Also on the INIT screen, where it adds the hub's own maximum |
| Encoder reversed | The value of `ENCODER_REVERSED` |

---

## First run

Do these **in order**, with the module lifted off the table. Each step assumes
the ones above it are already right.

**Point the wheel straight forward by hand, then hold A through steps 1 to 4,
including the moment you press START.** Holding **A** keeps everything off.

### 1. Is the encoder alive?

INIT (or hold **A** after START — both cut all power) and turn the module by hand.

- `Servo raw` and `Encoder volts` must both change smoothly. At a ratio of 8,
  a small module turn makes `Servo raw` race round and wrap several times. That
  is the gearing, not a fault.
- `Noise p-p` must stay small while you hold the module still.

If `Servo raw` never moves, the feedback path is dead: check the feedback wire,
the analog port, and that `swerve_encoder` is configured as **Analog Input**.
Nothing below this step can work until it moves.

While you are here, turn the module slowly and read `Volts seen` — it is on both
the INIT screen and the calibration screen. If it is not
close to 0.00 – 3.30 V, put the real numbers into `ANALOG_MIN_VOLTAGE` and
`ANALOG_MAX_VOLTAGE` and reinstall. Do this **before** measuring the ratio: a
wrong voltage span scales every angle, including the ratio you are about to
measure.

### 2. Measure `STEERING_RATIO`

Still holding **A**. You need something square — a block against the module, a
line on the tile, anything that gives you a repeatable 90°.

1. Square the wheel up at a known heading.
2. Press **X**. `Servo moved` resets to 0.
3. Turn the module **by hand, by exactly 90°**, to the next square face.
   `Servo moved` is **net** travel from where you pressed **X**, so overshooting
   and coming back to the square face is fine — it corrects itself. What does
   spoil the reading is stopping short of the face, or nudging the module after
   you have read the number. Turn slowly: spinning it fast by hand is the one
   thing that can alias the unwrapping.
4. Read `Implied ratio`. With the real ratio at 8 you should see `Servo moved`
   around 720 and `Implied ratio` around 8.00.
5. Repeat it two or three times. The readings should agree within a few percent.
6. Put the number into `STEERING_RATIO` and reinstall.

`RATIO_REFERENCE_DEGREES` (90.0) is what `Implied ratio` divides by. Change it if
your square gives you a different known angle — 180° is even better if you have a
stop for it, because hand error matters half as much.

Getting this wrong scales every error, every tolerance and every gain by the same
factor. Too small and the module creeps and never arrives; too large and it
overshoots, gets pulled back and twitches. That twitching is exactly what the old
1:1 code did.

### 3. Which way does the encoder count?

Still holding **A**: **turn the wheel to the RIGHT by hand. `Module angle` must
go UP.**

If it falls, set `ENCODER_REVERSED = true` and reinstall.

This matters more than it looks. With the encoder counting backwards the loop
still settles — on the **mirrored** heading, so stick right points the wheel
left. `STEER_REVERSED` does **not** fix that; it flips the servo, not the
measurement, and with it the loop runs away instead.

### 4. Set the zero

**Keep holding A** — everything stays off — point the wheel straight forward by
hand, and press **B**. `Module angle` must read about 0. Use dpad left/right,
still holding **A**, for the last degree.

Do **not** release **A** first. Releasing it re-arms the closed loop, which then
fights your hand at up to `STEER_MAX_POWER` while you are trying to square the
wheel, and the stall watchdog arms once you are more than about 8 module degrees
off. **B** works in every mode, including while **A** is held.

There is nothing to copy into the file. The zero is taken fresh every run, which
is why step 4 of the quick start says to point the wheel forward before START.

### 5. Which way does the servo turn?

Release **A** and **LB**, then push the left stick a little to the right.
`Error` must get **smaller** every loop and the module must settle.

**If the module spins without stopping, or `Error` grows, press STOP
immediately.** Set `STEER_REVERSED = true` and reinstall. The stall watchdog
also cuts it after about 2 s on its own, but STOP is faster and cannot be
argued with.

If it settles neatly but on the opposite side from the stick, that is step 3, not
this one.

### 6. Which way does the wheel roll?

Push the stick straight forward. The wheel should point forward and roll
**forward**. If it rolls backward, set `DRIVE_REVERSED = true`.

### 7. Tune the gains

Only once everything above is right, and **the gains must be retuned** — see
*Tuning*. The defaults were changed when the ratio went in and they are a
starting point, not a setting.

---

## Constants

All at the top of `SingleSwerveTeleOp.java`. Reinstall after any change.

### Gearing

| Constant | Default | What it does |
| --- | --- | --- |
| `STEERING_RATIO` | 8.0 | Servo degrees per module degree. **Measure it** (first run step 2). Zero or negative falls back to 1.00 with a warning on screen |
| `RATIO_REFERENCE_DEGREES` | 90.0 | The known module angle you hand-turn during the measurement. `Implied ratio` divides by this |

### Directions and feel

| Constant | Default | What it does |
| --- | --- | --- |
| `STEER_REVERSED` | false | Set true when the servo drives the module **away** from the target |
| `DRIVE_REVERSED` | false | Set true when the wheel rolls backward while pointing forward |
| `STICK_DEADZONE` | 0.15 | How far the stick must move before anything happens |
| `DRIVE_POWER_SCALE` | 1.0 | Wheel speed limit. 0.5 = half speed |
| `FOLD_HYSTERESIS_DEGREES` | 15.0 | Module degrees past 90° the error must go before the module flips to the other side and the motor reverses |

### Encoder

| Constant | Default | What it does |
| --- | --- | --- |
| `ENCODER_REVERSED` | false | Set true when the analog reading **falls** as the wheel turns right. First-run step 3 |
| `ANALOG_MIN_VOLTAGE` | 0.0 | Voltage the Axon reports at its lowest angle |
| `ANALOG_MAX_VOLTAGE` | 3.3 | Voltage the Axon reports at its highest angle |
| `TRIM_STEP_DEGREES` | 0.5 | How much one dpad press moves the zero, in **module** degrees |
| `NOISE_WINDOW_SECONDS` | 1.0 | Window for the `Noise p-p` figure |
| `ALIAS_WARN_SERVO_DEGREES` | 90.0 | A single-loop **wrapped** servo step bigger than this raises the alias warning. The hard limit where unwrapping actually breaks is 180 |
| `MAX_SERVO_DEGREES_PER_SECOND` | 600.0 | Your servo's free speed. A loop longer than 180 divided by this raises the alias warning, because a servo at full speed could have moved more than half a turn inside it. Lower it if your servo is slower and you want a later warning |

There is **no `ENCODER_OFFSET_DEGREES`**. It was removed rather than kept for a
1:1 case, because a stored absolute zero is meaningless once the servo wraps more
than once per module turn. See *Zeroing*.

### Steering loop

All of these are in **module** degrees.

| Constant | Default | What it does |
| --- | --- | --- |
| `STEER_KP` | 0.05 | Power per **module** degree of error. Saturates at `STEER_MAX_POWER` around 11 module degrees |
| `STEER_KD` | 0.002 | Damping, taken on the measured angle, in module degrees per second |
| `STEER_KS` | 0.05 | Constant push that gets the module over its own friction. Unit-free, unchanged by the ratio |
| `STEER_TOLERANCE_DEGREES` | 1.0 | Inside this module error the servo is given 0 power. Tighter than the old 2.0, because 1 module degree is already 8 servo degrees |
| `STEER_MAX_POWER` | 0.6 | Power ceiling for the servo. Deliberately below 1.0 as a shipped default for a geared module — raise it once the loop is tuned |

### Watchdog and homing

| Constant | Default | What it does |
| --- | --- | --- |
| `STALL_POWER_THRESHOLD` | 0.45 | Steer power above which the watchdog starts watching. With the default gains that is a module error of about **8°** — `(0.45 − STEER_KS) / STEER_KP`. **It has to move whenever `STEER_KP` moves**, see *Tuning* |
| `STALL_TIME_SECONDS` | 2.0 | How long steering may be commanded above that power without progress |
| `STALL_MOVE_DEGREES` | 1.0 | Module degrees of progress that count as progress. 1 module degree is 8 servo degrees, so this is not a tight ask |
| `HOME_TOLERANCE_DEGREES` | 2.0 | Module degrees from forward that count as done |
| `HOME_TIMEOUT_SECONDS` | 6.0 | Give up homing after this long and report how far off, in module degrees. Longer than the old 4 s because the module moves 8× slower than the servo |

### Analog range

The code turns a voltage into a **servo** angle with
`(volts − ANALOG_MIN_VOLTAGE) / (ANALOG_MAX_VOLTAGE − ANALOG_MIN_VOLTAGE) × 360°`.

It does **not** assume 0 V to the hub maximum, because a real Axon MINI often
swings over a narrower band. Assume a 3.3 V swing on a servo that really delivers
0.1–3.2 V and every servo angle is scaled slightly wrong — and with the ratio in
play that error is multiplied into the module angle on every turn.

Measure yours with `Volts seen` in calibration mode, then redo the ratio.

---

## Tuning

**The gains must be retuned for your module.** The defaults were rescaled for a
ratio of 8 and are only a safe starting point: an error of 90 module degrees is
720 servo degrees of travel, so a gain that means "power per degree" means
something completely different now than it did in the 1:1 code.

Rules of thumb when you change `STEERING_RATIO`:

- **`STEER_KP` scales with the ratio.** Double the ratio and the same feel needs
  roughly double the `STEER_KP`, because the same module error is twice as much
  servo work.
- **`STEER_KD` scales with the ratio too**, for the same reason — `Module speed`
  gets smaller as the ratio grows.
- **`STEER_KS` does not scale.** It is a push against friction, not per degree.
- **`STEER_TOLERANCE_DEGREES` scales the other way.** A bigger ratio means finer
  module resolution, so you can afford a tighter band.
- **`STALL_POWER_THRESHOLD` must move with `STEER_KP`.** The watchdog only
  watches while `|error| ≥ (STALL_POWER_THRESHOLD − STEER_KS) / STEER_KP`.
  Raising `STEER_KP` without raising the threshold pulls that band down toward
  zero, and the watchdog starts cutting the **drive motor** for ordinary small
  parking errors. Keep the band somewhere between roughly half and all of the
  error at which `STEER_MAX_POWER` saturates (11° with the shipped gains).

Start with `STEER_KD = 0`, `STEER_KS = 0`, and `STEER_MAX_POWER` around 0.3 so
mistakes are slow. Push the stick to a fixed direction and watch `Error`.

**While `STEER_MAX_POWER` is below `STALL_POWER_THRESHOLD` (0.45) the stall
watchdog cannot arm at all**, because the steer power never reaches the level it
watches. That is deliberate — a deliberately weak loop is not a stall — but it
means a runaway during tuning will keep going. Keep your hand on **STOP**, or
raise `STEER_MAX_POWER` back above the threshold before trusting the watchdog.

| What you see | Change |
| --- | --- |
| Turns slowly, or stops short of the target | Raise `STEER_KP` |
| Overshoots and wobbles back and forth | Lower `STEER_KP`, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Twitches or hums when already pointed right | Raise `STEER_TOLERANCE_DEGREES` |
| Jerky and noisy at every small correction | Lower `STEER_KD` — D amplifies encoder noise |
| Full-power jolt the instant the stick moves | Not the D term. It is measured on the module angle, not on the error. Look at `STEER_KP` and `STEER_MAX_POWER` |
| Too fast to watch | Lower `STEER_MAX_POWER` |
| Overshoots by a consistent *factor*, or creeps by one | Not a gain. `STEER_TOLERANCE_DEGREES` is fine — go and re-measure `STEERING_RATIO` |

Raise `STEER_MAX_POWER` toward 1.0 when you are happy.

`STEER_KS` is a static push, not a gain: it is added at full size as soon as the
error leaves the tolerance band, so too much of it makes the module hunt.

---

## Troubleshooting

### The module does not turn at all

Work down this list in order.

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
3. **Does `Servo raw` change while the wheel turns under LB?**
   - **No** → the feedback path is dead. Check the analog wire and the analog
     port. The loop was chasing a number that never changes, which is also what
     fires `!! STALLED / NO FEEDBACK`.
   - **Yes** → the hardware is all fine and the *reference* is wrong. Redo first
     run steps 2, 3, 4 and 5 in that order.
4. **Does it turn under LB but creep under the left stick?** Either
   `STEERING_RATIO` is too small — the loop thinks a big move is a small one — or
   the gains are too low for the friction. Check the ratio first, then raise
   `STEER_KS`, then `STEER_KP`.

### Everything else

| Problem | Fix |
| --- | --- |
| INIT fails naming `swerve_servo` and `CRServo` | The device is configured as a plain Servo, or the name is wrong |
| `Servo raw` never changes | Feedback wire, analog port, or the `swerve_encoder` configuration |
| I moved the module 10° and `Servo raw` moved 80° | That is the gearing. It is what `STEERING_RATIO` is for |
| Module overshoots, gets dragged back, twitches | `STEERING_RATIO` too large, or the gains never retuned after changing it |
| Module creeps, never arrives, gives up short | `STEERING_RATIO` too small, or `STEER_KS` too low |
| `Module angle` says 0 but the wheel is not forward | The zero is stale. Point the wheel forward and press **B** |
| `Module angle` jumped by about 45° on its own | 360 ÷ ratio — the unwrap aliased. Check `Slowest loop` first (a long loop is the usual cause and the only reliable signal), then press **B**. `Biggest wrapped servo step` may look innocent even when this happens |
| `!! Unwrap may have aliased` | Either a wrapped step passed 90 servo degrees or a loop ran long enough for a full-speed servo to cross 180. Usually spinning the module fast by hand, or a slow loop. Re-zero with **B**, which also clears the warning and both worst-case figures |
| The alias warning will not go away | It is cleared only by **B**. If it comes straight back, look at `Slowest loop`: something is stalling the loop |
| `Module continuous` reads 500° | Normal. It is the unnormalized running value. `Module angle` is the controlled one |
| The module creeps slowly while you hold **A**, or after homing says `done` | The servo's own CR centre is off. `setPower(0.0)` is a neutral pulse, not an off switch. Trim the centre on the servo itself. It cannot happen at INIT, where the port is never written at all |
| `Noise p-p` is several tenths of a volt while still | Floating analog input — the wire is unplugged or broken. Note the figure is in **servo** degrees; divide by the ratio for what the module sees |
| `Module angle` flickers between +179 and −180 | The ±180 wrap sitting where the wheel happens to point. Point forward and press **B** |
| Module spins and never stops, `Error` grows | STOP. Set `STEER_REVERSED = true` |
| Module settles, but stick right points the wheel left | `ENCODER_REVERSED` is wrong. Do **not** reach for `STEER_REVERSED` |
| Wheel faces forward but rolls backward | Set `DRIVE_REVERSED = true` |
| Every angle is off by the same amount | Redo the zero with **B** |
| Every angle is off by the same *proportion* | Re-measure `STEERING_RATIO` |
| Small angles right, big angles drift further off | Measure `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE` with `Volts seen`, then redo the ratio |
| `Steering ratio` replaced by a warning | `STEERING_RATIO` is zero or negative. The code is running at 1.00 |
| `!! STALLED / NO FEEDBACK` | See *The module does not turn at all*, steps 2 and 3. Push the stick a new way, or hold **A**, to clear the latch |
| `Homing: TIMEOUT, still N module deg off` | The module could not reach forward in 6 s. Gains, ratio, a jam, or dead feedback. The module then holds where it stopped |
| `Homing: STALLED N module deg off - check STEER_REVERSED` | The home was pushing hard and getting no closer. Almost always `STEER_REVERSED`, otherwise a jam or dead feedback |
| `Homing: cancelled by re-zero` | You pressed **B** during a home. Not a fault |
| Module hunts around the target | Lower `STEER_KS`, then `STEER_KP` |
| Module slams 180° back and forth near sideways | Raise `FOLD_HYSTERESIS_DEGREES` |
| Module points correctly but the wheel does not spin | `swerve_motor` wiring, or `DRIVE_POWER_SCALE` is 0 |
| Numbers right on the bench, wrong on the robot | The horn slipped, or the gearing changed. Redo the ratio and the zero after any mechanical work |

---

## Things that look wrong but are normal

**`Servo raw` races round and wraps several times for a small module turn.**
That is the gear ratio. It is the whole reason the code unwraps instead of
reading the encoder as an absolute module position.

**Nothing moves at START.** The zero is taken at START from wherever the wheel is
pointing, so the home that follows is already finished. Press **Y** after turning
the module away to see a real home.

**`Module continuous` keeps growing.** It is a running total and is allowed to.

**The wheel drives backward when you push the stick backward.**
Pointing the wheel forward and reversing the motor is the same motion as turning
180°, and it is a much shorter turn. Telemetry shows `(reversed)`.

**The wheel speeds up gradually as the module lines up.**
Drive power is scaled by `cos(error)`, so the wheel does not scrub sideways
mid-turn.

**The module keeps its heading when you let go of the stick.**
That is deliberate. Hold **A** to cut all power and turn it by hand.

**`Steer power` reads 0.00 while the module is pointed correctly.**
That is `STEER_TOLERANCE_DEGREES`. Inside 1 module degree the servo is
deliberately off.

**The module parks slightly off what the stick asks for near sideways.**
That is the fold hysteresis, up to 15° in a 30° window.

**`Encoder volts` jumps from the top of the range to the bottom several times per
module turn.** That is the seam in the sensor, once per *servo* revolution. Real
noise is a jump in the **voltage** while the module is still; the seam is a jump
in `Servo raw` with a smooth voltage behind it, and `Servo unwrapped` steps
straight through it.

---

## See also

- `SwerveServoBenchTest.java` — **Swerve Servo Bench Test**, a pure open-loop
  diagnostic with stepped test powers and a moved-since-press readout. It reports
  **servo** degrees and knows nothing about the gear ratio.
- `swerveDriveTesting.java` — **Swerve Drive Testing (1 module)**, the same
  closed-loop steering plus four tank motors. It still assumes **1:1** and still
  uses `ENCODER_OFFSET_DEGREES`, so its angles are servo angles and its gains do
  not transfer.
- `README.md` in this folder — the three testers side by side.
