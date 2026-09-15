# Single Swerve Module

**Driver Station name:** `Single Swerve Module` (TeleOp, group *Testing*)
**Code:** `SingleSwerveTeleOp.java`

Tests one swerve module on its own. The module behaves like an office chair
wheel: push the left stick in any direction and the module turns itself to face
that way, then drives. The further you push the stick, the faster it drives.

Use this tester for bench testing a module by itself. If the module is mounted
on the tank test robot, use **Swerve Drive Testing (1 module)** instead — see
`README.md` in this folder.

---

## Quick start

1. Configure `swerve_motor`, `swerve_servo`, `swerve_encoder` (below).
2. Lift the module off the table so the wheel spins freely.
3. INIT → turn the module by hand → `Raw encoder` must change.
4. Point the wheel straight forward → copy `Raw encoder` into
   `ENCODER_OFFSET_DEGREES` → Run again.
5. START → push the left stick gently → `Error` must shrink toward 0.

Everything after this section explains each step in detail.

---

## Hardware

| Part | Job |
| --- | --- |
| DC motor | Spins the wheel. Its encoder cable is **not** needed |
| Axon servo in **continuous rotation (CR) mode** | Turns the module |
| Axon analog feedback wire | Tells the code which way the module points. Plug it into an **analog** port on the hub |

The feedback is absolute: the module knows its angle the moment it powers on.
There is no homing step.

## Robot configuration

| Name (exact) | Device type |
| --- | --- |
| `swerve_motor` | DC Motor |
| `swerve_servo` | Continuous Rotation Servo |
| `swerve_encoder` | Analog Input |

A wrong name makes INIT fail with *Could not find a hardware device*. Fix it in
the Driver Station configuration.

## Angles

Looking down at the module from above:

| Angle | Wheel points |
| --- | --- |
| 0° | Forward |
| +90° | Right |
| −90° | Left |
| ±180° | Backward |

## Controls (gamepad 1)

| Input | What happens |
| --- | --- |
| Left stick, any direction | Module turns to that direction and drives |
| Left stick, how far | Speed. Nothing happens until the stick passes 15% |
| Let go of the stick | Motor and servo both turn off |
| Hold **A** | Calibration mode: everything off, wheel can be turned by hand |
| Dpad left / right **while holding A** | Nudge the zero point by 0.5° |

---

## The screens

### During INIT (before START)

| Line | Meaning |
| --- | --- |
| Module angle | Where the module points, with the zero offset applied |
| Raw encoder | The encoder reading before the offset, 0–360° |

The motors are not powered yet, so this is the safest time to check the encoder.

### Driving (stick pushed)

| Line | Meaning |
| --- | --- |
| Stick | Raw stick position, x (right +) and y (forward +) |
| Module angle | Where the module points now |
| Target angle | Where it is trying to point |
| Error | Target − actual. Should quickly shrink toward 0 |
| Steer power | Power sent to the servo, −1 to 1 |
| Drive power | Power sent to the wheel motor, −1 to 1. May show `(reversed)` — that is normal |

### Stick released

| Line | Meaning |
| --- | --- |
| Stick | `released` |
| Module angle | Where the module points now |
| Held angle | The last target it was aiming at |

### Calibration mode (holding A)

| Line | Meaning |
| --- | --- |
| Raw encoder | Encoder reading before the offset |
| Offset | The zero offset in use right now (changes with the dpad) |
| Module angle | Raw encoder − Offset. Reads 0 when calibrated and pointing forward |

---

## First-time setup

Do these in order. Keep a hand on STOP from step 5 on.

### 1. Lift the module
The wheel must not touch anything.

### 2. Check the encoder is alive
Press INIT. Turn the module slowly by hand. `Raw encoder` must change smoothly.

If it does not change: check the feedback wire is in an **analog** port, and that
`swerve_encoder` is configured as **Analog Input**.

### 3. Check the encoder counts the right way
Still in INIT, turn the module so the wheel points more to the **right**
(clockwise, looking from above). `Module angle` must go **up**.

If it goes **down**, the encoder counts the opposite way from the code. Stop here
and tell a programmer. None of the settings in the file can fix this: pushing the
stick right would make the wheel face left.

### 4. Set the zero
1. Point the wheel straight forward by hand.
2. Press START, then hold **A**.
3. Read `Raw encoder`. Put that number into `ENCODER_OFFSET_DEGREES` at the top of
   `SingleSwerveTeleOp.java`.
4. Press Run in Android Studio to reinstall.
5. INIT again: with the wheel straight, `Module angle` should read about 0.

Fine-tuning option: while holding A, use dpad left/right until `Module angle`
reads 0, then copy the **Offset** value into `ENCODER_OFFSET_DEGREES`. Dpad
changes are lost when the OpMode stops, so always copy the final number into the
file.

### 5. Check steering direction
Release A. Push the left stick a little to the right. `Error` must shrink toward 0
and the module must settle.

**If the module keeps spinning or `Error` grows, press STOP right away.** Set
`STEER_REVERSED = true` and reinstall.

### 6. Check drive direction
Push the stick forward. The wheel should face forward and roll forward. If it
faces forward but rolls **backward**, set `DRIVE_REVERSED = true` and reinstall.

### 7. Check all four directions
Push forward, right, backward, left. The wheel should point and roll that way
each time.

---

## Settings

All of these are at the top of `SingleSwerveTeleOp.java`. Reinstall after any
change.

| Constant | Default | What it does |
| --- | --- | --- |
| `ENCODER_OFFSET_DEGREES` | 0.0 | Raw encoder reading when the wheel points forward. Set in setup step 4 |
| `STEER_REVERSED` | false | Flip if the module spins away from the target |
| `DRIVE_REVERSED` | false | Flip if the wheel rolls backward when pointing forward |
| `STEER_KP` | 0.012 | Main steering strength |
| `STEER_KD` | 0.0006 | Braking that stops overshoot |
| `STEER_KS` | 0.05 | Small constant push to overcome friction |
| `STEER_TOLERANCE_DEGREES` | 2.0 | Close enough — stop steering inside this many degrees |
| `STEER_MAX_POWER` | 1.0 | Steering power limit. Lower it while tuning |
| `STICK_DEADZONE` | 0.15 | How far the stick must move before anything happens |
| `DRIVE_POWER_SCALE` | 1.0 | Wheel speed limit. 0.5 = half speed |
| `TRIM_STEP_DEGREES` | 0.5 | How much one dpad press moves the zero |

## Tuning the steering

Start with `STEER_KD = 0` and `STEER_KS = 0`, and tune `STEER_KP` first.

| What you see | Change |
| --- | --- |
| Turns slowly, or stops well short of the target | Raise `STEER_KP` |
| Overshoots and swings back and forth | Lower `STEER_KP` a little, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Buzzes or twitches when already pointed correctly | Raise `STEER_TOLERANCE_DEGREES` |
| Moves too fast to watch | Lower `STEER_MAX_POWER` while tuning |

Change one number at a time, by about 20–30%, and test each change.

---

## Things that look wrong but are normal

**The wheel sometimes drives backward.**
If the target is more than 90° from where the module points, the module turns to
the opposite direction and drives backward instead. It moves the same way, with
at most a 90° turn. Telemetry shows `(reversed)`.

**The wheel speeds up gradually after a big turn.**
Drive power is reduced while the module is still turning, and reaches full power
once it lines up. That stops the wheel scrubbing sideways.

**The module goes limp when you let go.**
Power is cut on purpose so nothing buzzes and you can turn it by hand. It does not
hold its angle.

**Nothing happens with a tiny stick push.**
That is the 15% deadzone (`STICK_DEADZONE`).

---

## Troubleshooting

| Problem | Fix |
| --- | --- |
| INIT fails: *Could not find a hardware device* | A configuration name is wrong |
| `Raw encoder` never changes | Feedback wire not in an analog port, or `swerve_encoder` is not Analog Input |
| `Raw encoder` jumps around while still | Loose feedback wire or ground |
| Module spins forever | `STEER_REVERSED` is wrong. STOP, flip it |
| Stick right makes the wheel face left | Encoder counts the other way. Needs a code change — tell a programmer |
| Wheel faces forward but rolls backward | Flip `DRIVE_REVERSED` |
| Wheel points about the right way but always a bit off | Redo the zero (setup step 4) |
| Module points correctly but wheel does not spin | `swerve_motor` wiring, or `DRIVE_POWER_SCALE` is 0 |
| Wobbles around the target | See *Tuning the steering* |
| Stops 3–10° short | Raise `STEER_KS` |
