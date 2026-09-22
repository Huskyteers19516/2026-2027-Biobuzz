# Single Swerve Module

**Driver Station name:** `Single Swerve Module` (TeleOp, group *Testing*)
**Code:** `SingleSwerveTeleOp.java`

Tests one swerve module on its own. Push the left stick in a direction and the
module points that way, then drives. The further you push the stick, the faster
it drives.

This tester now steers with a **normal position servo**. There is no analog
encoder and no feedback loop: the code tells the servo where to go and trusts it
to get there. Read *Limits of open-loop steering* at the bottom before you rely
on it.

Use this tester for bench testing a module by itself. If the module is mounted on
the tank test robot, use **Swerve Drive Testing (1 module)** instead — see
`README.md` in this folder. **That OpMode needs a different configuration** (CR
servo plus an analog encoder). Do not assume one configuration works for both.

---

## Quick start

1. Configure `swerve_motor` and `swerve_servo` (below). There is **no**
   `swerve_encoder` for this OpMode.
2. Lift the module off the table so the wheel spins freely.
3. INIT → the servo snaps to its center position. The wheel should be pointing
   straight forward. If it is not, do the full setup below.
4. START → hold **A** → dpad left/right until the wheel is exactly forward.
5. Copy `Center trim` into `SERVO_CENTER`, reinstall.
6. Release A, push the left stick gently in each direction.

Everything after this section explains each step in detail.

---

## Hardware

| Part | Job |
| --- | --- |
| DC motor | Spins the wheel. Its encoder cable is **not** needed |
| Position servo (Axon in standard mode, or a hobby servo) | Turns the module |

Nothing is plugged into an analog port. The Axon's feedback wire, if the module
still has one, is simply unused by this OpMode.

The servo has no feedback, so the code has no idea where the module actually
points. It only knows where it last **told** the servo to go.

## Robot configuration

| Name (exact) | Device type |
| --- | --- |
| `swerve_motor` | DC Motor |
| `swerve_servo` | **Servo** (not Continuous Rotation Servo) |

If `swerve_servo` is configured as a Continuous Rotation Servo, INIT fails with
*Unable to find a hardware device with name "swerve_servo" and type Servo*. In
the SDK a CR servo is a completely different kind of device, so the OpMode never
starts — it cannot run away. Change the device type in the Driver Station
configuration, not the code.

A wrong name makes INIT fail with *Could not find a hardware device*.

## Angles

Looking down at the module from above:

| Angle | Wheel points |
| --- | --- |
| 0° | Forward |
| +90° | Right |
| −90° | Left |

**The module never turns past ±90°.** A standard servo has limited travel, so a
stick push toward the back is handled by pointing the wheel the *opposite* way
within ±90° and running the drive motor backward. See *Things that look wrong but
are normal*.

The decision to flip is **sticky**. Pointing the stick almost exactly sideways
sits right on the boundary between "point right and drive forward" and "point
left and drive backward", and stick noise alone is enough to cross it. Without
stickiness the servo would be slammed nearly half a turn back and forth every
loop. So the code only flips once the stick passes 105° from forward, and only
flips back once it comes inside 75° (`FOLD_HYSTERESIS_DEGREES`). Inside that 30°
band the wheel simply parks at ±90° and keeps the direction it already had.

## Controls (gamepad 1)

| Input | What happens |
| --- | --- |
| Left stick, any direction | Module points that way (folded into ±90°) and drives |
| Left stick, how far | Speed. Nothing happens until the stick passes 15% |
| Let go of the stick | Motor stops. The servo **holds** its last angle |
| Hold **A** | Calibration mode: motor off, servo commanded to center |
| Dpad left / right **while holding A** | Nudge the center by 0.002 servo units |

---

## The screens

### During INIT (before START)

| Line | Meaning |
| --- | --- |
| Servo position | The position command sent to the servo, 0–1 |
| Center trim | The center position in use right now |
| Reachable | The largest right / left angle the servo can actually reach with the current center and travel. If either number is under 90, the module cannot make a full ±90° |
| Extended PWM | `on` only if `USE_EXTENDED_PWM` is true and the servo supports it |

### Driving (stick pushed)

| Line | Meaning |
| --- | --- |
| Stick | Raw stick position, x (right +) and y (forward +) |
| Target angle | The commanded angle, always between −90° and +90° |
| Servo position | The position command sent to the servo, 0–1 |
| Center trim | The center position in use right now |
| Ramp | Drive power multiplier while the servo is still swinging, 0 → 1 |
| Drive power | Power sent to the wheel motor. May show `(reversed)` — that is normal |
| `!! OUT OF SERVO RANGE` | Only appears when the commanded angle is past what the servo can reach. The servo is sitting at the end of its travel, so the real wheel angle is **less** than `Target angle` says. See troubleshooting |

### Stick released

| Line | Meaning |
| --- | --- |
| Stick | `released` |
| Held angle | The last angle commanded. The servo is still holding it |
| Servo position | The position still being commanded |
| Drive power | 0.00 |

### Calibration mode (holding A)

| Line | Meaning |
| --- | --- |
| Servo position | Whatever the trimmed center works out to |
| Center trim | The number to copy into `SERVO_CENTER` |
| Reachable | Right / left angles still reachable after the trim. Trimming the center off 0.5 takes travel away from one side |

---

## First-time setup

Do these in order. Keep a hand on STOP from step 4 on.

### 1. Lift the module
The wheel must not touch anything. An open-loop servo will happily push the
module into a hard stop, so give it room.

### 2. Mechanical zero
With the robot **off**, set the servo horn on the module so that the wheel points
straight forward when the servo is near the middle of its travel. Get this as
close as you can by hand — the trim in step 3 is for the last degree or two, not
for a horn that is a whole spline tooth off.

### 3. Trim `SERVO_CENTER`
1. Press INIT, then START, then hold **A**.
2. Use dpad left/right until the wheel is exactly straight forward.
3. Read `Center trim`. Put that number into `SERVO_CENTER` at the top of
   `SingleSwerveTeleOp.java`.
4. Press Run in Android Studio to reinstall.

Dpad trims are lost when the OpMode stops, so always copy the final number into
the file.

### 4. Check `SERVO_REVERSED`
Release A. Push the left stick a little to the **right**. The wheel must turn to
the **right**.

If it turns left, set `SERVO_REVERSED = true` and reinstall. Recheck the center
afterwards — reversing mirrors the angles around the center, it does not move the
center itself.

### 5. Check `DRIVE_REVERSED`
Push the stick straight forward. The wheel should face forward and roll forward.
If it faces forward but rolls **backward**, set `DRIVE_REVERSED = true` and
reinstall.

### 6. Check `SERVO_TRAVEL_DEGREES`
This is the one number a standard servo cannot tell you, and getting it wrong
makes every angle wrong in proportion.

1. Push the stick straight **right** and hold it. That commands +90°.
2. Look at the module. The wheel should be at a real 90° to straight forward.
3. Repeat straight **left** for −90°.

| What you see at a commanded 90° | Fix |
| --- | --- |
| Wheel turns noticeably **less** than 90° | `SERVO_TRAVEL_DEGREES` is too big — lower it |
| Wheel turns noticeably **more** than 90° | `SERVO_TRAVEL_DEGREES` is too small — raise it |
| Servo buzzes or hits a hard stop | Travel is too small for ±90°, or the horn is off-center. Redo steps 2–3 |
| `!! OUT OF SERVO RANGE` on the screen | The commanded angle is past the end of the servo's travel, so it is not reaching 90° no matter what the constant says. Check `Reachable` — if one side is much smaller than the other the **center trim** is off, not the travel |

An Axon in standard mode is about 270°. A typical hobby servo is about 180°. If
the module is geared (the servo turns more than the module does), the correct
value is the servo's travel divided by the gear ratio — measure it, do not guess.

### 7. Check all four directions
Push forward, right, backward, left. Forward and backward should both point the
wheel forward, with the motor running the opposite way for backward.

---

## Settings

All of these are at the top of `SingleSwerveTeleOp.java`. Reinstall after any
change.

| Constant | Default | What it does |
| --- | --- | --- |
| `SERVO_TRAVEL_DEGREES` | 270.0 | How many degrees the module turns across the servo's full 0–1 range. 270 for an Axon in standard mode, 180 for a typical hobby servo. Set in setup step 6 |
| `SERVO_CENTER` | 0.5 | Servo position where the wheel points straight forward. Set in setup step 3 |
| `SERVO_REVERSED` | false | Flip if the wheel turns the wrong way |
| `DRIVE_REVERSED` | false | Flip if the wheel rolls backward when pointing forward |
| `STICK_DEADZONE` | 0.15 | How far the stick must move before anything happens |
| `DRIVE_POWER_SCALE` | 1.0 | Wheel speed limit. 0.5 = half speed |
| `TRIM_STEP` | 0.002 | How much one dpad press moves the center |
| `SERVO_SLEW_SECONDS` | 0.25 | How long a full 90° swing is assumed to take. Used only for the drive power ramp |
| `FOLD_HYSTERESIS_DEGREES` | 15.0 | How far past sideways the stick must go before the wheel flips to the other side and the motor reverses. Stops the module flapping when the stick sits near sideways. Raise it if it still flips too eagerly |
| `USE_EXTENDED_PWM` | false | Widen the servo's pulse range (see below) |
| `PWM_LOWER_MICROSECONDS` | 500.0 | Low end of the extended pulse range |
| `PWM_UPPER_MICROSECONDS` | 2500.0 | High end of the extended pulse range |

### Extended PWM range

Servos like the Axon reach their full travel only with a pulse range wider than
the default. Setting `USE_EXTENDED_PWM = true` applies
`PWM_LOWER_MICROSECONDS`–`PWM_UPPER_MICROSECONDS` to the servo at INIT, if the
hub supports it. Telemetry shows `Extended PWM: on` when it was applied, and
`off` when it was not.

Changing this changes the real travel, so **redo setup steps 3 and 6** after
turning it on or off.

Do not widen the range on a servo whose datasheet does not allow it. Too wide a
pulse drives the servo into its internal stops and it will buzz and overheat.

---

## How the drive power ramp works

There is no feedback, so the code cannot tell when the servo has finished
turning. Instead it keeps a **guess at where the servo is**, and moves that guess
toward the commanded angle at a fixed speed — 90° per `SERVO_SLEW_SECONDS`, so
360° per second with the default 0.25.

Drive power is then scaled by how close the guess is to the command: exactly on
it gives full power, 90° behind gives zero. `Ramp` in telemetry shows that
multiplier.

The point of tracking a position rather than timing each jump is that it behaves
sensibly whichever way you move the stick:

| What you do | What happens |
| --- | --- |
| Flick the stick 90° in one frame | The guess is far behind, so power starts near 0 and reaches full after about `SERVO_SLEW_SECONDS` |
| Sweep the stick around steadily | The guess trails just behind, so you keep most of your power instead of losing it all |
| Nudge the stick mid-swing | The guess is unaffected, so the ramp already in progress keeps running instead of restarting short |
| Let go mid-swing | The motor stops, but the guess keeps catching up, so driving again immediately does not get a false full-power start |

This is still a guess, not a measurement. If the wheel scrubs sideways after a
big stick swing, raise `SERVO_SLEW_SECONDS`. If it feels sluggish to start, lower
it.

---

## Things that look wrong but are normal

**The wheel drives backward when you push the stick backward.**
The module only turns within ±90°, so a backward target is handled by keeping the
wheel pointed forward and running the motor in reverse. It moves the same way.
Telemetry shows `(reversed)`.

**The wheel speeds up gradually after a big turn.**
That is the drive power ramp described above, so the wheel does not scrub
sideways while the servo is still swinging.

**The module stays put when you let go of the stick.**
A position servo holds its last command. This is deliberate — the module keeps its
heading like a caster instead of flopping. You cannot turn it by hand while the
OpMode is running.

**Nothing happens with a tiny stick push.**
That is the 15% deadzone (`STICK_DEADZONE`).

**The servo snaps to center the instant you press INIT.**
Expected. Keep fingers out of the module when you press INIT.

---

## Troubleshooting

| Problem | Fix |
| --- | --- |
| INIT fails: *Could not find a hardware device* | A configuration name is wrong |
| INIT fails: *Unable to find a hardware device with name "swerve_servo" and type Servo* | `swerve_servo` is configured as a Continuous Rotation Servo. Change it to Servo |
| Module twitches hard back and forth when the stick is held sideways | The servo fitted is physically in CR mode, or the horn is binding. This is **not** the fold boundary — that has hysteresis and cannot chatter |
| Stick right makes the wheel face left | Set `SERVO_REVERSED = true`, then recheck the center |
| Wheel faces forward but rolls backward | Flip `DRIVE_REVERSED` |
| Wheel is consistently a few degrees off in both directions | Redo the center trim (setup step 3) |
| Small angles are right but 90° falls short or overshoots | `SERVO_TRAVEL_DEGREES` is wrong (setup step 6) |
| 90° falls short on **one side only**, `!! OUT OF SERVO RANGE` showing | The center trim pushed the usable range off one end. Re-centre the horn mechanically (setup step 2) so the trim can stay near 0.5, rather than lowering `SERVO_TRAVEL_DEGREES` |
| Servo buzzes and gets hot while holding | It is jammed against a hard stop or the module. Power down and check travel and the horn position |
| Module points correctly but wheel does not spin | `swerve_motor` wiring, or `DRIVE_POWER_SCALE` is 0 |
| Wheel scrubs sideways right after a big stick swing | Raise `SERVO_SLEW_SECONDS` |
| Module does not reach full travel | Try `USE_EXTENDED_PWM = true`, then redo steps 3 and 6 |

---

## Limits of open-loop steering

The old version of this tester ran a CR servo against an analog encoder, so the
code always knew the module's real angle. This version does not. That costs you
three things, and you have to work around them by hand.

**No way to detect a stalled or blocked module.**
If a wire snags the module, the servo horn strips, or the module jams against the
frame, the code sees nothing. Telemetry keeps reporting the commanded angle and
the drive motor keeps pushing on a wheel pointing the wrong way. Watch the module
itself, not the screen, and stop if it does not move.

**The zero has to be re-checked mechanically.**
With feedback, the zero was a number you read off the encoder. Now it is a
physical relationship between the servo horn and the wheel. Any time the horn is
removed, slips, or is remounted a spline tooth over, every angle shifts and
nothing in telemetry says so. Re-do setup steps 2 and 3 after any mechanical work
on the module.

**A wrong `SERVO_TRAVEL_DEGREES` makes every angle proportionally wrong.**
The code converts degrees to a servo position by dividing by this constant. If it
is 180 and the real travel is 270, every angle comes out at two thirds of what you
asked for — 90° becomes 60°, and small angles are wrong by too little to notice
until the module is on a real drivetrain. Verify it with the ±90° check, do not
copy it from another robot.

**Also:** the commanded angle is folded into ±90° relative to the *zero*, not
relative to where the module currently points. With feedback the module took the
shorter path from its actual angle. Now it always swings back through center, so
a hard left-to-right stick flick is a full 180° of servo travel.

Because that fold is decided against a fixed boundary instead of a lagging
measurement, it needs the hysteresis described under *Angles* to stay still near
sideways. The cost is that in a 30° band around sideways the wheel parks at ±90°
and is up to 15° off what the stick asks for. The closed-loop version had no such
band. If you are testing behaviour right at sideways, approach it from one side
and note which way you came from.

If you need any of this back, use the CRServo version described in `README.md`.
