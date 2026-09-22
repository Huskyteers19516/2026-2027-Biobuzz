# Swerve Module Testers

Both OpModes make the swerve module point where the left stick points and then
drive. Push the stick further to go faster.

**They no longer use the same steering hardware.** Read the next section before
you configure anything.

| Name on the Driver Station | Steering hardware | Use it when |
| --- | --- | --- |
| **Single Swerve Module** | **Position Servo**, no feedback | Testing the module by itself |
| **Swerve Drive Testing (1 module)** | **CR Servo + analog encoder** | The module is mounted on the tank test robot. Adds the 4 tank motors |

Step-by-step guide for Single Swerve Module:
[SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md)

---

## The configuration trap — read this first

Both OpModes ask for a device named `swerve_servo`, but they need it configured
as **different device types**. A configuration that works for one will fail or
misbehave on the other.

| | Single Swerve Module | Swerve Drive Testing (1 module) |
| --- | --- | --- |
| `swerve_servo` device type | **Servo** | **Continuous Rotation Servo** |
| `swerve_encoder` needed? | **No** — do not configure it | **Yes** — Analog Input |

What goes wrong if you get it backwards:

| Mistake | Symptom |
| --- | --- |
| Running Single Swerve Module with `swerve_servo` set to CR Servo | INIT fails: *Unable to find a hardware device with name "swerve_servo" and type Servo*. The OpMode never starts |
| Running Swerve Drive Testing with `swerve_servo` set to Servo | INIT fails, or the module never turns |
| Running Swerve Drive Testing with no `swerve_encoder` | INIT fails: *Could not find a hardware device* |
| Leaving `swerve_encoder` configured while running Single Swerve Module | Harmless — that OpMode ignores it |

The practical fix: keep **two saved configurations** on the Driver Station, named
after the OpMode you use them with, and switch between them. Do not try to make
one configuration serve both.

Mechanically, the module also has to actually have the right servo fitted. These
are not two settings for one servo — an Axon in CR mode and an Axon in standard
position mode are different modes of the physical servo, and the standard-mode
one does not need its feedback wire connected.

---

## Hardware

| Config name | Device | Used by | Notes |
| --- | --- | --- | --- |
| `swerve_motor` | DC Motor | both | drives the wheel |
| `swerve_servo` | **Servo** | Single Swerve Module | position servo, turns the module, no feedback |
| `swerve_servo` | **Continuous Rotation Servo** | Swerve Drive Testing | Axon in CR mode, turns the module |
| `swerve_encoder` | Analog Input | **Swerve Drive Testing only** | the Axon's feedback wire, in an analog port |
| `left_front`, `left_back`, `right_front`, `right_back` | DC Motor | **Swerve Drive Testing only** | tank drive |

---

## Per-file notes

### Single Swerve Module — `SingleSwerveTeleOp.java`

Open loop. The code commands a servo position and never finds out whether the
module got there.

- The commanded angle is always folded into **±90° of straight forward**, and the
  drive motor reverses when folded. The module never swings past ±90°.
- Releasing the stick stops the motor; the servo **holds** its last angle, so the
  module keeps its heading and cannot be turned by hand.
- The flip is **sticky**: the stick has to pass 105° from forward before the wheel
  swaps sides, and come back inside 75° before it swaps back
  (`FOLD_HYSTERESIS_DEGREES`). Without that, stick noise at exactly sideways would
  make the servo slam back and forth every loop.
- Drive power fades in using a **modelled** servo position: the code assumes the
  servo covers 90° per `SERVO_SLEW_SECONDS` and scales power by how far its guess
  still is from the command. There is no error signal to fade it in with, so this
  stands in for one.
- Calibration (hold **A**) commands the servo to center so the wheel can be
  squared up; dpad left/right trims `SERVO_CENTER` live.
- Key constants: `SERVO_TRAVEL_DEGREES`, `SERVO_CENTER`, `SERVO_REVERSED`,
  `DRIVE_REVERSED`, `SERVO_SLEW_SECONDS`, `FOLD_HYSTERESIS_DEGREES`.

Full setup, telemetry and troubleshooting: [SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md).

### Swerve Drive Testing (1 module) — `swerveDriveTesting.java`

Closed loop. A CR servo is driven by a PD + static-friction loop against the
absolute analog encoder, so the module knows its real angle at power-on with no
homing step. Also drives the 4 tank motors from the right stick.

- The commanded angle is folded to the **shorter path from the measured angle**,
  so the module never turns more than 90° from where it actually is.
- Drive power is scaled by the cosine of the steering error, so the wheel fades in
  as the module lines up.
- Releasing the stick cuts power to both the motor and the servo — the module goes
  limp and can be turned by hand.
- Calibration (hold **A**) cuts all power; dpad left/right nudges the encoder zero
  by 0.5°.
- Key constants: `ENCODER_OFFSET_DEGREES`, `STEER_REVERSED`, `STEER_KP`,
  `STEER_KD`, `STEER_KS`, `STEER_TOLERANCE_DEGREES`, `STEER_MAX_POWER`,
  `SCALE_DRIVE_BY_ERROR`, `TANK_POWER_SCALE`.

Note it has **no** `DRIVE_REVERSED` constant — flipping the drive direction there
is a code change in `init()`, unlike Single Swerve Module.

The two files have **separate constants** and now **different steering models**.
Do not copy numbers between them: `SERVO_CENTER` and `ENCODER_OFFSET_DEGREES`
mean completely different things, and the PD gains have no counterpart in the
position-servo version.

---

## Controls

| Input | Single Swerve Module | Swerve Drive Testing |
| --- | --- | --- |
| Left stick | Aim and drive the module | Aim and drive the module |
| Right stick up/down | — | Tank robot forward / backward |
| Release left stick | Motor off, servo **holds** its angle | Motor and servo both off, module goes limp |
| Hold A | Calibration: motor off, servo commanded to center | Calibration: everything off (tank too) |
| Dpad left/right (while holding A) | Trim `SERVO_CENTER` by 0.002 | Nudge the encoder zero by 0.5° |

## What the telemetry means

Angles: **0° = straight forward, positive = to the right.**

| Line | Which OpMode | Meaning |
| --- | --- | --- |
| Target angle | both | Where it is trying to point |
| Servo position | Single Swerve Module | The 0–1 command sent to the servo |
| Center trim | Single Swerve Module | Current center position, to copy into `SERVO_CENTER` |
| Ramp | Single Swerve Module | Drive power multiplier while the servo is still swinging |
| Reachable | Single Swerve Module | Largest right / left angle the servo can reach with the current center and travel |
| `!! OUT OF SERVO RANGE` | Single Swerve Module | The commanded angle is past the end of the servo's travel, so the real wheel angle is less than `Target angle` |
| Module angle | Swerve Drive Testing | Where the module actually points right now |
| Error | Swerve Drive Testing | Target minus actual. Should shrink toward 0 |
| Steer power | Swerve Drive Testing | Power sent to the CR servo |
| Raw encoder | Swerve Drive Testing | Encoder reading before the zero offset |
| Offset | Swerve Drive Testing | Current encoder zero offset |
| Drive power | both | Power sent to the wheel motor. `(reversed)` is normal, see below |
| Tank power | Swerve Drive Testing | Tank drive output |

---

## First-time setup

**Lift the module off the ground first**, for either OpMode.

### Single Swerve Module

1. Set the servo horn so the wheel is straight forward near mid-travel.
2. Hold **A**, dpad-trim until the wheel is exactly forward, copy `Center trim`
   into `SERVO_CENTER`, reinstall.
3. Push the stick right — the wheel must turn right. If not, `SERVO_REVERSED = true`.
4. Push the stick forward — the wheel must roll forward. If not, `DRIVE_REVERSED = true`.
5. Push the stick fully right and check the wheel is at a real 90°. If not, fix
   `SERVO_TRAVEL_DEGREES`.

Details for each step are in [SingleSwerveTeleOp_README.md](SingleSwerveTeleOp_README.md).

### Swerve Drive Testing (1 module)

1. **Check the encoder.** In calibration mode (hold A), turn the module by hand.
   `Raw encoder` must change. If it does not, check the feedback wire, the analog
   port, and that `swerve_encoder` is set to Analog Input.
2. **Set the zero.** Point the wheel straight forward, hold A, read `Raw encoder`,
   put that number into `ENCODER_OFFSET_DEGREES`. Reinstall, then check
   `Module angle` reads about 0 when the wheel is straight. Dpad trims only last
   until the OpMode stops — always copy the final value into the file.
3. **Check steering direction.** Release A and push the left stick a little.
   `Error` must get smaller. **If the module keeps spinning or the error grows,
   press STOP immediately** and set `STEER_REVERSED = true`.
4. **Check drive direction.** Point the stick forward. If the wheel faces forward
   but rolls backward, ask a programmer to flip `driveMotor.setDirection` in
   `init()`.
5. **Check the tank drive.** Push the right stick forward. If the robot goes
   backward or spins, the tank motor directions need flipping in `init()`.

## Tuning the steering — Swerve Drive Testing only

Single Swerve Module has no steering loop to tune; its accuracy comes from
`SERVO_CENTER` and `SERVO_TRAVEL_DEGREES`.

Tune in this order. Start with `STEER_KD = 0` and `STEER_KS = 0`.

| What you see | Change |
| --- | --- |
| Turns slowly or stops short of the target | Raise `STEER_KP` |
| Overshoots and wobbles back and forth | Lower `STEER_KP`, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Buzzes or twitches when it is already pointed right | Raise `STEER_TOLERANCE_DEGREES` |
| Too fast to watch while tuning | Lower `STEER_MAX_POWER` |

## Things that look wrong but are normal

- **The module drives backward sometimes.** Both OpModes point the wheel the
  opposite way and reverse the motor rather than make a big turn. Telemetry shows
  `(reversed)`.
- **The wheel speeds up gradually after a big turn.** Swerve Drive Testing fades
  power in using the real steering error; Single Swerve Module fades it in against
  a modelled servo position instead. Either way it stops the wheel scrubbing
  sideways.
- **Single Swerve Module parks at 90° while the stick is roughly sideways.** That
  is the fold hysteresis. It is up to 15° off what you asked for in that band, and
  it is deliberate — without it the module would flap between pointing left and
  pointing right. Swerve Drive Testing does not need it because it folds against
  its measured angle.
- **Swerve Drive Testing goes limp when you let go of the stick.** Power is cut on
  purpose so nothing buzzes and you can turn it by hand.
- **Single Swerve Module does *not* go limp.** A position servo holds its last
  command, so the module keeps its heading. This is the intended difference.

## Warning for the tank test robot

If the swerve wheel is on the ground and pointing sideways, driving the tank
forward drags the swerve wheel sideways. That wears the wheel and loads the
steering. Either keep the module lifted, or do not drive the tank hard while the
module is turned.
