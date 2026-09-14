# Swerve Module Testers

Both OpModes make the swerve module behave like an office chair wheel: push the
left stick in a direction and the module turns itself to face that way, then
drives. Push the stick further to go faster.

| Name on the Driver Station | Use it when |
| --- | --- |
| **Single Swerve Module** | Testing the module by itself |
| **Swerve Drive Testing (1 module)** | The module is mounted on the tank test robot. Adds the 4 tank motors. |

The two files have **separate constants**. If you calibrate one, copy the values
into the other too.

## Hardware

| Config name | Device | Notes |
| --- | --- | --- |
| `swerve_motor` | DC Motor | drives the wheel |
| `swerve_servo` | Continuous Rotation Servo | turns the module (Axon in CR mode) |
| `swerve_encoder` | Analog Input | the Axon's extra feedback wire, plugged into an analog port |
| `left_front`, `left_back`, `right_front`, `right_back` | DC Motor | **Swerve Drive Testing only** — tank drive |

## Controls

| Input | Single Swerve Module | Swerve Drive Testing |
| --- | --- | --- |
| Left stick | Aim and drive the module | Aim and drive the module |
| Right stick up/down | — | Tank robot forward / backward |
| Release left stick | Module motor and servo turn off | Module motor and servo turn off |
| Hold A | Calibration mode, everything off | Calibration mode, everything off (tank too) |
| Dpad left/right (while holding A) | Nudge the zero point 0.5° | Nudge the zero point 0.5° |

## What the telemetry means

Angles: **0° = straight forward, positive = to the right.**

| Line | Meaning |
| --- | --- |
| Module angle | Where the module is pointing right now |
| Target angle | Where it is trying to point |
| Error | Target minus actual. Should shrink toward 0 |
| Steer power | Power sent to the servo |
| Drive power | Power sent to the wheel motor. `(reversed)` is normal, see below |
| Raw encoder | Encoder reading before the zero offset is applied |
| Offset | Current zero offset |
| Tank power | Swerve Drive Testing only |

## First-time setup — do these in order

1. **Lift the module off the ground.**
2. **Check the encoder.** In calibration mode (hold A) — or during INIT on Single
   Swerve Module — turn the module by hand. `Raw encoder` must change. If it does
   not, check the feedback wire, the analog port, and that `swerve_encoder` is set
   to Analog Input.
3. **Set the zero.** Point the wheel straight forward, hold A, and read
   `Raw encoder`. Put that number into `ENCODER_OFFSET_DEGREES` at the top of the
   file. Press Run again, then check `Module angle` reads about 0 when the wheel
   is straight. (Dpad trims only last until the OpMode stops — always copy the
   final value into the file.)
4. **Check steering direction.** Release A and push the left stick a little.
   `Error` must get smaller. **If the module keeps spinning or the error grows,
   press STOP immediately** and set `STEER_REVERSED = true`.
5. **Check drive direction.** Point the stick forward. If the wheel faces forward
   but rolls backward:
   - Single Swerve Module: set `DRIVE_REVERSED = true`.
   - Swerve Drive Testing: ask a programmer to flip `driveMotor.setDirection` in `init()`.
6. **Swerve Drive Testing only:** push the right stick forward. If the robot goes
   backward or spins, the tank motor directions need flipping in `init()`.

## Tuning the steering

Tune in this order. Start with `STEER_KD = 0` and `STEER_KS = 0`.

| What you see | Change |
| --- | --- |
| Turns slowly or stops short of the target | Raise `STEER_KP` |
| Overshoots and wobbles back and forth | Lower `STEER_KP`, then raise `STEER_KD` |
| Gets within a few degrees but will not finish | Raise `STEER_KS` |
| Buzzes or twitches when it is already pointed right | Raise `STEER_TOLERANCE_DEGREES` |
| Too fast to watch while tuning | Lower `STEER_MAX_POWER` |

## Things that look wrong but are normal

- **The module drives backward sometimes.** If the target is more than 90° away,
  the module turns to face the opposite way and drives backward instead — it is
  the same motion, with half the turning. Telemetry shows `(reversed)`.
- **The wheel speeds up gradually after a big turn.** Drive power is reduced while
  the module is still turning, so the wheel does not scrub sideways.
- **The module goes limp when you let go of the stick.** Power is cut on purpose so
  nothing buzzes and you can turn it by hand.

## Warning for the tank test robot

If the swerve wheel is on the ground and pointing sideways, driving the tank
forward drags the swerve wheel sideways. That wears the wheel and loads the
steering. Either keep the module lifted, or do not drive the tank hard while the
module is turned.
