# Tank Drive Auto (Square)

An Autonomous for the 4-motor tank test robot. After START it:

1. drives forward `SIDE_LENGTH_INCHES` (24 in by default),
2. turns right 90°,
3. repeats 4 times,

and ends back where it started, facing the same direction.

Straight lines use the motor encoders to measure distance. Turns use the Control
Hub's built-in IMU, which is much more accurate than encoders for turning.

## Hardware

| Config name | Device | Notes |
| --- | --- | --- |
| `left_front`, `left_back` | DC Motor | direction FORWARD |
| `right_front`, `right_back` | DC Motor | direction REVERSE |
| `imu` | IMU | built into the Control Hub |

> **The encoder cable on all 4 drive motors must be plugged in.** Without it the
> code never sees the robot arrive, and the robot drives forward at full speed
> until you press STOP.

## Set these before the first run

| Constant | Default | What to put |
| --- | --- | --- |
| `COUNTS_PER_MOTOR_REV` | 537.7 | From your motor's spec sheet. 537.7 is goBILDA 5203 312 RPM |
| `WHEEL_DIAMETER_INCHES` | 3.78 | Measure the wheel. 3.78 in = 96 mm |
| `DRIVE_GEAR_REDUCTION` | 1.0 | Extra gearing between motor and wheel, if any |
| `HUB_LOGO_DIRECTION` | UP | Which way the REV logo on the Control Hub faces |
| `HUB_USB_DIRECTION` | FORWARD | Which way the Control Hub's USB ports face |
| `SIDE_LENGTH_INCHES` | 24.0 | Size of the square |
| `DRIVE_SPEED` / `TURN_SPEED` | 0.4 / 0.3 | Lower these for the first run |

**The two hub directions must match how the Control Hub is actually mounted.**
If they are wrong, turns go the wrong way or never stop.

## Running it

1. Driver Station → Autonomous list → **Tank Drive Auto (Square)** → INIT.
2. Place the robot with room for the square in front of and to the right of it.
3. Press START. The IMU heading is zeroed at that moment, so the robot's direction
   when you press START is "0°".

## Telemetry

| Line | Meaning |
| --- | --- |
| Side | Which side of the square (1–4) |
| Mode | `driving`, `turning`, or `holding` (settling after a turn) |
| Target heading | The heading it should hold, in degrees |
| Heading | What the IMU reads now |
| Error | Target minus actual |
| Final heading | Shown at the end. Should be close to 0 |

## Troubleshooting

| Problem | Fix |
| --- | --- |
| Square is too big or too small | Check `COUNTS_PER_MOTOR_REV`, `WHEEL_DIAMETER_INCHES`, `DRIVE_GEAR_REDUCTION` |
| Robot drives forward and never stops | Encoder cables. Press STOP |
| Robot drives backward or spins on the first side | Motor directions are wrong |
| Turns the wrong way, or spins forever | `HUB_LOGO_DIRECTION` / `HUB_USB_DIRECTION` are wrong |
| Drifts sideways on straight lines | Raise `P_DRIVE_GAIN` |
| Wobbles at the end of each turn | Lower `P_TURN_GAIN` |
| Stops turning before 90° | Raise `P_TURN_GAIN`. A turn also gives up after `TURN_TIMEOUT_SECONDS` (4 s) |
| Square does not close | Compare `Final heading` with 0. If the angle is off, it is the turns; if the angle is fine, it is the distances |
