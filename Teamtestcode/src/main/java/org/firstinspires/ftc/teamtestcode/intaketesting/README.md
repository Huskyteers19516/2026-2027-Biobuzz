# Intake Motor Test

Runs the intake motor inward and shows how fast it is spinning and how much
current it draws. Built for the build team to compare intake designs.

The intake only runs **inward**. There is no outtake in this tester.

## Hardware

| Config name | Device | Notes |
| --- | --- | --- |
| `intake_motor` | DC Motor | **Set the Motor Type to your real motor model** in the configuration |

- The **encoder cable must be plugged in**, or speed always reads 0.
- Ticks per revolution and rated free speed come from the Motor Type in the
  configuration. If the type is wrong, every RPM number is wrong. INIT shows the
  motor type it is using — check it.

## Controls

| Button | Action |
| --- | --- |
| Right trigger | Run the intake. Press harder = faster |
| A | Start / stop running at the **set power** (hands-free) |
| B | Stop |
| Dpad up / down | Set power +0.05 / −0.05 (0 to 1) |
| X | Reset the peak values |

The trigger always wins over the set power. Use A for tests so the power is the
same every run.

## Telemetry

| Line | Meaning |
| --- | --- |
| Control | What is driving the motor: `triggers`, `set power`, or `stopped` |
| Set power | Power A will use |
| Output power | Power actually sent to the motor now |
| Motor speed | Motor output shaft RPM |
| Roller speed | Motor speed × `EXTERNAL_GEAR_RATIO` |
| Encoder velocity | Raw encoder ticks per second |
| Of rated free speed | How fast it spins compared with the motor's no-load speed |
| Current | Amps the motor is drawing right now |
| Peak speed / Peak current | Highest values since the last X press |
| Encoder position | Total encoder ticks |

## How to run a comparison test

1. Set the power with the dpad, for example 0.8.
2. **Empty run:** press A, wait 2 seconds, write down Motor speed and Current.
   Press B.
3. Press X to reset the peaks.
4. **Loaded run:** press A and feed game pieces into the intake. Press B.
5. Write down Peak current and the lowest Motor speed you saw.
6. Repeat with the other design or setting, at the **same set power**.

Reading the results:

- Speed drops a lot and current jumps → the intake is struggling or jamming.
- Peak current catches short jams that happen too fast to see on the live number.
- Current keeps climbing while speed is near 0 → stalled. Stop quickly to protect
  the motor.

## Constants you may change

| Constant | Default | Meaning |
| --- | --- | --- |
| `INTAKE_REVERSED` | false | Set `true` if the intake pushes pieces out |
| `EXTERNAL_GEAR_RATIO` | 1.0 | Belt or gear ratio from motor to roller. 2:1 speed-up = `2.0` |
| `TICKS_PER_REV_OVERRIDE` | 0.0 | Only if the Motor Type cannot be set. Put the real ticks per rev |
| `DEFAULT_SET_POWER` | 0.5 | Set power when the OpMode starts |
| `POWER_STEP` | 0.05 | How much one dpad press changes the set power |

## Troubleshooting

| Problem | Fix |
| --- | --- |
| Motor spins but speed is 0 | Encoder cable |
| INIT shows `ticks per rev is 0` | Set the Motor Type, or `TICKS_PER_REV_OVERRIDE` |
| RPM looks far too high or too low | Wrong Motor Type in the configuration |
| Intake pushes pieces out | `INTAKE_REVERSED = true` |
