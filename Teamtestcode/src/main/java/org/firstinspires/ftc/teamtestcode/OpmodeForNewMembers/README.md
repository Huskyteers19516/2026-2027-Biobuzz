# Meet 0 — Starter Code

`Meet0TeleOp.java` and `Meet0Auto.java` are set up for you: every motor and servo
is declared, found in the hardware map, and configured. **Your job is to make the
robot do things.**

On the Driver Station they are **Meet0 TeleOp** (TeleOp list) and **Meet0 Auto**
(Autonomous list), in the *Meet0* group.

## The robot

| Variable in code | Config name | Device | What it is |
| --- | --- | --- | --- |
| `leftFront` | `left_front` | DC Motor | Tank drive, left side |
| `leftBack` | `left_back` | DC Motor | Tank drive, left side |
| `rightFront` | `right_front` | DC Motor | Tank drive, right side |
| `rightBack` | `right_back` | DC Motor | Tank drive, right side |
| `leftIntake` | `left_intake` | Continuous Rotation Servo | Collects balls |
| `rightIntake` | `right_intake` | Continuous Rotation Servo | Collects balls |
| `launcher` | `launcher` | DC Motor (with encoder) | Shoots balls |

## Already done for you

- All 7 devices are connected in `init()` / at the top of `runOpMode()`.
- **Directions:** the right drive motors are reversed, so `setPower(1.0)` on all
  four drives forward. `leftIntake` is reversed, so `setPower(1.0)` on both
  intake servos should pull a ball in.
- **Drive motors** brake when power is 0.
- **Launcher** coasts when power is 0 and uses its encoder, so `setVelocity()`
  works.
- Everything is set to power 0 at the start. The TeleOp also stops everything when
  you press STOP.
- `runtime` is a timer that starts at 0 when you press START.

Don't change `init()` unless a direction turns out to be wrong on the real robot.

## Commands you will need

| To do this | Write this |
| --- | --- |
| Run a drive motor | `leftFront.setPower(0.5);` — from −1.0 (full backward) to 1.0 (full forward) |
| Run an intake servo | `leftIntake.setPower(1.0);` — −1.0 to 1.0, 0 stops it |
| Spin the launcher at a set speed | `launcher.setVelocity(1500);` — ticks per second |
| Read launcher speed | `launcher.getVelocity()` |
| Read a joystick | `gamepad1.left_stick_y` — **pushing up gives a negative number** |
| Read a button | `gamepad1.a`, `gamepad1.right_bumper` — `true` while held |
| Read a trigger | `gamepad1.right_trigger` — 0.0 to 1.0 |
| Show a value on the Driver Station | `telemetry.addData("Launcher", launcher.getVelocity());` |
| Time since START | `runtime.seconds()` |

---

## Your tasks — Meet0 TeleOp

Write everything inside `loop()`. It runs about 50 times per second.

### Task 1 — Tank drive
- Left stick up/down controls both left motors.
- Right stick up/down controls both right motors.
- Remember the stick gives a **negative** number when pushed up.
- Test: push both sticks up → robot drives forward. Left up, right down → robot
  spins right.

### Task 2 — Intake
- Pick a button. While it is held, both intake servos run at full power.
- When it is released, they stop.
- Test: does the ball go **in**? If one servo pushes the wrong way, flip its
  direction in `init()`.

### Task 3 — Launcher
- One button starts the launcher with `setVelocity(...)`. Another button stops it.
- Show `launcher.getVelocity()` with telemetry so you can see the speed.
- Find a speed that shoots well by testing. Start low.

### Task 4 — Challenge
- Put the launcher speed in a variable at the top of the class instead of typing
  the number twice.
- Only let the intake feed a ball into the launcher when the launcher is close to
  its target speed.

---

## Your tasks — Meet0 Auto

Write your steps **after** `runtime.reset();` and **before** the last `stopAll();`.
Auto runs from top to bottom, one line after another, then ends.

### Task 1 — Drive forward for a set time
To do something for a set time:
1. Set the motor powers.
2. Wait in a loop until the time is up:
   ```java
   runtime.reset();
   while (opModeIsActive() && runtime.seconds() < 1.5) {
       telemetry.addData("Step", "driving forward");
       telemetry.update();
   }
   ```
3. Set the motor powers back to 0.

`opModeIsActive()` makes the robot stop right away if someone presses STOP. Always
include it.

### Task 2 — Shoot
1. Start the launcher.
2. Wait for it to reach speed (time, or check `getVelocity()`).
3. Run the intake to feed a ball.
4. Stop the intake and the launcher.

### Task 3 — Challenge
- Turn in place for a set time.
- Put a repeated step (like "drive for N seconds") in its own method so you can
  reuse it.

---

## Before you test on the real robot

- **Put the robot on a box so the wheels are off the ground** for the first run.
- Keep a hand on STOP.
- If the launcher **encoder cable is not plugged in**, `setVelocity()` runs the
  launcher at full speed. Check the cable before testing the launcher.
- If the app stops with *Could not find a hardware device*, a name in the Driver
  Station configuration does not match the table above.

## Examples to learn from

These come with the FTC SDK, in
`FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples`:

| File | Shows |
| --- | --- |
| `RobotTeleopTank_Iterative.java` | Tank drive with two sticks |
| `BasicOpMode_Iterative.java` | Structure of a TeleOp like Meet0 TeleOp |
| `RobotAutoDriveByTime_Linear.java` | Auto that drives using a timer |
| `ConceptGamepadEdgeDetection.java` | Doing something once per button press |
