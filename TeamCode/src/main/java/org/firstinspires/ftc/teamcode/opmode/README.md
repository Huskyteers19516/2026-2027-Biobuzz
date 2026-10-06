# Competition OpModes

| File | What it is |
| --- | --- |
| `RobotTeleOp.java` | Driver-controlled OpMode. Owns every shooter constant (velocity, tolerance, spin-up timeout, feed power/time, servo aim limits). Nothing else may redefine them. |
| `MatchAuto.java` | The competition autonomous. Pedro-based `LinearOpMode` that walks the step list from `FieldPoints`. |
| `FieldPoints.java` | All field poses, the mission step list, and every tunable number for autonomous, including the vision-aiming constants. |
| `GoalScanner.java` | Owns the camera: one `VisionPortal` + one `AprilTagProcessor`, and turns AprilTag cluster detections into "where is our goal right now". |

---

## Why the robot re-scans before every shot

The BIOBUZZ AprilTag clusters **move during the match**. Each goal flips, so the cluster that is facing our
camera changes while we are still shooting — and the alliance partner shoots too, which can flip a goal
between our own shots.

Two consequences:

1. **A single scan at INIT is worthless.** `MatchAuto` re-scans immediately before *every individual shot*,
   not once per shooting step. A three-shot step performs three independent scan-aim-shoot cycles.
2. **The clusters are never used for localization.** The SDK 12 release notes say plainly that the BIOBUZZ
   tags move and are therefore unsuitable for absolute field localization. The Pinpoint odometry stays the
   only source of truth for the robot pose. Vision only ever changes two things: the robot *heading* and the
   launcher servo *aim*. It never calls `follower.setPose`.

Each cluster's origin sits at the centre of the Cell opening, which is exactly what we want to point at.

---

## The `ALLIANCE` constant

```java
public static final GoalScanner.Alliance ALLIANCE = GoalScanner.Alliance.RED;
```

in `FieldPoints.java`. **Set this before every match.** It selects which two clusters count as "our goal":

| `ALLIANCE` | Candidate clusters | Tag IDs |
| --- | --- | --- |
| `RED` | `RED SCORING`, `RED AUDIENCE` | 30-33, 34-37 |
| `BLUE` | `BLUE SCORING`, `BLUE AUDIENCE` | 42-45, 38-41 |

Cluster short names come straight from `AprilTagGameDatabase.getBioBuzzTagLibrary()`; they are declared once
as constants in `GoalScanner` so a typo cannot silently disable aiming.

### How a target is chosen

A cluster sighting must clear **two gates** before it can be aimed with:

| Gate | Constant | Value | Why |
| --- | --- | --- | --- |
| Fresh enough | `GoalScanner.SIGHTING_FRESHNESS_MS` | 250 ms | The goals flip. A sighting older than this may describe a goal that is no longer there. It is deliberately shorter than `SHOT_GAP_SECONDS` (0.4 s), so a cluster that vanishes between two of our own shots has always aged out by the time the next shot scans. |
| Strong enough | `GoalScanner.MIN_PERCENT_CLUSTER_FOUND` | 50 % | Each BIOBUZZ cluster has four member tags. The SDK emits a cluster detection when it sees **even one** of them (25 %), then extrapolates the cluster origin up to 6.5 in sideways from that single 3.25 in tag — a range-ambiguous, bearing-biased solve. Requiring two or more tags keeps the geometry honest. Weak sightings are still shown in telemetry, tagged `WEAK<50%`. |

Among the two candidates that pass both gates, `GoalScanner.bestTargetFor(alliance)` picks:

1. the **more recent** sighting, if the two were observed more than `RECENCY_PREFERENCE_MS` (50 ms) apart —
   what the camera can see *now* beats what it could see a moment ago, which is the whole point of the
   feature;
2. otherwise (same frame, effectively) the **highest `percentClusterFound`** — more member tags visible
   means a better-conditioned pose solve;
3. on a tie, the **shorter `range`**.

A tag that belongs to a cluster is never also returned as a single detection, so the scanner ignores
`AprilTagSingleDetection` entirely.

**On "age".** `Sighting.ageMs()` measures from the moment `GoalScanner.update()` first *ingested* that
camera frame, using `System.nanoTime()`. It deliberately does not use the detector's
`frameAcquisitionNanoTime` for age, because that is the camera's own clock while our timeouts are
wall-clock. `frameAcquisitionNanoTime` is still used, but only to tell a genuinely new frame from the same
frame being handed back again by `getDetections()`. Add roughly one pipeline latency (tens of ms) if you
want true end-to-end age.

---

## Scan -> aim -> shoot, per shot

For every shot, `MatchAuto` runs:

0. **Spin up first.** `outtake.setVelocity(OUTTAKE_VELOCITY)` is commanded *before* aiming starts, so the
   flywheel comes up to speed while the camera and the drivetrain do their work. Nothing in the aiming phase
   depends on the shooter being stopped, and the spin-up wait afterwards then usually exits on its first
   loop. On the first shot of each shooting step this removes the whole aiming duration of dead time.
1. **Scan.** Poll `GoalScanner` for up to `SCAN_TIMEOUT_SECONDS`, exiting the moment a usable candidate
   appears. `MatchAuto.tick()` calls `follower.update()` and `scanner.update()` together, and every wait
   loop in the autonomous uses `tick()`, so the scan stays warm while driving and intaking — by the time a
   shot starts there is usually already a usable sighting and this phase exits immediately. The robot never
   stops holding its pose.
2. **Turn.** If `|bearing| > AIM_BEARING_TOLERANCE_DEGREES`, rotate to reduce the bearing.
   **Bearing sign: positive bearing means the target is to the LEFT**, so the goal heading is
   `currentHeading + toRadians(bearing)` (Pedro headings are CCW-positive, in radians).
   The correction is clipped to `AIM_BEARING_MAX_CORRECTION_DEGREES` (30 deg — inside a typical webcam's
   ~60 deg horizontal field of view, so the clip can actually reject a spurious pose instead of being
   unreachable). The turn is a heading-only `follower.hold(follower.pose().withHeading(goalHeading))` — the
   x/y of the current pose are preserved, so the robot pivots in place.
   *Pedro v3.0.1 has no `turnTo` and no `holdPoint`; `hold(Pose)` / `hold(Pose, boolean)` is the only
   heading command on `Follower`. Verified with `javap` against `core-3.0.1.jar`.*
   The turn exits when the odometry heading error is within `AIM_HEADING_SETTLE_TOLERANCE_DEGREES`, or on
   `AIM_TURN_TIMEOUT_SECONDS`.
3. **Confirm, then close the loop.** After the turn, a `CONFIRM_SCAN_SECONDS` re-read gives the *actual*
   post-turn bearing and a fresh range. Two rules make that trustworthy:
   - the confirm is **locked to the cluster we just turned toward**. It never re-runs the full candidate
     selection, so it cannot silently swap to the other alliance cluster and hand us the wrong goal's range
     while the robot is pointed at the first one;
   - the confirm only accepts a sighting **observed strictly after the turn finished**. Without that rule a
     cluster blurred out by the pivot would let the pre-turn sighting be reported as the post-turn result —
     telemetry would read `bearing 8.0 -> 8.0 deg`, which looks exactly like an inverted turn direction and
     sends people hunting a sign bug that does not exist.

   If the confirmed bearing is still outside tolerance, the turn/confirm pair runs **once more**
   (`AIM_MAX_TURN_PASSES = 2`), budget permitting. A pass that *timed out* is not retried — the robot is
   fighting something and a second turn will not help inside the budget. Steps 2-3 together are what make
   the aim closed-loop against the camera rather than open-loop against odometry.
4. **Aim.** The launcher servo position is a linear map of the **confirmed** range, then clipped to the
   servo limits from `RobotTeleOp`:

   ```
   t   = clip((range - RANGE_NEAR_INCHES) / (RANGE_FAR_INCHES - RANGE_NEAR_INCHES), 0, 1)
   aim = clip(AIM_AT_NEAR + t * (AIM_AT_FAR - AIM_AT_NEAR), AIM_MIN_POSITION, AIM_MAX_POSITION)
   ```

5. **Shoot.** Wait for the flywheel with the existing `SPINUP_TIMEOUT_SECONDS`, then feed for
   `FEED_SECONDS`. This part is behaviourally what was there before; vision only changed which servo
   position and which heading it fires from.

Then the loop repeats for the next shot — including a completely fresh scan.

### Telemetry to read while it happens

`Aim source` (VISION / STATIC FALLBACK), `Aim cluster` + percent, `Aim range`,
`Aim bearing: before X -> after Y`, `Sighting age`, `Aim turn passes`, `Aim turn` (what the turn did, or why
it stopped) and `Aim note`. During a scan the `Candidates` line shows both alliance clusters with their
percent, range, age and any `STALE` / `WEAK<50%` flag.

---

## What to measure for the range-to-aim map

The four numbers live in `FieldPoints.java` and **the shipped values are placeholders**:

```java
public static final double RANGE_NEAR_INCHES = 40.0;
public static final double RANGE_FAR_INCHES  = 100.0;
public static final double AIM_AT_NEAR       = 0.45;
public static final double AIM_AT_FAR        = 0.65;
```

Procedure, on the real field, with a full hopper:

1. Park the robot at the **closest** realistic shooting distance. Run `AprilTag Tester (Logitech)` and write
   down the reported `Distance` in inches — that is `RANGE_NEAR_INCHES`.
2. Without moving the robot, run `Robot TeleOp` and use gamepad 2 d-pad (up/down = 0.05, left/right = 0.01)
   to walk the aim until the ball scores repeatedly. That value is `AIM_AT_NEAR`.
3. Repeat at the **farthest** realistic shooting distance for `RANGE_FAR_INCHES` and `AIM_AT_FAR`.
4. Check one distance in the middle. If the middle shot is consistently short or long, the real relationship
   is not linear over that span — narrow `RANGE_NEAR`/`RANGE_FAR` to the band you actually shoot from rather
   than trying to cover the whole field with one straight line.

Ranges outside `[RANGE_NEAR, RANGE_FAR]` are clamped to the nearest endpoint, so a wild range reading can
never drive the servo somewhere absurd.

---

## Fallback behaviour — autonomous never stalls on vision

Vision is strictly an *improvement*. Every failure path ends in a shot being taken.

| Situation | What happens |
| --- | --- |
| Camera missing / `VisionPortal` throws at construction | `GoalScanner` catches it, `isAvailable()` is false, INIT telemetry says `!! CAMERA NOT AVAILABLE`. Every shot uses `step.aim`. |
| `VISION_AIMING_ENABLED = false` | Same as above, deliberately. Use this to fall back to last year's behaviour at an event. |
| `getDetections()` throws mid-match | Caught inside `update()` and recorded in `fault()`. Existing sightings simply age out and the next shot falls back. |
| No cluster both fresh enough and strong enough within `SCAN_TIMEOUT_SECONDS` | Shot is taken from the step's static pose and static `step.aim`. Telemetry reads `Aim source: STATIC FALLBACK` plus a reason naming **both cluster short names**, the percent floor, the freshness window and the scan limit that was actually used. |
| Only a weak sighting (`< MIN_PERCENT_CLUSTER_FOUND`) | Treated as "nothing seen", i.e. the static fallback above. A one-tag solve is worse than the step's measured static aim. Telemetry still shows it, tagged `WEAK<50%`. |
| Turn does not settle within `AIM_TURN_TIMEOUT_SECONDS` | **The shot is taken anyway**, from wherever the robot got to, with the vision range-derived aim. Telemetry reads `TURN TIMEOUT ... - shooting anyway`. No second pass is attempted. |
| Confirm scan sees nothing new after the turn | The pre-turn range drives the aim; telemetry appends `(no confirm sighting, kept pre-turn range)`. No further passes. |
| Overall `AUTONOMOUS_BUDGET_SECONDS` reached | The existing budget logic stops the mission, exactly as before. |

**Deliberate choice: a shot that cannot be aimed is TAKEN, never SKIPPED.** A ball that stays in the robot
scores zero with certainty; a badly aimed ball scores zero *or* scores. The only thing vision is allowed to
cost us is time, and that is what the timeouts bound.

### Time budget

| Constant | Value | Bounds |
| --- | --- | --- |
| `SCAN_TIMEOUT_SECONDS` | 0.7 s | Waiting for a usable cluster. Exits early the instant one is seen. |
| `AIM_TURN_TIMEOUT_SECONDS` | 1.0 s | One heading-only turn. Exits early on settle. |
| `CONFIRM_SCAN_SECONDS` | 0.25 s (`MatchAuto`) | One post-turn re-read. |
| `AIM_MAX_TURN_PASSES` | 2 (`MatchAuto`) | At most two turn+confirm pairs per shot. |
| `AIM_MIN_PASS_SECONDS` | 0.40 s (`MatchAuto`) | A further pass only starts if at least this much of the per-shot budget is left. |
| `PER_SHOT_TIMEOUT_SECONDS` | 4.0 s | Hard ceiling on **the whole aiming phase** of one shot. The scan, each turn and each confirm get `min(their own timeout, what is left of this)`. |

Worst case aiming cost per shot is `0.7 + 2 x (1.0 + 0.25) = 3.2 s`, under the 4.0 s ceiling. That worst case
needs the first turn to settle *and* the goal to still be outside tolerance afterwards, which is rare. The
typical cost, when the goal is already roughly in front of the robot, is a few tens of milliseconds: the scan
exits on the first usable frame and the turn is skipped entirely inside `AIM_BEARING_TOLERANCE_DEGREES`.
Because the flywheel now spins up *during* aiming, most of even the worst case overlaps work the robot had to
do anyway.

Spin-up and feed are *not* charged against `PER_SHOT_TIMEOUT_SECONDS`; they keep their own existing
`SPINUP_TIMEOUT_SECONDS` and `FEED_SECONDS`. The global `AUTONOMOUS_BUDGET_SECONDS` (28.0 s) gates everything
through `canKeepRunning()`, which every scan, turn, spin-up and hold loop tests, so vision can never push the
autonomous past the 30 s match period.

The one loop that deliberately does **not** test the budget is the feed loop: once the transfer has started
pushing a ball into a spinning flywheel, aborting halfway is a jam risk. Worst case it runs `FEED_SECONDS`
(1.0 s) past the 28.0 s budget, which is still inside the 30 s match.

> **Separate, pre-existing concern — the mission is longer than 30 s.** With 15 steps at
> `DEFAULT_PATH_TIMEOUT_SECONDS = 6.0` plus 9 shots, the worst case is far past `AUTONOMOUS_BUDGET_SECONDS`,
> and even a benign run (~1.5 s per leg) spends ~29 s before the first ball is fired. The budget guard
> truncates the mission safely, but because `park` is the **last** step it is what gets dropped. This is not
> caused by vision aiming — vision's own worst case is ~3.2 s per shot and typically near zero — but it must
> be resolved before the mission is trusted: cut the step list to what actually fits, and lower
> `DEFAULT_PATH_TIMEOUT_SECONDS` to something near the measured leg times.

---

## Camera cost

- **One `VisionPortal` for the whole autonomous.** It is built in `runOpMode` before INIT and closed in a
  `finally` at the very end. It is never rebuilt per shot — building a portal costs seconds, which we do not
  have.
- **640x480.** Enough resolution for a multi-tag cluster at the distances we shoot from, and the cheapest
  standard mode the webcam offers.
- **Decimation 2.0.** The detector downsamples by 2 before finding quads. At 640x480 this roughly quarters
  the quad-detection work while still resolving the cluster tags out past our far shooting distance. Pose
  solving still runs at full resolution, so range and bearing accuracy are not thrown away. This is the same
  value the proven `AprilTag Tester` OpMode uses, so field measurements taken with the tester transfer
  directly.
- **Live view off** (`LIVE_VIEW = false` in `GoalScanner`). The driver station preview costs real CPU and
  nobody watches it during a match. Flip it to `true` while debugging on the bench; the tag outline and ID
  overlays follow the same flag.
- `GoalScanner.update()` only calls `getDetections()`, which returns the last completed frame's list and
  never blocks. It is called from `MatchAuto.tick()` alongside `follower.update()`, so the scan stays warm
  during driving and intaking and a sighting is usually already usable by the time the shot starts.

---

## Verifying the cluster mapping on the field

Use the existing tester — it lives in the other module and is **not** part of the competition build:

`Teamtestcode/src/main/java/org/firstinspires/ftc/teamtestcode/apriltagtesting/AprilTagTester.java`
(Driver Station: **TeleOp → Testing → "AprilTag Tester (Logitech)"**)

1. INIT it and wait for `Camera: STREAMING`. The INIT screen lists every cluster in the library with its
   member tag IDs — check you see the four clusters and the ID ranges in the table above.
2. Press START and point the robot at **our** goal.
3. Read the `Target` line. It must say `Cluster RED SCORING` / `RED AUDIENCE` when we are red, or
   `BLUE SCORING` / `BLUE AUDIENCE` when we are blue. **If it says the other alliance's name, the
   `ALLIANCE` constant in `FieldPoints.java` is wrong — fix that, not the code.**
4. Check `Tags seen: n of 4 (p %)`. That `p` is the same `percentClusterFound` `GoalScanner` gates on. It
   must read **50 % or more** from every pose you intend to shoot from, or `Match Auto` will refuse the
   sighting and fall back to the static aim. If it sits at 25 % from a shooting spot, lower or re-aim the
   camera until at least two member tags are in frame.
5. Check the bearing sign: slide the robot so the goal sits to its **left** and confirm the tester prints a
   **positive** bearing and `(tag is LEFT, turn left)`. `MatchAuto` assumes exactly that convention. If your
   camera is mounted facing backwards or mirrored, this is where you will find out.
   While you are here, also check that the camera and the launcher point the **same way**: put the goal dead
   ahead of the launcher and confirm the tester reads a bearing near 0. A consistent non-zero offset means
   the camera is mounted at an angle to the launcher axis, and that offset has to be subtracted from the
   bearing before the turn.
6. Note the `Distance` readings from your two shooting poses — those are the numbers for
   `RANGE_NEAR_INCHES` / `RANGE_FAR_INCHES` above.

Only one OpMode may own the camera at a time. Stop the tester before running `Match Auto`.

---

## Pre-match checklist

- [ ] `FieldPoints.ALLIANCE` matches the alliance we are actually on.
- [ ] `FieldPoints.POINTS_CONFIRMED` is `true` and the poses have been measured (auto refuses to drive otherwise).
- [ ] `RANGE_NEAR/FAR` and `AIM_AT_NEAR/FAR` measured on this field, with this ball batch.
- [ ] INIT telemetry shows `Camera: STREAMING`, not `NO CAMERA`.
- [ ] INIT `Candidates` line shows our clusters at 50 % or better from the first shooting pose.
- [ ] Webcam is configured as `Webcam 1` in the robot configuration (`GoalScanner.WEBCAM_NAME`).
