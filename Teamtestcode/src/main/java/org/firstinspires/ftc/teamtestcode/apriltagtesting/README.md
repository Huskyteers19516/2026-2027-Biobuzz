# AprilTag Testers

| Name on the Driver Station | Who it is for |
| --- | --- |
| **AprilTag Tester (Logitech)** | Anyone. No buttons. Shows whether a tag is seen, its ID, and where it is |
| **AprilTag Test** | Programmers. Tunes camera exposure, gain, and decimation |

## Setup

- The webcam must be named `Webcam 1` in the robot configuration.
- The SDK has built-in calibration for **Logitech C270, C310, and C920** at
  640×480, which is what both testers use. Other cameras still detect tags, but
  distances may be wrong.
- Needs FTC SDK 12.0. If Android Studio shows `AprilTagClusterDetection` in red,
  run File → Sync Project with Gradle Files.

## BIOBUZZ tag IDs

Field tags come in **clusters**: groups of 4 tags that act as one target. A
cluster's position is the center of the Cell opening, so aiming at the cluster
means aiming at the opening.

| Cluster | Tag IDs |
| --- | --- |
| RED SCORING | 30, 31, 32, 33 |
| RED AUDIENCE | 34, 35, 36, 37 |
| BLUE AUDIENCE | 38, 39, 40, 41 |
| BLUE SCORING | 42, 43, 44, 45 |

The SDK library also contains sample tags that are **not on the field**, handy
for testing with printouts:

| Sample | IDs |
| --- | --- |
| GOAL (Center Goal cluster) | 581, 582 |
| Robbie / Number 5 / C3PO / K9 | 583 / 584 / 585 / 586 |

BIOBUZZ tags move during a match, so use them for **aiming**, not for working
out where the robot is on the field.

---

## AprilTag Tester (Logitech)

### During INIT

Shows the camera state and every cluster and tag ID in the library. Wait until
**Camera = STREAMING**, then press START.

### After START

**No tag visible:**

```
DETECTED:  NO
Last seen   2.3 s ago  (Cluster RED SCORING (IDs 30, 31, 32, 33))
```

**Tag visible:**

| Line | Meaning |
| --- | --- |
| DETECTED / IDs | YES, how many targets, and their IDs |
| Target | The one being measured. Clusters win over single tags; then the closest |
| Tag IDs in cluster / Tags seen | For a cluster: its IDs and about how many of them are visible |
| Tag ID / Confidence (margin) | For a single tag: its ID and how clean the detection is (higher is better) |
| Distance | Straight-line distance from the camera, inches and cm |
| Left / Right | How far the tag is to the side of the camera |
| Forward | How far ahead of the camera |
| Up / Down | How far above or below the camera |
| Bearing | Angle to turn to face the tag. **Positive = tag is on the LEFT.** Within ±3° shows `CENTERED` |
| Elevation | Angle up or down to the tag |
| Tag yaw | How much the tag is turned relative to the camera |
| Frame age | How old the picture is, in ms. Consistently above 100 ms means it is lagging |
| ALL VISIBLE | Every target in view with distance and bearing |

### Good to know

- **All positions are measured from the camera, not the robot center.** If the
  camera is mounted off to one side, Left / Right includes that offset.
- **For a cluster, the SDK does not say exactly which of its tags it saw** — only
  what percentage. `Tags seen` is worked out from that percentage.
- **DETECTED keeps flipping between YES and NO** → the image is blurry or the tag
  is too far. Tune the camera with AprilTag Test.
- **Tag shows `(unknown)` / not in library** → that ID is not in the SDK 12.0
  library, for example an old DECODE tag.

---

## AprilTag Test (camera tuning)

### Controls

| Button | Action |
| --- | --- |
| A | Camera preview on / off (off saves CPU — compare FPS) |
| B | Auto / manual exposure |
| Dpad up / down | Manual exposure +1 / −1 ms |
| Dpad left / right | Manual gain −10 / +10 |
| X | Decimation 1 → 2 → 3 |

### What it shows

- **INIT:** camera state and the tag library.
- **Running:** FPS, decimation, exposure and gain, then an `AIM >` line for the
  closest cluster, then every detection with range / bearing / elevation (RBE),
  X / Y / Z, and pitch / roll / yaw.

### Tuning steps

1. **Exposure.** Press B for manual (starts at 6 ms, gain 250). Drive or turn the
   robot while watching the tag. If detections drop out while moving, lower
   exposure. If the image is too dark, raise gain.
2. **Decimation.** At your normal shooting distance, try 2 and 3. Use the highest
   number that still detects reliably — it gives more FPS. Use 1 only if you need
   extra range.
3. **Check distance.** Hold a tape measure to a cluster and compare with `range`.
4. **Save the values.** Settings reset every time the OpMode starts. Put your final
   numbers into `DEFAULT_EXPOSURE_MS`, `DEFAULT_GAIN`, and
   `DEFAULT_DECIMATION_INDEX` (0 = 1, 1 = 2, 2 = 3).

## Troubleshooting (both testers)

| Problem | Fix |
| --- | --- |
| INIT fails: cannot find `Webcam 1` | Configuration name, or the camera is not plugged in |
| Camera never reaches STREAMING | Replug the USB cable. Use a powered hub if on a long cable |
| Distance is clearly wrong | Camera is not a C270 / C310 / C920, or resolution was changed from 640×480 |
| Exposure / gain says "waiting for camera" | Camera is not streaming yet, or it does not support manual control |
