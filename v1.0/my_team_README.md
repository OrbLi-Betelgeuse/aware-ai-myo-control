# Our controller: drive the Jackal with one arm

Code: [`my_team.py`](my_team.py). It uses the Myo's EMG (muscle activity) and IMU (arm angle).
No machine learning. A 30 s calibration per driver.

## How to drive

Rest your elbow and hold the forearm in the air, level. The **arm points the way**, the **squeeze is the gas**.

| Arm | Robot |
|---|---|
| Hand up | forward |
| Hand down | back (slow) |
| Twist left / right | turn left / right |
| Forearm level, no twist | stop |
| Squeeze the fist | faster (relaxed hand = 30 % speed, medium fist = full speed) |

## Run it

1. `jackal-bridge --serve-site --open` (the sim opens in the browser)
2. `python examples/myo_lsl_publisher.py` (Myo on the arm)
3. `python examples/my_team.py`, then follow the screen. Calibration is 6 poses of 3 s each:
   relax, twist left, twist right, hand up, hand down, medium fist.

The terminal shows one live line: direction, squeeze bar, steer bar, speed, distance ahead.

## The algorithm (every 50 ms)

1. **Read** the last 200 ms of EMG (8 channels) and IMU.
2. **Squeeze**: RMS of each EMG channel, averaged over all 8. Scaled to 0 (relaxed) to 1 (medium fist).
   The average does not change if the band is rotated on the arm.
3. **Arm direction**: from the IMU, find which way gravity points in the band. Compare with the
   calibrated poses: the left-right part gives the steering (-1 to 1), the up-down part gives forward or back.
4. **Command**: `speed = 0.3 + 0.7 × squeeze`, then
   `v = speed × 2.0` (forward) or `speed × -0.3` (back) or 0, and `w = speed × 1.5 × steer`.
5. **Lidar assist**: forward speed is limited to `0.8 × (distance ahead - 0.3 m)`.
   It only slows the robot down. It never steers.
6. **Safety**: no Myo data for 0.25 s means stop. After a signal loss, it waits until the arm is level.
7. **Send** `(v, w)` to the bridge, 20 times a second.

## Why not a gesture classifier

Our classifier scored 98-100 % on recorded data but answered "forward" for every gesture live.
The band sat about 2 electrodes rotated. The EMG average and the gravity direction do not depend on that.

## Settings (top of `my_team.py`)

| Setting | Value | Meaning |
|---|---|---|
| `V_MAX` | 2.0 m/s | top forward speed (the sim's limit) |
| `V_BACK` | 0.3 m/s | top backward speed (the lidar cannot see behind) |
| `W_MAX` | 1.5 rad/s | top turn rate |
| `BASE_SPEED` | 0.3 | speed with a relaxed hand, as a fraction of the top speed |
| `ASSIST_GAIN` | 0.8 1/s | how early the lidar assist slows down |

## Output

Each run saves `results/my_team/<date_time>/calibration.json` and `drive_log.csv` (one row per 50 ms).

## Tested so far

With a scripted fake Myo and the real sim: the robot reaches 2 m/s and stops about 0.3 m before an
obstacle, also with the judged link delay (300 ms, 10 % drop). Course scores with a real driver: still to do.

## Known limits

- An arm that sinks while resting makes the robot back up slowly. Hold the arm level to stop.
- About 0.2 s lag after changing the pose.
- No lidar protection when backing up.
