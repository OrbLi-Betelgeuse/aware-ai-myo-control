# Our controller: drive the Jackal with one arm

Code: [`my_team.py`](my_team.py). It uses the Myo's EMG (muscle activity) and IMU (arm angle).
No pre-trained model: a 45 s calibration at the start learns each driver.

## How to drive

Rest your elbow and hold the forearm in the air, level. The robot **always creeps forward**; you steer,
speed up, or back up.

| Arm / hand | Robot |
|---|---|
| Nothing special (forearm level, hand relaxed) | forward at the slow default speed (0.3 m/s) |
| Make a fist | faster forward (firm fist = 2.0 m/s) |
| Twist left / right | turn left / right (bigger twist and firmer fist = sharper turn) |
| Hand up | back, straight, slow (0.2 m/s) |

It stops only when the Myo signal is lost, at the start until it has seen your arm level once, and when
the lidar assist stops it in front of an obstacle.

## Run it

1. `jackal-bridge --serve-site --open` (the sim opens in the browser)
2. `python examples/myo_lsl_publisher.py` (Myo on the arm)
3. `python examples/my_team.py`, then follow the screen. Calibration is 5 poses (about 45 s): for each,
   a 6 s countdown to read and get ready, then 3 s to hold. The poses: relax, twist left, twist right,
   hand up, firm fist. The numbers are saved in `calibration.json`.

The terminal shows one live line: direction, effort bar, steer bar, speed, distance ahead.

## The algorithm (every 50 ms)

1. **Read** the last 200 ms of EMG (8 channels) and IMU.
2. **Effort**: RMS of each EMG channel, averaged over all 8. Scaled to 0 (relaxed) to 1 (firm fist).
   The average does not change if the band is rotated on the arm.
3. **Arm direction**: from the IMU, find which way gravity points in the band. Compared with the
   calibrated poses, the left-right part gives the steering, the up part means back.
   Steering ignores the first 25 % of the twist and is squared, so small wobbles do not turn the robot.
4. **Command**: hand up → `v = -0.2` (no turning). Otherwise `v = 0.3 + 1.7 × effort` and
   `w = (0.3 + 0.7 × effort) × W_LEFT or W_RIGHT × steer`.
5. **Lidar assist**: forward speed is limited to `0.8 × (distance ahead - 0.3 m)`.
   It only slows the robot down. It never steers.
6. **Safety**: no Myo data for 0.25 s means stop. At the start and after a signal loss, it waits until the arm is level and the hand relaxed.
7. **Send** `(v, w)` to the bridge, 20 times a second.

## Why not a pre-trained gesture classifier

Our classifier scored 98-100 % on recorded data but answered "forward" for every gesture live.
The band sat about 2 electrodes rotated. The EMG average and the arm angle do not depend on that.

## Settings (top of `my_team.py`)

| Setting | Value | Meaning |
|---|---|---|
| `V_MIN` | 0.3 m/s | default forward speed |
| `V_MAX` | 2.0 m/s | top forward speed with a firm fist (the sim's limit) |
| `V_BACK` | 0.2 m/s | fixed back speed with the hand up (the lidar cannot see behind) |
| `W_LEFT`, `W_RIGHT` | 1.0 rad/s | top turn rate to the left, to the right |
| `STEER_DEADZONE` | 0.25 | part of the twist that does nothing |
| `BASE_SPEED` | 0.3 | turn rate with a relaxed hand, as a fraction of the top turn rate |
| `ASSIST_GAIN` | 0.8 1/s | how early the lidar assist slows down |
| `USE_BACK` | False | an extra "open hand = back" gesture (off: it got mixed up with the fist) |

## Output

Each run saves `results/my_team/<date_time>/calibration.json` and `drive_log.csv` (one row per 50 ms).

## Tested so far

With a scripted fake Myo and the real sim: forward up to 2 m/s with a stop about 0.3 m before an
obstacle (also with the judged 300 ms delay and 10 % drop). Course scores with a real driver: still to do.

## Known limits

- The robot never stops by itself in open space. To wait, back up a little or let the lidar stop it at a wall.
- Our left twist is often much smaller than the right one. If left turns feel weak, raise `W_LEFT`.
- While driving, holding a twist can add some speed for some drivers (their twisting muscles show up in the EMG).
- About 0.2 s lag after changing the pose.
- No lidar protection when backing up.
