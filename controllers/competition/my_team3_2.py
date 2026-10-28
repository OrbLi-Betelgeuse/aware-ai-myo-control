#!/usr/bin/env python3
"""My team's controller: the forearm points the way, the hand sets the speed, the lidar keeps you off walls.

How the driver controls the robot (elbow resting, forearm held in the air)
    nothing special          → forward at the slow default speed V_MIN (the robot does not stop by itself)
    squeeze the fist         → faster forward, up to V_MAX with a firm fist
    twist the forearm        → turn left / right; a bigger twist and a firmer fist turn more (W_LEFT, W_RIGHT)
    hand up                  → back, straight, at the fixed slow speed V_BACK (the lidar cannot see behind)
    open hand, fingers wide  → also back, only with USE_BACK = True
    lidar assist             → slows down and stops by itself before hitting something in its path.
                               It never steers: the driver stays in charge
    Myo signal lost (0.25 s) → stop. At the start and after a signal loss, the robot moves only after
                               it has seen the forearm level once (so it never drives off by surprise)

Why this design: our gesture classifier failed live (every gesture came out "forward" once the band
sat about two electrodes rotated). Here the effort is the mean EMG over all 8 electrodes (the same for any
band rotation), the arm directions come from the IMU, and the fist / open-hand patterns are learned fresh
in the ~30 s calibration at the start (about 36 s with USE_BACK), so they match how the band sits right now.

Run, from the repo folder (jackal-bridge running, the sim open, myo_lsl_publisher.py streaming):
    python controllers/competition/my_team3_2.py
Saves, in a new folder per run:
    data/driving-sessions/<date_time>/calibration.json   the driver's calibration numbers
    data/driving-sessions/<date_time>/drive_log.csv      one row per 50 ms: signals, command, lidar, course
"""
from __future__ import annotations

import asyncio
import csv
import json
import os
import socket
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pylsl
import websockets

SOURCE = "my-team"            # shown in the sim's HUD; change it to your team name
RATE_HZ = 20                  # commands per second (the robot stops if none arrives for 250 ms)
BRIDGE = "ws://localhost:8765"
EMG_STREAM = "Myo_EMG"        # LSL stream names from myo_lsl_publisher.py
IMU_STREAM = "Myo_IMU"

V_MAX = 2.0                   # m/s forward at full speed (the sim's limit; L3 caps it at 1.0)
V_MIN = 0.4                   # m/s, default forward speed: the robot creeps forward when no other pose is seen
FORWARD_GAIN = 1.8            # m/s per unit of rescaled forward effort
FULL_SPEED_EFFORT = 0.70      # fraction of relaxed-to-fist calibration range
USE_BACK = False              # True = the open hand also backs up (adds the open-hand calibration pose). Off:
                              # in tests the open hand got mixed up with the fist
V_BACK = 0.6                  # m/s, fixed back speed with the hand up (the lidar cannot see behind: keep it slow)
W_LEFT = 1.3                  # rad/s, fastest LEFT turn (full twist + firm fist; limit 4.0, L3 caps it at 2.0).
                              # Raise it if left turns feel weak (our left twist is often much smaller than the right)
W_RIGHT = 1.8                 # rad/s, fastest RIGHT turn
BASE_SPEED = 0.3              # fraction of the full turn rate with a relaxed hand; the fist adds the rest
DIR_THRESHOLD = 0.55           # lift the hand more than half of your calibrated "up" to back up
DIR_EXIT_THRESHOLD = 0.40
STATE_TIMEOUT = 0.50
BACK_ACTIVE = False
BACK_REQUESTED = False
SPEED_DEADZONE = 0.15         # the first 15 % of the effort range does nothing: a loose hand does not speed up
STEER_DEADZONE = 0.25         # the first 25 % of the twist range does nothing: small arm wobbles do not turn
OPEN_TICKS = 5                # the open hand must be seen 5 ticks in a row (250 ms) before backing up. 3 was too
                              # few: moving into a twist looked "open" for 3 ticks in a test (0.2 s window blend)
MIN_EFFORT_RATIO = 1.3        # calibration check: fist and open hand must be at least 1.3 × the relaxed EMG
MAX_FALSE_BACK = 0.10         # calibration check: at most 10 % of the fist / twist / hand-up windows may look open
EMG_WINDOW = 40               # samples (200 ms at 200 Hz) for the EMG level
IMU_WINDOW = 10               # samples (200 ms at 50 Hz) averaged for the forearm direction
SIGNAL_TIMEOUT = 0.25         # s without new EMG or IMU samples → stop
READ_SECONDS = 3              # s to read each calibration instruction and get into the pose (countdown)
CALIB_SECONDS = 3.0           # s each calibration pose is then held and recorded
PATH_HALF_WIDTH = 0.30        # m, lidar points closer than this to the robot's centre line are "in the path"
                              # (the Jackal is 0.43 m wide, so this leaves about 8 cm each side)
# Lidar assist: top forward speed for the distance to the nearest point in the path (measured from the lidar).
# Straight lines between the points; closer than the first distance = stop, farther than the last = no limit.
ASSIST_DIST = [0.20, 0.50, 0.93, 1.55]    # m, distance ahead (stop at 0.20 m: front bumper about 0.06 m away)
ASSIST_SPEED = [0.0, 0.70, 1.50, 2.00]    # m/s, top forward speed at that distance
ASSIST_RELEASE_ACCEL = 0.5    # m/s², how quickly the lidar speed limit may rise as the path clears; decreases stay instant
SQUEEZE_TAU = 0.30            # s, low-pass on the fist effort: the EMG level is noisy, so the speed jumped around
STEER_TAU = 0.15              # s, low-pass on the twist
ACCEL = 1.0                   # m/s², fastest speed-up (0.4 → 2.0 m/s takes ~1.6 s)
DECEL = 2.0                   # m/s², fastest slow-down by the driver (the lidar assist and the stops are instant)
W_ACCEL = 2.5                 # rad/s², fastest turn-in: a twist no longer snaps the robot round (0 → 1.8 in ~0.7 s)
W_DECEL = 4.0                 # rad/s², fastest turn-out: untwisting straightens up quicker than it turned in

# Terminal colors (ANSI codes). Each calibration pose has its own color; the live status line uses the same ones.
WHITE, BLUE, MAGENTA, GREEN, RED, YELLOW, RESET = ("\033[1;97m", "\033[1;94m", "\033[1;95m", "\033[1;92m",
                                                   "\033[1;91m", "\033[1;93m", "\033[0m")
POSES = [  # (name, color, title, what to do)
    ("relax", GREEN, "RELAX = SLOW FORWARD", "Elbow resting, forearm in the air and level (your driving position). Hand loose."),
    ("left", BLUE, "TWIST LEFT = TURN LEFT", "Twist the forearm to the left, as far as you want for a full turn. Hand loose."),
    ("right", MAGENTA, "TWIST RIGHT = TURN RIGHT", "Twist the forearm to the right, as far as you want for a full turn. Hand loose."),
    ("up", RED, "HAND UP = BACK", "Lift your hand up (tilt the forearm up). Hand loose."),
    ("open", WHITE, "OPEN HAND = BACK", "Forearm level. Open your hand and spread all fingers wide, firmly."),
    ("squeeze", YELLOW, "FIST = FASTER", "Forearm level. Make a firm fist: this is top speed (a lighter fist is slower)."),
]

EMG: pylsl.StreamInlet = None     # set in main()
IMU: pylsl.StreamInlet = None
CAL: dict = {}                    # calibration, set in main()
LOG = None                        # csv writer for drive_log.csv, set in main()
EMG_BUF = deque(maxlen=EMG_WINDOW)
IMU_BUF = deque(maxlen=IMU_WINDOW)
LAST_SEEN = {"emg": float("-inf"), "imu": float("-inf")}   # time.monotonic() of the newest samples
ARMED = {"on": False}             # True once the forearm was seen level; False again after a signal loss
OPEN = {"ticks": 0}               # how many ticks in a row the open hand was seen
SMOOTH = {"squeeze": 0.0, "steer": 0.0, "v": 0.0, "w": 0.0, "assist_limit": V_MAX}


def reset_control():
    global BACK_ACTIVE, BACK_REQUESTED
    ARMED["on"] = False
    OPEN["ticks"] = 0
    BACK_ACTIVE = BACK_REQUESTED = False
    SMOOTH.update(squeeze=0.0, steer=0.0, v=0.0, w=0.0, assist_limit=V_MAX)
    EMG_BUF.clear()
    IMU_BUF.clear()
    LAST_SEEN.update(emg=float("-inf"), imu=float("-inf"))


def wants_back(tilt):
    global BACK_ACTIVE
    if tilt > DIR_THRESHOLD:
        BACK_ACTIVE = True
    elif tilt < DIR_EXIT_THRESHOLD:
        BACK_ACTIVE = False
    return BACK_ACTIVE


def low_pass(name: str, x: float, tau: float) -> float:
    """First-order low-pass (exponential average) with time constant tau, one tick at RATE_HZ."""
    dt = 1 / RATE_HZ
    SMOOTH[name] += dt / (tau + dt) * (x - SMOOTH[name])
    return SMOOTH[name]


def slew(cur: float, target: float, rise: float, fall: float) -> float:
    """Move cur toward target by at most rise/RATE_HZ when growing (away from zero), fall/RATE_HZ otherwise."""
    rate = (rise if abs(target) > abs(cur) and target * cur >= 0 else fall) / RATE_HZ
    return cur + float(np.clip(target - cur, -rate, rate))


def channel_rms(emg) -> np.ndarray:
    """RMS of each of the 8 electrodes over the window (offset removed). Its mean is the muscle effort,
    which does not change when the band is rotated. Its shape (which electrodes are active) tells a
    fist from an open hand."""
    x = np.asarray(emg, dtype=float)
    x = x - x.mean(axis=0)
    return np.sqrt(np.mean(x * x, axis=0))


def gravity(imu) -> np.ndarray:
    """Direction of 'up' as seen by the band, from the Myo's orientation quaternion [qw qx qy qz ...].
    Twisting or tilting the forearm turns this vector. It is referenced to gravity, so it does not drift.
    (Checked on our Myo: this formula points the same way as the accelerometer, mean dot product 0.996.)"""
    w, x, y, z = np.asarray(imu, dtype=float)[:, 0:4].T
    up = np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=1).mean(axis=0)
    return up / np.linalg.norm(up)


def open_stream(name: str) -> pylsl.StreamInlet:
    """The one LSL stream with this name on THIS laptop (other teams' Myos are on the same network)."""
    found = pylsl.resolve_bypred(f"name='{name}' and hostname='{socket.gethostname()}'", 2, 2.0)
    if len(found) != 1:
        raise SystemExit(f"Expected one {name} stream on this laptop, found {len(found)}. "
                         "Is myo_lsl_publisher.py running (only once)?")
    inlet = pylsl.StreamInlet(found[0], max_buflen=1)
    inlet.open_stream(timeout=5)
    return inlet


class CalibrationAttemptError(Exception):
    """A calibration attempt failed its signal-quality check and should be recorded again."""


def _calibrate_once(wait_for_start: bool = True) -> dict:
    """Collect one calibration attempt. Five poses take about 30 s (36 s with USE_BACK)."""
    poses = [p for p in POSES if USE_BACK or p[0] != "open"]   # no open-hand pose when back is off
    # Intro: what will happen and every pose, each in its own color. The driver reads at their own pace.
    print(f"\n{WHITE}CALIBRATION{RESET}: the controller learns how YOUR arm and hand look to the band.")
    print(f"You will hold {len(poses)} poses. For each one: a {READ_SECONDS} s countdown to read and get ready, "
          f"then hold still for {CALIB_SECONDS:.0f} s while it records.\n"
          f"Total about {len(poses) * (READ_SECONDS + CALIB_SECONDS):.0f} s. Keep the elbow resting the whole time.\n")
    for i, (_, color, title, what) in enumerate(poses, 1):
        print(f"  {color}{i}. {title:26s}{RESET} {what}")
    if wait_for_start:
        input(f"\nRead the list, then press Enter to start ... ")

    levels, rmss, ups = {}, {}, {}
    for i, (name, color, title, what) in enumerate(poses, 1):
        print(f"\n{color}Pose {i}/{len(poses)}: {title}\n{what}{RESET}")
        for k in range(READ_SECONDS, 0, -1):   # time to read the instruction and get into the pose
            print(f"\r{color}  get ready ... {k} {RESET}", end="", flush=True)
            time.sleep(1.0)
        print(f"\r{color}  HOLD IT, recording ...{RESET}", flush=True)
        EMG.pull_chunk(timeout=0.0)  # throw away the samples from moving into the pose
        IMU.pull_chunk(timeout=0.0)
        emg, imu = deque(maxlen=EMG_WINDOW), deque(maxlen=IMU_WINDOW)
        levels[name], rmss[name], ups[name] = [], [], []
        n_emg = n_imu = 0
        last_emg = last_imu = time.monotonic()
        max_gap_emg = max_gap_imu = 0.0
        end = time.monotonic() + CALIB_SECONDS
        while time.monotonic() < end:
            chunk_emg, chunk_imu = EMG.pull_chunk(timeout=0.0)[0], IMU.pull_chunk(timeout=0.0)[0]
            n_emg, n_imu = n_emg + len(chunk_emg), n_imu + len(chunk_imu)
            now = time.monotonic()
            max_gap_emg = max(max_gap_emg, now - last_emg)
            max_gap_imu = max(max_gap_imu, now - last_imu)
            if chunk_emg:
                last_emg = now
            if chunk_imu:
                last_imu = now
            emg.extend(chunk_emg)
            imu.extend(chunk_imu)
            if chunk_emg and chunk_imu and len(emg) == EMG_WINDOW and len(imu) == IMU_WINDOW:
                rms = channel_rms(emg)
                rmss[name].append(rms)
                levels[name].append(float(rms.mean()))
                ups[name].append(gravity(imu))
            time.sleep(0.05)
        now = time.monotonic()
        max_gap_emg = max(max_gap_emg, now - last_emg)
        max_gap_imu = max(max_gap_imu, now - last_imu)
        if (len(levels[name]) < 20 or n_emg < 200 * CALIB_SECONDS * 0.7
                or n_imu < 50 * CALIB_SECONDS * 0.7
                or max(max_gap_emg, max_gap_imu) > SIGNAL_TIMEOUT):
            raise CalibrationAttemptError(
                f"Not enough Myo samples for pose '{name}' ({n_emg} EMG samples; {n_imu} IMU samples; "
                f"longest gaps EMG={max_gap_emg:.2f}s, IMU={max_gap_imu:.2f}s). Check that the publisher is "
                "still streaming, then retry the full calibration.")
        print(f"{color}  done ✓{RESET}  (EMG level {np.median(levels[name]):.1f})")

    # Effort: only the relaxed hand (zero) and the firm fist (full) set the speed; the open hand sets the full
    # back speed. The other poses are not used here: for some drivers twisting is almost as strong as the fist
    # (twist right 12.5 vs fist 15.9 at 10:03), which made calibration fail. Side effect: holding a twist can
    # add some speed (30-48 % instead of 30 % in most of our calibrations). Median = ignores short spikes.
    zero = float(np.median(levels["relax"]))
    full = float(np.median(levels["squeeze"]))
    rms_relax = np.mean(rmss["relax"], axis=0)
    back = {}   # open-hand calibration, only with USE_BACK
    if USE_BACK:
        full_open = float(np.median(levels["open"]))
        # Open hand or not: which electrodes are active above the relaxed level, as unit vectors ("shapes").
        # Besides the fist, the twist and hand-up poses are "not back" references too: holding them also uses
        # forearm muscles (their EMG was close to the open hand in our 09:4x runs), so they must never look open.
        shapes = {}
        for p in ("open", "squeeze", "left", "right", "up"):
            a = np.clip(np.mean(rmss[p], axis=0) - rms_relax, 0, None)
            shapes[p] = a / max(float(np.linalg.norm(a)), 1e-9)
        others = np.array([shapes[p] for p in ("squeeze", "left", "right", "up")])
        # Check on the calibration windows themselves, with the same rule as live (optimistic: same data).
        threshold = zero + SPEED_DEADZONE * (full_open - zero)   # effort the live rule needs before "open"
        said_open = {}
        for p in ("open", "squeeze", "left", "right", "up"):
            act = np.clip(np.array(rmss[p]) - rms_relax, 0, None)
            act = act / np.maximum(np.linalg.norm(act, axis=1, keepdims=True), 1e-9)
            said_open[p] = (np.array(levels[p]) > threshold) & (act @ shapes["open"] > (act @ others.T).max(axis=1))
        false_back = {p: float(said_open[p].mean()) for p in ("squeeze", "left", "right", "up")}
        back = {"full_open": full_open, "shape_open": shapes["open"].tolist(), "shape_others": others.tolist(),
                "open_recognized": float(said_open["open"].mean()), "false_back": false_back,
                "false_back_all": float(np.mean(np.concatenate([said_open[p] for p in false_back])))}

    up0, up_left, up_right, up_up = (np.mean(ups[p], axis=0) for p in ("relax", "left", "right", "up"))
    # Steering: how far 'up' moves along the left-right direction, from the driving position.
    axis = (up_left - up_right) / np.linalg.norm(up_left - up_right)
    reach_left = float(np.dot(up_left - up0, axis))       # > 0
    reach_right = float(np.dot(up_right - up0, axis))     # < 0
    # Forward: the hand-up direction without its twist part, so twisting never drives forward.
    lift = up_up - up0
    lift = lift - np.dot(lift, axis) * axis
    reach_up = float(np.linalg.norm(lift))
    tilt_axis = lift / reach_up
    angles = {k: float(np.degrees(2 * np.arcsin(abs(r) / 2))) for k, r in
              [("left", reach_left), ("right", reach_right), ("up", reach_up)]}
    print(f"\nEffort: zero at EMG {zero:.1f}, fist {full:.1f}"
          + (f", open hand {back['full_open']:.1f}" if USE_BACK else "") + ". Angles (approx.): "
          + ", ".join(f"{k} {a:.0f}°" for k, a in angles.items()))
    # Collect every problem; main() saves the calibration file first, then stops if there is any.
    problems = []
    if not np.isfinite([zero, full, reach_left, reach_right, reach_up, *axis, *tilt_axis]).all() or full <= zero:
        problems.append("Invalid calibration values. Check signals and repeat distinct, steady poses.")
    if full < MIN_EFFORT_RATIO * zero:
        problems.append(f"The fist ({full:.1f}) must be at least {MIN_EFFORT_RATIO * zero:.1f} ({MIN_EFFORT_RATIO} × "
                        f"your relaxed level {zero:.1f}): make a firmer fist, and relax fully in pose 1.")
    if min(reach_left, -reach_right, reach_up) < 0.15:
        problems.append("Move clearly (at least ~10°) left, right and up.")
    if USE_BACK:
        print(f"Open hand recognized in {back['open_recognized']:.0%} of its calibration windows. Other poses taken "
              f"for an open hand: {back['false_back_all']:.0%} ("
              + ", ".join(f"{p} {f:.0%}" for p, f in back["false_back"].items()) + ").")
        if back["full_open"] < MIN_EFFORT_RATIO * zero:
            problems.append(f"The open hand ({back['full_open']:.1f}) must be at least {MIN_EFFORT_RATIO * zero:.1f} "
                            f"({MIN_EFFORT_RATIO} × your relaxed level {zero:.1f}): spread the fingers wider and "
                            "harder, and relax fully in pose 1.")
        if back["open_recognized"] < 0.9 or back["false_back_all"] > MAX_FALSE_BACK:
            problems.append("The band cannot tell your open hand from your other poses well enough: spread the "
                            "fingers wider and firmer, keep the hand loose while twisting, or move the band a little.")
    return {"zero": zero, "full": full, "up0": up0.tolist(), "axis": axis.tolist(),
            "reach_left": reach_left, "reach_right": reach_right, "tilt_axis": tilt_axis.tolist(),
            "reach_up": reach_up, "rms_relax": rms_relax.tolist(), **back,
            "angles_deg": angles, "median_emg_level": {p: float(np.median(levels[p])) for p in levels},
            "p90_emg_level": {p: float(np.percentile(levels[p], 90)) for p in levels}, "problems": problems}


def calibrate() -> dict:
    """Repeat the full calibration after any detected signal or pose-quality error."""
    wait_for_start = True
    while True:
        try:
            result = _calibrate_once(wait_for_start=wait_for_start)
            problems = result["problems"]
        except CalibrationAttemptError as exc:
            result = None
            problems = [str(exc)]

        if result is not None and not problems:
            return result

        print("\nCalibration was not accepted:")
        for problem in problems:
            print(f"- {problem}")
        input("Press Enter to re-record all poses (Ctrl+C to quit) ... ")
        # The retry prompt above is the only Enter needed; show the pose list and start immediately.
        wait_for_start = False


def my_input() -> dict:
    """Read the newest Myo samples: squeeze 0..1, open hand yes/no and its effort 0..1, steering -1..1
    (twist, + = left) and forward yes/no (hand up)."""
    now = time.monotonic()
    chunk, _ = EMG.pull_chunk(timeout=0.0)
    if chunk:
        EMG_BUF.extend(chunk)
        LAST_SEEN["emg"] = now
    chunk, _ = IMU.pull_chunk(timeout=0.0)
    if chunk:
        IMU_BUF.extend(chunk)
        LAST_SEEN["imu"] = now
    if now - LAST_SEEN["emg"] > SIGNAL_TIMEOUT or now - LAST_SEEN["imu"] > SIGNAL_TIMEOUT:
        reset_control()  # require fresh windows and a relaxed pose after dropout
    if len(EMG_BUF) < EMG_WINDOW or len(IMU_BUF) < IMU_WINDOW:
        return {"fresh": False, "level": 0.0, "squeeze": 0.0, "open": False, "open_effort": 0.0,
                "steer": 0.0, "tilt": 0.0, "up": False}

    rms = channel_rms(EMG_BUF)
    level = float(rms.mean())
    squeeze = (level - CAL["zero"]) / (CAL["full"] - CAL["zero"])
    open_effort, hand_open = 0.0, False
    if USE_BACK:
        # Open hand: some effort, and the active electrodes look more like the calibrated open hand than like
        # the fist, the twists or the hand-up pose.
        open_effort = (level - CAL["zero"]) / (CAL["full_open"] - CAL["zero"])
        act = np.clip(rms - CAL["rms_relax"], 0, None)
        act = act / max(float(np.linalg.norm(act)), 1e-9)
        hand_open = bool(open_effort > SPEED_DEADZONE and act @ CAL["shape_open"] > (CAL["shape_others"] @ act).max())

    up = gravity(IMU_BUF) - CAL["up0"]
    p = float(np.dot(up, CAL["axis"]))        # twist along the left-right direction
    tilt = float(np.dot(up, CAL["tilt_axis"])) / CAL["reach_up"]   # 1 = calibrated hand up, < 0 = lower
    steer = p / CAL["reach_left"] if p > 0 else -p / CAL["reach_right"]
    # Dead zones, then scale so that the firm fist / the open hand / the full calibrated twist gives 1.
    # Steering is squared: half a twist turns gently (1/4), a full twist still gives the full turn rate.
    squeeze = float(np.clip((squeeze - SPEED_DEADZONE) / (1 - SPEED_DEADZONE), 0, 1))
    open_effort = float(np.clip((open_effort - SPEED_DEADZONE) / (1 - SPEED_DEADZONE), 0, 1))
    steer = float(np.sign(steer) * np.clip((abs(steer) - STEER_DEADZONE) / (1 - STEER_DEADZONE), 0, 1) ** 2)
    return {"fresh": True, "level": level, "squeeze": squeeze, "open": hand_open, "open_effort": open_effort,
            "steer": steer, "tilt": tilt, "up": wants_back(tilt)}


def step(state: dict, inp: dict, events: list[str]) -> tuple[float, float]:
    """Default = slow forward (the fist makes it faster), twist = turn (the fist makes it faster);
    hand up = back, straight, slow. The lidar assist can only lower the forward speed, never steer.
    The fist and twist are low-passed and v, w are rate-limited, so the robot eases into speed and turns."""
    global BACK_REQUESTED
    OPEN["ticks"] = OPEN["ticks"] + 1 if inp["open"] else 0
    squeeze = low_pass("squeeze", inp["squeeze"], SQUEEZE_TAU)
    steer = low_pass("steer", inp["steer"], STEER_TAU)
    if inp["up"] or inp["open"]:
        # Back up straight at the fixed slow speed: hand up, or the open hand (only with USE_BACK, after
        # OPEN_TICKS in a row; stand still while confirming so a fist that looks open never jerks backwards).
        v = -V_BACK if inp["up"] or OPEN["ticks"] >= OPEN_TICKS else 0.0
        w = 0.0
    else:
        # Forward speed only; steering effort and obstacle braking remain unchanged.
        full_speed_input = (FULL_SPEED_EFFORT - SPEED_DEADZONE) / (1.0 - SPEED_DEADZONE)
        forward_effort = float(np.clip(squeeze / full_speed_input, 0.0, 1.0))
        v = min(V_MAX, V_MIN + FORWARD_GAIN * forward_effort)
        speed = BASE_SPEED + (1 - BASE_SPEED) * squeeze            # relaxed = slow turn, firm fist = full turn
        w = speed * (W_LEFT if steer > 0 else W_RIGHT) * steer
    back_requested = inp["up"] or inp["open"]
    if back_requested != BACK_REQUESTED:
        # One explicit zero-command tick on each direction transition.
        v = w = 0.0
        SMOOTH.update(v=0.0, w=0.0, steer=0.0, squeeze=0.0)
    else:
        v = slew(SMOOTH["v"], v, ACCEL, DECEL)
        w = 0.0 if back_requested else slew(SMOOTH["w"], w, W_ACCEL, W_DECEL)
    BACK_REQUESTED = back_requested
    ahead = path_clearance(state)
    target_limit = min(V_MAX, float(np.interp(ahead, ASSIST_DIST, ASSIST_SPEED)))   # your table: distance → top speed
    current_limit = SMOOTH["assist_limit"]
    if target_limit < current_limit:
        current_limit = target_limit  # preserve immediate braking when the path gets closer
    else:
        current_limit = min(target_limit, current_limit + ASSIST_RELEASE_ACCEL / RATE_HZ)
    SMOOTH["assist_limit"] = current_limit
    v_allowed = current_limit
    assist = v > v_allowed   # never true backing up (v < 0): the lidar cannot see behind
    v = min(v, v_allowed)    # not rate-limited: the brake acts at once
    if not inp["fresh"] or state.get("status") == "estop":
        ARMED["on"] = False   # after a signal loss, the driver must hold the arm level once before driving again
    elif not inp["open"] and not inp["up"] and inp["steer"] == 0 and inp["squeeze"] == 0:
        ARMED["on"] = True    # forearm level, not twisted, hand relaxed: a calm start before creeping forward
                              # (calibration ends with the fist, which is still in the EMG window for 0.2 s)
    if not ARMED["on"]:
        v = w = 0.0
        SMOOTH.update(squeeze=0.0, steer=0.0)   # start again from rest, not from the old filtered values
    SMOOTH["v"], SMOOTH["w"] = v, w

    # Feedback for the driver: one status line, in the same colors as the calibration poses
    effort = inp["open_effort"] if inp["open"] else inp["squeeze"]
    g, s = round(10 * effort), round(5 * abs(inp["steer"]))
    effort_bar = (RED if inp["open"] else YELLOW) + "#" * g + RESET + "." * (10 - g)
    steer_bar = ("." * (5 - s) + BLUE + "<" * s + RESET + "|....." if inp["steer"] > 0
                 else ".....|" + MAGENTA + ">" * s + RESET + "." * (5 - s))
    way = f"{RED}BACK   {RESET}" if inp["up"] or inp["open"] else f"{GREEN}FORWARD{RESET}"
    course = state.get("course") or {}
    note = ("E-STOP ACTIVE" if state.get("status") == "estop" else "NO FRESH MYO / SIM DATA: STOPPED" if not inp["fresh"] else "ARM LEVEL, HAND RELAXED TO START" if not ARMED["on"]
            else "ASSIST: obstacle ahead" if assist else "BACKING UP: no lidar behind" if v < 0 else "")
    print(f"\r{way}  EFFORT [{effort_bar}]  STEER [{steer_bar}]  v {v:+.2f}  w {w:+.2f}  "
          f"path {ahead:4.1f} m  cap {v_allowed:.2f}  "
          f"gates {course.get('gates_passed', '-')}  hits {state.get('collisions', '-')}  {note:24s}",
          end="", flush=True)

    pose = state.get("pose") or {}
    LOG.writerow([f"{time.time():.3f}", int(inp["fresh"]), int(ARMED["on"]), f"{inp['level']:.2f}",
                  f"{inp['squeeze']:.3f}", int(inp["open"]), f"{inp['open_effort']:.3f}", f"{inp['steer']:.3f}",
                  f"{inp['tilt']:.3f}", int(inp["up"]), f"{v:.3f}", f"{w:.3f}", f"{ahead:.3f}",
                  f"{v_allowed:.3f}", int(assist),
                  state.get("status", ""), pose.get("x", ""), pose.get("y", ""), pose.get("yaw", ""),
                  course.get("id", ""), course.get("gates_passed", ""), state.get("collisions", ""),
                  " ".join(events)])
    return v, w


def path_clearance(state: dict) -> float:
    """Distance (m, from the lidar) to the nearest point in the strip the robot is about to drive
    through: straight ahead, PATH_HALF_WIDTH to each side. range_max if nothing is there."""
    scan = state.get("scan")
    if not scan:
        return 10.0
    r = np.asarray(scan["ranges"], dtype=float)
    a = scan["angle_min"] + scan["angle_inc"] * np.arange(len(r))
    ahead, side = r * np.cos(a), r * np.sin(a)
    inside = (ahead > 0) & (np.abs(side) < PATH_HALF_WIDTH)
    return float(ahead[inside].min()) if inside.any() else float(scan["range_max"])


# The plumbing below is the same as controller_template.py.
async def run(url: str, seconds: float = 0) -> None:
    base = url.rstrip("/")
    state: dict = {}
    events: list[str] = []
    t_end = time.monotonic() + seconds if seconds else None
    backoff = 1.0
    while t_end is None or time.monotonic() < t_end:
        try:
            async with websockets.connect(base + "/state") as st, websockets.connect(base + "/cmd") as cmd:
                print(f"✓ connected to {base}; running step() at {RATE_HZ} Hz (Ctrl+C to stop)")
                backoff = 1.0
                reset_control()
                state.clear()
                events.clear()
                last_state = float("-inf")

                async def listen() -> None:
                    nonlocal last_state
                    async for raw in st:
                        m = json.loads(raw)
                        if m.get("type") == "state":
                            state.clear()
                            state.update(m)
                            last_state = time.monotonic()
                        elif m.get("type") == "event":
                            events.append(m["name"])

                listener = asyncio.create_task(listen())
                seq = 0
                try:
                    while t_end is None or time.monotonic() < t_end:
                        t0 = time.monotonic()
                        if listener.done():
                            listener.result()  # re-raise a dropped connection
                            raise OSError("State connection ended")
                        happened, events[:] = list(events), []
                        inp = my_input()
                        if time.monotonic() - last_state > STATE_TIMEOUT:
                            reset_control()
                            inp = dict(inp, fresh=False, up=False, open=False, squeeze=0.0, steer=0.0)
                        v, w = step(state, inp, happened)
                        await cmd.send(json.dumps({"type": "cmd", "v": float(v), "w": float(w),
                                                   "source": SOURCE, "seq": seq, "t": time.time()}))
                        seq += 1
                        await asyncio.sleep(max(0.0, 1 / RATE_HZ - (time.monotonic() - t0)))
                finally:
                    reset_control()
                    listener.cancel()
                    try:
                        await cmd.send(json.dumps({"type": "cmd", "v": 0.0, "w": 0.0, "source": SOURCE}))
                    except websockets.ConnectionClosed:
                        pass
        except (OSError, websockets.ConnectionClosed, websockets.InvalidHandshake) as e:
            print(f"✗ bridge not reachable ({e.__class__.__name__}); retrying in {backoff:.0f} s")
            await asyncio.sleep(backoff)
            backoff = min(5.0, backoff * 2)


def main() -> None:
    global EMG, IMU, CAL, LOG
    print("Controller 3.1: 3-second calibration countdown, Enter-to-retry, and slow lidar speed-limit recovery.")
    os.system("")  # turns on the colors in the Windows console (no effect on Mac / Linux)
    EMG, IMU = open_stream(EMG_STREAM), open_stream(IMU_STREAM)
    print(f"✓ {EMG_STREAM} and {IMU_STREAM} found on this laptop.")
    CAL = calibrate()
    EMG.pull_chunk(timeout=0.0)
    IMU.pull_chunk(timeout=0.0)
    reset_control()

    out = Path(__file__).resolve().parents[2] / "data" / "driving-sessions" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True)
    (out / "calibration.json").write_text(json.dumps(CAL, indent=2) + "\n")
    if CAL["problems"]:
        raise SystemExit(f"\nCalibration failed (numbers saved in {out / 'calibration.json'}):\n- "
                         + "\n- ".join(CAL["problems"]) + "\nRun again.")
    for k in ["up0", "axis", "tilt_axis", "rms_relax"] + (["shape_open", "shape_others"] if USE_BACK else []):
        CAL[k] = np.array(CAL[k])
    LOG = csv.writer(open(out / "drive_log.csv", "w", newline="", buffering=1))
    LOG.writerow(["time_s", "fresh", "armed", "emg_level", "squeeze", "open", "open_effort", "steer", "tilt",
                  "hand_up", "v_cmd", "w_cmd", "path_clear_m", "distance_v_limit", "assist", "status", "x", "y", "yaw", "course",
                  "gates_passed", "collisions", "events"])
    print(f"Saving to {out}\nDefault = slow forward, fist = faster, twist = turn, hand up = back"
          f"{', open hand = back' if USE_BACK else ''}. Ctrl+C stops.\n")
    try:
        asyncio.run(run(BRIDGE))
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
