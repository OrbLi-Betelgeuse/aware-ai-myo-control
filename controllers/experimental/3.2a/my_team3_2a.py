#!/usr/bin/env python3
"""3.2a experimental safety variant of the final 3.2 competition controller.

Preserves 3.2 motion mapping, piecewise LiDAR cap and gradual release.
Adapts validation/watchdogs from the teammate's 2.6 safety controller.
Run only with the external Myo publisher and jackal-bridge ready:
    python controllers/experimental/3.2a/my_team3_2a.py

Relaxed means forward after arming. Reverse has no rear obstacle protection.
This is software-checked experimental code, not a validated physical safety system.
All files for a run stay together in data/driving-sessions/<timestamp>_3_2a/.
"""
from __future__ import annotations

import asyncio
import csv
from contextlib import suppress
import json
import math
import os
import socket
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pylsl
import websockets

VERSION = "3.2a-experimental"
BASE_SHA256 = "c2d1d1f0d6bfea675f3c323cd012214255523d26117223bc805a44a7ecb894a8"
SAFETY_DONOR_SHA256 = "75dc2d507f7a9b4c77ee2e4f48f9f1abc5890d19566987e38744c08c8578ef39"
EPS = 1e-9
LOOP_TIMEOUT = 0.25
WINDOW_MAX_AGE = 0.30
FUTURE_TOLERANCE = 0.05
LIDAR_BEAMS = 180
LAST_STEP = None
LAST_FAULT = ""
PREPARE_SECONDS = 3.0
READY_SECONDS = 0.8
READY_STARTED = None
PREPARE_STARTED = None
PREPARE_DONE = False
PREPARE_UI = {"phase": "waiting", "remaining": 3, "reason": "Waiting for fresh neutral input"}
PREPARE_PORT = 8766
EMG_TIMES = deque(maxlen=40)
IMU_TIMES = deque(maxlen=10)
LAST_SOURCE = {"emg": float("-inf"), "imu": float("-inf")}

SOURCE = "my-team-3.2a"            # shown in the sim's HUD; change it to your team name
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


class ControlFault(ValueError):
    """A command cannot safely be calculated from the current data."""

def finite_number(value) -> bool:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        return False
    try:
        return math.isfinite(float(value))
    except (ValueError, TypeError, OverflowError):
        return False

def validate_configuration() -> None:
    values = (V_MIN, V_MAX, FORWARD_GAIN, FULL_SPEED_EFFORT, V_BACK, W_LEFT, W_RIGHT,
              BASE_SPEED, SPEED_DEADZONE, STEER_DEADZONE, DIR_THRESHOLD, DIR_EXIT_THRESHOLD,
              SQUEEZE_TAU, STEER_TAU, ACCEL, DECEL, W_ACCEL, W_DECEL)
    if not all(finite_number(v) for v in values):
        raise ControlFault("Configuration contains nonfinite values")
    if not (0 <= V_MIN <= V_MAX <= 2 and FORWARD_GAIN > 0 and 0 <= V_BACK <= 2
            and 0 <= W_LEFT <= 4 and 0 <= W_RIGHT <= 4 and 0 <= BASE_SPEED <= 1
            and 0 <= SPEED_DEADZONE < FULL_SPEED_EFFORT <= 1 and 0 <= STEER_DEADZONE < 1
            and 0 <= DIR_EXIT_THRESHOLD < DIR_THRESHOLD <= 1
            and min(SQUEEZE_TAU, STEER_TAU) >= 0
            and min(ACCEL, DECEL, W_ACCEL, W_DECEL) > 0 and RATE_HZ > 0):
        raise ControlFault("Invalid speed, dead-zone, direction, or filter configuration")
    if (len(ASSIST_DIST) != len(ASSIST_SPEED) or len(ASSIST_DIST) < 2
            or not all(finite_number(x) for x in (*ASSIST_DIST, *ASSIST_SPEED, ASSIST_RELEASE_ACCEL))
            or ASSIST_DIST[0] < 0 or any(a >= b for a, b in zip(ASSIST_DIST, ASSIST_DIST[1:]))
            or any(a > b for a, b in zip(ASSIST_SPEED, ASSIST_SPEED[1:]))
            or min(ASSIST_SPEED) < 0 or max(ASSIST_SPEED) > V_MAX or ASSIST_RELEASE_ACCEL <= 0):
        raise ControlFault("Invalid 3.2 LiDAR speed table/release configuration")

def sample_array(samples, kind: str) -> np.ndarray:
    try:
        x = np.asarray(samples, dtype=float)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ControlFault(f"Malformed {kind.upper()} samples") from exc
    expected = 8 if kind == "emg" else 10
    if x.ndim != 2 or not len(x) or x.shape[1] != expected or not np.isfinite(x).all():
        raise ControlFault(f"{kind.upper()} needs finite rows of {expected} channels")
    return x

def unit(vector, name: str) -> np.ndarray:
    try:
        x = np.asarray(vector, dtype=float)
        with np.errstate(over="raise", invalid="raise"):
            norm = float(np.linalg.norm(x))
    except (ValueError, TypeError, OverflowError, FloatingPointError) as exc:
        raise ControlFault(f"Invalid {name}") from exc
    if x.shape != (3,) or not np.isfinite(x).all() or not finite_number(norm) or norm <= EPS:
        raise ControlFault(f"Degenerate {name}")
    return x / norm

def read_samples(inlet, kind: str):
    """Same-host LSL timestamps use local_clock, not time.monotonic().

    Within-chunk timestamp reordering is allowed. Replayed chunk maxima cannot
    refresh the watchdog. Arrival order is preserved rather than sorted.
    """
    try:
        chunk, stamps = inlet.pull_chunk(timeout=0.0)
        if not len(chunk):
            if len(stamps):
                raise ControlFault(f"Unpaired {kind.upper()} timestamps")
            return None
        samples = sample_array(chunk, kind)
        stamps = np.asarray(stamps, dtype=float)
        if stamps.shape != (len(samples),) or not np.isfinite(stamps).all():
            raise ControlFault(f"Invalid {kind.upper()} timestamps")
        source_now = pylsl.local_clock()
        newest = float(stamps.max())
        if (not finite_number(source_now) or not -FUTURE_TOLERANCE <= source_now - newest <= SIGNAL_TIMEOUT
                or source_now - float(stamps.min()) > WINDOW_MAX_AGE):
            raise ControlFault(f"Old/future {kind.upper()} measurements")
        if newest <= LAST_SOURCE[kind]:
            return None
        if kind == "imu":
            gravity(samples)
        LAST_SOURCE[kind] = newest
        return samples, stamps
    except ControlFault:
        raise
    except Exception as exc:
        raise ControlFault(f"Cannot read {kind.upper()}: {exc}") from exc

def validate_calibration(cal: dict) -> None:
    keys = ("zero", "full", "reach_left", "reach_right", "reach_up")
    if not isinstance(cal, dict) or not all(finite_number(cal.get(k)) for k in keys):
        raise ControlFault("Missing/nonfinite calibration values")
    if cal["zero"] < 0 or cal["full"] - cal["zero"] <= EPS * max(1.0, abs(cal["full"])):
        raise ControlFault("No usable relaxed-to-fist effort span")
    if cal["full"] < MIN_EFFORT_RATIO * cal["zero"]:
        raise ControlFault("Fist effort must be at least 1.3 times relaxed effort")
    for key in ("up0", "axis", "tilt_axis"):
        unit(cal.get(key), key)
    if min(cal["reach_left"], -cal["reach_right"], cal["reach_up"]) < 0.15:
        raise ControlFault("Reference twists/lift are too small or have incorrect signs")
    if USE_BACK:
        if not finite_number(cal.get("full_open")) or cal["full_open"] - cal["zero"] <= EPS:
            raise ControlFault("No usable open-hand effort span")
        for key, shape in (("rms_relax", (8,)), ("shape_open", (8,)), ("shape_others", (4, 8))):
            values = np.asarray(cal.get(key), dtype=float)
            if values.shape != shape or not np.isfinite(values).all():
                raise ControlFault(f"Invalid {key}")

def input_failure(reason: str, waiting: bool = False) -> dict:
    reset_control(clear_buffers=not waiting)
    return {"fresh": False, "level": 0.0, "squeeze": 0.0, "open": False,
            "open_effort": 0.0, "steer": 0.0, "tilt": 0.0, "up": False,
            "new_emg": False, "waiting": waiting, "error": reason}

def _state_fields(state: dict) -> None:
    if not isinstance(state, dict) or state.get("type") != "state" or not finite_number(state.get("t")):
        raise ControlFault("Malformed simulator state")
    for key, fields in (("pose", ("x", "y", "yaw")), ("vel", ("v", "w")), ("cmd_applied", ("v", "w"))):
        value = state.get(key)
        if not isinstance(value, dict) or not all(finite_number(value.get(k)) for k in fields):
            raise ControlFault(f"Invalid simulator {key}")
    hits = state.get("collisions")
    if isinstance(hits, bool) or not isinstance(hits, (int, np.integer)) or hits < 0:
        raise ControlFault("Invalid simulator collisions")
    if state.get("status") not in ("ok", "idle", "deadman", "estop", "finished"):
        raise ControlFault("Invalid simulator status")
    if state.get("cmd_echo_t") is not None and not finite_number(state["cmd_echo_t"]):
        raise ControlFault("Invalid echoed command timestamp")
    course = state.get("course")
    if course is not None:
        if (not isinstance(course, dict) or not isinstance(course.get("id"), str)
                or not finite_number(course.get("elapsed")) or course["elapsed"] < 0
                or not isinstance(course.get("done"), bool)):
            raise ControlFault("Invalid course state")
        for key in ("next_gate", "gates_passed"):
            if isinstance(course.get(key), bool) or not isinstance(course.get(key), int) or course[key] < 0:
                raise ControlFault("Invalid course progress")

def state_clearance(state: dict, now: float) -> float:
    _state_fields(state)
    stamp = state.get("_received_at")
    if not finite_number(stamp) or not 0 <= now - stamp <= STATE_TIMEOUT:
        raise ControlFault("Simulator state missing/stale")
    if state["status"] not in ("ok", "idle", "deadman"):
        raise ControlFault(f"Simulator {state['status']}")
    if state.get("_terminal") is True:
        raise ControlFault("Terminal event received; reset the course before rearming")
    if isinstance(state.get("course"), dict) and state["course"]["done"]:
        raise ControlFault("Course finished or DNF; reset the course before rearming")
    return path_clearance(state)

async def listen_state(st, state: dict, events: list[str]) -> None:
    last_t = None
    last_course = None
    terminal = False
    try:
        async for raw in st:
            try:
                message = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ControlFault(f"Invalid JSON {value}")))
            except (ValueError, TypeError) as exc:
                raise ControlFault("Malformed state-channel JSON") from exc
            if not isinstance(message, dict):
                raise ControlFault("State-channel payload is not an object")
            if message.get("type") == "event":
                if not isinstance(message.get("name"), str):
                    raise ControlFault("Invalid event name")
                events.append(message["name"])
                if message["name"] in ("estop", "finish", "dnf"):
                    reset_control()
                    state["status"] = "estop" if message["name"] == "estop" else "finished"
                    if message["name"] in ("finish", "dnf"):
                        terminal = True
                        state["_terminal"] = True
                continue
            _state_fields(message)
            path_clearance(message)
            stamp = message["t"]
            course_id = (message.get("course") or {}).get("id")
            if last_t is not None and course_id != last_course:
                reset_control()
                terminal = False
            last_course = course_id
            if last_t is not None and stamp == last_t:
                if message["status"] in ("estop", "finished"):
                    state["status"] = message["status"]
                    reset_control()
                continue
            if last_t is not None and stamp < last_t:
                reset_control()
                terminal = False  # Simulator reset() restarts its source clock at zero.
            if message["status"] in ("estop", "finished"):
                reset_control()
            if message["status"] == "finished" or (isinstance(message.get("course"), dict) and message["course"]["done"]):
                terminal = True
            last_t = stamp
            state.clear()
            state.update(message)
            state["_received_at"] = time.monotonic()
            if terminal:
                state["_terminal"] = True
        state.clear()
        reset_control()
    except BaseException:
        state.clear()
        reset_control()
        raise

async def _send_zero(cmd) -> None:
    with suppress(Exception):
        await asyncio.wait_for(cmd.send(json.dumps({"type": "cmd", "v": 0.0, "w": 0.0,
                                                  "source": SOURCE}, allow_nan=False)), timeout=SIGNAL_TIMEOUT)

def reset_control(clear_buffers: bool = True) -> None:
    global BACK_ACTIVE, BACK_REQUESTED, LAST_STEP, PREPARE_STARTED, PREPARE_DONE, READY_STARTED
    PREPARE_STARTED, PREPARE_DONE = None, False
    READY_STARTED = None
    PREPARE_UI.update(phase="waiting", remaining=3, reason="Return to neutral to prepare")
    ARMED["on"], OPEN["ticks"] = False, 0
    BACK_ACTIVE = BACK_REQUESTED = False
    LAST_STEP = None
    SMOOTH.update(squeeze=0.0, steer=0.0, v=0.0, w=0.0, assist_limit=V_MAX)
    if clear_buffers:
        for buffer in (EMG_BUF, IMU_BUF, EMG_TIMES, IMU_TIMES):
            buffer.clear()
        LAST_SEEN.update(emg=float("-inf"), imu=float("-inf"))
        LAST_SOURCE.update(emg=float("-inf"), imu=float("-inf"))


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
    x = sample_array(emg, "emg")
    if len(x) < 2:
        raise ControlFault("EMG RMS needs at least two samples")
    try:
        with np.errstate(over="raise", invalid="raise"):
            x = x - x.mean(axis=0)
            result = np.sqrt(np.mean(x * x, axis=0))
    except FloatingPointError as exc:
        raise ControlFault("EMG arithmetic overflow") from exc
    if not np.isfinite(result).all():
        raise ControlFault("Invalid EMG RMS")
    return result


def gravity(imu) -> np.ndarray:
    q = sample_array(imu, "imu")[:, :4]
    with np.errstate(over="ignore", invalid="ignore"):
        norms = np.linalg.norm(q, axis=1)
    if not np.isfinite(norms).all() or np.any(norms <= EPS):
        raise ControlFault("Zero/invalid IMU quaternion")
    w, x, y, z = (q / norms[:, None]).T
    up = np.stack((2 * (x * z - w * y), 2 * (y * z + w * x),
                   1 - 2 * (x * x + y * y)), axis=1).mean(axis=0)
    return unit(up, "IMU gravity direction")


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
            chunk_emg = calibration_chunk(EMG, "emg")
            chunk_imu = calibration_chunk(IMU, "imu")
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
    axis = unit(up_left - up_right, "left/right calibration axis")
    reach_left = float(np.dot(up_left - up0, axis))       # > 0
    reach_right = float(np.dot(up_right - up0, axis))     # < 0
    # Forward: the hand-up direction without its twist part, so twisting never drives forward.
    lift = up_up - up0
    lift = lift - np.dot(lift, axis) * axis
    reach_up = float(np.linalg.norm(lift))
    tilt_axis = unit(lift, "hand-up calibration axis")
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
            reset_control()
            result = _calibrate_once(wait_for_start=wait_for_start)
            validate_calibration(result)
            problems = result["problems"]
        except (CalibrationAttemptError, ValueError, TypeError, KeyError, OverflowError, FloatingPointError, OSError) as exc:
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
    now, new_emg = time.monotonic(), False
    try:
        for kind, inlet, values, times in (("emg", EMG, EMG_BUF, EMG_TIMES),
                                          ("imu", IMU, IMU_BUF, IMU_TIMES)):
            packet = read_samples(inlet, kind)
            if packet is not None:
                rows, stamps = packet
                values.extend(rows)
                times.extend(stamps)
                LAST_SEEN[kind] = now
                new_emg = new_emg or kind == "emg"
        if any(now - stamp > SIGNAL_TIMEOUT for stamp in LAST_SEEN.values() if stamp != float("-inf")):
            raise ControlFault("Myo stream stopped advancing")
        source_now = pylsl.local_clock()
        if any(stamps and source_now - min(stamps) > WINDOW_MAX_AGE for stamps in (EMG_TIMES, IMU_TIMES)):
            raise ControlFault("Myo window too old; sample rate dropped")
        if (any(stamp == float("-inf") for stamp in LAST_SEEN.values())
                or len(EMG_BUF) < EMG_WINDOW or len(IMU_BUF) < IMU_WINDOW):
            return input_failure("Waiting for fresh Myo windows", waiting=True)
        if len(EMG_TIMES) != len(EMG_BUF) or len(IMU_TIMES) != len(IMU_BUF):
            raise ControlFault("Sample/timestamp history mismatch")
        validate_calibration(CAL)
        rms = channel_rms(EMG_BUF)
        level = float(rms.mean())
        raw = (level - CAL["zero"]) / (CAL["full"] - CAL["zero"])
        open_effort, hand_open = 0.0, False
        if USE_BACK:
            open_effort = (level - CAL["zero"]) / (CAL["full_open"] - CAL["zero"])
            active = np.clip(rms - np.asarray(CAL["rms_relax"]), 0, None)
            active /= max(float(np.linalg.norm(active)), EPS)
            hand_open = bool(open_effort > SPEED_DEADZONE
                             and active @ np.asarray(CAL["shape_open"])
                             > (np.asarray(CAL["shape_others"]) @ active).max())
        change = gravity(IMU_BUF) - np.asarray(CAL["up0"])
        p = float(np.dot(change, CAL["axis"]))
        tilt = float(np.dot(change, CAL["tilt_axis"])) / CAL["reach_up"]
        steer = p / CAL["reach_left"] if p > 0 else -p / CAL["reach_right"]
        squeeze = float(np.clip((raw - SPEED_DEADZONE) / (1 - SPEED_DEADZONE), 0, 1))
        steer = float(np.sign(steer) * np.clip((abs(steer) - STEER_DEADZONE) / (1 - STEER_DEADZONE), 0, 1) ** 2)
        open_effort = float(np.clip((open_effort - SPEED_DEADZONE) / (1 - SPEED_DEADZONE), 0, 1))
        if not all(finite_number(x) for x in (level, squeeze, steer, tilt, open_effort)):
            raise ControlFault("Nonfinite Myo features")
        return {"fresh": True, "level": level, "squeeze": squeeze, "open": hand_open,
                "open_effort": open_effort, "steer": steer, "tilt": tilt, "up": wants_back(tilt),
                "new_emg": new_emg, "waiting": False, "error": ""}
    except Exception as exc:
        return input_failure(f"Invalid/unavailable Myo data: {exc}")


def _motion_step(state: dict, inp: dict, events: list[str]) -> tuple[float, float]:
    """Default = slow forward (the fist makes it faster), twist = turn (the fist makes it faster);
    hand up = back, straight, slow. The lidar assist can only lower the forward speed, never steer.
    The fist and twist are low-passed and v, w are rate-limited, so the robot eases into speed and turns."""
    global BACK_REQUESTED
    OPEN["ticks"] = (OPEN["ticks"] + int(inp.get("new_emg", True))) if inp["open"] else 0
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
    elif not inp["open"] and not inp["up"] and inp["steer"] == 0 and inp["squeeze"] == 0 and abs(inp["tilt"]) <= DIR_EXIT_THRESHOLD:
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
    write_log([f"{time.time():.3f}", int(inp["fresh"]), int(ARMED["on"]), f"{inp['level']:.2f}",
                  f"{inp['squeeze']:.3f}", int(inp["open"]), f"{inp['open_effort']:.3f}", f"{inp['steer']:.3f}",
                  f"{inp['tilt']:.3f}", int(inp["up"]), f"{v:.3f}", f"{w:.3f}", f"{ahead:.3f}",
                  f"{v_allowed:.3f}", int(assist),
                  state.get("status", ""), pose.get("x", ""), pose.get("y", ""), pose.get("yaw", ""),
                  course.get("id", ""), course.get("gates_passed", ""), state.get("collisions", ""),
                  " ".join(events), ""])
    return v, w


def path_clearance(state: dict) -> float:
    if not isinstance(state, dict) or not isinstance(state.get("scan"), dict):
        raise ControlFault("Missing lidar scan")
    scan = state["scan"]
    try:
        ranges = np.asarray(scan["ranges"], dtype=float)
        amin, inc, maximum = scan["angle_min"], scan["angle_inc"], scan["range_max"]
    except (KeyError, ValueError, TypeError, OverflowError) as exc:
        raise ControlFault("Malformed lidar scan") from exc
    if (not all(finite_number(v) for v in (amin, inc, maximum)) or inc <= 0 or maximum <= 0
            or ranges.ndim != 1 or len(ranges) != LIDAR_BEAMS or not np.isfinite(ranges).all()
            or np.any(ranges < 0) or np.any(ranges > maximum)):
        raise ControlFault("Invalid lidar ranges/geometry")
    angles = amin + inc * np.arange(len(ranges))
    if (not np.isfinite(angles).all() or angles[0] > 0 or angles[-1] < 0
            or not np.any(np.abs(angles) <= 0.1)):
        raise ControlFault("Lidar does not cover forward")
    # A zero reading is valid contact, never a no-hit return. Conservatively stop.
    if np.any(ranges == 0):
        return 0.0
    ahead, side = ranges * np.cos(angles), ranges * np.sin(angles)
    inside = (ahead > 0) & (np.abs(side) < PATH_HALF_WIDTH)
    return float(ahead[inside].min()) if inside.any() else float(maximum)


# Transport watchdogs adapted from the teammate safety controller.
async def run(url: str, seconds: float = 0) -> None:
    end = time.monotonic() + seconds if seconds else None
    backoff = 1.0
    while end is None or time.monotonic() < end:
        state, events = {}, []
        reset_control()
        try:
            async with websockets.connect(url.rstrip("/") + "/state") as st, websockets.connect(url.rstrip("/") + "/cmd") as cmd:
                listener = asyncio.create_task(listen_state(st, state, events))
                seq, backoff = 0, 1.0
                try:
                    while end is None or time.monotonic() < end:
                        start = time.monotonic()
                        if listener.done():
                            listener.result()
                            raise OSError("State connection ended")
                        happened, events[:] = list(events), []
                        v, w = step(state, my_input(), happened)
                        if time.monotonic() - start > LOOP_TIMEOUT:
                            reset_control()
                            v = w = 0.0
                        await asyncio.wait_for(cmd.send(json.dumps({"type": "cmd", "v": v, "w": w,
                                                                   "source": SOURCE, "seq": seq, "t": time.time()},
                                                                  allow_nan=False)), timeout=SIGNAL_TIMEOUT)
                        seq += 1
                        await asyncio.sleep(max(0.0, 1 / RATE_HZ - (time.monotonic() - start)))
                finally:
                    reset_control()
                    listener.cancel()
                    with suppress(asyncio.CancelledError, Exception):
                        await listener
                    await _send_zero(cmd)
        except (OSError, ValueError, TypeError, websockets.ConnectionClosed, websockets.InvalidHandshake) as exc:
            reset_control()
            with suppress(OSError):
                print(f"\nConnection/state fault: {exc}; retry in {backoff:g}s")
            if end is not None and time.monotonic() >= end:
                break
            await asyncio.sleep(backoff)
            backoff = min(5.0, backoff * 2)


def calibration_chunk(inlet, kind):
    packet = read_samples(inlet, kind)
    return [] if packet is None else packet[0].tolist()


def write_log(row):
    if LOG is not None:
        with suppress(OSError, ValueError, TypeError, csv.Error):
            LOG.writerow(row)


def step(state: dict, inp: dict, events: list[str]) -> tuple[float, float]:
    """Validate first, then apply the 3.2 motion law. Faults disarm and return zero."""
    global LAST_STEP, LAST_FAULT
    preserve_partial = False
    try:
        validate_configuration()
        now = time.monotonic()
        if LAST_STEP is not None and not 0 <= now - LAST_STEP <= LOOP_TIMEOUT:
            raise ControlFault("Control loop stalled; return to neutral")
        LAST_STEP = now
        state_clearance(state, now)
        keys = ("level", "squeeze", "open_effort", "steer", "tilt")
        valid = (isinstance(inp, dict) and all(finite_number(inp.get(k)) for k in keys)
                 and inp["level"] >= 0 and 0 <= inp["squeeze"] <= 1
                 and 0 <= inp["open_effort"] <= 1 and abs(inp["steer"]) <= 1
                 and isinstance(inp.get("up"), bool) and isinstance(inp.get("open"), bool))
        if not valid or inp.get("fresh") is not True:
            preserve_partial = bool(valid and inp.get("waiting") is True and inp.get("fresh") is False)
            raise ControlFault(inp.get("error", "Invalid/stale Myo input") if isinstance(inp, dict) else "Invalid input")
        if inp["open"] and not USE_BACK:
            raise ControlFault("Open-hand reverse is disabled")
        if not preparation_ready(inp, now):
            LAST_FAULT = ""
            log_hold(PREPARE_UI["reason"])
            return 0.0, 0.0
        v, w = _motion_step(state, inp, events)
        if not finite_number(v) or not finite_number(w) or not -V_BACK <= v <= V_MAX or abs(w) > max(W_LEFT, W_RIGHT):
            raise ControlFault("Invalid outgoing command")
        LAST_FAULT = ""
        return float(v), float(w)
    except (ValueError, TypeError, KeyError, OverflowError, FloatingPointError) as exc:
        reset_control(clear_buffers=not preserve_partial)
        LAST_FAULT = str(exc)
        PREPARE_UI.update(phase="stopped", remaining=3, reason=LAST_FAULT)
        with suppress(OSError):
            print(f"\rSTOP: {LAST_FAULT:80s}", end="", flush=True)
        # Same 24-column schema; unavailable measurements are blank, never fabricated.
        row = [""] * 24
        row[0], row[1], row[2] = f"{time.time():.3f}", 0, 0
        row[10], row[11], row[23] = 0.0, 0.0, LAST_FAULT
        write_log(row)
        return 0.0, 0.0


def log_hold(reason):
    row = [""] * 24
    row[0], row[1], row[2] = f"{time.time():.3f}", 1, 0
    row[10], row[11], row[23] = 0.0, 0.0, reason
    write_log(row)


def preparation_ready(inp, now):
    """Require three continuous seconds of fresh neutral input before motion."""
    global PREPARE_STARTED, PREPARE_DONE, READY_STARTED
    if PREPARE_DONE:
        return True
    neutral = (not inp["open"] and not inp["up"] and inp["steer"] == 0
               and inp["squeeze"] == 0 and abs(inp["tilt"]) <= DIR_EXIT_THRESHOLD)
    ARMED["on"] = False
    SMOOTH.update(v=0.0, w=0.0, squeeze=0.0, steer=0.0)
    if not neutral:
        PREPARE_STARTED = None
        READY_STARTED = None
        PREPARE_UI.update(phase="waiting", remaining=3, reason="Hold arm level and relax to prepare")
        return False
    if READY_STARTED is None:
        READY_STARTED = now
    if round(now - READY_STARTED, 9) < READY_SECONDS:
        PREPARE_UI.update(phase="waiting", remaining=3, reason="Ready - hold neutral")
        return False
    if PREPARE_STARTED is None:
        PREPARE_STARTED = now
    elapsed = round(now - PREPARE_STARTED, 9)
    if elapsed < PREPARE_SECONDS:
        remaining = max(1, math.ceil(PREPARE_SECONDS - elapsed))
        PREPARE_UI.update(phase="countdown", remaining=remaining, reason=f"Prepare: {remaining}")
        with suppress(OSError):
            print(f"\rPREPARE {remaining} - hold neutral; zero commands                ", end="", flush=True)
        return False
    PREPARE_DONE = True
    PREPARE_UI.update(phase="running", remaining=0, reason="GO - relaxed hand now requests forward motion")
    return True


async def preparation_client(socket):
    """Read-only loopback status feed; it never accepts robot commands."""
    try:
        while True:
            payload = dict(PREPARE_UI, type="preparation", version=VERSION)
            if LAST_FAULT == "Waiting for fresh Myo windows":
                payload.update(phase="waiting", remaining=3, reason="Preparing fresh input")
            elif payload["phase"] == "waiting" and LAST_FAULT:
                payload.update(phase="stopped", remaining=3, reason=LAST_FAULT)
            await socket.send(json.dumps(payload, allow_nan=False))
            await asyncio.sleep(1 / RATE_HZ)
    except websockets.ConnectionClosed:
        pass


async def run_with_preparation_ui(url):
    # Bind before starting control: a port conflict fails without driving the robot.
    async with websockets.serve(preparation_client, "127.0.0.1", PREPARE_PORT,
                                origins=["http://localhost:8765", "http://127.0.0.1:8765"],
                                max_size=1024):
        await run(url)


def main() -> None:
    global EMG, IMU, CAL, LOG
    validate_configuration()
    print(f"Controller {VERSION}: 3.2 motion law with experimental safety checks.")
    os.system("")  # turns on the colors in the Windows console (no effect on Mac / Linux)
    EMG, IMU = open_stream(EMG_STREAM), open_stream(IMU_STREAM)
    print(f"✓ {EMG_STREAM} and {IMU_STREAM} found on this laptop.")
    CAL = calibrate()
    EMG.pull_chunk(timeout=0.0)
    IMU.pull_chunk(timeout=0.0)
    reset_control()

    out = Path(__file__).resolve().parents[3] / "data" / "driving-sessions" / datetime.now().strftime("%Y%m%d_%H%M%S_%f_3_2a")
    out.mkdir(parents=True)
    (out / "configuration.json").write_text(json.dumps({
        "version": VERSION, "base_sha256": BASE_SHA256, "safety_donor_sha256": SAFETY_DONOR_SHA256,
        "assist_dist": ASSIST_DIST, "assist_speed": ASSIST_SPEED,
        "assist_release_accel": ASSIST_RELEASE_ACCEL, "rate_hz": RATE_HZ,
        "signal_timeout": SIGNAL_TIMEOUT, "state_timeout": STATE_TIMEOUT,
        "loop_timeout": LOOP_TIMEOUT, "window_max_age": WINDOW_MAX_AGE,
        "prepare_seconds": PREPARE_SECONDS,
    }, indent=2, allow_nan=False) + "\n")
    (out / "calibration.json").write_text(json.dumps(CAL, indent=2) + "\n")
    if CAL["problems"]:
        raise SystemExit(f"\nCalibration failed (numbers saved in {out / 'calibration.json'}):\n- "
                         + "\n- ".join(CAL["problems"]) + "\nRun again.")
    for k in ["up0", "axis", "tilt_axis", "rms_relax"] + (["shape_open", "shape_others"] if USE_BACK else []):
        CAL[k] = np.array(CAL[k])
    logfile = open(out / "drive_log.csv", "w", newline="", buffering=1)
    LOG = csv.writer(logfile)
    LOG.writerow(["time_s", "fresh", "armed", "emg_level", "squeeze", "open", "open_effort", "steer", "tilt",
                  "hand_up", "v_cmd", "w_cmd", "path_clear_m", "distance_v_limit", "assist", "status", "x", "y", "yaw", "course",
                  "gates_passed", "collisions", "events", "fault_reason"])
    print(f"Saving to {out}\nDefault = slow forward, fist = faster, twist = turn, hand up = back"
          f"{', open hand = back' if USE_BACK else ''}. Ctrl+C stops.\n")
    try:
        asyncio.run(run_with_preparation_ui(BRIDGE))
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        logfile.close()
        LOG = None
        reset_control()


if __name__ == "__main__":
    main()
