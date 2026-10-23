#!/usr/bin/env python3
"""LiDAR Safety Enhanced / LiDAR 安全增强版: standalone EC alternative to 2.6.

No hardware is opened on import.

This merged 2.7 variant adds a 0.5 m/s forward cap during turns.

The original 2.6 controls are preserved: relaxed = 0.4 m/s forward after arming,
fist = faster, twist = steering, hand up = straight reverse at 0.6 m/s. Relaxing
does NOT stop an armed robot. Use the simulator's E-stop or Ctrl+C to stop.
Each direction change includes a zero-command tick, not a measured standstill.

EC rejects malformed/stale sensor, simulator and lidar data, resets control on
faults, and requires fresh relaxed neutral input before rearming. Five-pose
calibration remains 3 s preparation + 3 s acquisition, with Enter-to-retry.
The EMG mean is invariant to channel permutation, not arbitrary band movement.
Lidar only limits forward speed and does not guarantee collision avoidance.

Run after the local Myo publisher and jackal-bridge are ready:
    python my_team2_6_lidar_safety_enhanced.py
Logs are saved beside this script under results/my_team2_6_lidar_safety_enhanced/<timestamp>/.
Calibration attempts include diagnostics; drive logs contain fault reasons.
"""
from __future__ import annotations

import asyncio
import csv
import json
import math
import socket
import time
from collections import deque
from contextlib import suppress
from datetime import datetime
from pathlib import Path

import numpy as np
import pylsl
import websockets

VERSION = "2.7-EC"
ORIGINAL_SHA256 = "a7ab6b4505b0eafc6ee8e3c61e183eb5d7e9fd67e8346150083ce60d340048f3"
SOURCE, RATE_HZ, BRIDGE = "my-team", 20, "ws://localhost:8765"
EMG_STREAM, IMU_STREAM = "Myo_EMG", "Myo_IMU"
V_MAX, V_MIN, FORWARD_GAIN, FULL_SPEED_EFFORT = 2.0, 0.4, 1.8, 0.70
TURN_V_MAX = 0.5  # Maximum forward speed while steering; reverse remains unchanged.
V_BACK, W_LEFT, W_RIGHT, BASE_SPEED = 0.6, 1.3, 1.8, 0.3
USE_BACK = False
DIR_THRESHOLD, DIR_EXIT_THRESHOLD = 0.55, 0.40
SPEED_DEADZONE, STEER_DEADZONE = 0.15, 0.25
OPEN_TICKS, MIN_EFFORT_RATIO, MAX_FALSE_BACK = 5, 1.3, 0.10
EMG_WINDOW, IMU_WINDOW = 40, 10
SIGNAL_TIMEOUT, STATE_TIMEOUT, LOOP_TIMEOUT = 0.25, 0.50, 0.25
WINDOW_MAX_AGE, FUTURE_TOLERANCE = 0.30, 0.05
READ_SECONDS, CALIB_SECONDS = 3, 3.0
PATH_HALF_WIDTH, STOP_DIST, ASSIST_GAIN = 0.30, 0.30, 0.8
LIDAR_BEAMS = 180
SQUEEZE_TAU, STEER_TAU = 0.30, 0.15
ACCEL, DECEL, W_ACCEL, W_DECEL = 1.0, 2.0, 2.5, 4.0
EPS = 1e-9

POSES = [
    ("relax", "RELAX = SLOW FORWARD", "Elbow supported; forearm level in the air; hand loose."),
    ("left", "TWIST LEFT = TURN LEFT", "Twist comfortably left, keeping the hand loose."),
    ("right", "TWIST RIGHT = TURN RIGHT", "Twist comfortably right, keeping the hand loose."),
    ("up", "HAND UP = BACK", "Lift the hand/forearm, keeping the hand loose."),
    ("open", "OPEN HAND = BACK", "Forearm level; spread the fingers firmly."),
    ("squeeze", "FIST = FASTER", "Forearm level; make a steady comfortable firm fist."),
]
EMG = IMU = LOG = None
CAL: dict = {}
EMG_BUF, IMU_BUF = deque(maxlen=EMG_WINDOW), deque(maxlen=IMU_WINDOW)
EMG_TIMES, IMU_TIMES = deque(maxlen=EMG_WINDOW), deque(maxlen=IMU_WINDOW)
LAST_SEEN = {"emg": float("-inf"), "imu": float("-inf")}
LAST_SOURCE = {"emg": float("-inf"), "imu": float("-inf")}
ARMED, OPEN = {"on": False}, {"ticks": 0}
SMOOTH = {"squeeze": 0.0, "steer": 0.0, "v": 0.0, "w": 0.0}
BACK_ACTIVE = BACK_REQUESTED = False
LAST_STEP = None
CALIB_ATTEMPT: dict = {}
ATTEMPT_PATH = None


class ControlFault(ValueError):
    """A command cannot safely be calculated from the current data."""


class CalibrationAttemptError(ValueError):
    """Expected acquisition/quality failure: repeat the calibration."""


def finite_number(value) -> bool:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        return False
    try:
        return math.isfinite(float(value))
    except (ValueError, TypeError, OverflowError):
        return False


def validate_configuration() -> None:
    values = (V_MIN, V_MAX, FORWARD_GAIN, FULL_SPEED_EFFORT, V_BACK, W_LEFT, W_RIGHT,
              BASE_SPEED, SPEED_DEADZONE, STEER_DEADZONE, DIR_THRESHOLD, DIR_EXIT_THRESHOLD, TURN_V_MAX,
              SQUEEZE_TAU, STEER_TAU, ACCEL, DECEL, W_ACCEL, W_DECEL)
    if not all(finite_number(v) for v in values):
        raise ControlFault("Configuration contains nonfinite values")
    if not (0 <= V_MIN <= V_MAX <= 2 and FORWARD_GAIN > 0 and 0 <= V_BACK <= 2
            and 0 <= W_LEFT <= 4 and 0 <= W_RIGHT <= 4 and 0 <= BASE_SPEED <= 1
            and 0 <= TURN_V_MAX <= V_MAX
            and 0 <= SPEED_DEADZONE < FULL_SPEED_EFFORT <= 1 and 0 <= STEER_DEADZONE < 1
            and 0 <= DIR_EXIT_THRESHOLD < DIR_THRESHOLD <= 1
            and min(SQUEEZE_TAU, STEER_TAU) >= 0
            and min(ACCEL, DECEL, W_ACCEL, W_DECEL) > 0 and RATE_HZ > 0):
        raise ControlFault("Invalid speed, dead-zone, direction, or filter configuration")


def reset_control(clear_buffers: bool = True) -> None:
    global BACK_ACTIVE, BACK_REQUESTED, LAST_STEP
    ARMED["on"], OPEN["ticks"] = False, 0
    BACK_ACTIVE = BACK_REQUESTED = False
    LAST_STEP = None
    SMOOTH.update(squeeze=0.0, steer=0.0, v=0.0, w=0.0)
    if clear_buffers:
        for buffer in (EMG_BUF, IMU_BUF, EMG_TIMES, IMU_TIMES):
            buffer.clear()
        LAST_SEEN.update(emg=float("-inf"), imu=float("-inf"))
        LAST_SOURCE.update(emg=float("-inf"), imu=float("-inf"))


def sample_array(samples, kind: str) -> np.ndarray:
    try:
        x = np.asarray(samples, dtype=float)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ControlFault(f"Malformed {kind.upper()} samples") from exc
    expected = 8 if kind == "emg" else 10
    if x.ndim != 2 or not len(x) or x.shape[1] != expected or not np.isfinite(x).all():
        raise ControlFault(f"{kind.upper()} needs finite rows of {expected} channels")
    return x


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


def open_stream(name: str):
    found = pylsl.resolve_bypred(f"name='{name}' and hostname='{socket.gethostname()}'", 2, 2.0)
    if len(found) != 1:
        raise SystemExit(f"Expected one local {name} stream, found {len(found)}. Check the publisher.")
    inlet = pylsl.StreamInlet(found[0], max_buflen=1)
    inlet.open_stream(timeout=5)
    return inlet


def forward_target(squeeze: float) -> float:
    """Exact 2.6 target from filtered POST-dead-zone effort in [0,1].

    FULL_SPEED_EFFORT=.70 is a rescaling reference, not the saturation point:
    with gain=1.8 and baseline=.4, steady raw effort reaches the cap at ~.6389.
    """
    validate_configuration()
    if not finite_number(squeeze) or not 0 <= squeeze <= 1:
        raise ControlFault("Invalid forward effort")
    reference = (FULL_SPEED_EFFORT - SPEED_DEADZONE) / (1 - SPEED_DEADZONE)
    effort = float(np.clip(squeeze / reference, 0, 1))
    return min(V_MAX, V_MIN + FORWARD_GAIN * effort)


def wants_back(tilt: float) -> bool:
    global BACK_ACTIVE
    if not finite_number(tilt):
        raise ControlFault("Invalid forearm lift")
    if tilt > DIR_THRESHOLD:
        BACK_ACTIVE = True
    elif tilt < DIR_EXIT_THRESHOLD:
        BACK_ACTIVE = False
    return BACK_ACTIVE


def low_pass(name: str, value: float, tau: float) -> float:
    if not all(finite_number(x) for x in (value, tau, SMOOTH[name])) or tau < 0:
        raise ControlFault("Invalid filter state")
    dt = 1 / RATE_HZ
    SMOOTH[name] += dt / (tau + dt) * (value - SMOOTH[name])
    return SMOOTH[name]


def slew(current: float, target: float, rise: float, fall: float) -> float:
    if not all(finite_number(x) for x in (current, target, rise, fall)) or min(rise, fall) <= 0:
        raise ControlFault("Invalid slew state")
    rate = (rise if abs(target) > abs(current) and target * current >= 0 else fall) / RATE_HZ
    return current + float(np.clip(target - current, -rate, rate))


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


def valid_state(state: dict, now: float | None = None) -> bool:
    try:
        state_clearance(state, time.monotonic() if now is None else now)
        return True
    except (ValueError, TypeError, OverflowError):
        return False


def _feedback(state, inp, events, v, w, ahead, reason):
    global LOG
    try:
        print(f"\rv {v:+.2f} w {w:+.2f} clearance {ahead:.2f} m  {reason:65s}", end="", flush=True)
    except OSError:
        pass
    if LOG is None:
        return
    state = state if isinstance(state, dict) else {}
    pose = state.get("pose") if isinstance(state.get("pose"), dict) else {}
    course = state.get("course") if isinstance(state.get("course"), dict) else {}
    safe = lambda key: float(inp[key]) if isinstance(inp, dict) and finite_number(inp.get(key)) else 0.0
    try:
        LOG.writerow([f"{time.time():.3f}", VERSION, int(bool(inp.get("fresh"))), int(ARMED["on"]),
                      safe("level"), safe("squeeze"), safe("steer"), safe("tilt"),
                      v, w, ahead, reason, state.get("status", ""), pose.get("x", ""), pose.get("y", ""),
                      pose.get("yaw", ""), course.get("id", ""), course.get("gates_passed", ""),
                      state.get("collisions", ""), " ".join(map(str, events))])
    except (OSError, ValueError, TypeError, csv.Error) as exc:
        LOG = None
        with suppress(OSError):
            print(f"\nWarning: drive logging disabled: {exc}")


def step(state: dict, inp: dict, events: list[str]) -> tuple[float, float]:
    global BACK_REQUESTED, LAST_STEP
    now, v, w, ahead, reason = time.monotonic(), 0.0, 0.0, 0.0, ""
    waiting = isinstance(inp, dict) and inp.get("waiting") is True
    preserve_partial = False
    try:
        if LAST_STEP is not None and not 0 <= now - LAST_STEP <= LOOP_TIMEOUT:
            raise ControlFault("Control loop stalled; return to neutral")
        LAST_STEP = now
        ahead = state_clearance(state, now)
        keys = ("level", "squeeze", "open_effort", "steer", "tilt")
        valid_fields = (isinstance(inp, dict) and all(finite_number(inp.get(k)) for k in keys)
                and 0 <= inp["squeeze"] <= 1 and 0 <= inp["open_effort"] <= 1
                and abs(inp["steer"]) <= 1 and inp["level"] >= 0
                and isinstance(inp.get("up"), bool) and isinstance(inp.get("open"), bool))
        if (not valid_fields or inp.get("fresh") is not True):
            preserve_partial = bool(valid_fields and waiting and inp.get("fresh") is False)
            raise ControlFault(inp.get("error", "Invalid Myo input") if isinstance(inp, dict) else "Invalid Myo input")
        if inp["open"] and not USE_BACK:
            raise ControlFault("Open-hand reverse is disabled")
        neutral = (not inp["open"] and not inp["up"] and inp["steer"] == 0
                   and inp["squeeze"] == 0 and abs(inp["tilt"]) <= DIR_EXIT_THRESHOLD)
        if neutral:
            ARMED["on"] = True
        if not ARMED["on"]:
            reason = "RELAX + NEUTRAL TO ARM"
            SMOOTH.update(squeeze=0.0, steer=0.0, v=0.0, w=0.0)
        else:
            if not inp["open"]:
                OPEN["ticks"] = 0
            elif inp.get("new_emg", True):
                OPEN["ticks"] += 1
            squeeze = low_pass("squeeze", inp["squeeze"], SQUEEZE_TAU)
            steer = low_pass("steer", inp["steer"], STEER_TAU)
            reverse = inp["up"] or inp["open"]
            if reverse:
                v = -V_BACK if inp["up"] or OPEN["ticks"] >= OPEN_TICKS else 0.0
                w = 0.0
            else:
                v = forward_target(squeeze)
                w = (BASE_SPEED + (1 - BASE_SPEED) * squeeze) * (W_LEFT if steer > 0 else W_RIGHT) * steer
            if reverse != BACK_REQUESTED:
                v = w = 0.0
                SMOOTH.update(v=0.0, w=0.0, steer=0.0, squeeze=0.0)
            else:
                v = slew(SMOOTH["v"], v, ACCEL, DECEL)
                w = 0.0 if reverse else slew(SMOOTH["w"], w, W_ACCEL, W_DECEL)
            BACK_REQUESTED = reverse
            allowed = ASSIST_GAIN * max(0.0, ahead - STOP_DIST)
            if v > allowed:
                reason = "LIDAR LIMIT"
            v = min(v, allowed)
            turning = not reverse and abs(inp["steer"]) > 0.0
            if turning and v > TURN_V_MAX:
                v = TURN_V_MAX
                if not reason:
                    reason = f"TURN SPEED LIMIT {TURN_V_MAX:.1f}"
            if not finite_number(v) or not finite_number(w) or not -V_BACK <= v <= V_MAX or abs(w) > max(W_LEFT, W_RIGHT):
                raise ControlFault("Invalid outgoing command")
            SMOOTH["v"], SMOOTH["w"] = v, w
    except (ValueError, TypeError, KeyError, OverflowError, FloatingPointError) as exc:
        reset_control(clear_buffers=not preserve_partial)
        v = w = 0.0
        reason = f"STOP: {exc}"
    _feedback(state, inp if isinstance(inp, dict) else {}, events, v, w, ahead, reason)
    return float(v), float(w)


def save_attempt(report: dict) -> None:
    if ATTEMPT_PATH is None:
        return
    try:
        Path(ATTEMPT_PATH).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    except (OSError, ValueError, TypeError) as exc:
        with suppress(OSError):
            print(f"Warning: cannot save calibration diagnostics: {exc}")


def _summary(levels: dict) -> dict:
    result = {}
    for name, values in levels.items():
        a = np.asarray(values, dtype=float)
        if a.ndim != 1 or not len(a) or not np.isfinite(a).all() or np.any(a < 0):
            raise CalibrationAttemptError(f"Invalid {name} effort observations")
        result[name] = {"observations": len(a), "median": float(np.median(a)),
                        "p10": float(np.percentile(a, 10)), "p90": float(np.percentile(a, 90))}
    return result


def _collect_pose(name: str):
    try:
        try:
            EMG.pull_chunk(timeout=0.0)
            IMU.pull_chunk(timeout=0.0)
        except Exception as exc:
            # LSL LostError inherits RuntimeError; a lost publisher is retryable.
            raise CalibrationAttemptError(f"Cannot flush Myo streams: {exc}") from exc
        LAST_SOURCE.update(emg=float("-inf"), imu=float("-inf"))
        buffers = {"emg": deque(maxlen=EMG_WINDOW), "imu": deque(maxlen=IMU_WINDOW)}
        times = {"emg": deque(maxlen=EMG_WINDOW), "imu": deque(maxlen=IMU_WINDOW)}
        totals, used = {"emg": 0, "imu": 0}, {"emg": 0, "imu": 0}
        start = time.monotonic()
        last = {"emg": start, "imu": start}
        first = {"emg": None, "imu": None}
        levels, rmss, ups = [], [], []
        while time.monotonic() - start < CALIB_SECONDS:
            now = time.monotonic()
            for kind, inlet in (("emg", EMG), ("imu", IMU)):
                packet = read_samples(inlet, kind)
                if packet is not None:
                    samples, stamps = packet
                    buffers[kind].extend(samples)
                    times[kind].extend(stamps)
                    totals[kind] += len(samples)
                    last[kind] = now
                    first[kind] = now if first[kind] is None else first[kind]
                if now - last[kind] > SIGNAL_TIMEOUT:
                    raise CalibrationAttemptError(f"{name}: {kind.upper()} stopped advancing")
                if times[kind] and pylsl.local_clock() - min(times[kind]) > WINDOW_MAX_AGE:
                    raise CalibrationAttemptError(f"{name}: {kind.upper()} window too old")
            if (len(buffers["emg"]) == EMG_WINDOW and len(buffers["imu"]) == IMU_WINDOW
                    and all(totals[k] > used[k] for k in totals)):
                rms = channel_rms(buffers["emg"])
                rmss.append(rms)
                levels.append(float(rms.mean()))
                ups.append(gravity(buffers["imu"]))
                used = totals.copy()
            time.sleep(1 / RATE_HZ)
        for kind, nominal in (("emg", 200), ("imu", 50)):
            if (time.monotonic() - last[kind] > SIGNAL_TIMEOUT
                    or totals[kind] < nominal * CALIB_SECONDS * 0.7):
                raise CalibrationAttemptError(f"{name}: insufficient fresh {kind.upper()} samples: {totals[kind]}")
        if len(levels) < 20:
            raise CalibrationAttemptError(f"{name}: fewer than 20 fresh feature observations")
        stats = {"samples": totals, "observations": len(levels),
                 "coverage_seconds": {k: last[k] - first[k] for k in totals}}
        return levels, rmss, ups, stats
    except (ControlFault, ValueError, TypeError, OverflowError, FloatingPointError, OSError) as exc:
        raise CalibrationAttemptError(f"Pose {name}: {exc}") from exc


def build_calibration(levels: dict, rmss: dict, ups: dict) -> dict:
    names = [p[0] for p in POSES if USE_BACK or p[0] != "open"]
    for name in names:
        if name not in levels or name not in rmss or name not in ups:
            raise CalibrationAttemptError(f"Missing pose {name}")
        a, r, u = np.asarray(levels[name], float), np.asarray(rmss[name], float), np.asarray(ups[name], float)
        if (a.ndim != 1 or len(a) < 20 or r.shape != (len(a), 8) or u.shape != (len(a), 3)
                or not np.isfinite(a).all() or not np.isfinite(r).all() or not np.isfinite(u).all()
                or np.any(a < 0) or np.any(r < 0)):
            raise CalibrationAttemptError(f"Invalid observations for {name}")
    summary = _summary(levels)
    zero, full = summary["relax"]["median"], summary["squeeze"]["median"]
    up0, left, right, lifted = (np.mean(ups[p], axis=0) for p in ("relax", "left", "right", "up"))
    unit(up0, "neutral reference")
    axis = unit(left - right, "left/right calibration axis")
    lift = lifted - up0
    lift -= np.dot(lift, axis) * axis
    reach_up = float(np.linalg.norm(lift))
    tilt_axis = unit(lift, "hand-up calibration axis")
    reach_left, reach_right = float(np.dot(left - up0, axis)), float(np.dot(right - up0, axis))
    back = {}
    rms_relax = np.mean(rmss["relax"], axis=0)
    if USE_BACK:
        full_open = summary["open"]["median"]
        shapes = {}
        for p in ("open", "squeeze", "left", "right", "up"):
            active = np.clip(np.mean(rmss[p], axis=0) - rms_relax, 0, None)
            norm = float(np.linalg.norm(active))
            if not finite_number(norm) or norm <= EPS:
                raise CalibrationAttemptError(f"No distinct EMG pattern for {p}")
            shapes[p] = active / norm
        others = np.array([shapes[p] for p in ("squeeze", "left", "right", "up")])
        said_open = {}
        for p in shapes:
            active = np.clip(np.asarray(rmss[p]) - rms_relax, 0, None)
            active /= np.maximum(np.linalg.norm(active, axis=1, keepdims=True), EPS)
            said_open[p] = ((np.asarray(levels[p]) > zero + SPEED_DEADZONE * (full_open - zero))
                            & (active @ shapes["open"] > (active @ others.T).max(axis=1)))
        false_back = {p: float(said_open[p].mean()) for p in ("squeeze", "left", "right", "up")}
        recognized, mistaken = float(said_open["open"].mean()), float(np.mean(np.concatenate([said_open[p] for p in false_back])))
        if full_open < MIN_EFFORT_RATIO * zero or recognized < 0.9 or mistaken > MAX_FALSE_BACK:
            raise CalibrationAttemptError("Open hand is not distinct enough; repeat or disable USE_BACK")
        back = {"full_open": full_open, "shape_open": shapes["open"].tolist(), "shape_others": others.tolist(),
                "open_recognized": recognized, "false_back": false_back, "false_back_all": mistaken}
    reaches = {"left": reach_left, "right": reach_right, "up": reach_up}
    cal = {"version": VERSION, "zero": zero, "full": full, "up0": up0.tolist(), "axis": axis.tolist(),
           "reach_left": reach_left, "reach_right": reach_right, "reach_up": reach_up,
           "tilt_axis": tilt_axis.tolist(), "rms_relax": rms_relax.tolist(), **back,
           "angles_deg": {k: float(np.degrees(2 * np.arcsin(np.clip(abs(v) / 2, 0, 1)))) for k, v in reaches.items()},
           "median_emg_level": {p: value["median"] for p, value in summary.items()},
           "p90_emg_level": {p: value["p90"] for p, value in summary.items()}, "emg_summary": summary,
           "problems": []}
    validate_calibration(cal)
    return cal


def _calibrate_once(wait_for_start: bool = True) -> dict:
    names = [pose for pose in POSES if USE_BACK or pose[0] != "open"]
    print(f"\nCALIBRATION: {len(names)} poses, {READ_SECONDS}s preparation + {CALIB_SECONDS:g}s recording each.")
    print("Elbow supported; forearm level in the air. Enter each pose during its countdown.")
    for _, title, instruction in names:
        print(f"  {title}: {instruction}")
    if wait_for_start:
        input("Read the list, then press Enter to start ... ")
    levels, rmss, ups = {}, {}, {}
    for name, title, instruction in names:
        CALIB_ATTEMPT["active_pose"] = name
        print(f"\n{title}: {instruction}")
        for seconds in range(READ_SECONDS, 0, -1):
            print(f"  get ready ... {seconds}", flush=True)
            time.sleep(1)
        print("  HOLD IT, recording ...", flush=True)
        levels[name], rmss[name], ups[name], stats = _collect_pose(name)
        CALIB_ATTEMPT.setdefault("pose_quality", {})[name] = stats
        CALIB_ATTEMPT["emg_summary"] = _summary(levels)
        info = CALIB_ATTEMPT["emg_summary"][name]
        print(f"  EMG median {info['median']:.2f}; p10 {info['p10']:.2f}; p90 {info['p90']:.2f}")
        save_attempt(CALIB_ATTEMPT)
    CALIB_ATTEMPT["active_pose"] = "validation"
    return build_calibration(levels, rmss, ups)


def calibrate(report_dir: Path | None = None) -> dict:
    global CALIB_ATTEMPT, ATTEMPT_PATH
    attempt, wait_for_start = 0, True
    while True:
        attempt += 1
        reset_control()
        ATTEMPT_PATH = None if report_dir is None else Path(report_dir) / f"calibration_attempt_{attempt:02d}.json"
        CALIB_ATTEMPT = {"version": VERSION, "attempt": attempt, "started_at_s": time.time(),
                         "status": "collecting", "active_pose": None, "emg_summary": {}, "pose_quality": {}, "reason": ""}
        save_attempt(CALIB_ATTEMPT)
        try:
            result = _calibrate_once(wait_for_start=wait_for_start)
            if result.get("problems"):
                raise CalibrationAttemptError("; ".join(map(str, result["problems"])))
            validate_calibration(result)
        except KeyboardInterrupt:
            reset_control()
            CALIB_ATTEMPT.update(status="interrupted", reason="User interrupted calibration", finished_at_s=time.time())
            save_attempt(CALIB_ATTEMPT)
            raise
        except (ValueError, TypeError, KeyError, OverflowError, FloatingPointError, OSError) as exc:
            reset_control()
            CALIB_ATTEMPT.update(status="failed", reason=str(exc), finished_at_s=time.time())
            save_attempt(CALIB_ATTEMPT)
            print(f"\nCalibration was not accepted: {exc}")
            input("Press Enter to re-record all poses (Ctrl+C to quit) ... ")
            wait_for_start = False
            continue
        reset_control()
        CALIB_ATTEMPT.update(status="passed", active_pose=None, finished_at_s=time.time())
        save_attempt(CALIB_ATTEMPT)
        result["pose_quality"] = CALIB_ATTEMPT["pose_quality"]
        return result


async def listen_state(st, state: dict, events: list[str]) -> None:
    last_t = None
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


def main() -> None:
    global EMG, IMU, CAL, LOG
    validate_configuration()
    out = Path(__file__).resolve().parent / "results" / "my_team2_7_lidar_safety_enhanced" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out.mkdir(parents=True)
    print(f"Controller {VERSION}. Relaxed means 0.4m/s FORWARD, not stop. E-stop/Ctrl+C stops.")
    print(f"Logs: {out}")
    config = {"version": VERSION, "original_sha256": ORIGINAL_SHA256, "rate_hz": RATE_HZ,
              "v_min": V_MIN, "v_max": V_MAX, "forward_gain": FORWARD_GAIN,
              "full_speed_effort_reference": FULL_SPEED_EFFORT, "speed_deadzone": SPEED_DEADZONE,
              "v_back": V_BACK, "w_left": W_LEFT, "w_right": W_RIGHT, "use_back": USE_BACK,
              "window_max_age": WINDOW_MAX_AGE, "signal_timeout": SIGNAL_TIMEOUT, "state_timeout": STATE_TIMEOUT, "turn_v_max": TURN_V_MAX}
    (out / "configuration.json").write_text(json.dumps(config, indent=2, allow_nan=False) + "\n")
    EMG, IMU = open_stream(EMG_STREAM), open_stream(IMU_STREAM)
    CAL = calibrate(out)
    (out / "calibration.json").write_text(json.dumps(CAL, indent=2, allow_nan=False) + "\n")
    reset_control()
    try:
        with (out / "drive_log.csv").open("w", newline="", buffering=1) as logfile:
            LOG = csv.writer(logfile)
            LOG.writerow(["time_s", "version", "fresh", "armed", "emg_level", "squeeze", "steer", "tilt",
                          "v_cmd", "w_cmd", "clearance_m", "reason", "status", "x", "y", "yaw",
                          "course", "gates_passed", "collisions", "events"])
            asyncio.run(run(BRIDGE))
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        LOG = None
        reset_control()


if __name__ == "__main__":
    main()
