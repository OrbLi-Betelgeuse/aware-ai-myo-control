#!/usr/bin/env python3
"""My team's controller: the forearm points the way, the squeeze is the speed, the lidar keeps you off walls.

How the driver controls the robot (elbow resting, forearm held in the air)
    hand up                  → forward
    hand down                → back (slower: the lidar cannot see behind)
    twist the forearm        → turn left / right; a bigger twist turns more. With the forearm level,
                               the robot turns on the spot
    forearm level, no twist  → stop (no clear direction = do nothing)
    squeeze the fist         → faster. With a relaxed hand the robot still moves slowly the way the arm
                               points (BASE_SPEED); a medium fist = full speed
    lidar assist             → slows down and stops by itself before hitting something in its path.
                               It never steers: the driver stays in charge
    Myo signal lost (0.25 s) → stop. At the start and after a signal loss, the robot moves only after
                               it has seen the forearm level once (so it never drives off by surprise)

Why this design: our gesture classifier failed live (every gesture came out "forward" once the band
sat about two electrodes rotated). The mean EMG level over all 8 electrodes does not change when the
band is rotated, and the arm directions are learned from the driver's own poses. So a new driver only
needs the ~30 s calibration at the start.

Run, from the repo folder (jackal-bridge running, the sim open, myo_lsl_publisher.py streaming):
    python examples/my_team.py
Saves, in a new folder per run:
    results/my_team/<date_time>/calibration.json   the driver's calibration numbers
    results/my_team/<date_time>/drive_log.csv      one row per 50 ms: signals, command, lidar, course
"""
from __future__ import annotations

import asyncio
import csv
import json
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
V_BACK = 0.3                  # m/s backward at full speed (the lidar cannot see behind: keep it slow)
W_MAX = 1.5                   # rad/s at a full twist and full speed (protocol limit 4.0, L3 caps it at 2.0)
BASE_SPEED = 0.3              # fraction of full speed with a relaxed hand (the arm must point a clear way)
DIR_THRESHOLD = 0.5           # tilt the hand more than half of your calibrated up / down to choose forward / back
SPEED_DEADZONE = 0.15         # the first 15 % of the squeeze range does nothing: a loose hand does not move
STEER_DEADZONE = 0.15         # the first 15 % of the twist range does nothing: a resting arm does not turn
EMG_WINDOW = 40               # samples (200 ms at 200 Hz) for the EMG level
IMU_WINDOW = 10               # samples (200 ms at 50 Hz) averaged for the forearm direction
SIGNAL_TIMEOUT = 0.25         # s without new EMG or IMU samples → stop
CALIB_SECONDS = 3.0           # s each calibration pose is held (after a 2 s countdown)
PATH_HALF_WIDTH = 0.30        # m, lidar points closer than this to the robot's centre line are "in the path"
                              # (the Jackal is 0.43 m wide, so this leaves about 8 cm each side)
STOP_DIST = 0.30              # m, the assist stops when the nearest point in the path is this far from the
                              # lidar (the front bumper is then about 0.16 m from it)
ASSIST_GAIN = 0.8             # 1/s, the assist allows at most ASSIST_GAIN × (distance − STOP_DIST) m/s forward,
                              # so 2.0 m/s from 2.8 m away. Gentle enough for the 300 ms degraded link: with
                              # gain × delay below 0.37 a delayed brake slows smoothly and does not overshoot

EMG: pylsl.StreamInlet = None     # set in main()
IMU: pylsl.StreamInlet = None
CAL: dict = {}                    # calibration, set in main()
LOG = None                        # csv writer for drive_log.csv, set in main()
EMG_BUF = deque(maxlen=EMG_WINDOW)
IMU_BUF = deque(maxlen=IMU_WINDOW)
LAST_SEEN = {"emg": float("-inf"), "imu": float("-inf")}   # time.monotonic() of the newest samples
ARMED = {"on": False}             # True once the forearm was seen level; False again after a signal loss


def emg_level(emg) -> float:
    """Muscle effort: RMS of each electrode over the window (offset removed), averaged over all 8.
    The average does not change when the band is rotated on the arm."""
    x = np.asarray(emg, dtype=float)
    x = x - x.mean(axis=0)
    return float(np.sqrt(np.mean(x * x, axis=0)).mean())


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


def calibrate() -> dict:
    """About 30 s: six poses, each held 3 s after a 2 s countdown. The result maps this driver's own
    squeeze onto speed 0..1, their own twist onto steering -1..1, and their own hand up / down onto
    forward / back."""
    poses = [("relax", "RELAX: elbow resting, forearm in the air and level (driving position), hand loose."),
             ("left", "STEER LEFT: twist the forearm as far LEFT as you want for a full turn. Hand loose."),
             ("right", "STEER RIGHT: twist the forearm as far RIGHT as you want for a full turn. Hand loose."),
             ("up", "FORWARD: hand loose, lift your HAND UP (tilt the forearm up)."),
             ("down", "BACK: hand loose, drop your HAND DOWN (tilt the forearm down)."),
             ("squeeze", "SPEED: forearm level, make a MEDIUM fist that is easy to hold (= full speed).")]
    levels, ups = {}, {}
    for name, text in poses:
        print(f"\n{text}")
        for k in (2, 1):
            print(f"  {k} …", flush=True)
            time.sleep(1.0)
        print("  hold it …", flush=True)
        EMG.pull_chunk(timeout=0.0)  # throw away the samples from moving into the pose
        IMU.pull_chunk(timeout=0.0)
        emg, imu = deque(maxlen=EMG_WINDOW), deque(maxlen=IMU_WINDOW)
        levels[name], ups[name] = [], []
        end = time.monotonic() + CALIB_SECONDS
        while time.monotonic() < end:
            emg.extend(EMG.pull_chunk(timeout=0.0)[0])
            imu.extend(IMU.pull_chunk(timeout=0.0)[0])
            if len(emg) == EMG_WINDOW and len(imu) == IMU_WINDOW:
                levels[name].append(emg_level(emg))
                ups[name].append(gravity(imu))
            time.sleep(0.05)
        if len(levels[name]) < 20:
            raise SystemExit("Not enough Myo samples during calibration. Check the publisher and run again.")
        print(f"  EMG level {np.median(levels[name]):.1f}")

    # Speed: zero at the strongest loose-hand level with the forearm level (twisting included), full at
    # the medium fist. The up / down poses are left out: holding the arm there raises the EMG (to 11
    # against 4.8 relaxed in our 21:23 run), which made the squeeze needed much harder.
    zero = float(np.percentile(levels["relax"] + levels["left"] + levels["right"], 90))
    full = float(np.median(levels["squeeze"]))
    up0, up_left, up_right, up_up, up_down = (np.mean(ups[p], axis=0)
                                              for p in ("relax", "left", "right", "up", "down"))
    # Steering: how far 'up' moves along the left-right direction, from the driving position.
    axis = (up_left - up_right) / np.linalg.norm(up_left - up_right)
    reach_left = float(np.dot(up_left - up0, axis))       # > 0
    reach_right = float(np.dot(up_right - up0, axis))     # < 0
    # Forward / back: the up-down direction without its twist part, so twisting never picks a direction.
    tilt = up_up - up_down
    tilt = tilt - np.dot(tilt, axis) * axis
    tilt_axis = tilt / np.linalg.norm(tilt)
    reach_up = float(np.dot(up_up - up0, tilt_axis))      # > 0
    reach_down = float(np.dot(up_down - up0, tilt_axis))  # < 0
    angles = {k: float(np.degrees(2 * np.arcsin(abs(r) / 2))) for k, r in
              [("left", reach_left), ("right", reach_right), ("up", reach_up), ("down", reach_down)]}
    print(f"\nSpeed: zero at EMG {zero:.1f}, full at {full:.1f}. Angles (approx.): "
          + ", ".join(f"{k} {a:.0f}°" for k, a in angles.items()))
    if full < 1.5 * zero or min(reach_left, -reach_right, reach_up, -reach_down) < 0.15:
        raise SystemExit("Calibration failed: make a clearer fist, and move clearly (at least ~10°) in each "
                         "direction: twist left and right, hand up and down. Run again.")
    return {"zero": zero, "full": full, "up0": up0.tolist(), "axis": axis.tolist(),
            "reach_left": reach_left, "reach_right": reach_right,
            "tilt_axis": tilt_axis.tolist(), "reach_up": reach_up, "reach_down": reach_down,
            "angles_deg": angles, "median_emg_level": {p: float(np.median(levels[p])) for p in levels}}


def my_input() -> dict:
    """Read the newest Myo samples: squeeze 0..1, steering -1..1 (twist, + = left) and
    direction +1 / 0 / -1 (hand up / level / down)."""
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
        EMG_BUF.clear()  # after a dropout, wait for a full window of new samples
        IMU_BUF.clear()
    if len(EMG_BUF) < EMG_WINDOW or len(IMU_BUF) < IMU_WINDOW:
        return {"fresh": False, "level": 0.0, "squeeze": 0.0, "steer": 0.0, "tilt": 0.0, "direction": 0}

    level = emg_level(EMG_BUF)
    squeeze = (level - CAL["zero"]) / (CAL["full"] - CAL["zero"])
    up = gravity(IMU_BUF) - CAL["up0"]
    p = float(np.dot(up, CAL["axis"]))        # twist along the left-right direction
    q = float(np.dot(up, CAL["tilt_axis"]))   # tilt along the up-down direction
    steer = p / CAL["reach_left"] if p > 0 else -p / CAL["reach_right"]
    tilt = q / CAL["reach_up"] if q > 0 else -q / CAL["reach_down"]   # +1 = calibrated up, -1 = calibrated down
    direction = 1 if tilt > DIR_THRESHOLD else -1 if tilt < -DIR_THRESHOLD else 0
    # Dead zones, then scale so that the medium fist / the full calibrated twist gives 1.
    squeeze = float(np.clip((squeeze - SPEED_DEADZONE) / (1 - SPEED_DEADZONE), 0, 1))
    steer = float(np.sign(steer) * np.clip((abs(steer) - STEER_DEADZONE) / (1 - STEER_DEADZONE), 0, 1))
    return {"fresh": True, "level": level, "squeeze": squeeze, "steer": steer, "tilt": tilt, "direction": direction}


def step(state: dict, inp: dict, events: list[str]) -> tuple[float, float]:
    """The arm chooses the way (up = forward, down = back, twist = turn; level = stop); the squeeze makes
    every movement faster. The lidar assist can only lower the forward speed, never steer."""
    speed = BASE_SPEED + (1 - BASE_SPEED) * inp["squeeze"]   # relaxed hand = slow, medium fist = full
    v = speed * {1: V_MAX, 0: 0.0, -1: -V_BACK}[inp["direction"]]
    w = speed * W_MAX * inp["steer"]
    ahead = path_clearance(state)
    v_allowed = ASSIST_GAIN * max(0.0, ahead - STOP_DIST)
    assist = v > v_allowed   # never true backing up (v < 0): the lidar cannot see behind
    v = min(v, v_allowed)
    if not inp["fresh"]:
        ARMED["on"] = False   # after a signal loss, the driver must relax once before driving again
    elif inp["direction"] == 0 and inp["steer"] == 0:
        ARMED["on"] = True    # forearm level and not twisted: the robot would stand still
    if not ARMED["on"]:
        v = w = 0.0

    # Feedback for the driver: one status line
    g, s = round(10 * inp["squeeze"]), round(5 * abs(inp["steer"]))
    squeeze_bar = "#" * g + "." * (10 - g)
    steer_bar = "." * (5 - s) + "<" * s + "|....." if inp["steer"] > 0 else ".....|" + ">" * s + "." * (5 - s)
    way = {1: "FORWARD", 0: "LEVEL  ", -1: "BACK   "}[inp["direction"]]
    course = state.get("course") or {}
    note = ("NO MYO SIGNAL: STOPPED" if not inp["fresh"] else "HOLD THE ARM LEVEL TO START" if not ARMED["on"]
            else "ASSIST: obstacle ahead" if assist else "BACKING UP: no lidar behind" if v < 0 else "")
    print(f"\r{way}  SQUEEZE [{squeeze_bar}]  STEER [{steer_bar}]  v {v:+.2f}  w {w:+.2f}  path {ahead:4.1f} m  "
          f"gates {course.get('gates_passed', '-')}  hits {state.get('collisions', '-')}  {note:24s}",
          end="", flush=True)

    pose = state.get("pose") or {}
    LOG.writerow([f"{time.time():.3f}", int(inp["fresh"]), int(ARMED["on"]), f"{inp['level']:.2f}",
                  f"{inp['squeeze']:.3f}", f"{inp['steer']:.3f}", f"{inp['tilt']:.3f}", inp["direction"],
                  f"{v:.3f}", f"{w:.3f}", f"{ahead:.3f}", int(assist),
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

                async def listen() -> None:
                    async for raw in st:
                        m = json.loads(raw)
                        if m.get("type") == "state":
                            state.clear()
                            state.update(m)
                        elif m.get("type") == "event":
                            events.append(m["name"])

                listener = asyncio.create_task(listen())
                seq = 0
                try:
                    while t_end is None or time.monotonic() < t_end:
                        t0 = time.monotonic()
                        if listener.done():
                            listener.result()  # re-raise a dropped connection
                        happened, events[:] = list(events), []
                        v, w = step(state, my_input(), happened)
                        await cmd.send(json.dumps({"type": "cmd", "v": float(v), "w": float(w),
                                                   "source": SOURCE, "seq": seq, "t": time.time()}))
                        seq += 1
                        await asyncio.sleep(max(0.0, 1 / RATE_HZ - (time.monotonic() - t0)))
                finally:
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
    EMG, IMU = open_stream(EMG_STREAM), open_stream(IMU_STREAM)
    print(f"✓ {EMG_STREAM} and {IMU_STREAM} found on this laptop. Calibration takes about 30 s.")
    CAL = calibrate()

    out = Path(__file__).resolve().parents[1] / "results" / "my_team" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True)
    (out / "calibration.json").write_text(json.dumps(CAL, indent=2) + "\n")
    CAL["up0"], CAL["axis"], CAL["tilt_axis"] = (np.array(CAL[k]) for k in ("up0", "axis", "tilt_axis"))
    LOG = csv.writer(open(out / "drive_log.csv", "w", newline="", buffering=1))
    LOG.writerow(["time_s", "fresh", "armed", "emg_level", "squeeze", "steer", "tilt", "direction", "v_cmd", "w_cmd",
                  "path_clear_m", "assist", "status", "x", "y", "yaw", "course", "gates_passed", "collisions",
                  "events"])
    print(f"Saving to {out}\nHand up = forward, hand down = back, twist = turn, squeeze = faster. "
          "Forearm level = stop. Ctrl+C stops.\n")
    try:
        asyncio.run(run(BRIDGE))
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
