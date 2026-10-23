"""Offline EC regressions: synthetic samples and fake sockets, never hardware.

Run from the repository root: python -m pytest 2.0/tests/test_my_team2_6_lidar_safety_enhanced.py -q
"""
from __future__ import annotations

import asyncio
import csv
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


@pytest.fixture(scope="module")
def controller():
    """Stub LSL before import, so even accidental discovery cannot touch a device."""
    stub = ModuleType("pylsl")
    stub.StreamInlet = object
    stub.local_clock = lambda: 100.0

    def forbidden(*args, **kwargs):
        raise AssertionError("Offline tests must not discover or open real LSL streams")

    stub.resolve_bypred = forbidden
    path = Path(__file__).resolve().parents[1] / "my_team2_6_lidar_safety_enhanced.py"
    spec = importlib.util.spec_from_file_location("ec_controller_under_test", path)
    module = importlib.util.module_from_spec(spec)
    with pytest.MonkeyPatch.context() as patch:
        patch.setitem(sys.modules, "pylsl", stub)
        patch.setitem(sys.modules, spec.name, module)
        spec.loader.exec_module(module)
        yield module


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def time(self):
        return 1_700_000_000.0 + self.now

    def sleep(self, seconds):
        self.now += seconds


class Inlet:
    def __init__(self, clock, packets=(), hz=200):
        self.clock, self.packets, self.hz = clock, list(packets), hz

    def pull_chunk(self, timeout=0.0, **kwargs):
        if not self.packets:
            return [], []
        packet = self.packets.pop(0)
        if isinstance(packet, tuple):
            return packet
        return packet, [self.clock.now - (len(packet) - 1 - i) / self.hz
                        for i in range(len(packet))]


def emg(level=4.0, count=40):
    return [[level if i % 2 == 0 else -level] * 8 for i in range(count)]


def imu(angle=0.0, count=10, axis=0):
    q = [math.cos(angle / 2), 0.0, 0.0, 0.0]
    q[axis + 1] = math.sin(angle / 2)
    return [q + [0.0] * 6 for _ in range(count)]


def calibration():
    reach = math.sin(0.5)
    return {"zero": 4.0, "full": 32.0, "up0": np.array([0.0, 0.0, 1.0]),
            "axis": np.array([1.0, 0.0, 0.0]), "reach_left": reach,
            "reach_right": -reach, "tilt_axis": np.array([0.0, 1.0, 0.0]),
            "reach_up": reach, "rms_relax": np.full(8, 4.0), "problems": []}


def state(clock, **overrides):
    result = {"type": "state", "t": clock.now, "_received_at": clock.now,
              "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
              "vel": {"v": 0.0, "w": 0.0}, "cmd_applied": {"v": 0.0, "w": 0.0},
              "status": "ok", "collisions": 0, "course": None,
              "scan": {"angle_min": -2.356, "angle_inc": 0.02618,
                       "range_max": 10.0, "ranges": [10.0] * 180}}
    result.update(overrides)
    return result


def inp(*, squeeze=0.0, steer=0.0, up=False, fresh=True, hand_open=False):
    return {"fresh": fresh, "level": 4.0 + 28.0 * squeeze, "squeeze": squeeze,
            "open": hand_open, "open_effort": 0.0, "steer": steer,
            "tilt": 1.0 if up else 0.0, "up": up, "new_emg": fresh}


@pytest.fixture
def harness(controller, monkeypatch):
    clock, log = Clock(), io.StringIO()
    monkeypatch.setattr(controller, "time", clock)
    monkeypatch.setattr(controller.pylsl, "local_clock", clock.monotonic)
    monkeypatch.setattr(controller, "print", lambda *a, **k: None, raising=False)
    monkeypatch.setattr(controller, "LOG", csv.writer(log))
    monkeypatch.setattr(controller, "CAL", calibration())
    monkeypatch.setattr(controller, "EMG", Inlet(clock))
    monkeypatch.setattr(controller, "IMU", Inlet(clock, hz=50))
    controller.reset_control()
    return SimpleNamespace(c=controller, clock=clock, log=log)


def tick(h, record=None, frame=None):
    command = h.c.step(state(h.clock) if frame is None else frame,
                       inp() if record is None else record, [])
    assert np.isfinite(command).all()
    h.clock.sleep(1 / h.c.RATE_HZ)
    return command


def settle(h, record, count=250):
    commands = [tick(h, record) for _ in range(count)]
    return commands[-1], commands


def test_relaxed_level_arms_and_preserves_point_four_creep(harness):
    final, commands = settle(harness, inp())
    assert harness.c.ARMED["on"]
    assert final == pytest.approx((0.4, 0.0))
    assert all(0 <= v <= 0.4 and w == 0 for v, w in commands)


def test_reverse_is_straight_and_both_direction_changes_have_zero_tick(harness):
    h = harness
    settle(h, inp())
    assert tick(h, inp(up=True, squeeze=1.0, steer=1.0)) == (0.0, 0.0)
    reverse, _ = settle(h, inp(up=True, squeeze=1.0, steer=1.0))
    assert reverse == pytest.approx((-0.6, 0.0))
    assert tick(h, inp()) == (0.0, 0.0)
    assert settle(h, inp())[0] == pytest.approx((0.4, 0.0))


def test_raised_arm_cannot_arm_on_start_or_after_fault(harness):
    h = harness
    assert settle(h, inp(up=True))[0] == (0.0, 0.0)
    tick(h, inp())
    assert h.c.ARMED["on"]
    assert tick(h, inp(fresh=False)) == (0.0, 0.0)
    assert not h.c.ARMED["on"]
    assert settle(h, inp(squeeze=1.0, steer=1.0))[0] == (0.0, 0.0)
    assert tick(h, inp())[0] > 0.0


@pytest.mark.parametrize("raw_effort,expected", [(0.0, 0.4), (0.15, 0.4),
    (0.30, 0.890909090909), (0.50, 1.545454545455), (0.638888888889, 2.0), (0.70, 2.0), (1.0, 2.0)])
def test_forward_curve_preserves_existing_steady_speed(harness, raw_effort, expected):
    squeeze = max(0.0, min(1.0, (raw_effort - 0.15) / 0.85))
    assert harness.c.forward_target(squeeze) == pytest.approx(expected)
    tick(harness, inp())
    assert settle(harness, inp(squeeze=squeeze))[0] == pytest.approx((expected, 0.0))


def test_speed_changes_obey_slew_but_obstacle_brakes_immediately(harness):
    h = harness
    settle(h, inp())
    _, speeding = settle(h, inp(squeeze=1.0), count=80)
    assert np.diff([0.4] + [v for v, _ in speeding]).max() <= h.c.ACCEL / h.c.RATE_HZ + 1e-12
    _, slowing = settle(h, inp(), count=100)
    assert np.diff([speeding[-1][0]] + [v for v, _ in slowing]).min() >= -h.c.DECEL / h.c.RATE_HZ - 1e-12
    settle(h, inp(squeeze=1.0))
    blocked = state(h.clock)
    angle = blocked["scan"]["angle_min"] + 90 * blocked["scan"]["angle_inc"]
    blocked["scan"]["ranges"][90] = 0.3 / math.cos(angle)
    assert tick(h, inp(squeeze=1.0), blocked)[0] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("steer,expected", [(1.0, 1.3), (-1.0, -1.8)])
def test_asymmetric_turn_limits_and_lidar_never_adds_steering(harness, steer, expected):
    tick(harness, inp())
    assert settle(harness, inp(squeeze=1.0, steer=steer))[0][1] == pytest.approx(expected)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -0.1, 1.1])
def test_forward_target_rejects_invalid_effort(harness, bad):
    with pytest.raises(ValueError):
        harness.c.forward_target(bad)


def test_rms_removes_channel_offsets(harness):
    samples = np.asarray(emg(12.0)) + np.arange(8) * 50.0
    assert harness.c.channel_rms(samples) == pytest.approx([12.0] * 8)


@pytest.mark.parametrize("stream", ["EMG", "IMU"])
def test_publisher_loss_during_calibration_flush_is_retryable(harness, monkeypatch, stream):
    def lost_publisher(*args, **kwargs):
        raise RuntimeError("publisher disconnected")

    monkeypatch.setattr(getattr(harness.c, stream), "pull_chunk", lost_publisher)
    with pytest.raises(harness.c.CalibrationAttemptError, match="Cannot flush Myo streams"):
        harness.c._collect_pose("relax")


@pytest.mark.parametrize("bad", [[], [[1.0] * 7] * 40, [[1.0] * 9] * 40,
    [[float("nan")] * 8] * 40, [[float("inf")] * 8] * 40])
def test_bad_emg_cannot_produce_feature(harness, bad):
    with pytest.raises(ValueError):
        harness.c.channel_rms(bad)


def test_gravity_tracks_quaternion_orientation(harness):
    assert harness.c.gravity(imu(math.pi / 6)) == pytest.approx([0.0, 0.5, math.sqrt(3) / 2])


@pytest.mark.parametrize("bad", [[], [[0.0] * 10] * 10, [[1.0] * 9] * 10,
    [[float("nan")] + [0.0] * 9] * 10, [[1.0] + [0.0] * 8 + [float("inf")]] * 10])
def test_bad_imu_cannot_produce_direction(harness, bad):
    with pytest.raises(ValueError):
        harness.c.gravity(bad)


@pytest.mark.parametrize("fault", ["missing", "empty", "nan", "negative", "too_far", "short",
    "nan_angle", "negative_increment", "rear_only", "nan_max"])
def test_invalid_lidar_stops_and_disarms_even_during_reverse(harness, fault):
    h, frame = harness, state(harness.clock)
    if fault == "missing": del frame["scan"]
    elif fault == "empty": frame["scan"] = {}
    elif fault == "nan": frame["scan"]["ranges"][0] = float("nan")
    elif fault == "negative": frame["scan"]["ranges"][0] = -1.0
    elif fault == "too_far": frame["scan"]["ranges"][0] = 11.0
    elif fault == "short": frame["scan"]["ranges"].pop()
    elif fault == "nan_angle": frame["scan"]["angle_min"] = float("nan")
    elif fault == "negative_increment": frame["scan"]["angle_inc"] = -0.02618
    elif fault == "rear_only": frame["scan"]["angle_min"] = math.pi / 2
    elif fault == "nan_max": frame["scan"]["range_max"] = float("nan")
    with pytest.raises(ValueError): h.c.path_clearance(frame)
    h.c.ARMED["on"] = True
    h.c.BACK_REQUESTED = True
    h.c.SMOOTH["v"] = -0.6
    assert tick(h, inp(up=True), frame) == (0.0, 0.0)
    assert not h.c.ARMED["on"]
    assert tick(h, inp(up=True)) == (0.0, 0.0)


def test_zero_range_contact_stops_forward_but_valid_scan_allows_reverse(harness):
    h = harness
    frame = state(h.clock)
    frame["scan"]["ranges"][90] = 0.0
    assert h.c.path_clearance(frame) == 0.0
    h.c.ARMED["on"] = True
    assert tick(h, inp(squeeze=1.0), frame)[0] == 0.0
    assert tick(h, inp(up=True), state(h.clock, scan=frame["scan"])) == (0.0, 0.0)
    commands = [tick(h, inp(up=True), state(h.clock, scan=frame["scan"])) for _ in range(20)]
    assert commands[-1] == pytest.approx((-0.6, 0.0))


@pytest.mark.parametrize("fault", ["missing_receipt", "old", "future_receipt", "nan_receipt",
    "missing_time", "nan_time", "wrong_type", "estop", "finished", "unknown_status",
    "nan_pose", "missing_vel", "nan_applied", "negative_collisions"])
def test_invalid_state_stops_and_requires_neutral_rearm(harness, fault):
    h, frame = harness, state(harness.clock)
    if fault == "missing_receipt": del frame["_received_at"]
    elif fault == "old": frame["_received_at"] -= h.c.STATE_TIMEOUT + 0.01
    elif fault == "future_receipt": frame["_received_at"] += 1.0
    elif fault == "nan_receipt": frame["_received_at"] = float("nan")
    elif fault == "missing_time": del frame["t"]
    elif fault == "nan_time": frame["t"] = float("nan")
    elif fault == "wrong_type": frame["type"] = "event"
    elif fault in ("estop", "finished", "unknown_status"): frame["status"] = fault
    elif fault == "nan_pose": frame["pose"]["x"] = float("nan")
    elif fault == "missing_vel": del frame["vel"]
    elif fault == "nan_applied": frame["cmd_applied"]["w"] = float("nan")
    elif fault == "negative_collisions": frame["collisions"] = -1
    h.c.ARMED["on"] = True
    assert tick(h, inp(squeeze=1.0, steer=1.0), frame) == (0.0, 0.0)
    assert not h.c.ARMED["on"]
    assert tick(h, inp(squeeze=1.0)) == (0.0, 0.0)
    assert tick(h, inp())[0] > 0.0


@pytest.mark.parametrize("status", ["idle", "deadman"])
def test_valid_startup_status_does_not_prevent_neutral_arming(harness, status):
    assert tick(harness, inp(), state(harness.clock, status=status))[0] > 0.0


def test_completed_or_dnf_course_cannot_rearm_with_status_ok(harness):
    h = harness
    course = {"id": "L1-gates", "next_gate": 7, "gates_passed": 7,
              "elapsed": 120.0, "done": True}
    h.c.ARMED["on"] = True
    for record in (inp(), inp(squeeze=1.0), inp(up=True)):
        assert tick(h, record, state(h.clock, status="ok", course=course)) == (0.0, 0.0)
        assert not h.c.ARMED["on"]


def test_partial_startup_windows_can_fill_while_commands_stay_stopped(harness, monkeypatch):
    h = harness
    monkeypatch.setattr(h.c, "EMG", Inlet(h.clock, [emg(count=10) for _ in range(4)]))
    monkeypatch.setattr(h.c, "IMU", Inlet(h.clock, [imu(count=3) for _ in range(4)], hz=50))
    for i in range(4):
        record = h.c.my_input()
        command = tick(h, record)
        if i < 3:
            assert not record["fresh"] and command == (0.0, 0.0)
            assert len(h.c.EMG_BUF) == (i + 1) * 10
        else:
            assert record["fresh"] and command[0] > 0.0


@pytest.mark.parametrize("fault", ["missing_scan", "stale_state", "malformed_waiting_input"])
def test_partial_waiting_does_not_preserve_samples_when_another_fault_occurs(harness, monkeypatch, fault):
    h = harness
    monkeypatch.setattr(h.c, "EMG", Inlet(h.clock, [emg(count=10)]))
    monkeypatch.setattr(h.c, "IMU", Inlet(h.clock, [imu(count=3)], hz=50))
    record, frame = h.c.my_input(), state(h.clock)
    assert record["waiting"] and len(h.c.EMG_BUF) == 10
    if fault == "missing_scan":
        del frame["scan"]
    elif fault == "stale_state":
        frame["_received_at"] -= h.c.STATE_TIMEOUT + 0.01
    else:
        record["level"] = float("nan")
    assert tick(h, record, frame) == (0.0, 0.0)
    assert not h.c.ARMED["on"]
    assert not h.c.EMG_BUF and not h.c.IMU_BUF
    assert not h.c.EMG_TIMES and not h.c.IMU_TIMES


@pytest.mark.parametrize("stream", ["emg", "imu"])
def test_one_missing_sensor_stops_even_when_other_is_fresh(harness, monkeypatch, stream):
    h = harness
    monkeypatch.setattr(h.c, "EMG", Inlet(h.clock, [emg()] + ([] if stream == "emg" else [emg()])) )
    monkeypatch.setattr(h.c, "IMU", Inlet(h.clock, [imu()] + ([] if stream == "imu" else [imu()]), hz=50))
    assert tick(h, h.c.my_input())[0] > 0.0
    h.clock.sleep(h.c.SIGNAL_TIMEOUT + 0.01)
    record = h.c.my_input()
    assert not record["fresh"]
    assert tick(h, record) == (0.0, 0.0)
    assert not h.c.ARMED["on"]


@pytest.mark.parametrize("fault", ["old", "future", "replay", "nan_sample", "zero_quaternion"])
def test_bad_source_packets_never_keep_motion_alive(harness, monkeypatch, fault):
    h = harness
    if fault == "replay":
        stamps_e, stamps_i = [h.clock.now] * 40, [h.clock.now] * 10
    else:
        stamp = h.clock.now - 1 if fault == "old" else h.clock.now + 60 if fault == "future" else h.clock.now
        stamps_e, stamps_i = [stamp] * 40, [stamp] * 10
    e, q = emg(32.0), imu(0.4)
    if fault == "nan_sample": e[0][0] = float("nan")
    if fault == "zero_quaternion": q[0][:4] = [0.0] * 4
    monkeypatch.setattr(h.c, "EMG", Inlet(h.clock, [(e, stamps_e)] * 5))
    monkeypatch.setattr(h.c, "IMU", Inlet(h.clock, [(q, stamps_i)] * 5, hz=50))
    h.c.ARMED["on"] = True
    record = h.c.my_input()
    if fault == "replay":
        assert record["fresh"]
        last = dict(h.c.LAST_SEEN)
        for _ in range(4):
            h.clock.sleep(0.10)
            record = h.c.my_input()
        assert all(h.c.LAST_SEEN[k] <= last[k] for k in last)
    assert not record["fresh"]
    assert tick(h, record) == (0.0, 0.0)
    assert not h.c.ARMED["on"]


def test_small_batched_timestamp_regression_is_supported(harness, monkeypatch):
    h = harness
    stamps_e = np.linspace(h.clock.now - 0.195, h.clock.now, 40).tolist()
    stamps_e[20], stamps_e[21] = stamps_e[21], stamps_e[20]
    monkeypatch.setattr(h.c, "EMG", Inlet(h.clock, [(emg(), stamps_e)]))
    monkeypatch.setattr(h.c, "IMU", Inlet(h.clock, [imu()], hz=50))
    assert h.c.my_input()["fresh"]


@pytest.mark.parametrize("slow_stream", ["emg", "imu"])
def test_new_trickle_cannot_reuse_old_motion_window(harness, monkeypatch, slow_stream):
    h = harness
    tick(h, inp())
    emg_packets = [emg(32.0)] + [emg(32.0, count=1 if slow_stream == "emg" else 40) for _ in range(8)]
    imu_packets = [imu(0.1)] + [imu(0.1, count=1 if slow_stream == "imu" else 10) for _ in range(8)]
    monkeypatch.setattr(h.c, "EMG", Inlet(h.clock, emg_packets))
    monkeypatch.setattr(h.c, "IMU", Inlet(h.clock, imu_packets, hz=50))
    assert tick(h, h.c.my_input())[0] > 0.0
    for _ in range(8):
        h.clock.sleep(0.10)
        record = h.c.my_input()
        if not record["fresh"]:
            break
        tick(h, record)
    assert not record["fresh"]
    assert tick(h, record) == (0.0, 0.0)
    assert not h.c.ARMED["on"]


def test_calibration_retries_all_poses_after_quality_and_signal_failure(harness, monkeypatch):
    h, calls, prompts = harness, [], []
    results = [{"problems": ["poses overlap"]}, h.c.CalibrationAttemptError("missing IMU"), calibration()]
    def attempt(wait_for_start=True):
        calls.append(wait_for_start)
        result = results.pop(0)
        if isinstance(result, Exception): raise result
        return result
    monkeypatch.setattr(h.c, "_calibrate_once", attempt)
    monkeypatch.setattr("builtins.input", lambda text: prompts.append(text) or "")
    accepted = h.c.calibrate()
    assert accepted["problems"] == [] and accepted["full"] == 32.0
    assert calls == [True, False, False] and len(prompts) == 2


def install_calibration_streams(h, monkeypatch, fault=None):
    """Exercise the real collection loops with fresh packets at nominal rates."""
    driver = {"pose": "relax"}
    names = ["relax", "left", "right", "up", "squeeze"]
    monkeypatch.setattr(h.c, "POSES", [(p, p.upper(), p) for p in names])

    def report(*args, **kwargs):
        text = str(args[0]) if args else ""
        for pose in names:
            if text.startswith("\n" + pose.upper() + ":"):
                driver["pose"] = pose
    monkeypatch.setattr(h.c, "print", report)

    class Sensor:
        def __init__(self, kind):
            self.kind, self.last, self.reads = kind, None, 0
        def pull_chunk(self, timeout=0.0, **kwargs):
            now = h.clock.now
            if now == self.last or (fault == "one_burst" and self.reads > 1):
                return [], []
            self.last, self.reads = now, self.reads + 1
            pose = driver["pose"]
            if self.kind == "emg":
                level = 32.0 if pose == "squeeze" and fault != "no_fist" else 4.0
                samples, hz = emg(level, count=10), 200
                if fault == "nan": samples[0][0] = float("nan")
            else:
                angle, axis = {"left": (-0.5, 1), "right": (0.5, 1),
                               "up": (0.5, 0)}.get(pose, (0.0, 0))
                if fault == "no_move": angle = 0.0
                samples, hz = imu(angle, count=2 + self.reads % 2, axis=axis), 50
            return samples, [now - (len(samples) - 1 - i) / hz for i in range(len(samples))]
    monkeypatch.setattr(h.c, "EMG", Sensor("emg"))
    monkeypatch.setattr(h.c, "IMU", Sensor("imu"))


def test_nominal_rate_fresh_samples_complete_real_calibration(harness, monkeypatch):
    h = harness
    install_calibration_streams(h, monkeypatch)
    result = h.c._calibrate_once(wait_for_start=False)
    assert result["problems"] == []
    assert result["zero"] == pytest.approx(4.0)
    assert result["full"] == pytest.approx(32.0)
    assert result["reach_left"] > 0 and result["reach_right"] < 0 and result["reach_up"] > 0


@pytest.mark.parametrize("fault", ["no_fist", "no_move", "nan", "one_burst"])
def test_bad_calibration_never_returns_a_drivable_mapping(harness, monkeypatch, fault):
    h = harness
    install_calibration_streams(h, monkeypatch, fault)
    try:
        result = h.c._calibrate_once(wait_for_start=False)
    except (ValueError, h.c.CalibrationAttemptError):
        return
    assert result["problems"], "A bad attempt must be rejected before the driving loop"


class Frames:
    def __init__(self, clock, frames, after_frame=None):
        self.clock, self.frames, self.after_frame = clock, frames, after_frame
    async def __aiter__(self):
        for frame in self.frames:
            self.clock.sleep(0.10)
            yield frame if isinstance(frame, str) else json.dumps(frame)
            if self.after_frame:
                self.after_frame()


def test_duplicate_sim_time_and_events_do_not_refresh_watchdog(harness):
    h, current, events, snapshots = harness, {}, [], []
    first = state(h.clock, t=10.0)
    frames = Frames(h.clock, [first, {"type": "event", "name": "gate:1"}, first],
                    after_frame=lambda: snapshots.append(dict(current)))
    asyncio.run(h.c.listen_state(frames, current, events))
    assert all(s["_received_at"] == pytest.approx(100.1) for s in snapshots)
    assert events == ["gate:1"]
    assert not current, "Clean EOF must not leave a cached state usable"


def test_sim_clock_restart_disarms_and_clears_old_motion(harness):
    h = harness
    h.c.ARMED["on"] = True
    h.c.SMOOTH.update(v=1.0, w=1.0)
    current, snapshots = {}, []
    frames = Frames(h.clock, [state(h.clock, t=10.0), state(h.clock, t=1.0)],
                    after_frame=lambda: snapshots.append((dict(current), h.c.ARMED["on"], dict(h.c.SMOOTH))))
    asyncio.run(h.c.listen_state(frames, current, []))
    assert snapshots[-1][0]["t"] == 1.0
    assert not snapshots[-1][1]
    assert snapshots[-1][2]["v"] == snapshots[-1][2]["w"] == 0.0
    assert not h.c.ARMED["on"]
    assert h.c.SMOOTH["v"] == h.c.SMOOTH["w"] == 0.0


@pytest.mark.parametrize("payload", ["{broken-json", "[]", "null", '{"type":"state","t":NaN}'])
def test_malformed_state_invalidates_previous_state(harness, payload):
    h, current = harness, state(harness.clock)
    h.c.ARMED["on"] = True
    with pytest.raises(ValueError):
        asyncio.run(h.c.listen_state(Frames(h.clock, [payload]), current, []))
    assert not current and not h.c.ARMED["on"]


@pytest.mark.parametrize("terminal_event", ["finish", "dnf"])
def test_terminal_event_stays_stopped_until_course_reset_and_neutral_handshake(harness, terminal_event):
    h, current, observed = harness, {}, []
    ready = {"id": "L1-gates", "next_gate": 0, "gates_passed": 0,
             "elapsed": 0.0, "done": False}
    completed = dict(ready, elapsed=120.0, done=True)
    frames = [state(h.clock, t=10.0, course=ready),
              {"type": "event", "name": terminal_event},
              state(h.clock, t=10.1, status="ok", course=ready),
              state(h.clock, t=10.2, status="ok", course=completed),
              state(h.clock, t=0.05, course=ready),
              state(h.clock, t=0.10, course=ready)]

    def after_frame():
        # An active hand immediately after reset must still wait for neutral.
        record = inp(squeeze=1.0) if len(observed) == 4 else inp()
        command = tick(h, record, current)
        observed.append((command, h.c.ARMED["on"]))

    asyncio.run(h.c.listen_state(Frames(h.clock, frames, after_frame), current, []))
    assert observed[0][0][0] > 0.0, "Begin with a genuinely armed controller"
    assert all(command == (0.0, 0.0) and not armed for command, armed in observed[1:5])
    assert observed[5][0][0] > 0.0 and observed[5][1]


@pytest.mark.parametrize("failure", ["clean_eof", "malformed_json"])
def test_state_connection_failure_stops_reconnects_and_rearms_using_real_step(
        harness, monkeypatch, failure):
    """No real socket: exercise run, listener and actual command generation together."""
    h, real_sleep = harness, asyncio.sleep
    connections, commands, reads = [], {}, {}

    class Socket:
        def __init__(self, number, is_state):
            self.number, self.is_state = number, is_state
        async def __aenter__(self):
            return self
        async def __aexit__(self, *args):
            return False
        async def __aiter__(self):
            frames = 6 if self.number == 1 else 1000
            for i in range(frames):
                yield json.dumps(state(h.clock, t=self.number * 100.0 + i * 0.05))
                await real_sleep(0)
            if failure == "malformed_json":
                yield "{broken-json"
        async def send(self, raw):
            message = json.loads(raw)
            assert math.isfinite(message["v"]) and math.isfinite(message["w"])
            commands.setdefault(self.number, []).append(message)

    def connect(url):
        if url.endswith("/state"):
            connections.append(len(connections) + 1)
        return Socket(connections[-1], url.endswith("/state"))

    def read_input():
        number = connections[-1]
        reads[number] = reads.get(number, 0) + 1
        # On reconnect, active intent must not restore the previous armed state.
        return inp(squeeze=1.0, steer=1.0) if number > 1 and reads[number] <= 3 else inp()

    async def advance(seconds):
        h.clock.sleep(seconds)
        await real_sleep(0)

    monkeypatch.setattr(h.c.websockets, "connect", connect)
    monkeypatch.setattr(h.c, "my_input", read_input)
    monkeypatch.setattr(h.c, "asyncio", SimpleNamespace(
        create_task=asyncio.create_task, sleep=advance,
        CancelledError=asyncio.CancelledError, gather=asyncio.gather, wait_for=asyncio.wait_for))
    asyncio.run(h.c.run("ws://offline.invalid", seconds=2.0))
    assert len(connections) >= 2
    assert any(m["v"] > 0.0 for m in commands[1]), "First connection must actually drive before the fault"
    assert (commands[1][-1]["v"], commands[1][-1]["w"]) == (0.0, 0.0)
    assert all((m["v"], m["w"]) == (0.0, 0.0) for m in commands[2][:3])
    assert any(m["v"] > 0.0 for m in commands[2][3:]), "Neutral handshake should permit creep again"
    assert (commands[2][-1]["v"], commands[2][-1]["w"]) == (0.0, 0.0)
    assert not h.c.ARMED["on"]


def test_delayed_control_cycle_sends_zero_instead_of_stale_motion(harness, monkeypatch):
    h, real_sleep, actual_step = harness, asyncio.sleep, harness.c.step
    commands, computed = [], []
    reads = {"count": 0}

    class Socket:
        async def __aenter__(self): return self
        async def __aexit__(self, *args): return False
        async def __aiter__(self):
            for i in range(1000):
                yield json.dumps(state(h.clock, t=10.0 + i * 0.05))
                await real_sleep(0)
        async def send(self, raw): commands.append(json.loads(raw))

    def read_input():
        reads["count"] += 1
        return inp() if reads["count"] <= 3 else inp(squeeze=1.0, steer=1.0)

    def slow_step(frame, record, events):
        result = actual_step(frame, record, events)
        computed.append(result)
        if len(computed) == 3:
            h.clock.sleep(h.c.SIGNAL_TIMEOUT + 0.01)
        return result

    async def advance(seconds):
        h.clock.sleep(seconds)
        await real_sleep(0)

    monkeypatch.setattr(h.c.websockets, "connect", lambda url: Socket())
    monkeypatch.setattr(h.c, "my_input", read_input)
    monkeypatch.setattr(h.c, "step", slow_step)
    monkeypatch.setattr(h.c, "asyncio", SimpleNamespace(
        create_task=asyncio.create_task, sleep=advance,
        CancelledError=asyncio.CancelledError, gather=asyncio.gather, wait_for=asyncio.wait_for))
    asyncio.run(h.c.run("ws://offline.invalid", seconds=0.8))
    assert computed[2][0] > 0.0, "The delayed cycle must have generated a real movement request"
    assert (commands[2]["v"], commands[2]["w"]) == (0.0, 0.0)
    assert all((m["v"], m["w"]) == (0.0, 0.0) for m in commands[3:])
    assert not h.c.ARMED["on"]
