#!/usr/bin/env python3
"""Keyboard-to-synthetic-Myo adapter; imports unchanged 3.2a in this process only."""
import asyncio
import contextlib
import csv
import json
import math
import time
from datetime import datetime
import numpy as np
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'3.2a'))
import simulate_3_2a as synthetic

c = synthetic.controller
ROOT = synthetic.ROOT
KEYS = frozenset(('KeyW', 'KeyA', 'KeyS', 'KeyD'))

class KeyboardState:
    def __init__(self): self.pause(); self.owner = None; self.seq = -1; self.input_active = False
    def pause(self): self.keys = set(); self.enabled = False; self.last = float('-inf')
    def accept(self, message):
        if (not isinstance(message, dict) or message.get('type') != 'keys'
                or type(message.get('seq')) is not int or message['seq'] <= self.seq
                or type(message.get('enabled')) is not bool
                or not isinstance(message.get('keys'), list)
                or any(not isinstance(k,str) or k not in KEYS for k in message['keys'])):
            self.pause(); return False
        self.seq = message['seq']; self.last = time.monotonic()
        self.enabled = message['enabled']; self.keys = set(message['keys']) if self.enabled else set()
        return True
    def active(self): return self.enabled and 0 <= time.monotonic()-self.last <= .30

keyboard = KeyboardState()
ACTUAL_INPUT = c.my_input

def keyboard_input():
    if not keyboard.active():
        keyboard.input_active = False
        return c.input_failure("Keyboard test paused/disconnected; press Enter in the test page")
    if not keyboard.input_active:
        # Paused synthetic sensors must not replay a backlog with expired timestamps.
        now = c.pylsl.local_clock()
        c.EMG.last = now
        c.IMU.last = now
        c.reset_control()
        keyboard.input_active = True
    return ACTUAL_INPUT()

class KeyboardInlet(synthetic.SyntheticInlet):
    def pull_chunk(self, timeout=0.):
        rows, stamps = super().pull_chunk(timeout)
        if not rows: return rows, stamps
        keys = keyboard.keys if keyboard.active() else set()
        if self.kind == 'emg':
            amplitude = math.sqrt(2)*10 if 'KeyW' in keys else .3
            rows = [[amplitude*math.sin(2*math.pi*25*t+j*.2) for j in range(8)] for t in stamps]
        else:
            # Produce a quaternion whose gravity projection matches the fixed fixture.
            ux = .5 * (int('KeyA' in keys)-int('KeyD' in keys))
            uy = .65 if 'KeyS' in keys else 0.
            uz = math.sqrt(1-ux*ux-uy*uy)
            qw = math.sqrt((1+uz)/2)
            rows = [[qw,uy/(2*qw),-ux/(2*qw),0.,0.,0.,1.,0.,0.,0.] for _ in stamps]
        return rows, stamps

async def keyboard_client(socket):
    # A connected but paused page must not reserve control indefinitely.
    local = KeyboardState()
    last_diagnostic = None
    try:
        async for raw in socket:
            try: message=json.loads(raw)
            except (ValueError,TypeError):
                await socket.close(code=1008,reason='Invalid JSON');break
            if not local.accept(message):
                await socket.close(code=1008,reason='Invalid keyboard input');break
            diagnostic = (local.enabled, str(message.get('reason', ''))[:160],
                          str(message.get('camera', ''))[:160])
            if diagnostic != last_diagnostic:
                print('\n[KEYBOARD]', time.time(), diagnostic, flush=True)
                last_diagnostic = diagnostic
            busy = keyboard.owner not in (None, socket) and keyboard.active()
            if keyboard.owner is socket or (local.enabled and not busy):
                keyboard.owner = socket
                keyboard.keys = local.keys.copy()
                keyboard.enabled = local.enabled
                keyboard.last = local.last
                if not local.enabled: keyboard.owner = None
            await socket.send(json.dumps(dict(type='keyboard_status',
                enabled=keyboard.owner is socket and keyboard.active(),
                busy=busy and local.enabled, fault=c.LAST_FAULT)))
    finally:
        if keyboard.owner is socket:
            keyboard.pause();keyboard.owner=None

async def test_preparation_client(socket):
    """Report completed control decisions, not transient reset UI state."""
    try:
        while True:
            payload = dict(c.PREPARE_UI)
            if c.LAST_FAULT == 'Waiting for fresh Myo windows':
                payload.update(phase='waiting', remaining=3, reason='Preparing fresh synthetic input')
            elif payload['phase'] == 'waiting' and c.LAST_FAULT:
                payload.update(phase='stopped', remaining=3, reason=c.LAST_FAULT)
            await socket.send(json.dumps(dict(payload, type='preparation', version=c.VERSION)))
            await asyncio.sleep(1 / c.RATE_HZ)
    except c.websockets.ConnectionClosed:
        pass

async def run():
    async with c.websockets.serve(keyboard_client,'127.0.0.1',8767,
            origins=['http://localhost:8765','http://127.0.0.1:8765'],max_size=2048):
        await c.run_with_preparation_ui(c.BRIDGE)

def main():
    c.validate_configuration();c.SOURCE='keyboard-3.2aTest'
    c.EMG=KeyboardInlet('emg');c.IMU=KeyboardInlet('imu')
    c.CAL=dict(zero=1.,full=10.,up0=np.array([0.,0.,1.]),axis=np.array([1.,0.,0.]),
               tilt_axis=np.array([0.,1.,0.]),reach_left=.5,reach_right=-.5,reach_up=.5,
               rms_relax=np.zeros(8),problems=[])
    c.validate_calibration(c.CAL)
    actual_step = c.step
    last_fault = [None]
    def traced_step(state, inp, events):
        command = actual_step(state, inp, events)
        if c.LAST_FAULT != last_fault[0]:
            print('\n[CONTROL]', time.time(), c.LAST_FAULT or 'valid input/state', flush=True)
            last_fault[0] = c.LAST_FAULT
        return command
    c.step = traced_step
    c.my_input=keyboard_input
    c.preparation_client=test_preparation_client
    out=ROOT/'data/driving-sessions'/datetime.now().strftime('%Y%m%d_%H%M%S_%f_synthetic_3_2aTest')
    out.mkdir(parents=True)
    (out/'configuration.json').write_text(json.dumps(dict(test_tool='3.2aTest',controller=c.VERSION,
        synthetic_input=True,input='keyboard -> synthetic EMG/quaternion -> actual 3.2a my_input',
        calibration='fixed synthetic fixture',prepare_seconds=c.PREPARE_SECONDS),indent=2)+'\n')
    (out/'calibration.json').write_text(json.dumps({k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in c.CAL.items()},indent=2)+'\n')
    print('3.2aTest keyboard adapter. No Myo / Bluetooth. Controller sources unchanged.')
    print('Open http://localhost:8765/Hackathon/?controller=3.2a&keyboard=1')
    print('Enter: prepare/resume. W: fist. A/D: twist. S: lift/reverse. Space: pause.')
    print('Releasing keys means relaxed forward, NOT stop. Blur/heartbeat loss pauses.')
    print('Logs:',out,flush=True)
    c.reset_control()
    with (out/'drive_log.csv').open('w',newline='',buffering=1) as f:
        c.LOG=csv.writer(f)
        c.LOG.writerow(['time_s','fresh','armed','emg_level','squeeze','open','open_effort','steer','tilt',
                        'hand_up','v_cmd','w_cmd','path_clear_m','distance_v_limit','assist','status','x','y','yaw',
                        'course','gates_passed','collisions','events','fault_reason'])
        try:asyncio.run(run())
        except KeyboardInterrupt:print('\nKeyboard test stopped.')
        finally:keyboard.pause();c.LOG=None;c.reset_control()

if __name__=='__main__': main()
