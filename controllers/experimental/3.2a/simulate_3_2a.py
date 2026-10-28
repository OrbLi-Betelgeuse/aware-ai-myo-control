#!/usr/bin/env python3
"""No-Myo simulator runner: synthetic local sample arrays, real 3.2a/bridge path.

No Bluetooth or LSL stream is opened. Fixed synthetic calibration is NOT a user
calibration. Use only with the local browser simulator, one controller at a time.
"""
import argparse
import asyncio
import csv
import importlib.util
import json
import math
from datetime import datetime
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('controller32a', ROOT/'controllers/experimental/3.2a/my_team3_2a.py')
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)

class SyntheticInlet:
    """Generate nominal-rate samples with real local LSL-clock timestamps."""
    def __init__(self, kind):
        self.kind = kind
        self.rate = 200 if kind == 'emg' else 50
        self.last = controller.pylsl.local_clock()
    def pull_chunk(self, timeout=0.):
        now = controller.pylsl.local_clock()
        n = min(200, max(0, int((now-self.last)*self.rate)))
        if not n: return [], []
        # Preserve genuine pauses as gaps instead of manufacturing stale backlog.
        stamps = [now-(n-1-i)/self.rate for i in range(n)]
        self.last = now
        if self.kind == 'emg':
            rows = [[0.3*math.sin(2*math.pi*25*t+j*.2) for j in range(8)] for t in stamps]
        else:
            rows = [[1.,0.,0.,0.,0.,0.,1.,0.,0.,0.] for _ in stamps]
        return rows, stamps

async def run(seconds):
    task = asyncio.create_task(controller.run_with_preparation_ui(controller.BRIDGE))
    try:
        if seconds:
            await asyncio.wait_for(asyncio.shield(task), timeout=seconds)
        else:
            await task
    except asyncio.TimeoutError:
        pass
    finally:
        task.cancel()
        try: await task
        except asyncio.CancelledError: pass

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds',type=float,default=0,help='optional duration; zero runs until Ctrl+C')
    args=ap.parse_args()
    if args.seconds < 0: ap.error('--seconds must be nonnegative')
    controller.validate_configuration()
    controller.SOURCE='synthetic-3.2a'
    controller.EMG=SyntheticInlet('emg');controller.IMU=SyntheticInlet('imu')
    controller.CAL=dict(zero=1.,full=10.,up0=np.array([0.,0.,1.]),axis=np.array([1.,0.,0.]),
                        tilt_axis=np.array([0.,1.,0.]),reach_left=.5,reach_right=-.5,reach_up=.5,
                        rms_relax=np.zeros(8),problems=[])
    controller.validate_calibration(controller.CAL)
    folder=ROOT/'data/driving-sessions'/datetime.now().strftime('%Y%m%d_%H%M%S_%f_synthetic_3_2a')
    folder.mkdir(parents=True)
    config=dict(version=controller.VERSION,synthetic_input=True,calibration='fixed synthetic fixture',
                prepare_seconds=controller.PREPARE_SECONDS,bridge=controller.BRIDGE,
                base_sha256=controller.BASE_SHA256,safety_donor_sha256=controller.SAFETY_DONOR_SHA256)
    (folder/'configuration.json').write_text(json.dumps(config,indent=2)+'\n')
    cal={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in controller.CAL.items()}
    (folder/'calibration.json').write_text(json.dumps(cal,indent=2)+'\n')
    print('SYNTHETIC INPUT: neutral EMG/IMU; no Myo, Bluetooth or LSL stream connection.')
    print('After preparation the simulator creeps forward. Use simulator E-stop or Ctrl+C.')
    print('Open http://localhost:8765/Hackathon/?controller=3.2a')
    print('Logs:',folder,flush=True)
    controller.reset_control()
    with (folder/'drive_log.csv').open('w',newline='',buffering=1) as f:
        controller.LOG=csv.writer(f)
        controller.LOG.writerow(['time_s','fresh','armed','emg_level','squeeze','open','open_effort','steer','tilt',
            'hand_up','v_cmd','w_cmd','path_clear_m','distance_v_limit','assist','status','x','y','yaw','course',
            'gates_passed','collisions','events','fault_reason'])
        try: asyncio.run(run(args.seconds))
        except KeyboardInterrupt: print('\nSynthetic run stopped.')
        finally: controller.LOG=None;controller.reset_control()

if __name__=='__main__':main()
