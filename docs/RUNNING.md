# Setup and running

## Prerequisites

Use the separately distributed hackathon environment. It must provide Python, `numpy`, `pylsl` with native liblsl, `websockets`, `jackal-bridge`, and `Hackathon/examples/myo_lsl_publisher.py`. Python 3.11 was available for this audit. This repository does not contain the publisher or bridge source; installing the Python imports alone does not install the entire system.

The packaged ML requirements and version snapshot are in `archive/gesture-classification/gesture_baseline/requirements.txt` and `requirements-lock.txt`; these are for the ML package, not a verified lockfile for every controller and the simulator.

## Three terminals (same computer)

Replace every `/path/to/...` and `YOUR_MYO_BLUETOOTH_ADDRESS`. No device address is universal. Stop other controllers/keyboard teleop before starting one controller.

Terminal 1 - publisher:

```bash
cd /path/to/hackathon-environment
./jackal-runtime/bin/python Hackathon/examples/myo_lsl_publisher.py --addr YOUR_MYO_BLUETOOTH_ADDRESS
```

Terminal 2 - bridge and bundled site:

```bash
cd /path/to/aware-ai-myo-control
/path/to/hackathon-environment/jackal-runtime/bin/jackal-bridge --serve-site --site-dir ./jackal-site
```

Terminal 3 - final competition controller (3.2):

```bash
cd /path/to/aware-ai-myo-control
/path/to/hackathon-environment/jackal-runtime/bin/python controllers/competition/my_team3_2.py
```

Open `http://localhost:8765/Hackathon/`. Its steering overlay is included in `jackal-site/index.html`; the asset should also be available at `/Hackathon/wheel-overlay.js`. The external bridge must support these paths. The owner has reviewed synthetic simulator operation; real Myo validation of the experimental variants remains pending.

For the separate experimental 3.2a variant, see [safety integration and testing](SAFETY_3_2A.md). Run only one controller at a time; 3.2a has been exercised with synthetic input; real Myo testing remains pending. It includes a three-second neutral preparation gate; use `http://localhost:8765/Hackathon/?controller=3.2a` for the optional countdown overlay.

## Calibration and operation

Follow five poses: relax, left, right, hand-up, fist. Each has 3 seconds preparation + 3 seconds acquisition; nominal recording sequence is 30 seconds plus prompts/retries. Open-hand reverse is disabled by default. A rejected calibration prompts a complete retry.

After calibration, hold a relaxed, level, untwisted pose to arm. Relaxed means slow forward, fist adds speed, twist steers, hand-up requests straight reverse. Use the simulator E-stop or Ctrl+C to stop. A direction change inserts one zero-command tick; it does not measure actual standstill. Check control direction in the sandbox course before navigating obstacles.

The competition entry point logs to `data/driving-sessions/<timestamp>/`. Runtime-generated logs should be reviewed before committing; original historical logs are preserved.

## Offline ML, no robot required

The ZIP has been extracted without changing package contents:

```bash
cd /path/to/aware-ai-myo-control/archive/gesture-classification/gesture_baseline
python -m pip install -r requirements.txt
python -m unittest test_baseline
python train.py --data ../../../data/gesture-recordings/Myodata.zip --out /tmp/aware-ai-baseline-run
python predict.py --help
```

Use a new `--out` directory to keep historical outputs intact. See the archive wrapper README and `predict.py --help` for replay/model-path options. Its historical README assumes a shallower directory; from this extracted location use `../../../data/gesture-recordings/Myodata.zip`, not `../Myodata.zip`. Stored verification and metrics are historical artifacts, not a claim that a new live test passed.

For an isolated audit environment, create a virtual environment outside the repository or in `.venv/`. Do not overwrite the shared hackathon runtime. Native LSL setup is platform-dependent.

## Troubleshooting

- No Myo streams: check publisher, device connection, and matching host/stream names. 3.x expects exactly one local `Myo_EMG` and `Myo_IMU`.
- Bridge connection failure: check the external runtime, port 8765, and that bridge and browser are running.
- Calibration rejected: follow prompts and inspect signal/pose quality before retrying.
- Missing overlay: serve `jackal-site/`, verify asset URL, reload; FPV intentionally hides it.
- Script says “3.1” while running 3.2: inherited display text is stale; the actual 3.2 file changes the LiDAR table. The inherited display text was preserved; only run/path references and the log destination were changed.
