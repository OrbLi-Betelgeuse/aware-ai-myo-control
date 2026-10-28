# No-Myo 3.2a simulator trial

For interactive steering and reverse, use the separate [3.2aTest keyboard guide](../keyboard-demo/README.md). The runner described here supplies continuous neutral input and therefore drives straight.

No Myo, Bluetooth, sensor publisher or physical calibration is required. Synthetic eight-channel EMG and ten-channel IMU samples pass through the actual 3.2a input processing, preparation gate and WebSocket control path. Fixed synthetic calibration does not represent a wearer test.

## Start the bridge and site

In terminal A:

```sh
cd "/Users/betelgeuseorb/Library/Mobile Documents/com~apple~CloudDocs/RIT/AWEAR-AI/Oct 23_Hackathon/aware-ai-myo-control"
sh controllers/experimental/start.sh bridge
```

Do not start a second bridge if one is already running. This binds local port 8765, disables LSL ingress, accepts WebSocket commands and serves the bundled website.

Open <http://localhost:8765/Hackathon/?controller=3.2a>. Keep only one connected simulator page across all browsers.

## Start neutral synthetic input

In terminal B:

```sh
cd "/Users/betelgeuseorb/Library/Mobile Documents/com~apple~CloudDocs/RIT/AWEAR-AI/Oct 23_Hackathon/aware-ai-myo-control"
sh controllers/experimental/start.sh controller
```

The runner generates relaxed, neutral input. Valid simulator state should lead to READY, 3, 2, 1 and GO. Commands stay zero during preparation, then increase toward the default forward target. Front obstacles limit forward speed. This runner supplies no steering and is not an autonomous course solver.

## Manual checks

- Switch courses and check preparation before motion at the new start.
- Reset the current course and check preparation again.
- Apply E-stop; motion must stay stopped until it is cleared and preparation completes.
- Compare terminal preparation messages, displayed numbers and vehicle motion.

Preparation begins after valid state and neutral input reach the controller, not at the instant a course selector is clicked. Queued commands and simulation latency require manual checks.

## Stop

Use simulator E-stop if needed. Stop terminal B with Ctrl+C before stopping terminal A. The competition 3.2 and original teammate safety file remain unchanged.

## Runtime and troubleshooting

The launcher defaults to the existing local environment at `/Users/betelgeuseorb/Documents/Codex/2026-10-22/new-chat/jackal-runtime`. On another machine, set `AWARE_RUNTIME` in both terminals to the installed runtime directory. It must provide jackal-bridge, numpy, pylsl (including native liblsl) and websockets. No LSL inlet is opened, but the controller still uses the pylsl clock.

- Port 8765 or UDP 9870 occupied: stop the known old bridge; do not terminate unidentified processes.
- Port 8766 occupied: stop the old controller before launching another.
- Missing countdown: check the `controller=3.2a` query parameter and refresh the page.
- Malformed or stale simulator state: verify that one simulator page is connected and producing state.
- Countdown disconnected: check port 8766. Supported origins are `http://localhost:8765` and `http://127.0.0.1:8765`. A disconnected display is not an E-stop.

## Logs and validation scope

Each run keeps its configuration, calibration and drive log together in `data/driving-sessions/<timestamp>_synthetic_3_2a/`. Configuration marks `synthetic_input: true`; Git ignores these local test records.

The offline `test_simulate_3_2a.py` check exercises actual input processing, the complete countdown and the eventual 0.4 m/s target. The user has confirmed the countdown visually. Assistant browser access was blocked by tool policy, so the assistant has not independently verified live visuals or course-switch interactions.
