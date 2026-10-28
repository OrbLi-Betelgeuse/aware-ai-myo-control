# 3.2a experimental safety variant

`controllers/experimental/3.2a/my_team3_2a.py` starts from the final competition 3.2 controller and adapts selected checks from `controllers/archive/safety-donor/my_team2_6_lidar_safety_enhanced.py`. Both original files are unchanged by this integration. The owner clarified that the teammate's work enhanced an older version. This variant is named **3.2a**, not 3.3, to make the relationship to the competition controller explicit.

Status: 22 deterministic controller tests pass. The owner reviewed the synthetic simulator interface and preparation sequence. Real Myo integration and physical robot testing remain pending. Do not describe it as the competition version or a certified safety system.

## Choose your input

- **No Myo available:** use the [straight-driving interface preview](PREVIEW.md). It runs actual 3.2a with synthetic neutral input; it is not a keyboard controller or a static screenshot.
- **Myo available:** use the real publisher and bridge setup below.
- **Keyboard steering:** use [Keyboard Demo](../keyboard-demo/README.md).

## Shared experimental presentation

3.2a and Keyboard Demo use the same interface: Ready for 0.8 seconds of valid neutral input, then 3/2/1 for one second each. Motion is permitted when 1 ends. The central card disappears and green GO text appears alone at the same position and size for 0.45 seconds. Emergency Controls and Restart Course are shared; keyboard-only Start / Resume and Stop Driving buttons are absent in Myo/neutral mode.

The experimental webpage pauses practice scoring during preparation, emergency stop and invalid input. State updates and safety checks continue. These times are not competition scores. Fault-state display avoids transient READY flashes. No trained ML model is used by this controller.

## Preserved from 3.2

- Effort-to-speed mapping, asymmetric steering gains, dead zones, low-pass filters and slew limits.
- Reverse mapping, direction-change zero-command tick, and disabled-by-default open-hand reverse.
- The exact piecewise LiDAR knots: distances `[0.20, 0.50, 0.93, 1.55]` and speeds `[0, 0.70, 1.50, 2.00]`.
- Immediate command-level cap reduction and cap release at 0.5 m/s².
- Five-pose calibration instructions and reference calculations, with added validation.

The donor's linear LiDAR policy was not copied. After preparation, normal valid-input regression sequences compare 3.2 and 3.2a over 190 command ticks, covering steering, speed changes, distance boundaries/release and reversal.

## Adapted safety components

| Component | Integration |
| --- | --- |
| `sample_array`, `channel_rms`, `gravity`, `unit` | Finite sensor rows, expected channel counts, nondegenerate/normalized quaternions |
| `read_samples`, `my_input` | Source-age and advancing-timestamp checks; buffer/window freshness; invalid input disarms |
| `path_clearance`, `_state_fields`, `state_clearance` | Required scan/state schema; missing/malformed scan rejected; zero ranges treated conservatively |
| `listen_state` | Malformed JSON rejection, repeated-state freshness handling, terminal latch until simulator clock reset |
| `run`, `_send_zero` | Loop/send timeouts, reconnect reset and best-effort zero command on exit |
| New guarded `step` | Validates before calling retained 3.2 motion logic; faults return zero and require rearming |

Calibration retains the 3.2 prompts but reads validated packets, rejects degenerate axes and validates final references. Failed attempts can be retried. The donor's separate per-attempt diagnostic-file system was not imported; this is selected safety integration, not a wholesale copy of every donor feature.

## Intentional behavior differences

- Missing or invalid required scan/state now disarms and returns zero instead of treating a missing scan as clear.
- Fresh relaxed input must also satisfy `abs(tilt) <= 0.40` to arm. The driver must return to neutral after a fault.
- EMG has 8 channels; IMU has 10. Expected scan has 180 finite nonnegative ranges within `range_max`. Other sensor formats need an explicit compatibility change.
- Source timestamps are checked against same-host LSL `local_clock`; delayed/replayed chunks do not refresh the watchdog. This assumes the documented local publisher setup.
- Non-unit valid quaternions are normalized. Optional open-hand confirmation counts new EMG updates instead of repeated controller ticks.
- Terminal finish/DNF state is latched until the simulator's source clock resets. This reset assumption must be checked in live testing.
- No rear-obstacle detection or new autonomous steering was added. A zero forward cap can still allow rotation, as in 3.2. A zero-command transition is not measured standstill.

A process blocked or crashed cannot itself send a stop; bridge-side command expiry remains an external requirement. State receipt checks are not independent LiDAR acquisition timestamps. Software stops do not establish physical stopping distance.

## Run separately

Use the publisher/bridge setup in [RUNNING.md](../../../docs/RUNNING.md). Stop any other controller, then from repository root:

```bash
/path/to/jackal-runtime/bin/python controllers/experimental/3.2a/my_team3_2a.py
```

Calibration, configuration and drive logs stay together under `data/driving-sessions/<timestamp>_3_2a/`. The configuration records the version and SHA-256 of both input source files plus timing/LiDAR settings. The drive CSV retains 3.2 columns and adds `fault_reason`; unavailable fault-row measurements are blank. Logging is best-effort, not a stopping guarantee.

## Offline verification

```bash
/path/to/jackal-runtime/bin/python tests/test_controller_3_2a.py
```

The original 18 tests cover normal-motion parity, LiDAR table/release, malformed/missing/zero scan data, invalid inputs, stale/terminal state, neutral recovery, loop stalls, sample shape/quaternions, source age/replays, sensor buffer loss, degenerate calibration, invalid configuration, terminal/reset handling, repeated state, invalid JSON, zero-command sending, and cancellation cleanup with mocked transport.

Before promoting this variant, run a separate live Myo + simulator test: calibrate, verify neutral/steering/reverse, exercise obstacle approach and cap release, disconnect the publisher/bridge, exercise E-stop/finish/reset, and review versioned logs. No such live test was performed here.

## Three-second preparation gate

3.2a now holds zero linear and angular commands for **three continuous seconds of fresh neutral input** before allowing its normal motion law. Neutral means relaxed effort, no steering/reverse request and tilt within the neutral threshold. Leaving neutral restarts preparation. Missing/stale input or state, E-stop and other faults cancel it; recovery requires a fresh full countdown.

Preparation is required on initial connection, reconnection, observed course-ID changes and observed simulator-time rollback (same-course reset). Finish/DNF remains stopped until reset. The listener detects transitions from received state; it cannot retract an old command already in the bridge. Reset/command timing must be checked in the actual simulator, especially with artificial link delay.

No nonzero command is produced by the preparation gate before three seconds. On completion, the existing 3.2 ramp begins; relaxed still means forward. This does not add a permanent idle mode or physical brake.

### Optional screen animation

Start 3.2a and open:

```text
http://localhost:8765/Hackathon/?controller=3.2a
```

`jackal-site/preparation-entry.js` loads the shared `preparation-demo.js` display for 3.2a. Digits switch directly without scaling or fading; GO is green text without a card. The older `preparation-overlay.js` is retained as an earlier implementation and is no longer loaded by this entry. It reads the authoritative Python phase over a read-only loopback WebSocket on `127.0.0.1:8766`; the page does not independently count down or release the robot. Without the query parameter the overlay does nothing, including no socket connection, so the ordinary competition page remains unchanged.

The status feed accepts the local bridge page origins (`http://localhost:8765` and `http://127.0.0.1:8765`). Use the documented same-computer setup. A port conflict fails before control starts. The countdown also works with terminal feedback without the optional screen overlay. Losing the display connection shows an unavailable message; it is not itself a robot E-stop. Sensor/bridge watchdogs remain responsible for control faults. Display transport/rendering has ordinary latency; no frame-exact synchronization is claimed.

Preparation holds are recorded as zero-command rows with a preparation message in `fault_reason` (the column also contains actual fault reasons). The session configuration records the three-second duration.

### Additional checks

Four new Python tests exercise the full 3-second zero-output interval, neutral interruption, fault cancellation/restart, and course changes. Existing motion parity tests explicitly start after preparation. The total is now 22 passing tests.

```bash
python tests/test_controller_3_2a.py
node tests/test_preparation_overlay.cjs
```

The JavaScript tests use a mocked DOM and socket: opt-in behavior, 3/2/1, invalid countdown payloads, GO/hiding, stale feed, STOP and READY passed. They are not a visual browser test. Next live trial: initial launch, L1→L2 switch, same-course reset, neutral interruption during countdown and E-stop/recovery. Check zero applied velocity during preparation and correspondence between screen/terminal states. Live Myo, browser rendering and competition timing have not been verified in this change.

Countdown digits change directly in place, without scaling or fading. This presentation change does not alter controller timing.

At course completion or DNF, the simulator result banner takes priority. Shared preparation/stop overlays and GO remain hidden while results are displayed, then become available again after Restart Course.
