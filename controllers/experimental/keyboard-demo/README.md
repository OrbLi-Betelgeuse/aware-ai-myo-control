# Keyboard-to-Myo Test (3.2aTest)

For controller testing by teammates. For the smoother built-in keyboard interface preview, use the [main README](../../../README.md#no-myo-try-keyboard-demo).

This separate test adapter generates eight-channel EMG and IMU quaternion samples for the unchanged 3.2a controller. Samples pass through its input processing, steering, reverse logic, forward LiDAR limits and three-second preparation gate. Fixed synthetic calibration is not a real wearer calibration. The adapter imports the experimental 3.2a controller. The shared Ready/countdown behavior lives in 3.2a; competition 3.2 remains unchanged.

## Startup

Stop any existing straight-input controller with Ctrl+C before starting this adapter. Run one bridge, one controller and one simulator page. Close simulator pages in other browsers, including the Codex browser: even paused keyboard pages can still publish simulator state.

In terminal A, from the repository root:

```sh
sh controllers/experimental/start.sh bridge
```

In terminal B, from the same directory:

```sh
sh controllers/experimental/start.sh keyboard
```

Do not repeat these commands if their services are already running. The script uses the existing local runtime; set `AWARE_RUNTIME` to another installed runtime directory if needed. See [runtime details](../3.2a/PREVIEW.md).

Open <http://localhost:8765/Hackathon/?controller=3.2a&keyboard=1>. A course may be selected with `&course=L3-corridor-dock`. Refresh after a tool update. Click **Start / Resume** in the keyboard panel, or focus the simulator and press Enter. Release WASD throughout the countdown; drive after GO.

## Controls

| Input | Effect |
| --- | --- |
| Start / Resume or Enter | Enable input; initial start and recovery require neutral preparation |
| Hold W | Simulate a fist to increase forward speed, subject to LiDAR limits |
| Hold A / D | Simulate left / right arm orientation |
| W+A / W+D | Combine effort and steering |
| Hold S | Simulate hand lift; transition into straight reverse |
| Stop Driving or Space | Pause test input and request zero velocity |
| Release WASD | Neutral, relaxed input: approximately 0.4 m/s forward after preparation, not a stop |

Reverse does not steer and has no rear LiDAR protection. Key presses are discrete while arm orientation is continuous. This adapter checks controller behavior, not realistic Myo ergonomics. Existing smoothing and asymmetric steering gains remain active.

Changing focus, hiding the page or losing the keyboard heartbeat pauses input. Return to the page and explicitly resume. An idle keyboard connection no longer reserves control; another actively driving page cannot be displaced. Still use only one simulator page, because all simulator pages may feed the same bridge.

## Reset and fault diagnosis

After switching or resetting a course, release WASD and wait for preparation. After a finish, reset the course. After E-stop, clear E-stop before resuming. The keyboard panel reports controller faults while enabled.

The test-only preparation publisher keeps STOP visible while a recorded fault remains active. This avoids transient READY messages emitted by state resets between control ticks. It does not suppress faults, bypass safety checks or change the countdown. The competition source remains unchanged by this feature; 3.2a supplies the shared preparation gate.

Check countdown gating, left/right turns, acceleration, straight reverse, forward LiDAR limiting, pause/resume and course reset separately. Browser behavior still requires manual validation; offline tests are not proof of successful browser interaction.

## Shutdown and records

Stop driving or use simulator E-stop, then stop terminal B with Ctrl+C, followed by terminal A. Port 8765 serves the bridge/site, 8766 serves preparation status and 8767 accepts keyboard input. Stop the known old service before reusing its port.

Each session writes calibration, configuration and drive logs together in `data/driving-sessions/<timestamp>_synthetic_3_2aTest/`. These local files are ignored by Git and explicitly marked synthetic. They are not competition results or real Myo measurements.

Regression scripts cover controller behavior (`test_keyboard_3_2aTest.py`), connection ownership (`test_keyboard_connections.py`) and offline UI events (`test_keyboard_overlay.cjs`).

## Interface and stop states

Normal use: **Start / Resume → WASD → Space to stop driving**.

| Control or state | Meaning |
| --- | --- |
| Start / Resume | Enable synthetic input; does not clear emergency stop |
| Stop Driving / Space | Stop test input and request zero motion; demo timer pauses |
| Restart Course | Reset the course and its timer; preparation runs again |
| Emergency Controls: E-stop | Activate simulator emergency stop; demo timer pauses |
| Emergency Controls: Clear E-stop | Explicitly release emergency stop; neutral preparation is still required |
| Driving Stopped | Input is stopped, including page focus loss; select Start / Resume |
| Emergency Stop Active | Release the simulator emergency stop before driving |
| Connection / Input Lost | Valid input or simulator state is unavailable; inspect the stated reason |
| Course Ended | Restart the course to try again |

The center card is a status display, not a stop button. Demo scoring time pauses while driving is stopped, during E-stop, input loss and preparation. Physics and safety/state updates continue. These practice times are not comparable to competition scores. Switching camera views should not restart preparation. The demo UI changes only apply with `controller=3.2a&keyboard=1`; the same presentation is shared with the 3.2a preview, without keyboard-only buttons.

## Manual acceptance checklist

1. Start the demo and wait for 3, 2, 1, GO with WASD released.
2. Check A/D steering, W acceleration and S straight reverse.
3. Select Stop Driving or press Space: motion stops and the card says Driving Stopped; the running demo timer pauses.
4. Resume and verify a fresh preparation countdown.
5. Activate E-stop: the card says Emergency Stop Active. Resume must not clear it. Explicitly clear E-stop before driving again.
6. Switch Chase/Top/FPV during driving: it should change only the camera.
7. Restart Course: course time resets and preparation runs again.
8. Use the ordinary 3.2a URL without keyboard=1: demo buttons and emergency grouping must be absent.

Offline tests cover status mapping and keyboard events. Live browser interaction still needs manual confirmation.

Ready is held for at least 0.8 seconds on start/resume before the three-second countdown. Each countdown number occupies one second. When the controller permits motion at the end of 1, the central card disappears immediately on receipt of that state. GO appears alone at the center in green, at the same 48px size as the countdown numbers, for approximately 0.45 seconds. It has no card, background, border or subtitle and does not intercept input. No extra GO waiting period is added. The competition controller and competition timing remain unchanged.

Countdown digits change directly in place, without scaling or fading. This presentation change does not alter controller timing.

At course completion or DNF, the simulator result banner takes priority. Shared preparation/stop overlays and GO remain hidden while results are displayed, then become available again after Restart Course.
