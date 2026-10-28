# Myo Armband → Jackal Control

Control a Jackal in the hackathon browser simulator using Myo muscle activity and arm orientation. This RIT AWARE-AI Hackathon project combines EMG-based speed control, IMU-based steering and reverse, forward LiDAR speed limiting, and an on-robot steering-wheel display.

**3.2 is the final competition version.** It uses calibrated signal processing and rules, not a trained machine learning model. The separate gesture-classification experiment is preserved in the archive and is not required to run the controller.

[Presentation PDF](Project-Presentation.pdf) · [Competition 3.2 source](controllers/competition/my_team3_2.py) · [Try Keyboard Demo](#no-myo-try-keyboard-demo)

## Choose a version

| What you want to do | Version | Input |
| --- | --- | --- |
| Run the final competition controller | **3.2**, instructions below | Myo armband and publisher |
| Explore the interface without Myo | **Keyboard Demo** | Built-in simulator keyboard controls with the updated 3.2a interface |
| Test the experimental control logic | **3.2a / 3.2aTest** | Real Myo or synthetic sensor input; see the experimental guides |

The experimental versions add stricter checks and a preparation countdown. They are not the versions used in the competition. Their browser interface uses practice timing that pauses during stops and preparation. Do not compare those times directly with competition runs.

## Run competition version 3.2

The runnable controller is [`controllers/competition/my_team3_2.py`](controllers/competition/my_team3_2.py). It is the final competition script, not a shortcut to the experimental version.

### What you need

- A Myo armband, its Bluetooth address, and a working connection to your computer.
- This repository, including `jackal-site/`.
- The separately installed hackathon runtime and `Hackathon/examples/myo_lsl_publisher.py` from the [instructor's project](https://github.com/AHRTLab/Hackathon). The runtime must provide `jackal-bridge`, `numpy`, `pylsl` with native liblsl, and `websockets`.

The publisher and bridge are external dependencies, not bundled installers in this repository. Opening the webpage alone does not start the Myo controller.

### Start three terminals

Replace `/path/to/hackathon-environment`, `/path/to/aware-ai-myo-control`, and `YOUR_MYO_BLUETOOTH_ADDRESS` with your actual values. Run one controller and one simulator page at a time; stop any experimental controller first.

**Terminal 1: publish Myo sensor data**

```sh
cd /path/to/hackathon-environment
./jackal-runtime/bin/python Hackathon/examples/myo_lsl_publisher.py --addr YOUR_MYO_BLUETOOTH_ADDRESS
```

**Terminal 2: serve the simulator and start the bridge**

```sh
cd /path/to/aware-ai-myo-control
/path/to/hackathon-environment/jackal-runtime/bin/jackal-bridge --serve-site --site-dir ./jackal-site
```

Open <http://localhost:8765/Hackathon/>. Use this plain URL for 3.2, without `controller=3.2a` or `keyboard=1`.

**Terminal 3: run competition controller 3.2**

```sh
cd /path/to/aware-ai-myo-control
/path/to/hackathon-environment/jackal-runtime/bin/python controllers/competition/my_team3_2.py
```

Follow the terminal calibration prompts: **relax, left, right, hand-up, fist**. Then hold a relaxed, level, untwisted pose to arm. Competition 3.2 does not have the experimental Ready / 3 / 2 / 1 / GO overlay.

### Drive and stop

| Myo input or action | Result |
| --- | --- |
| Relaxed, neutral arm after arming | Slow forward motion, approximately 0.4 m/s target |
| Fist | Increase forward speed, subject to LiDAR limiting |
| Twist left / right | Steer |
| Hand up | Request straight reverse |
| Simulator E-stop | Stop vehicle control; the competition course timer continues |

**Relaxing is not a stop.** Use simulator E-stop before ending a session, then Ctrl+C in the controller terminal. Stop the publisher and bridge when finished. Calibration and driving logs stay together in `data/driving-sessions/<timestamp>/`.

For dependency details, calibration troubleshooting and connection issues, see the [setup and troubleshooting guide](docs/RUNNING.md).

## No Myo? Try Keyboard Demo

### Explore the interface with built-in keyboard controls

This is the recommended no-Myo experience. Drive using the instructor simulator's built-in keyboard mapping while viewing our steering-wheel overlay and updated 3.2a interface: Ready, 3/2/1, green GO, explicit stop states and practice timer pauses. **This mode does not run our Python controller, EMG/IMU processing or LiDAR speed-limiting rules.** It demonstrates the interface, not Myo control performance.

With the external hackathon runtime installed, run this from the repository root:

```sh
sh controllers/experimental/start.sh bridge
```

The launcher defaults to the maintainer's local runtime. On another computer, first set `AWARE_RUNTIME` to your installed `jackal-runtime` directory. No Myo publisher or Python controller is needed for this mode.

Open <http://localhost:8765/Hackathon/?controller=3.2a&input=native>. Select **Start / Resume**, then use **W/S** to drive forward/reverse, **A/D** to turn, **Shift** for fine control and **C** to change camera. Releasing movement keys stops driving. **Space / Stop Driving** explicitly stops driving and pauses the practice timer; resume includes the preparation countdown. E-stop must be cleared explicitly.

Use one simulator page and stop other controllers. The interface preview disconnects its command bridge to prevent remote commands overriding local controls; a disconnected bridge indicator is expected in this mode. The website is still served by the same process.

The steering wheel loads automatically from this repository's `jackal-site/`. Use Chase or Top view; FPV intentionally hides it. Serving a different copy of the instructor's website will not include our overlay.

### For controller testing: Keyboard-to-Myo Test

Teammates can use the separate [3.2aTest guide](controllers/experimental/keyboard-demo/README.md). It generates synthetic EMG/IMU samples from keys and runs the actual 3.2a controller, including its forward LiDAR speed limits. Its keyboard feel differs from native controls: releasing keys requests relaxed, slow forward motion rather than stopping.

This test tool is retained for development. It is not the recommended interface-only demo. Real-Myo and neutral-preview instructions are in the [experimental documentation](controllers/experimental/README.md).

## What the project implements

- Eight-channel EMG effort processing and per-wearer calibration.
- IMU orientation processing, dead zones, smoothing and velocity slew limits.
- A nominal 20 Hz control loop using LSL input and WebSocket commands.
- Piecewise forward speed limits based on LiDAR clearance.
- Paired calibration and driving logs for each session.
- A JavaScript steering-wheel overlay on the simulated robot, with chase and top views confirmed by the project owner.

Forward LiDAR limiting is not autonomous navigation or full collision avoidance. Competition 3.2 treats a missing scan as clear and does not protect reverse travel with rear LiDAR. The experimental variants add checks but still require real Myo validation. See the [implementation and limitations](docs/IMPLEMENTATION_AUDIT.md) and [validation record](docs/VALIDATION.md).

## Find your way around

| Location | Contents |
| --- | --- |
| `controllers/competition/` | Final competition 3.2 |
| `controllers/experimental/` | 3.2a and Keyboard Demo, each with its own instructions |
| `controllers/archive/` | Historical controllers and original teammate safety enhancement |
| `jackal-site/` | Simulator website and the active steering-wheel overlay |
| `data/` | Gesture recordings and complete driving sessions; see the [data guide](data/README.md) |
| `tests/` | Automated behavior and interface checks |
| `docs/` | [Architecture](docs/ARCHITECTURE.md), [version history](docs/VERSIONS.md), setup and validation |
| `archive/` | Unused classification experiment and original documents/component copies; see [archive notes](archive/README.md) |
| `Project-Presentation.pdf` | Hackathon presentation |

## Attribution

The instructor's project supplies the simulator, bridge, Myo publisher and controller examples. This repository preserves the team's controller iterations, recordings, driving logs and steering-overlay customization. The simulator is bundled build output, not a claim of team authorship of the entire simulator. See [provenance](docs/PROVENANCE.md) for component attribution. No new license is assigned to inherited materials.
