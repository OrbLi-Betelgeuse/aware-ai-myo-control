# Experimental versions

The competition version remains **3.2**. Choose one experimental workflow:

| Version | Use it for | Instructions |
| --- | --- | --- |
| 3.2a | Real Myo input and experimental safety control | [3.2a guide](3.2a/README.md) |
| 3.2a preview | View the interface without Myo; synthetic neutral input drives straight | [Preview setup](3.2a/PREVIEW.md) |
| Keyboard Demo (3.2aTest) | Try steering, reverse and LiDAR limits without Myo | [Demo setup and buttons](keyboard-demo/README.md) |

Run only one controller and one simulator page at a time. Both experimental modes share Ready, 3/2/1, the centered green GO text, explicit stop states and practice timer pauses. Practice timing is not comparable to competition timing.

## Start from the repository root

Run these in separate terminals, without starting duplicate services:

```sh
sh controllers/experimental/start.sh bridge
```

Then choose **one** controller:

```sh
# No-Myo straight-driving preview
sh controllers/experimental/start.sh controller
```

```sh
# Keyboard Demo
sh controllers/experimental/start.sh keyboard
```

See the selected version's guide for its browser URL and controls. Use `AWARE_RUNTIME` to select an installed runtime on another machine.

Each version folder contains its actual program and instructions. There are no forwarding `run.py` copies. Automated checks live in `tests/`. The simulator's built-in WASD controls are a separate path and do not run our controller; Keyboard Demo runs it using synthetic EMG/IMU.
