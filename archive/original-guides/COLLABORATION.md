# Team collaboration

This repository preserves the October 22–23 hackathon project folder, including recorded data, historical controllers, slides, and driving logs. The root README describes the earlier ML baseline inside `gesture_baseline.zip`.

## Current controller

`2.0/my_team2_2.py` is the 2.2 controller. Earlier versions are retained for comparison. The controller uses per-user calibration and rules; the separate ML baseline is not automatically used by it.

The Myo publisher, jackal-bridge, simulator site and Python environment are external prerequisites; this folder is not a complete runtime installation. Use the official hackathon setup for those components.

On Orb's existing setup, keep the publisher and bridge in separate terminals. In a third terminal run:

```bash
cd /Users/betelgeuseorb/Documents/Codex/2026-10-22/new-chat
./jackal-runtime/bin/python aware-ai-myo-control/2.0/my_team2_2.py
```

Only run one controller at a time. Stop the old controller with Ctrl+C before launching a new version. Calibrate for the current wearer. Relax means slow forward in this version, not stop; use the simulator E-stop or Ctrl+C to stop.

## Working together

1. Clone this private repository after accepting an invitation.
2. Pull the latest main branch before starting work.
3. Create a branch for each change, such as `fix/steering`.
4. Commit and push the branch, then open a pull request.
5. Include what changed and how it was tested; merge after a teammate reviews it.

Do not commit environments, credentials, or cache files. Keep the included recording and result files as historical data. Label new recordings with participant/session and gesture information.

This Git checkout is a copy of the original iCloud folder. Editing the iCloud copy does not update this repository automatically. Work in the checkout or explicitly copy intended changes into it before committing.
