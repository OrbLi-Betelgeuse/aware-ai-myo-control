# Data guide

These are two existing kinds of data, organized by purpose. Moving directories does not modify samples or divide a test session.

## Gesture recordings

`gesture-recordings/Myodata.zip` contains timestamped eight-channel EMG recordings under gesture folders. The independent classification archive uses them as input; the final controller does not read this ZIP while driving. The original ZIP is preserved intact, including original metadata. Historical metadata identifies Shima as the participant; personal identity was not independently verified in the repository audit.

## Driving sessions

Each `driving-sessions/<timestamp>/` directory is one historical test/calibration session:

- `calibration.json`: relaxed/fist effort references, orientation axes/ranges, and calibration diagnostics.
- `drive_log.csv`, when present: time, input summaries, arming state, velocity commands, forward clearance, simulator pose/course/status, and events. Schemas vary by controller iteration.

Keep these files together. Some sessions contain calibration only; do not invent a missing drive log or assume the trial completed. The exact generating controller version is unknown unless independently recorded; timestamp proximity alone does not establish it.

`v_cmd` and `w_cmd` are commands, not measured physical velocity. Differences of commands estimate command acceleration, not actual robot acceleration. Position-derived motion estimates require valid timestamps, handling gaps and appropriate filtering.

The competition entry point writes new sessions here too. Session names use the execution computer's local clock; do not infer UTC from their names. Existing files are never overwritten by this migration. Review recordings before publication and record script commit, environment and test scenario for future runs.
