# Version history

The original v1.0/v2.0/v3.0 labels describe hackathon work stages, not formal semantic-versioned releases. Original script names are retained within clearer stage folders.

| Stage | Meaning confirmed by owner | Current location |
| --- | --- | --- |
| v1.0 | First evening's prototype | `controllers/archive/day1-evening/` |
| v2.0 | Second morning's iterations | `controllers/archive/day2-morning/` |
| v3.0 | Final testing and competition stage | 3.1 in `controllers/archive/final-testing/`; 3.2 in `controllers/competition/` |

**3.2 was used for the final competition demonstration.** Its two principal differences from 3.1 are the piecewise LiDAR distance/speed table and its use in the cap calculation. Both have gradual speed-cap release.

Archived controller source files remain byte-identical to the pre-organization files. Their embedded run examples and relative log paths can be outdated after relocation; they are preserved as historical reference, not supported entry points. Use the competition controller for the documented runtime. The old Pygame window helper is retained with the second-morning files.

3.2 control logic is unchanged. Its output directory now resolves from the repository root to `data/driving-sessions/<timestamp>/`; its docstring run/path examples were updated. Each session still writes calibration and drive log together.

## Experimental 3.2a

`controllers/experimental/3.2a/my_team3_2a.py` is a separate safety variant based on 3.2, integrating selected checks from the older `controllers/archive/safety-donor/my_team2_6_lidar_safety_enhanced.py`. The original competition and donor files remain unchanged by this integration. The owner clarified that the teammate's enhancement targeted an older controller.

3.2a preserves the competition motion law and piecewise LiDAR policy. It adds stricter input/state checks, watchdogs and a three-second neutral preparation gate; 22 controller tests pass, and the owner has reviewed synthetic simulator operation. Real Myo and physical robot testing remain pending. See [integration details](SAFETY_3_2A.md). Enhanced 2.7a/3.1a historical files remain preserved in the original checkout/backup and Git history.

3.2 remains the competition reference. No 3.3 file was created.

## Keyboard Demo (3.2aTest)

A separate keyboard adapter synthesizes EMG/IMU input for the actual 3.2a controller. It is a no-hardware demonstration, not the competition controller and not a trained ML model. Both experimental modes share a 0.8-second Ready stage, three-second countdown, centered unboxed green GO and explicit stop states. Their webpage uses practice timer pauses; competition timing is unchanged. See [experimental entries](../controllers/experimental/README.md).
