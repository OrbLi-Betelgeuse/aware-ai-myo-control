# Validation record - 2026-10-27

Executed in the existing external hackathon Python environment: Python 3.11.17, NumPy 2.4.6, scikit-learn 1.9.1, websockets 17.3. No packages were installed or shared runtime files changed.

- All 17 project Python files present before adding the audit script parsed successfully.
- All 18 extracted ML archive members matched the ZIP bytes exactly, including historical outputs.
- `python -m unittest test_baseline` in `archive/gesture-classification/gesture_baseline`: 4 tests passed.
- `python train.py --data ../../../data/gesture-recordings/Myodata.zip --out /tmp/aware-ai-audit-20261027-model`: completed; serialized model reload check passed. Recording/window counts, three aggregate metrics and confusion matrix exactly matched stored `outputs/metrics.json`. Generated files remained outside the repository.
- Before reorganization, 9 synthetic checks passed, including three probes of the deferred safety file. After reorganization, the current `tests/audit_controllers.py` runs only the six competition-controller checks; all six passed. It imports the competition module without starting `main()`, uses a no-op log, and does not connect to hardware or the bridge.
- `git diff --check`: passed.

The nine synthetic checks cover: 3.2 missing-scan fallback, forward corridor distance, near-obstacle zero forward cap, zero direction-transition tick, reverse bypass of the forward cap, stale-input disarming/zero commands, enhanced 2.6 missing-scan rejection, zero-range clearance, and nonfinite-scan rejection.

Passing the missing-scan/reverse probes confirms existing behavior, including known limitations; it does not endorse that behavior. These are focused checks, not exhaustive controller tests. No live Myo acquisition, browser rendering, physical robot operation, or latency benchmark was performed.

## Organization checks

- Every moved historical file matched its pre-move SHA-256. The sole intentional controller change is the final 3.2 script's run/path references and log destination; the complete file was compared against precisely those replacements.
- The deferred safety file is byte-identical to remote main and remains at the same path.
- All 16 original session directories match the old checkout file-for-file; calibration/CSV pairs were not split. Five previously omitted sessions were restored.
- Python syntax and current documentation links passed. The relocated ML package's four existing tests passed, and its loader found the same 42 recordings / 3,164 windows through the new data path.

## Reproduce

Use an environment with `numpy`, `pylsl` (including native liblsl), `websockets`, and the ML dependencies. From repository root:

```bash
python tests/audit_controllers.py
cd archive/gesture-classification/gesture_baseline
python -m unittest test_baseline
python train.py --data ../../../data/gesture-recordings/Myodata.zip --out /tmp/aware-ai-baseline-new-run
```

Do not overwrite the historical `archive/gesture-classification/gesture_baseline/outputs/` when reproducing training.

## Later 3.2a integration

`python tests/test_controller_3_2a.py`: 18 tests passed with mocked time, sensor input and transport. Normal valid-input trajectories matched the unchanged competition 3.2 reference; fault and recovery tests exercised the new checks. Both the competition source and original donor were SHA-256 checked against the hashes embedded in 3.2a. This does not establish live hardware or simulator performance. See [details](SAFETY_3_2A.md).

## 3.2a preparation countdown

22 Python tests passed after adding four preparation tests and updating existing regression checks to start after preparation. The optional JavaScript overlay passed syntax checking and mocked DOM/socket checks with `node tests/test_preparation_overlay.cjs`. No live browser rendering, Myo or simulator reset-timing trial was performed. The 3.2 competition source and original safety donor still match their recorded SHA-256 hashes.

## Final review - current experimental interface

The earlier sections above record checks at earlier stages. Current results:

- All ten scripts under `tests/` used for audit/regression passed: the competition audit, controller checks, synthetic neutral input, keyboard control, connection ownership, preparation status, demo clock, keyboard UI, shared preparation UI and retained legacy overlay.
- 22 experimental-controller tests and 10 keyboard-controller tests passed. Neutral simulation passed its input-to-motion check.
- The owner manually reviewed the no-Myo preview and Keyboard Demo, including countdown presentation and the final direct digit transitions. This is user-reported simulator validation, not assistant-operated browser verification or real Myo validation.
- Every one of 40 deleted historical paths has a byte-identical relocated file. The remaining relocated file is competition 3.2: its complete diff contains only run examples and the log destination change documented above.
- Synthetic session outputs remain local and ignored by Git. Per-session calibration and driving records remain together.
- 3.2a now owns the shared Ready stage and three-second gate. Experiment-only webpage scoring pauses during holds. Real Myo/physical-robot validation remains pending.

The primary entry is [controllers/experimental/README.md](../controllers/experimental/README.md). No push was performed during final review.
