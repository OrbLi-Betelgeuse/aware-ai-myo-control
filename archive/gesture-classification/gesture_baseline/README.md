# Shima EMG gesture baseline — pilot v1

This package trains and evaluates a gesture classifier. It does not drive the robot.
All 42 current recordings belong to Shima, as confirmed by Orb.

## Algorithm

- Read the eight columns EMG_1 through EMG_8 in that order, at nominal 200 Hz.
- Use 40-sample windows (nominal 200 ms), moving by 10 samples (50 ms).
- Extract raw RMS separately for each channel: 8 features. No timestamp, filename, or recording ID is a feature.
- StandardScaler -> LinearDiscriminantAnalysis(solver="lsqr", shrinkage="auto").
- Use equal class priors; recording counts are unbalanced.
- Fit preprocessing on each training fold only.
- Validate with five folds stratified at the recording-file level. All overlapping windows from one file stay together.
- Retrain on all recordings to produce outputs/model.joblib. This final model is for later NEW trials, not a new evaluation on its training data.

The ZIP may be read directly; no manual extraction is needed. Folder names provide labels because every CSV's internal label is currently "data".

## Installation

Use the existing jackal-runtime environment on Orb's computer, or on another machine:

```bash
python -m pip install -r requirements.txt
```

Use Python 3.11 and the versions in requirements-lock.txt for the closest reproduction.
Native liblsl may be required on some machines for live LSL; the supplied hackathon Conda environment includes it.

## Train again

Run from this folder (gesture_baseline):

```bash
python train.py --data ../Myodata.zip
```

On Orb's current machine the installed interpreter is:

```text
/Users/betelgeuseorb/Documents/Codex/2026-10-22/new-chat/jackal-runtime/bin/python
```

Replace `python` in the commands with this absolute path if that environment is not activated.
Add or replace data, keep the format consistent, then rerun training. Replacing the ZIP alone does not update the model.
A source directory can also be passed instead of a ZIP.

Required folder layout, optionally nested under a common root:

```text
back/*.csv
forward/*.csv
left/*.csv
right/*.csv
```

Optional future neutral recordings: add a rest folder AND add "rest" to config.json labels, then retrain.
For new gestures, update the labels and downstream controller mapping too.
For a new participant, update participant_id when training a participant-specific dataset. This version does not implement multi-person LOSO or transfer learning.

The loader assumes each CSV contains one intended gesture throughout. It trims 250 ms from each edge (by nominal sample count), which does NOT establish that remaining samples are correctly labeled. Relabel/split files if they contain substantial rest or gesture transitions.
It rejects missing channels, nonfinite data, duplicate file bytes, inconsistent declared sample rates, missing classes, and recordings with no usable windows.

## Outputs

- outputs/model.joblib: fitted classifier, scaler, labels, channels, and feature settings.
- outputs/RESULTS.txt: readable results and limitations.
- outputs/metrics.json: scores, per-fold metrics, class metrics, matrix, and settings.
- outputs/confusion_matrix.csv: rows = actual; columns = predicted.
- outputs/fold_assignments.csv: reproducible file-level split assignments.
- outputs/oof_predictions.csv: held-out window predictions.
- outputs/recording_audit.csv and data_audit.json: counts, sampling/timestamp checks, hashes, participant ID.

Only load joblib artifacts you trust. This package's model is generated locally by train.py.

## Offline replay (no armband)

```bash
python predict.py --replay path/to/one_recording.csv
```

Outputs one JSON prediction per window. Replaying training recordings checks plumbing, not generalization.

## Live predictions (on the computer receiving the Myo stream)

Close the keyboard teleop controller before integrating any new robot controller.
Start the official Myo publisher first, then:

```bash
python predict.py
```

The predictor refuses to choose arbitrarily if multiple Myo_EMG streams exist. Use --source-id to select the intended publisher, or --stream-name for a deliberately different stream name.
It checks channel labels/order and nominal rate; reports invalid input if there are insufficient or stale samples. Time correction is requested from LSL at connection time.
This interface has been tested with a synthetic local LSL stream, not yet with Shima's live device.

Optional output stream:

```bash
python predict.py --lsl-output
```

GesturePrediction publishes `[class_index, confidence, valid]` at 20 Hz.
The class-label array is stored in the stream metadata under class_labels_json. Use it rather than assuming a numeric label mapping.
Invalid output is `[-1, 0, 0]`. On disconnect, a consumer MUST also independently detect a stale prediction stream and output zero commands; a final invalid sample is not guaranteed after a crash.
`confidence` is the model's uncalibrated probability, not a proven correctness or unknown-gesture estimate.

**Do not connect GesturePrediction directly to controller_template.py's default two-number (v,w) mapping.** Its values are gesture index/confidence/valid, not velocity. A separate gesture-to-motion controller is still required, including brake/reverse states, safe stopping, and command smoothing. No robot command is sent by predict.py.

## Current limitations and next experiment

- No rest/neutral class: the four-way classifier can label rest or unfamiliar motions as a movement, possibly with high confidence. Do not infer safe stopping from the four-class accuracy.
- Only Shima: no cross-user validation or adaptation is implemented/evaluated.
- Assumption: one CSV is one independent action trial. Recordings in a single session are still related.
- Collection occurred in class-specific time blocks. Time, armband shift, fatigue, or session effects may inflate file-held-out performance. Collect interleaved new trials to test this.
- Many timestamps move backward. Do not sort rows to manufacture chronology. The pipeline preserves order and uses RMS (order-invariant within each window).
- Some recordings have fewer samples per second than nominal. A 40-sample window can span longer than 200 ms. Large forward-gap windows are excluded, but timing quality is not repaired.
- Edge trimming is an unvalidated initial setting.
- Small trial counts and correlated overlapping windows mean thousands of windows do not equal thousands of independent observations.
- Smoothing, temporal confirmation, movement mapping, and physical stopping response are not implemented by this classifier package.

Next: test NEW interleaved Shima trials, including neutral and transitions; inspect live predictions before connecting a controller. Then test the complete system in L0. Add other users only after the initial mapping is usable.
