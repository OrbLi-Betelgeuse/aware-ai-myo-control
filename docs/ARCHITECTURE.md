# Architecture

## Runtime control path

```mermaid
flowchart LR
  A[Myo Armband] --> B[External Myo LSL publisher]
  B --> C[Myo_EMG: 8 channels]
  B --> D[Myo_IMU: quaternion]
  C --> E[Calibration + windowed RMS effort]
  D --> F[Calibrated orientation projections]
  E --> G[Rule-based controller + smoothing]
  F --> G
  H[Browser Jackal simulator] --> I[External jackal-bridge /state]
  I --> J[LiDAR forward clearance + speed cap]
  J --> G
  G --> K[WebSocket /cmd: v, w]
  K --> L[External jackal-bridge]
  L --> H
  G --> M[Calibration JSON + drive CSV]
```

The controller connects to `ws://localhost:8765/state` and `/cmd`. Commands include `type`, linear `v`, angular `w`, source, sequence, and timestamp. The state supplies scan geometry, pose, status, collisions, and course progress. The bundled site is served separately from Python control logic. The steering overlay reads simulator hooks and applied angular command; it is visual feedback, not a controller.

For 3.2, 40 EMG samples and 10 IMU samples are buffered (nominally 200 ms at 200/50 Hz). Per-channel mean is removed before RMS; mean channel RMS provides effort. Quaternion `[qw,qx,qy,qz]` is converted into an averaged normalized up-vector. Calibration defines neutral orientation, left/right projection axis, orthogonalized lift axis, and relaxed/fist effort references. This is calibrated signal processing and rule-based control, not training the offline LDA model.

## Separate machine-learning path

```mermaid
flowchart LR
  A[Myodata.zip: labeled recording files] --> B[40-sample raw RMS windows / hop 10]
  B --> C[Recording-file stratified folds]
  C --> D[StandardScaler + shrinkage LDA]
  D --> E[Held-out predictions / metrics]
  B --> F[Final fit on all recordings]
  F --> G[model.joblib]
  G --> H[predict.py: replay or live EMG]
  H --> I[JSON / optional GesturePrediction LSL]
```

There is no implemented connection from `GesturePrediction` to the documented robot controllers. The classifier emits class/confidence/valid, not `(v,w)`. Its raw RMS features differ from the controller's mean-centered RMS. Any future integration requires an explicit mapping, neutral/rest handling, freshness checks, and system-level validation.
