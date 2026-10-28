# Implementation audit - 2026-10-27

Scope: remote main `aba0fedfb61d4b6f3577b158fb42c8c4698ed528`, compared with the original local checkout at `4b79bcd`. This audit describes code behavior, not a certified physical-robot safety assessment. The owner confirmed 3.2 as the final competition version. Safety changes were not integrated in time for the competition. The comparisons below concern the historical implementations. The later 3.2a integration is documented separately in [SAFETY_3_2A.md](SAFETY_3_2A.md). Physical deployment remains unconfirmed. The organization pass changes only the competition script's run/path references and log destination; its control logic and the simulator code are unchanged.

## Inputs and control (3.2)

| Area | Verified implementation | Limits |
| --- | --- | --- |
| EMG | `channel_rms`, `my_input`: 40 samples, per-channel mean removal, RMS averaged over channels | Assumes stream shape; lacks enhanced version's finite/channel/timestamp checks |
| IMU | `gravity`: first 4 channels interpreted as w,x,y,z; 10-sample up-vector average | Not an independent raw gyro/accelerometer fusion algorithm; quaternion quality depends on publisher/device |
| Calibration | Five pose references, effort/geometry checks, complete retries | Same-session calibration is not held-out ML validation |
| Steering | Calibrated projection, 25% dead zone, squared response, low-pass and slew limits | User steering; no autonomous obstacle steering |
| Speed | 0.4 m/s relaxed target, maximum 2.0; reverse target -0.6 m/s | Targets, not measured achieved speeds; relaxed is not stop |
| Timing | Async loop targets 20 Hz; Myo receipt gap 0.25 s; state receipt gap 0.50 s | Nominal scheduling, not guaranteed real-time deadlines or measured latency |
| Recovery | Neutral arming, buffers reset after loss, reconnect loop, best-effort zero on exit | Receipt freshness differs from source timestamp freshness; crash delivery is not guaranteed |

Averaging electrode RMS is invariant to channel permutation. It does not prove invariance to arbitrary armband displacement, skin contact, or new users. Earlier comments asserting rotation robustness or measured IMU agreement are historical notes, not independently reproduced measurements.

## LiDAR policies differ

`path_clearance` projects polar beams into forward `x=r*cos(angle)` and lateral `y=r*sin(angle)`. It takes the minimum positive forward coordinate within `abs(y)<0.30 m`. This is a straight corridor approximation, not an arc following the commanded turn, a full footprint planner, or rear collision detection.

- **3.2:** `np.interp` distance knots `[0.20, 0.50, 0.93, 1.55]` m and caps `[0, 0.70, 1.50, 2.00]` m/s. Cap reduction is immediate at the command level; cap release increases at 0.5 m/s². Physical stopping is not instantaneous.
- **3.1:** `min(2.0, 0.8 * max(0, distance - 0.30))`, with the same gradual release.
- **Enhanced 2.6:** linear policy, plus scan/schema/finite-value checks, source timestamp checks, stale state/loop guards, terminal-event handling, and neutral rearming. Do not attribute these checks to all versions.

### Known issues retained for a separate behavioral change

1. 3.2 returns 10 m for a missing scan. Zero-length/invalid ranges can also be treated as clear, ignored, or raise rather than reliably stopping. This is a known fail-open scan behavior.
2. 3.2 ignores LSL source timestamps and validates freshness from chunk receipt. Buffered old samples can appear fresh.
3. 3.2 only explicitly handles `estop` in `step`; it does not implement the enhanced variant's terminal course validation.
4. LiDAR only caps forward linear command. Reverse is not protected; angular motion may remain nonzero at a zero forward cap.
5. A zero-command transition tick is not evidence that the robot has stopped before reversing.
6. Enhanced 2.6 validates a specific scan format (180 finite beams); alternate sensors/protocols can be rejected. No live validation was performed in this audit.
7. The ML training report prints a literal class-count expression in `RESULTS.txt`; structured `metrics.json` contains the actual counts. The historical source is preserved unchanged.
8. Some script headers/run examples/version strings are inherited and stale. Use the paths in the new run guide.

## Machine learning boundary

`archive/gesture-classification/gesture_baseline/train.py` trains `StandardScaler` followed by shrinkage `LinearDiscriminantAnalysis(solver="lsqr", shrinkage="auto")`, with equal class priors. It fits the scaler within each training fold and splits by recording files before window selection. Raw RMS gives 8 features from 40-sample windows with a 10-sample hop. It finally fits all data and writes a separate model artifact.

The data comprises four gesture folders. Participant identity is recorded as Shima in historical package metadata; this audit does not independently identify the wearer. There is no rest class, cross-user validation, or implemented classifier-to-robot command path. Optional calibrated open-hand template matching in the controllers is distinct from this LDA pipeline and disabled by default.

Historical metrics remain in the extracted package for traceability. Portfolio outcome summaries and resume claims are intentionally deferred at the owner's request.

## Verification

See [validation record](VALIDATION.md) for the checks actually run in this audit. Static inspection, offline model reproduction, and synthetic control probes do not establish hardware behavior or live timing.
