# Repository map

See the root README for the active directory layout. The supported entry point is `controllers/competition/my_team3_2.py`.

- Historical control iterations retain their original filenames under stage folders. No historical algorithm was rewritten.
- `data/gesture-recordings/Myodata.zip` is the unchanged original gesture recording archive.
- `data/driving-sessions/` retains entire timestamped session folders. Five sessions omitted by the earlier remote cleanup were recovered from the original checkout. Calibration-only sessions remain calibration-only; missing logs were not fabricated.
- `archive/gesture-classification/gesture_baseline/` is the unchanged expansion of the adjacent original ZIP. Source hashes are recorded in `docs/baseline-source-manifest.json`; its keys are paths relative to the classification archive directory.
- `jackal-site/wheel-overlay.js` is the active overlay. The unchanged standalone copy is retained in `archive/standalone-wheel-overlay/` for historical reference, not as a second maintained source.
- `archive/original-guides/` preserves previous documentation. Embedded paths and status claims may be obsolete; use `docs/` for current instructions.
- `Project-Presentation.pdf` at the repository root is the competition PDF already present in remote main.
- The original safety file is preserved unchanged in `controllers/archive/safety-donor/`. Selected checks are adapted in the separate `controllers/experimental/3.2a/my_team3_2a.py`; see `docs/SAFETY_3_2A.md`.

Local-only commits, presentations, enhanced variants and the original full checkout remain in a verified backup outside the repository. Local filesystem migration manifests are also kept outside this public-facing documentation. No original checkout was deleted or reset.
