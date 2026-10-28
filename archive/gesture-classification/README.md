# Archived gesture-classification experiment

This teammate-provided package was not used by the final competition controller. Original authorship remains unconfirmed. It is not the Myo device's built-in classifier and is not required to run controller 3.2.

`gesture_baseline.zip` is the original archive. `gesture_baseline/` is its byte-identical expansion, including historical source, model and outputs. Do not interpret the stored evaluation as robot-control performance. The historical package README contains old relative paths and a local interpreter path.

For optional offline inspection, from `archive/gesture-classification/gesture_baseline/` in an environment with its requirements:

```bash
python -m unittest test_baseline
python train.py --data ../../../data/gesture-recordings/Myodata.zip --out /tmp/aware-ai-baseline-new-run
python predict.py --help
```

Use a new output directory to preserve historical artifacts. The classifier has no neutral/rest class, no demonstrated cross-user validation and no connection to final robot commands. The data ZIP remains separate in `data/gesture-recordings/`; neither the data nor the original experiment files were edited.
