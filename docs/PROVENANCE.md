# Component provenance

Upstream: [AHRTLab/Hackathon](https://github.com/AHRTLab/Hackathon), inspected at commit `f410369286a9ef0ff7ac46abdbedd1983d990591`.

- `examples/myo_lsl_publisher.py` provides Bluetooth-to-LSL EMG/IMU acquisition. It can enable the device classifier for sync and publish pose events; the final team controller consumes EMG/IMU, not these gesture events.
- `examples/myo_emg_example.py` demonstrates RMS/threshold control.
- `examples/controller_template.py` provides controller/bridge integration scaffolding.
- The upstream project supplies the bridge and browser simulator. Team documentation should not attribute the complete simulation stack to the team.

The independent `gesture_baseline.zip` was present in the team's initial import commit `e239189`. The inspected upstream tree does not contain that package. Its original author/delivery path remains unconfirmed; the owner identifies it as teammate-provided. It is retained for provenance, not presented as the competition controller or a Myo driver dependency.

Git authorship establishes a commit record, not sole authorship of every imported file. No new blanket license is assigned to upstream code, assets or teammate materials.
