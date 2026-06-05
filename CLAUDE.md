# CLAUDE.md — teleop

Submodule of `SCM/`. Do not commit changes that span sibling repos.

User-facing setup, launch params, calibration, and recording workflow: `README.md`.

## Package layout

```
src/
├── input/   vive_ros2, manus_ros2, pedal_ros2
├── core/    vive_rby1 (C++ prod), manus_inspire, rby1_ik (Python debug only)
├── gui/     scm_gui
├── launch/  teleop_bringup
└── msgs/    rby1_core_msgs, scm_recording_msgs, manus_ros2_msgs, inspire_hand_msgs
             (interbotix_xs_msgs: COLCON_IGNORE — must be system-installed)
```

`vive_rby1_node` (C++) is the only production node — default launch only starts this. `vive_rby1_debug_node` (Python) is dev-only.

## Common gotchas

- **`inspire_hand_msgs` ABI mismatch** → same package in `teleop/src/msgs/` and `hw-core/inspire_hand/src/`. Keep `.msg` files identical.
- **`tf2_msgs/TFMessage` on `/rby1/cmd/pose` is not a TF broadcast** → topic is not under `/tf*`, TF subsystem ignores it intentionally.
- **Mirror mode uses LEFT tracker for RIGHT arm** — intentional, not a bug.
- **SDK Z-offset** (+3.9 cm left, +2.6 cm right): compensated at engage via `sdk_ee_*_0_` captures. Don't change without recalibrating.
- **`rby1_ik` package** is only used by the Python debug node. No production dependencies on it.
- **`empy==3.3.4`** required — 4.x breaks colcon silently.

## Reference docs

| Doc | Contents |
|---|---|
| [`docs/vive_rby1.md`](docs/vive_rby1.md) | vive_rby1_node: coordinate frames, engage/disengage, mirror mode, step clamp, warmup |
| [`docs/manus_inspire.md`](docs/manus_inspire.md) | Finger flex mapping, calibration logic, mirror mode |
| [`docs/scm_gui.md`](docs/scm_gui.md) | Threading model, node monitoring, service clients, config files |
| [`docs/input_nodes.md`](docs/input_nodes.md) | vive_ros2 (OpenVR→ROS transform, station alignment), pedal_ros2, manus_ros2 |
| [`docs/CHANGES.md`](docs/CHANGES.md) | Changelog |
