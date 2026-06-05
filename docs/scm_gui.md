# scm_gui — Architecture

Source: `src/gui/scm_gui/scm_gui/scm_gui_node.py`

## Threading model

Two threads: ROS2 node (background) + Qt GUI (foreground, main thread). `Signals(QObject)` is the thread-safe bridge — ROS callbacks emit Qt signals, UI slots consume them.

## Node monitoring

`_poll_nodes()` runs every 1s via ROS timer. Checks `get_node_names_and_namespaces()` against four groups:

| Group | Nodes watched |
|---|---|
| Core | `rby1_core_node` |
| Vision | vision-related nodes |
| Teleop | `vive_rby1_node`, `manus_inspire_node`, `pedal_node`, `vive_tracker_node` |
| Recording | `scm_recording_node` |

## Subscriptions

| Topic | Type | Notes |
|---|---|---|
| `/teleop/pedal` | `Joy` | `buttons[0:3]` → pedals A/B/C |
| `/teleop/rec_state` | `String` | `IDLE/ARMING/READY/RECORDING/PAUSED` |
| `/teleop/rec_episode` | `Int32` | Current episode number |
| `/teleop/tracker_status` | `String` | Parsed as `"L:OK R:OK B:LOST"` |
| `/teleop/clutch_state` | `String` | `ENGAGED/DISENGAGED` |
| `/rby1/state/status` | `String` (JSON) | `power_state`, `servo_state`, `stream_state` (bool), `ctr_type` |
| `/rby1/state/joint` | `JointState` | Cached; fetched on-demand via `request_next_joint_state(cb)` |
| `/rby1/state/battery` | `BatteryState` | |

## Publications

| Topic | Type | Notes |
|---|---|---|
| `/teleop/task_id` | `Int32` | From GUI spinner |
| `/teleop/mirror_mode` | `String` | `'mirror'` or `'normal'` |
| `/rby1/cmd/base_vel` | `Twist` | Keyboard drive at 10 Hz when enabled |

## Service clients

All async via `_call_async()` — background thread, 2s wait-for-service, 30s timeout (connect: 15s).

- **rby1_core**: `connect`, `power`, `servo`, `ctrl_enable`, `err_reset`, `gripper_init`, `stream`, `stop_move`, `ctrl_mode`, `move_joint`, `set_ci_limits`, `set_ns_weight`, `set_ns_ref`
- **vive_rby1**: `set_use_torso`, `teleop_start`, `teleop_stop`, `toggle_clutch`, `set_pose`, `toggle_episode`
- **manus_inspire**: `calibrate`

## Config files

| File | Contents |
|---|---|
| `config/named_poses.yaml` | `{teleop_pose, named_poses{}, robot_model}` |
| `config/impedance_presets.yaml` | `{impedance_presets{name: {joint_limits[], nullspace_weights{right_arm[], left_arm[]}, nullspace_ref}}}` |

## Gotchas

- **Robot model (A/M)** must be set before connect — buttons disabled after connecting.
- **`CALIB_DURATION = 4.0`** hardcoded here and in `manus_inspire.py` — change both.
- **`/rby1/state/joint`** is not subscribed continuously — fetched on-demand via one-shot callback. Don't expect live updates.
- **Recording state** disables certain controls (mode switching locked during active session).
- **Keyboard drive** (W/A/S/D/Q/E): only active when checkbox enabled, publishes at 10 Hz regardless of key hold frequency.
- Joint display: 20 joints in 3 columns — torso[0:6], right_arm[6:13], left_arm[13:20].
