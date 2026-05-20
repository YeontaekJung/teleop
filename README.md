# teleop

ROS2 Humble workspace for bimanual teleoperation using motion capture hardware.
Currently supports RB-Y1 robot with Vive Trackers (arm control) and Manus gloves (hand control).
Designed to be extensible — additional robot platforms and input devices can be added as new packages.

## Architecture

```
Input              Core                      Output
──────────────────────────────────────────────────────────────────────
manus_ros2      →  manus_inspire          →  /rt/inspire_hand/ctrl/{l,r}   (Inspire Hand driver)
vive_ros2       →  vive_rby1_node (C++)   →  /rby1/cmd/pose  (sdk_impedance/position → rby1_core_node)
                   vive_rby1_debug_node   →  /rby1/cmd/joint (pink_position/impedance, debug only)
                   (Python, manual only)  →  /rby1/cmd/pose  (sdk modes)
pedal_ros2      →  vive_rby1 state machine→  /scm_recording/{start,end,toggle_pause}
GUI             →  teleop_gui_node        →  /rby1/ctrl/mode, /rby1/stream services
```

All msg/srv/action definitions are under `src/msgs/`.

A GUI node (`teleop_gui`) provides live system status, teleop controls, tracker status, and calibration.

## Published Topics

### rby1_core_node (hw-core)

| Topic | Type | Description |
|-------|------|-------------|
| `/rby1/state/status` | `std_msgs/String` | JSON 시스템 상태 (control/power/servo/stream/gripper/ctr_type) |
| `/rby1/state/joint` | `sensor_msgs/JointState` | 전체 joint 실제값 (position / velocity / torque), 항상 100 Hz |
| `/rby1/state/ee_pose` | `geometry_msgs/PoseArray` | SDK FK 기반 EE pose — poses[0]=ee_right, poses[1]=ee_left (base frame) |
| `/rby1/cmd/joint_ik` | `sensor_msgs/JointState` | SDK IK reference (CartesianImpedance 스트림에서만 발행) |

### vive_rby1_node

| Topic | Type | Description |
|-------|------|-------------|
| `/rby1/cmd/pose` | `rby1_core_msgs/LinkPoseCommand` | sdk_position / sdk_impedance EE target (C++ 프로덕션 노드) |
| `/rby1/cmd/joint` | `sensor_msgs/JointState` | pink_position / pink_impedance joint 명령 (Python 디버그 노드만) |
| `/teleop/rec_state` | `std_msgs/String` | 녹화 상태 (IDLE / ARMING / READY / RECORDING / PAUSED) |
| `/teleop/rec_episode` | `std_msgs/Int32` | 현재 에피소드 번호 |
| `/teleop/tracker_status` | `std_msgs/String` | 트래커 상태 (L:OK/JITTER/LOST R:OK/JITTER/LOST) |
| `/teleop/clutch_state` | `std_msgs/String` | clutch engaged / disengaged |

## Hardware Requirements (Current Setup)

| Hardware | Role |
|----------|------|
| Rainbow Robotics RB-Y1 | Teleoperated robot |
| HTC Vive Tracker 3.0 × 2 | Arm pose tracking |
| HTC Vive Base Station × 2 | SteamVR tracking |
| Manus Prime X Haptic gloves | Hand/finger tracking |
| Inspire Hand × 2 | Robot hands |
| PCsensor FootSwitch (3-pedal) | Clutch / mode control |

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble (`ros-humble-desktop`)
- SteamVR running with Vive Trackers paired before launching
- ManusSDK (see Installation step 2)
- `rby1_core` package installed (provides `Rby1ControllerJointTeleop`, `Rby1ControllerJointImpedanceTeleop`, etc.)

## Quick Start

Start SteamVR and pair all Vive devices, then:

```bash
source install/setup.bash
ros2 launch teleop_bringup teleop.launch.py
```

This launches all nodes: pedal driver, Vive tracker, Manus publisher, arm IK bridge, hand mapper, and GUI.

The GUI shows live node status, pedal state, tracker status (OK / JITTER / LOST), recording state, control mode selector, teleop buttons, and the calibration panel.

## Installation

### 1. Clone the repository

```bash
git clone <repo-url> teleop
cd teleop
```

### 2. ManusSDK (manual copy required — binary too large for git)

Copy the ManusSDK folder to the repo root:

```
teleop/
└── ManusSDK/
    ├── include/
    │   ├── ManusSDK.h
    │   ├── ManusSDKTypeInitializers.h
    │   └── ManusSDKTypes.h
    └── lib/
        ├── libManusSDK.so
        └── libManusSDK_Integrated.so
```

Obtain from the Manus developer portal, or copy from a machine that already has it.

### 3. System dependencies

```bash
sudo apt install ros-humble-desktop python3-pip
```

```bash
pip3 install pin pink scipy openvr evdev PySide6
pip3 install empy==3.3.4   # required for colcon build — do NOT use empy 4.x
```

> **Pedal (evdev) — one-time setup:**
> The pedal driver requires read access to `/dev/input`. Add your user to the `input` group:
> ```bash
> sudo usermod -aG input $USER
> ```
> Then **fully log out and log back in** (closing the terminal is not enough).
> Verify with: `groups` — `input` should appear in the list.

> **Conda users:** Deactivate conda before building — its Python conflicts with ROS2.
> ```bash
> conda deactivate
> which python3  # should be /usr/bin/python3
> ```

### 4. Build

```bash
cd teleop
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> **Note:** `interbotix_xs_msgs` is included in the repo for reference but excluded from build
> (`COLCON_IGNORE` present) — it is assumed to be installed system-wide.

## Configuration

### Vive Tracker serial numbers

Edit `src/input/vive_ros2/vive_ros2/vive_tracker_node.py` (or pass as ROS parameters):

```python
self.declare_parameter('serial_station_left',  'LHB-XXXXXXXX')
self.declare_parameter('serial_station_right', 'LHB-XXXXXXXX')
self.declare_parameter('serial_tracker_left',  'LHR-XXXXXXXX')
self.declare_parameter('serial_tracker_right', 'LHR-XXXXXXXX')
```

Find serials via SteamVR → Devices menu, or:
```bash
ros2 run vive_ros2 vive_tracker_node  # serials printed on connect
```

### URDF / SRDF paths

Edit the default paths in `src/core/vive_rby1/config/vive_rby1.yaml`, or pass at runtime:

```bash
ros2 run vive_rby1 vive_rby1_node --ros-args \
  -p urdf_path:=/path/to/rby1.urdf \
  -p srdf_path:=/path/to/rby1.srdf
```

### IK / Teleop tuning parameters

Adjustable in `src/launch/teleop_bringup/launch/teleop.launch.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate` | 100.0 Hz | IK command publish rate |
| `ik_dt` | 0.05 s | Differential IK time step (larger = faster tracking, more overshoot) |
| `pos_scale` | 0.5 | Tracker-to-robot position scale (hand trackers) |
| `torso_pos_scale` | 1.0 | Body tracker position scale (torso) |

`max_teleop_dq` (joint velocity clamp, 1.5 rad/s) is hardcoded in `src/core/vive_rby1/src/vive_rby1_node.cpp`.

## Usage

### Pedal Mapping

| Pedal | Function |
|-------|----------|
| A (left) | Toggle arm engage / disengage |
| B (center) | Spare |
| C (right) | Toggle Start / End recording episode |

### Control Modes

Select in the GUI before starting a session (locked during active recording):

| Mode | Topic | Description |
|------|-------|-------------|
| SDK Impedance | `/rby1/cmd/pose` | Cartesian impedance targets (C++ node, default launch) |
| SDK Position | `/rby1/cmd/pose` | Cartesian position targets (C++ node) |
| Pink Position | `/rby1/cmd/joint` | Joint position tracking via differential IK (Python debug node only) |
| Pink Impedance | `/rby1/cmd/joint` | Joint impedance tracking via local IK (Python debug node only) |

### Teleoperation (without recording)

Use the GUI **Teleop** panel buttons directly:

| Button | Action |
|--------|--------|
| ▶ Teleop Start | Set control mode → start stream (`/rby1/ctrl/mode` + `/rby1/stream` services) |
| Ready Pose | Move robot to ready configuration (`/rby1/move_to_joint_position`) |
| VLA Pose | Move robot to VLA home pose |
| ■ Teleop Stop | Stop stream |

### Recording Workflow

Requires `scm_recording` core (`/scm_recording/*` services) and `rby1_core_node` to be running.

1. Select `task_id` in the GUI Recording panel.
2. Select control mode: **SDK Impedance** (default) or **SDK Position**.
3. Click **▶ Start Episode** (or press pedal C) — system enters **ARMING** (sets control mode → moves to ready pose → starts stream), then **READY**.
4. **Press pedal A** to engage arm → recording starts (RECORDING).
5. **Press pedal A** to disengage → recording pauses (PAUSED).
6. Repeat steps 4–5 to collect data across multiple engage cycles.
7. Click **■ End Episode** (or press pedal C) when PAUSED — episode saved, robot returns to IDLE.

Recording states:

| State | Color | Meaning |
|-------|-------|---------|
| IDLE | grey | No active session |
| ARMING | blue | Transient: setting mode + moving to ready pose + starting stream |
| READY | yellow | Session started, waiting for arm engage |
| RECORDING | red | Arm engaged, data being recorded |
| PAUSED | orange | Arm disengaged, session still active |

### Tracker Status

The GUI Node Status panel shows live tracker health:

| Status | Color | Meaning |
|--------|-------|---------|
| OK | green | Tracker data arriving normally |
| JITTER | yellow | High position variance detected (> 3mm σ over last 20 samples) |
| LOST | red | No data received for > 0.5s |

### Manus Hand Calibration

Finger sensor ranges are calibrated per session and saved to `~/.ros/manus_inspire_calib.yaml`.

- **First launch:** calibration starts automatically if no saved file is found.
- **Recalibrate:** click **Recalibrate** in the GUI, or call the service:
  ```bash
  ros2 service call /manus_inspire/calibrate std_srvs/srv/Trigger
  ```

Calibration procedure (16 seconds total, 4 seconds per phase):

| Phase | Pose | Calibrates |
|-------|------|-----------|
| 1 | Open hands fully | Finger min + thumb spread min |
| 2 | Thumbs up (fist, thumb pointing up) | Finger max + thumb MCPStretch max |
| 3 | Press thumb to side of index finger | Thumb spread max |
| 4 | Open fingers, bend thumb only | Thumb MCPStretch min |

> Delete `~/.ros/manus_inspire_calib.yaml` to force recalibration on next launch.

## Individual Nodes (Advanced)

Run components separately for development or partial setups:

```bash
# Input
ros2 run pedal_ros2 pedal_node
ros2 run vive_ros2 vive_tracker_node
ros2 run manus_ros2 manus_data_publisher

# Core
ros2 run vive_rby1 vive_rby1_node
ros2 run vive_rby1 vive_rby1_debug_node   # optional legacy Python debug node
ros2 run manus_inspire manus_inspire_node

# GUI
ros2 run teleop_gui teleop_gui_node
```

## Packages

| Package | Layer | Description |
|---------|-------|-------------|
| `pedal_ros2` | input | PCsensor FootSwitch → `sensor_msgs/Joy` on `/teleop/pedal` |
| `manus_ros2` | input | Manus glove SDK → ROS2 (C++) |
| `vive_ros2` | input | Vive Tracker 3.0 → `/teleop/tracker/left\|right` |
| `manus_ros2_msgs` | msgs | Manus glove message types |
| `inspire_hand_msgs` | msgs | Inspire hand message types |
| `scm_recording_msgs` | msgs | Recording core service definitions |
| `rby1_core_msgs` | msgs | RB-Y1 core srv/msg definitions (10 services + `LinkPoseCommand`) |
| `manus_inspire` | core | Manus glove data → Inspire hand commands + 4-phase calibration |
| `rby1_ik` | core | Legacy Python IK helper kept for debug/experiments |
| `vive_rby1` | core | Tracker delta → RB-Y1 joint commands, recording state machine |
| `teleop_gui` | gui | PySide6 GUI (node status, tracker status, teleop panel, recording, calibration) |
| `teleop_bringup` | launch | Launch file for full system |
| `inspire_driver` | output | Inspire hand hardware driver |

## Troubleshooting

**Pedal not detected**
- Check `input` group membership: `groups | grep input`
- If missing: `sudo usermod -aG input $USER` then fully log out and back in.
- Note: re-login (e.g. after adding `realtime` group) resets the active session — re-check `groups` after every re-login and re-add if needed. `usermod` itself is permanent; only the active session needs refreshing.

**Vive trackers not publishing**
- Ensure SteamVR is running and trackers show green before launching.
- Check serial numbers match the parameters in `vive_tracker_node.py`.

**Tracker shows LOST in GUI**
- Check SteamVR — tracker may have lost line-of-sight to base station.
- Tracker stamp timeout is 0.5s.

**Build fails with empy error**
- `pip3 install empy==3.3.4` — colcon requires 3.x, not 4.x.

**Build fails with conda Python**
- `conda deactivate` before sourcing ROS or running colcon.

**ManusSDK not found**
- Confirm `ManusSDK/include/ManusSDK.h` and `ManusSDK/lib/libManusSDK.so` exist at the repo root and are non-zero size.

**`rby1_core_node` services not available**
- `rby1_core_node` must be running separately (not part of this repo).
- Without it, mode switching and ready pose moves will fail, but the GUI still launches.

**Robot joint trembling**
- Reduce `max_teleop_dq` in `rby1_ik.py` (currently 1.5 rad/s).
- Reduce `ik_dt` in launch file (currently 0.1s).
- Both together determine max joint velocity: `max_teleop_dq × ik_dt = max Δq per step`.
