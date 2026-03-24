# teleoperation-core

ROS2 Humble workspace for bimanual teleoperation using motion capture hardware.
Currently supports RB-Y1 robot with Vive Trackers (arm control) and Manus gloves (hand control).
Designed to be extensible — additional robot platforms and input devices can be added as new packages.

## Architecture

```
Input              Core                      Output
──────────────────────────────────────────────────────────────
manus_ros2      →  manus_inspire          →  inspire_driver  (Inspire Hand)
vive_ros2       →  vive_rby1 (+ rby1_ik) →  /rby1_teleop_command  (RB-Y1)
pedal_ros2      →  (clutch / mode switch)
```

A GUI node (`teleop_gui`) provides live system status and calibration control.

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
- ManusSDK (see Installation step 1)

## Installation

### 1. Clone the repository

```bash
git clone <repo-url> 2026
cd 2026
```

### 2. ManusSDK (manual copy required — binary too large for git)

Copy the ManusSDK folder to the repo root:

```
2026/
└── ManusSDK/
    ├── include/
    │   ├── ManusSDK.h
    │   ├── ManusSDKTypes.h
    │   └── ManusSDKTypeInitializers.h
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
cd 2026
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

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

Edit the default paths in `src/core/vive_rby1/vive_rby1/vive_rby1_node.py`, or pass at runtime:

```bash
ros2 run vive_rby1 vive_rby1_node --ros-args \
  -p urdf_path:=/path/to/rby1.urdf \
  -p srdf_path:=/path/to/rby1.srdf
```

## Quick Start

Start SteamVR and pair all Vive devices, then:

```bash
source install/setup.bash
ros2 launch teleop_bringup teleop.launch.py
```

This launches all nodes: pedal driver, Vive tracker, Manus publisher, arm IK bridge, hand mapper, and GUI.

The GUI shows live node status, pedal state, and the calibration panel.

## Usage

### Teleoperation

1. Launch the full system with the command above.
2. Confirm all nodes show green in the GUI.
3. **Hold pedal A** to engage tracking (dead-man switch).
   - While held: tracker poses are mapped to robot joint commands in real time.
   - On release: robot holds its last position.
   - On re-engage: reference pose is re-captured — no position jump.

### Manus Hand Calibration

Finger sensor ranges are calibrated per session and saved to `~/.ros/manus_inspire_calib.yaml`.

- **First launch:** calibration starts automatically if no saved file is found.
- **Recalibrate:** click **Recalibrate** in the GUI, or call the service directly:
  ```bash
  ros2 service call /manus_inspire/calibrate std_srvs/srv/Trigger
  ```

Calibration procedure (8 seconds total):
1. **Phase 1 (4 s):** Fully open both hands and hold.
2. **Phase 2 (4 s):** Make tight fists and hold.
3. Calibration saves automatically.

## Individual Nodes (Advanced)

Run components separately for development or partial setups:

```bash
# Input
ros2 run pedal_ros2 pedal_node
ros2 run vive_ros2 vive_tracker_node
ros2 run manus_ros2 manus_data_publisher

# Core
ros2 run vive_rby1 vive_rby1_node
ros2 run manus_inspire manus_inspire_node

# Output
ros2 run inspire_driver inspire_driver_node

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
| `manus_inspire` | core | Manus glove data → Inspire hand commands |
| `rby1_ik` | core | Differential IK solver (pink + pinocchio) |
| `vive_rby1` | core | Tracker delta → RB-Y1 joint commands |
| `teleop_gui` | gui | PySide6 status/calibration GUI |
| `teleop_bringup` | launch | Launch file for full system |
| `inspire_driver` | output | Inspire hand hardware driver |

## Troubleshooting

**Pedal not detected**
- Check `input` group membership: `groups | grep input`
- If missing: `sudo usermod -aG input $USER` then fully log out and back in.

**Vive trackers not publishing**
- Ensure SteamVR is running and trackers show green before launching.
- Check serial numbers match the parameters in `vive_tracker_node.py`.

**Build fails with empy error**
- `pip3 install empy==3.3.4` — colcon requires 3.x, not 4.x.

**Build fails with conda Python**
- `conda deactivate` before sourcing ROS or running colcon.

**ManusSDK not found**
- Confirm `ManusSDK/include/ManusSDK.h` and `ManusSDK/lib/libManusSDK.so` exist at the repo root.

## Versions

| Tag | Description |
|-----|-------------|
| `v0.1-manus-only` | Manus glove + Inspire hand only |
| `v0.2-teleop-core` | Added Vive tracker + RB-Y1 IK packages |
