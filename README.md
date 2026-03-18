# teleoperation-core

ROS2 Humble workspace for bimanual teleoperation of RB-Y1 robot using Vive Trackers and Manus gloves.

## Architecture

```
Input                  Core                       Output
────────────────────────────────────────────────────────────
manus_ros2          →  manus_inspire           →  inspire_driver
vive_ros2           →  vive_rby1 (+ rby1_ik)  →  /rby1_teleop_command
```

## Packages

| Package | Layer | Description |
|---------|-------|-------------|
| `manus_ros2` | input | Manus glove SDK → ROS2 (C++) |
| `vive_ros2` | input | Vive Tracker 3.0 → `/teleop/tracker/left\|right` |
| `manus_ros2_msgs` | msgs | Manus glove message types |
| `inspire_hand_msgs` | msgs | Inspire hand message types |
| `manus_inspire` | core | Manus glove data → Inspire hand commands |
| `rby1_ik` | core | Differential IK library (pink + pinocchio) |
| `vive_rby1` | core | Tracker delta → RB-Y1 joint commands |
| `inspire_driver` | output | Inspire hand hardware driver |

## Setup

### 1. ManusSDK (manual copy required — not in git, file too large for GitHub)

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

Obtain from: Manus developer portal, or copy from a machine that already has it.

### 2. Dependencies

```bash
sudo apt install ros-humble-desktop python3-pip
pip3 install pin pink scipy openvr evdev PySide6
pip3 install empy==3.3.4   # required for colcon build (do NOT use empy 4.x)
```

> **Note (pedal):** `evdev` requires the user to be in the `input` group:
> ```bash
> sudo usermod -aG input $USER  # then re-login
> ```

> **Note:** If using conda, deactivate it before building — conda's Python conflicts with ROS2.
> ```bash
> conda deactivate
> which python3  # should be /usr/bin/python3
> ```

### 3. Build

```bash
cd ~/path/to/2026
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 4. Run

**Manus → Inspire hand:**
```bash
ros2 run manus_ros2 manus_data_publisher
ros2 run manus_inspire manus_inspire_node
ros2 run inspire_driver inspire_driver_node
```

#### Manus Glove Calibration

`manus_inspire` 노드는 손가락 센서 범위 보정을 지원합니다.

- 최초 실행 시 캘리브레이션 파일이 없으면 자동으로 시작
- 저장 위치: `~/.ros/manus_inspire_calib.yaml` (다음 실행부터 자동 로드)
- 재캘리브레이션:
  ```bash
  ros2 service call /manus_inspire/calibrate std_srvs/srv/Trigger
  ```
- **순서:** Phase 1 (4초) — 양손 쫙 펴고 유지 → Phase 2 (4초) — 주먹 꽉 쥐고 유지 → 자동 저장

**Vive → RB-Y1 teleoperation:**
```bash
# Requires: SteamVR running, Vive Trackers paired
ros2 run vive_ros2 vive_tracker_node
ros2 run vive_rby1 vive_rby1_node --ros-args \
  -p urdf_path:=/path/to/rby1.urdf \
  -p srdf_path:=/path/to/rby1.srdf
```

Pedal clutch: **hold = tracking active**, **release = robot holds position**

## Reference Files (root)

Original working code kept for reference — do not run directly:
- `vive_manager.py` — original OpenVR + ZMQ tracker reader
- `rby1_ik_pink.py` — original pink IK implementation
- `small_main.py` — original Qt GUI + delta IK node

## Versions

| Tag | Description |
|-----|-------------|
| `v0.1-manus-only` | Manus glove + Inspire hand only |
| `v0.2-teleop-core` | Added Vive tracker + RB-Y1 IK packages |

---

## Notes / Future Architecture Reference

이전에 설계했던 더 큰 구조 (SCM-teleoperation) 메모:

```
teleoperation/
├── inputs/
│   ├── base_input/          # 입력 디바이스 공통 인터페이스
│   ├── manus_vive/          # Manus Glove + Vive Tracker
│   └── xr_hmd/
│       ├── openxr/          # Meta Quest, Galaxy XR 등
│       └── visionos/        # Apple Vision Pro
│
├── robots/
│   ├── base_humanoid/       # 휴머노이드 공통 인터페이스
│   ├── rby1/                # RBY1 전용 제어
│   └── dummy_robot/         # 테스트용 가상 로봇
│
├── core/
│   ├── retargeting/         # 모션 → 로봇 IK 변환
│   ├── filtering/           # Kalman / Low-pass 필터
│   └── safety/              # 충돌 방지, 특이점 회피
│
├── comms/
│   ├── ros2/
│   └── webrtc_udp/          # 저지연 네트워크 래퍼
│
└── launch/
    ├── scripts/
    └── configs/
```
