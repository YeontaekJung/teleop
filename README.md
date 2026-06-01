# teleop

> RB-Y1 양손 텔레오퍼레이션 ROS2 Humble 워크스페이스 — Vive Tracker(팔) + Manus Glove(손) + PCsensor Pedal(클러치/녹화) + PySide6 GUI.
> ROS2 Humble workspace for bimanual teleoperation of RB-Y1 — Vive Trackers (arms), Manus gloves (hands), PCsensor pedals (clutch/recording), and a PySide6 GUI.

---

## 한국어 (Korean)

### 개요

`teleop`는 RB-Y1 로봇의 양손 텔레오퍼레이션을 위한 ROS2 워크스페이스입니다. 입력 디바이스(트래커, 글로브, 페달)에서 들어온 모션 캡처 데이터를 SDK Cartesian Impedance 명령으로 변환해 `rby1_core_node`(hw-core)로 흘려보냅니다. PySide6 기반 GUI가 라이브 모니터링과 라이프사이클 제어 일체를 통합합니다.

### 패키지 구조

```
teleop/
├── README.md                                    ← 이 문서
├── CHANGES.md                                   변경 이력
└── src/
    ├── input/                                     입력 디바이스 드라이버
    │   ├── pedal_ros2/                            PCsensor 3-pedal → /teleop/pedal (Joy)
    │   ├── vive_ros2/                             OpenVR → /teleop/tracker/{left,right,body}
    │   └── manus_ros2/                            Manus SDK → /manus_glove_*
    ├── core/                                      변환·매핑 노드
    │   ├── vive_rby1/                             트래커 → SDK Cartesian 명령 + 녹화 상태머신 (C++ 프로덕션 + Python 디버그)
    │   ├── manus_inspire/                         Manus glove → Inspire Hand 명령 + 4-phase 캘리브
    │   └── rby1_ik/                               (Legacy) pink IK 헬퍼 — debug 노드 전용
    ├── gui/scm_gui/                               PySide6 GUI (스코프: 모든 라이프사이클 + 모니터링)
    ├── launch/teleop_bringup/                     전체 시스템 launch
    └── msgs/                                      메시지 패키지 5개
        ├── rby1_core_msgs/                        ← hw-core 사본과 동기화 필수
        ├── inspire_hand_msgs/                     ← hw-core 사본과 동기화 필수
        ├── manus_ros2_msgs/                       Manus glove 메시지
        ├── scm_recording_msgs/                    외부 녹화 core와의 srv 계약
        └── interbotix_xs_msgs/                    참고용(COLCON_IGNORE)
```

각 패키지의 상세 개발자 가이드: 패키지 디렉토리 내부의 `DEVELOPER.ko.md` 참조.

### 시스템 아키텍처 (데이터 흐름)

```
Vive Tracker × 2~3 ──vive_ros2──► vive_rby1 ──► /rby1/cmd/pose       ──► rby1_core_node ──► RB-Y1
                                  (body tracker → torso in sdk_impedance)
Manus Glove × 2    ──manus_ros2──► manus_inspire ──► /rt/inspire_hand/ctrl/{l,r} ──► inspire_hand_driver ──► Inspire Hand
PCsensor Pedal     ──pedal_ros2──► vive_rby1 (recording state machine) ──► /scm_recording/*
PySide6 GUI        ──scm_gui──► /rby1/{ctrl/mode,stream,…} services + /vive_rby1/* services
```

### 하드웨어 요구사항

| 장비 | 역할 |
|---|---|
| Rainbow Robotics RB-Y1 | 텔레오퍼레이션 대상 로봇 |
| HTC Vive Tracker 3.0 × 2 (~3) | 팔 trajectory (옵션 body tracker) |
| HTC Vive Base Station × 2 | SteamVR 트래킹 |
| Manus Prime X Haptic glove | 손가락 트래킹 |
| Inspire Hand × 2 | 로봇 손 |
| PCsensor FootSwitch (3-pedal) | 클러치 / 녹화 페달 |

### 시스템 의존성

- Ubuntu 22.04
- ROS2 Humble (`ros-humble-desktop`)
- SteamVR (Vive paired 상태)
- ManusSDK 바이너리 (수동 복사 — `teleop/ManusSDK/{include,lib}/`)
- hw-core 패키지 (`rby1_core`, `inspire_hand_driver` 등) — 별개 워크스페이스에 설치

### 빌드 전 준비

```bash
sudo apt install ros-humble-desktop python3-pip libncurses-dev
pip3 install pin pink scipy openvr evdev PySide6 pyyaml
pip3 install empy==3.3.4    # ⚠ empy 4.x는 colcon 빌드 깨뜨림

# pedal 권한 (1회, 재로그인 필수)
sudo usermod -aG input $USER

# conda 사용자: 비활성화 권장 (ROS2와 Python 충돌)
conda deactivate
```

ManusSDK 배치:
```
teleop/
└── ManusSDK/
    ├── include/{ManusSDK.h, ManusSDKTypeInitializers.h, ManusSDKTypes.h}
    └── lib/{libManusSDK_Integrated.so, libManusSDK.so}
```

### 빌드

```bash
cd teleop
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> `interbotix_xs_msgs`는 `COLCON_IGNORE`로 빌드 제외 — 시스템 설치 가정.

### 실행

```bash
# 모든 노드 + GUI 한 번에
ros2 launch teleop_bringup teleop.launch.py

# 시뮬레이터 모드 (입력 노드 비활성, core + GUI만)
ros2 launch teleop_bringup teleop.launch.py sim:=true

# 일부 입력만 비활성
ros2 launch teleop_bringup teleop.launch.py use_vive:=false
```

GUI가 뜨면:
1. **Connect** 버튼 (hw-core `rby1_core_node` 연결)
2. **Power On** → **Servo On** → **Ctrl Enable**
3. 그리퍼 사용 시 **Gripper Init**
4. (선택) **Move To Pose** — preset 자세로 이동
5. **▶ Teleop Start** 또는 Recording 패널의 **▶ Start Episode**

### 페달 매핑

| 페달 | 함수 |
|---|---|
| A (왼쪽) | 클러치 engage/disengage (REC↔PAUSED) |
| B (가운데) | 에피소드 폐기 (`EndRecording(discard=true)`) |
| C (오른쪽) | 에피소드 시작/종료 (IDLE↔ARMING, PAUSED→IDLE) |

### 제어 모드 (Mode dropdown)

| 모드 | 토픽 | 설명 |
|---|---|---|
| SDK Impedance | `/rby1/cmd/pose` | Cartesian impedance (C++ 프로덕션 노드, default) |
| SDK Position | `/rby1/cmd/pose` | Cartesian position (C++ 노드) |
| Pink Position | `/rby1/cmd/joint` | Joint position via pink IK (**debug 노드 전용**, 수동 실행) |
| Pink Impedance | `/rby1/cmd/joint` | Joint impedance via pink IK (debug 노드 전용) |

### 녹화 워크플로

전제: `scm_recording` core (별도 패키지, `/scm_recording/*` 서비스 서버) + `rby1_core_node` 실행 중.

1. Task ID 선택 (Recording 패널)
2. Mode 선택 (`SDK Impedance` 권장)
3. **▶ Start Episode** (또는 페달 C) → ARMING (mode 설정 → ready pose 이동 → stream 시작) → READY
4. **페달 A** 눌러 클러치 engage → RECORDING
5. **페달 A** 눌러 disengage → PAUSED (에피소드 유지)
6. 4~5 반복 (여러 engage cycle)
7. **■ End Episode** (또는 PAUSED 시 페달 C) → 에피소드 저장 → IDLE

### 트래커 상태

GUI Node Status 패널 또는 `/teleop/tracker_status`:

| 상태 | 색상 | 의미 |
|---|---|---|
| OK | 초록 | 데이터 정상 |
| JITTER | 노랑 | 위치 분산 > 3mm σ (10+ 샘플, 20-deep buffer) |
| LOST | 빨강 | 0.5s 이상 stamp 갭 |

### Manus 캘리브레이션

- **첫 실행**: 파일 없으면 자동 시작
- **재캘리브레이션**: GUI **Recalibrate** 버튼 또는 `ros2 service call /manus_inspire/calibrate std_srvs/srv/Trigger`
- 4 phase × 4초 = 총 16초
- 캘리브 파일: `~/.ros/manus_inspire_calib.yaml`

| Phase | 자세 | 측정 |
|---|---|---|
| 1 | Open hands fully | 손가락 min + 엄지 spread min |
| 2 | Thumbs up (주먹 + 엄지) | 손가락 max + 엄지 MCPStretch max |
| 3 | Press thumb to side of index | 엄지 spread max |
| 4 | Open fingers, bend thumb only | 엄지 MCPStretch min |

### 키보드 드라이빙 (Mobile Base Panel, 2026-05-28 추가)

GUI의 Mobile Base Panel에서 **Enable** 체크 후 키보드로 베이스 주행:

| 키 | 동작 |
|---|---|
| W | 전진 (linear.x = +linear) |
| S | 후진 (linear.x = -linear) |
| A | 좌 strafe (linear.y = +linear) — Model M만 |
| D | 우 strafe (linear.y = -linear) — Model M만 |
| Q | 좌회전 (angular.z = +angular) |
| E | 우회전 (angular.z = -angular) |

기본값: linear = 0.05 m/s, angular = 0.10 rad/s (2026-05-30 변경).

가속 한계는 `Driving Parameter Manager`에서 런타임 변경 가능 (`/rby1_core_node/set_parameters`, 2026-05-28 (7)).

### vive_rby1_node 핵심 파라미터 (launch dict)

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `publish_rate` | 100.0 Hz | timer 주기 (hw-core RT 100Hz와 매치) |
| `ik_dt` | 0.05 s | Differential IK 시간 단계 |
| `pos_scale` | 0.5 | hand tracker 위치 스케일 |
| `torso_pos_scale` | 1.0 | body tracker 위치 스케일 |
| `use_torso` | False | body tracker 비활성 (런타임 `/vive_rby1/set_use_torso`로 토글) |
| `sdk_max_delta_pos` | 0.03 m | per-frame Cartesian step clamp |

`max_teleop_dq` (1.5 rad/s)는 `vive_rby1_node.cpp`에 하드코딩.

### 흔한 함정 / 트러블슈팅

**페달 미감지**
- `groups | grep input` 확인 → 없으면 `sudo usermod -aG input $USER` 후 **완전 재로그인**
- `evdev` 디바이스 이름 확인: `python3 -c "import evdev; [print(d.name) for d in [evdev.InputDevice(p) for p in evdev.list_devices()]]"`

**Vive trackers publish 안 됨**
- SteamVR 실행 + 모든 디바이스 녹색 확인
- `trackers.yaml`의 시리얼 번호가 실제 디바이스와 일치하는지 확인

**GUI에 LOST 표시**
- SteamVR — 트래커 시야 차폐? 0.5s 타임아웃.

**빌드 실패 (empy 에러)**
- `pip3 install empy==3.3.4` — colcon은 3.x 필요, 4.x 거부

**빌드 실패 (conda Python)**
- `conda deactivate` 후 `which python3` 가 `/usr/bin/python3`인지 확인

**ManusSDK not found**
- `ManusSDK/include/ManusSDK.h`와 `ManusSDK/lib/libManusSDK_Integrated.so` 존재 + 크기 0 아닌지 확인

**`rby1_core_node` 서비스 미가용**
- hw-core 워크스페이스에서 별도로 실행 필요. 없으면 mode/move 실패하지만 GUI 자체는 뜸.

**로봇 떨림**
- `vive_rby1_node.cpp`의 `max_teleop_dq` 낮춤
- launch의 `ik_dt` 낮춤 (현재 0.05s)
- 둘의 곱이 per-step Δq 한계

### 변경 이력 / 메모

- 2026-05-30: keyboard driving 기본값 0.05/0.10 변경, q/e 방향 정정, VLA indicator, 배터리 표시
- 2026-05-29: `Mobile On/Off` 버튼 — Servo와 wheel servo 분리 (hw-core `SetServo.wheel_only` 신규)
- 2026-05-28: Mobile Base Panel 추가, 키보드 드라이빙, accel_limit 런타임 변경
- 2026-05-27: `/rby1/cmd/pose` 타입을 `tf2_msgs/TFMessage`로 마이그레이션
- 2026-05-22: torso teleop (body tracker → link_torso_5) on/off 토글, Cartesian Impedance Params 패널, nullspace ref pose 통합
- 2026-05-21: publish_rate 100Hz 복원, tracker smoothing alpha 0.9

전체 항목은 [`CHANGES.md`](CHANGES.md) 참조.

---

## English

### Overview

`teleop` is a ROS2 Humble workspace for bimanual teleoperation of the RB-Y1 robot. It translates motion capture data from input devices (trackers, gloves, pedals) into SDK Cartesian Impedance commands consumed by `rby1_core_node` (in the hw-core workspace). A PySide6 GUI integrates live monitoring and lifecycle control.

### Package layout

```
teleop/
├── README.md                                   ← this file
├── CHANGES.md                                  changelog
└── src/
    ├── input/{pedal_ros2, vive_ros2, manus_ros2}                input device drivers
    ├── core/{vive_rby1, manus_inspire, rby1_ik}                 mapping/conversion nodes
    ├── gui/scm_gui                                                PySide6 GUI
    ├── launch/teleop_bringup                                      full-system launch
    └── msgs/                                                      5 message packages
        ├── rby1_core_msgs                                         ← keep in sync with hw-core copy
        ├── inspire_hand_msgs                                      ← keep in sync with hw-core copy
        ├── manus_ros2_msgs
        ├── scm_recording_msgs                                     contract with external recording core
        └── interbotix_xs_msgs                                     reference (COLCON_IGNORE)
```

Per-package developer guides are in each package's `DEVELOPER.ko.md` (Korean).

### Architecture (data flow)

```
Vive Tracker × 2~3 ──vive_ros2──► vive_rby1 ──► /rby1/cmd/pose       ──► rby1_core_node ──► RB-Y1
                                  (body tracker → torso in sdk_impedance)
Manus Glove × 2    ──manus_ros2──► manus_inspire ──► /rt/inspire_hand/ctrl/{l,r} ──► inspire_hand_driver ──► Inspire Hand
PCsensor Pedal     ──pedal_ros2──► vive_rby1 (recording state machine) ──► /scm_recording/*
PySide6 GUI        ──scm_gui──► /rby1/{ctrl/mode,stream,…} services + /vive_rby1/* services
```

### Hardware requirements

| Item | Role |
|---|---|
| Rainbow Robotics RB-Y1 | Teleoperated robot |
| HTC Vive Tracker 3.0 × 2 (~3) | Arm trajectories (optional body tracker) |
| HTC Vive Base Station × 2 | SteamVR tracking |
| Manus Prime X Haptic gloves | Finger tracking |
| Inspire Hand × 2 | Robot hands |
| PCsensor FootSwitch (3-pedal) | Clutch / recording pedals |

### System prerequisites

- Ubuntu 22.04
- ROS2 Humble (`ros-humble-desktop`)
- SteamVR running with Vive devices paired
- ManusSDK binary (manual copy — `teleop/ManusSDK/{include,lib}/`)
- hw-core packages installed in a separate workspace

### Pre-build setup

```bash
sudo apt install ros-humble-desktop python3-pip libncurses-dev
pip3 install pin pink scipy openvr evdev PySide6 pyyaml
pip3 install empy==3.3.4    # ⚠ empy 4.x breaks colcon

# Pedal permissions (one-time, full re-login required)
sudo usermod -aG input $USER

# Conda users: deactivate (Python conflicts with ROS2)
conda deactivate
```

ManusSDK layout:
```
teleop/
└── ManusSDK/
    ├── include/{ManusSDK.h, ManusSDKTypeInitializers.h, ManusSDKTypes.h}
    └── lib/{libManusSDK_Integrated.so, libManusSDK.so}
```

### Build

```bash
cd teleop
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> `interbotix_xs_msgs` is excluded from build (`COLCON_IGNORE`) — assumed installed system-wide.

### Run

```bash
# All nodes + GUI
ros2 launch teleop_bringup teleop.launch.py

# Sim mode (no hardware input nodes, just core + GUI)
ros2 launch teleop_bringup teleop.launch.py sim:=true

# Disable a specific input
ros2 launch teleop_bringup teleop.launch.py use_vive:=false
```

When the GUI is up:
1. Press **Connect** (links to `rby1_core_node`)
2. **Power On** → **Servo On** → **Ctrl Enable**
3. **Gripper Init** if using RB Gripper
4. (optional) **Move To Pose** — go to a preset
5. **▶ Teleop Start** or **▶ Start Episode** (Recording panel)

### Pedal mapping

| Pedal | Function |
|---|---|
| A (left) | Toggle clutch engage/disengage (RECORDING↔PAUSED) |
| B (center) | Discard episode (`EndRecording(discard=true)`) |
| C (right) | Start / end episode (IDLE↔ARMING, PAUSED→IDLE) |

### Control modes

| Mode | Topic | Notes |
|---|---|---|
| SDK Impedance | `/rby1/cmd/pose` | Cartesian impedance (C++ production, default) |
| SDK Position | `/rby1/cmd/pose` | Cartesian position |
| Pink Position | `/rby1/cmd/joint` | Joint position via pink IK (**debug node only**, manual launch) |
| Pink Impedance | `/rby1/cmd/joint` | Joint impedance via pink IK (debug node only) |

### Recording workflow

Requires `scm_recording` core (separate package, hosts `/scm_recording/*` services) + `rby1_core_node` running.

1. Select Task ID in Recording panel
2. Select mode (`SDK Impedance` recommended)
3. **▶ Start Episode** (or pedal C) → ARMING (set mode → move to ready pose → start stream) → READY
4. **Pedal A** — engage clutch → RECORDING
5. **Pedal A** — disengage → PAUSED (episode still active)
6. Repeat 4–5 for multiple engage cycles
7. **■ End Episode** (or pedal C from PAUSED) → save episode → IDLE

### Tracker status

GUI Node Status panel or `/teleop/tracker_status`:

| Status | Color | Meaning |
|---|---|---|
| OK | green | Data arriving normally |
| JITTER | yellow | Position variance > 3 mm σ (10+ samples, 20-deep buffer) |
| LOST | red | > 0.5 s stamp gap |

### Manus calibration

- **First launch**: auto-starts if no file found
- **Recalibrate**: GUI **Recalibrate** button or `ros2 service call /manus_inspire/calibrate std_srvs/srv/Trigger`
- 4 phases × 4 s = 16 s total
- Calibration file: `~/.ros/manus_inspire_calib.yaml`

### Keyboard driving (Mobile Base Panel, added 2026-05-28)

In the GUI's Mobile Base Panel, check **Enable** and use keyboard:

| Key | Action |
|---|---|
| W | Forward |
| S | Backward |
| A | Strafe left (Model M only) |
| D | Strafe right (Model M only) |
| Q | Yaw left |
| E | Yaw right |

Defaults: linear = 0.05 m/s, angular = 0.10 rad/s (changed 2026-05-30).

Acceleration limits are runtime-mutable via the `Driving Parameter Manager` (uses `/rby1_core_node/set_parameters`, added 2026-05-28 (7)).

### Key vive_rby1_node parameters (launch dict)

| Parameter | Default | Meaning |
|---|---|---|
| `publish_rate` | 100.0 Hz | Timer period (matches hw-core 100 Hz RT) |
| `ik_dt` | 0.05 s | Differential IK time step |
| `pos_scale` | 0.5 | Hand tracker → robot position scale |
| `torso_pos_scale` | 1.0 | Body tracker → torso scale |
| `use_torso` | False | Body tracker disabled (runtime toggle via `/vive_rby1/set_use_torso`) |
| `sdk_max_delta_pos` | 0.03 m | Per-frame Cartesian step clamp |

`max_teleop_dq` (1.5 rad/s) is hardcoded in `vive_rby1_node.cpp`.

### Common gotchas / troubleshooting

**Pedal not detected**
- `groups | grep input` — if missing, `sudo usermod -aG input $USER` and **fully re-login**
- Check evdev device name: `python3 -c "import evdev; [print(d.name) for d in [evdev.InputDevice(p) for p in evdev.list_devices()]]"`

**Vive trackers not publishing**
- SteamVR running + all devices green
- Serial numbers in `trackers.yaml` match actual hardware

**LOST in GUI**
- SteamVR — line-of-sight occluded? 0.5 s timeout.

**Build fails with empy error**
- `pip3 install empy==3.3.4` — colcon needs 3.x, not 4.x

**Build fails with conda Python**
- `conda deactivate` then `which python3` should be `/usr/bin/python3`

**ManusSDK not found**
- Confirm `ManusSDK/include/ManusSDK.h` and `ManusSDK/lib/libManusSDK_Integrated.so` exist and are non-zero

**`rby1_core_node` services unavailable**
- Run separately from the hw-core workspace. Without it, mode/move fail but GUI launches.

**Robot trembling**
- Lower `max_teleop_dq` in `vive_rby1_node.cpp`
- Lower `ik_dt` in launch (currently 0.05 s)
- Product of the two is the max Δq per step

### Recent changes (highlights)

- 2026-05-30: keyboard driving defaults 0.05/0.10, q/e direction fix, VLA indicator, battery display
- 2026-05-29: `Mobile On/Off` button — separates body servo from wheel servo (new hw-core `SetServo.wheel_only`)
- 2026-05-28: Mobile Base Panel + keyboard driving, runtime accel_limit changes
- 2026-05-27: `/rby1/cmd/pose` type migrated to `tf2_msgs/TFMessage`
- 2026-05-22: torso teleop (body tracker → link_torso_5) on/off toggle, Cartesian Impedance Params panel, nullspace ref pose integration
- 2026-05-21: publish_rate restored to 100 Hz, tracker smoothing alpha 0.9

See [`CHANGES.md`](CHANGES.md) for the full log.
