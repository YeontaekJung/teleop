# `scm_gui` 개발자 가이드

> teleop 전 시스템의 라이브 모니터링 + 사용자 제어를 한 화면에 통합한 PySide6 데스크톱 GUI. ROS2 노드 1개(`scm_gui`), 단일 메인 윈도우(`TeleopGuiWindow`).

---

## 1. 패키지 개요

- **PySide6 (Qt6) 기반 GUI** — 단일 메인 윈도우 + 다수의 `QGroupBox` 패널
- **백그라운드 ROS2 노드** (`ScmGuiNode`) — 별도 thread에서 `rclpy.spin`. 콜백은 Qt `Signal`을 통해 UI thread로 전달
- 5개 노드 그룹 모니터링: Core / Vision / VLA / Teleop / Recording (+ placeholder StateMachine, MotionPlanning, Driving)
- **모든 hw-core 라이프사이클 + 런타임 튜닝 서비스 호출자**: Connect, Power, Servo (body/wheel 분리), ControlEnable, ErrorReset, GripperInit, Stream, StopMove, ControlMode, MoveToJoint, SetCartesianJointLimits, SetNullspaceWeight, SetNullspaceJointRef, SetParameters (런타임 mobility accel)
- **모든 vive_rby1 서비스 호출자**: teleop_start/stop, toggle_clutch, set_teleop_pose, toggle_episode, set_use_torso
- Manus 캘리브레이션 트리거 (`/manus_inspire/calibrate`)
- 키보드 기반 mobile base driving (`/rby1/cmd/base_vel`)

---

## 2. 디렉토리 구조

```
src/gui/scm_gui/
├── package.xml                                ament_python
├── setup.py                                   entry_points: scm_gui_node = ...
├── setup.cfg
├── resource/scm_gui
├── config/
│   ├── named_poses.yaml                       Joint position preset (사용자 편집 가능)
│   └── impedance_presets.yaml                 Cartesian Impedance preset (런타임 저장)
└── scm_gui/
    ├── __init__.py
    └── scm_gui_node.py                        모든 코드 (2087줄)
```

---

## 3. 빌드 / 실행

```bash
cd teleop
source /opt/ros/humble/setup.bash
colcon build --packages-select scm_gui
source install/setup.bash

# 실행 (default launch에 포함됨; 단독 실행도 가능)
ros2 run scm_gui scm_gui_node
```

전제 — `pip install PySide6 pyyaml`.

---

## 4. 모듈 구조

### 4.1 두 주요 클래스 (`scm_gui_node.py`)

| 클래스 | 책임 | thread |
|---|---|---|
| `ScmGuiNode(Node)` | ROS2 인터페이스 전부 (구독자/발행자/서비스 클라이언트). 콜백은 등록된 callback 리스트 호출. | 백그라운드 (`rclpy.spin`) |
| `TeleopGuiWindow(QWidget)` | UI 빌드, 이벤트 처리, ROS 콜백 ↔ Qt Signal 브리지 | Qt UI thread |

`main()`에서 `rclpy.init()` → `ScmGuiNode` 생성 → daemon thread에서 spin → `QApplication` 실행 → 종료 시 cleanup.

### 4.2 Qt Signal-Slot 패턴

ROS 콜백은 백그라운드 thread에서 호출되지만 Qt 위젯은 UI thread에서만 변경 가능. 모든 ROS → UI 갱신은 `Signal`을 거침:

```python
class TeleopGuiWindow:
    rec_state_changed = Signal(str)              # 백그라운드에서 emit
    rec_episode_changed = Signal(int)
    rby1_status_changed = Signal(dict)
    tracker_status_changed = Signal(str, str, str)
    battery_changed = Signal(BatteryState)
    pedal_changed = Signal(list)
    clutch_state_changed = Signal(str)

# 백그라운드 콜백:
node.register_rec_state_cb(lambda s: self.rec_state_changed.emit(s))
# UI thread:
self.rec_state_changed.connect(self._on_rec_state)
```

---

## 5. 화면 레이아웃 (위에서 아래로)

### 5.1 상단 — RB-Y1 라이프사이클 행

| 위젯 | 동작 |
|---|---|
| `Connect` 버튼 + IP/no_gripper 인풋 | `/rby1/connect` 호출. 성공 시 응답의 `joint_names`/`q_lower`/`q_upper`를 spinbox tooltip으로 |
| `Power On/Off` 토글 | `/rby1/power` |
| `Servo On/Off` (`no_wheel=true` 호출) | `/rby1/servo {enable, no_wheel: true, wheel_only: false}` — body만 |
| `Mobile On/Off` (`wheel_only=true`) | `/rby1/servo {enable, wheel_only: true}` — wheel만 (2026-05-29 추가) |
| `Ctrl Enable` | `/rby1/control_enable` |
| `Gripper Init` | `/rby1/gripper_init` |
| `Error Reset` | `/rby1/error_reset` |
| `Stop Move` | `/rby1/stop_move` |

상태 표시줄(노드 status JSON 기반): `Ctrl: Enabled/FAULT/Idle | Power:✓ | Servo:✓ | Mobile:✓ | Stream:✓ | Gripper:✓ | Battery: 78% (green/yellow/red)`.

> 2026-05-30: `Battery` 추가. 50%↑ 초록 / 20%↑ 노랑 / 미만 빨강.
> 2026-05-22: `power_state/servo_state/stream_state`는 JSON bool로 직접 파싱. `has_gripper` 사용 (`gripper_state` 의미 반전 버그 수정).

### 5.2 Joint Position 패널

- **Preset dropdown** ← `config/named_poses.yaml` (예: `ready`, `test`, `zero`). 각 preset = `joint_names[20]` + `positions[20]`.
- **20개 spinbox** (관절별 deg 표시 + rad 입력). Connect 응답의 q_lower/q_upper를 tooltip으로, 범위 초과 시 배경 빨강.
- **버튼**: `Apply` (= `/rby1/move_to_joint_position`), `Save` (현재값을 preset 이름으로 저장 → `named_poses.yaml`).

### 5.3 Cartesian Impedance Params 패널 (2026-05-22 추가)

3개 sub-section:

1. **Joint Limits 테이블** — `[+ Add Joint]`으로 행 추가, dropdown으로 joint 선택, min/max spinbox, `[X]`로 삭제. `[Apply Joint Limits]` → `/rby1/set_cartesian_joint_limits` (**전체 교체**).
2. **Nullspace Weights 테이블** — 14개 spinbox (right/left arm 0~6). `[Apply Weights]` → `/rby1/set_nullspace_weight` (부분 갱신).
3. **Nullspace Ref Pose** — dropdown ← `named_poses.yaml` preset 목록. `[Apply Nullspace Ref]` → `/vive_rby1/set_teleop_pose` + `/rby1/set_nullspace_joint_ref` (둘 다 전송, 2026-05-22 (3)).

**Impedance Preset 저장/불러오기** — `impedance_presets.yaml`에 위 3가지를 묶어서 저장. `nullspace_ref` 필드 추가됨 (joint position preset 이름 참조).

### 5.4 Mobile Base Panel (2026-05-28 추가)

ROS2 Node Status 오른쪽(2:1 비율).

**Manual Driving (Keyboard)** sub-group:
- `Enable` 체크박스
- QWEASD 키 버튼 (시각적 하이라이트, 키보드 이벤트 처리)
- W=전진, S=후진, A=좌 strafe, D=우 strafe, Q=좌회전, E=우회전
- Linear/Angular velocity spinbox (기본 0.05 / 0.10, 2026-05-30 변경)
- `editingFinished → clearFocus()` 연결 — Enter 또는 GUI 클릭 시 포커스 해제되어 즉시 키보드 조작 가능

**Driving Parameter Manager** sub-group:
- `accel_limit_linear`/`accel_limit_angular` spinbox + `Apply`
- `/rby1_core_node/set_parameters`로 `rcl_interfaces/SetParameters` 호출 (hw-core가 atomic 갱신)

Model A에선 A/D(Y 성분)는 SDK가 자동 무시.

### 5.5 Teleop 패널

- **▶ Teleop Start** → `/vive_rby1/teleop_start` (mode + move + stream + nullspace ref 자동)
- **■ Teleop Stop** → `/vive_rby1/teleop_stop`
- **Move To Pose** → preset 선택 후 `/rby1/move_to_joint_position`
- **Use Torso** 체크박스 → `/vive_rby1/set_use_torso` (런타임 토글, 2026-05-22 추가)
- **Mode dropdown**: SDK Impedance(기본) / SDK Position / Pink Position / Pink Impedance (마지막 두 개는 debug 노드 전용)
- **Mirror Mode** 체크박스 → `/teleop/mirror_mode` (`std_msgs/String`: `"mirror"`/`"normal"`)

### 5.6 Recording 패널

- **Task ID** spinbox + 발행 (`/teleop/task_id`)
- **Episode** 라벨 (현재 활성 에피소드, 또는 `-`)
- **Recording State** 색상 표시 — REC_STATE_STYLE dict로 매핑:
  - IDLE: 회색
  - ARMING: 파랑
  - READY: 노랑
  - RECORDING: 빨강
  - PAUSED: 주황
- **▶ Start Episode** → `/vive_rby1/toggle_episode` (IDLE 시)
- **■ End Episode** → `/vive_rby1/toggle_episode` (PAUSED 시)
- **Discard Episode** → `/vive_rby1/...` 또는 `/scm_recording/end` (`discard=true`) — 페달 B와 동일

### 5.7 ROS2 Node Status 패널

`NODES_TO_WATCH` 리스트의 각 노드를 1초마다 폴링 (`get_node_names_and_namespaces`). 발견 시 초록, 미발견 시 회색. `MODULE_ORDER`에 따라 그룹 표시:

| 그룹 | 노드 |
|---|---|
| Core | `rby1_core_node`, `inspire_hand_driver` |
| Vision | `cam_high/camera`, `cam_left_wrist/camera`, `cam_right_wrist/camera` |
| StateMachine | (placeholder, 빈 리스트) |
| MotionPlanning | (placeholder) |
| Driving | (placeholder) |
| VLA | `rby1_vla_client` (2026-05-30 추가) |
| Teleop | `pedal_node`, `vive_tracker_node`, `manus_data_publisher`, `vive_rby1_node`, `manus_inspire` |
| Recording | `scm_recording` |

### 5.8 Tracker Status 표시 (Teleop 그룹 내)

`/teleop/tracker_status` 문자열 파싱:
- `L:OK R:OK` 또는 `L:JITTER R:LOST` 등 — `L:`, `R:`, `B:` 순서, 공백 분리
- 색상: OK 초록 / JITTER 노랑 / LOST 빨강

### 5.9 Manus Calibration

- **Recalibrate** 버튼 → `/manus_inspire/calibrate` (`std_srvs/Trigger`)
- 진행률 표시 — `CALIB_DURATION = 4.0s` × 4 phase = 16초 진행 바
- 단계 안내 텍스트:
  1. Open hands fully — Finger min + thumb spread min
  2. Thumbs up (fist) — Finger max + thumb MCPStretch max
  3. Press thumb to side of index — Thumb spread max
  4. Open fingers, bend thumb only — Thumb MCPStretch min

---

## 6. ROS 인터페이스 전체

### 6.1 구독 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/teleop/pedal` | `sensor_msgs/Joy` | 페달 상태 시각화 (3개 버튼) |
| `/teleop/rec_state` | `std_msgs/String` | 녹화 상태 색상 매핑 |
| `/teleop/rec_episode` | `std_msgs/Int32` | 활성 에피소드 번호 |
| `/teleop/tracker_status` | `std_msgs/String` | L/R/B 헬스 시각화. depth=1 (실시간 비필수, 2026-05-30 depth 최소화) |
| `/rby1/state/status` | `std_msgs/String` (JSON) | 라이프사이클 표시. depth=1 |
| `/teleop/clutch_state` | `std_msgs/String` | engaged/disengaged 표시 |
| `/rby1/state/joint` | `sensor_msgs/JointState` | 현재 자세 → spinbox preview, "Save" 시 사용. depth=1 |
| `/rby1/state/battery` | `sensor_msgs/BatteryState` | 배터리 % 표시. depth=1 (2026-05-30 추가) |

### 6.2 발행 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/teleop/task_id` | `std_msgs/Int32` | Recording 패널 spinbox |
| `/teleop/mirror_mode` | `std_msgs/String` | "mirror"/"normal" |
| `/rby1/cmd/base_vel` | `geometry_msgs/Twist` | 키보드 driving (Mobile Base Panel) |

### 6.3 서비스 클라이언트 — hw-core (13개)

`ConnectRobot`, `SetPower`, `SetServo`, `SetStream`, `SetControlMode`, `MoveToJointPosition`, `Trigger`(control_enable, error_reset, gripper_init, stop_move), `SetCartesianJointLimits`, `SetNullspaceWeight`, `SetNullspaceJointRef`, `SetParameters`(rcl_interfaces).

### 6.4 서비스 클라이언트 — vive_rby1 (6개) + manus_inspire (1개)

`SetBool`(set_use_torso), `Trigger`(teleop_start, teleop_stop, toggle_clutch, toggle_episode), `SetTeleOpPose`, `Trigger`(calibrate).

### 6.5 파라미터

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `teleop_panel_expanded` | `false` | 시작 시 Teleop 패널 펼침 여부 |

---

## 7. 설정 파일

### 7.1 `config/named_poses.yaml`

```yaml
named_poses:
  ready:
    joint_names: [torso_0, torso_1, ...]   # 20개
    positions:   [0.0, 0.5236, ...]          # 20개 rad
  test:
    ...
robot_model: m       # GUI가 IK 관련 표시에 쓸 수 있음
teleop_pose: ready   # 시작 시 선택될 preset
```

GUI에서 preset 추가/수정 시 자동 저장. `share/scm_gui/config/`가 아닌 source 디렉토리 직접 수정 (install 후엔 source dir로 fallback).

### 7.2 `config/impedance_presets.yaml`

```yaml
impedance_presets:
  default:
    joint_limits: []
    nullspace_weights: [0.05, 0.05, ...]    # 14개
    nullspace_ref: ''                          # 또는 preset 이름
  my_preset:
    joint_limits:
      - {name: right_arm_3, min: -2.6, max: -0.5}
    nullspace_weights: [...]
    nullspace_ref: ready
```

저장 시 위 YAML로. 시작 시 GUI가 dropdown 채움.

---

## 8. 확장 / 수정 가이드

### 8.1 새 노드 그룹 추가 (예: Sensors)

1. `XXX_NODES = [('node_name', 'package_name'), ...]` 추가
2. `MODULE_ORDER`에 `('Sensors', XXX_NODES)` 추가
3. `NODES_TO_WATCH = ... + XXX_NODES` 갱신

### 8.2 새 서비스 호출 버튼 추가

1. `ScmGuiNode.__init__`에 `self._cli_foo = self.create_client(FooSrv, '/foo')` 추가
2. `ScmGuiNode.call_foo(...)` 메서드 작성 — `wait_for_service` + `call_async` + 결과 콜백 등록
3. `TeleopGuiWindow`에서 `QPushButton` + `clicked → self.ros.call_foo(...)`

### 8.3 새 ROS subscriber + UI 갱신

1. `ScmGuiNode.__init__`에 `self.create_subscription(Type, '/topic', self._cb_xxx, 1)`
2. `_cb_xxx`에서 등록된 callback 리스트 호출
3. `register_xxx_cb(cb)` 메서드 추가 (외부에서 콜백 등록)
4. `TeleopGuiWindow`에서 `Signal` 정의 + `connect` + handler 메서드

### 8.4 새 키보드 단축키

`TeleopGuiWindow.keyPressEvent`/`keyReleaseEvent`에서 이벤트 처리. `_DRIVE_BTN_PRESSED`/`_DRIVE_BTN_DEFAULT` 스타일 토글로 시각적 피드백.

---

## 9. 흔한 함정

- **포커스 문제**: spinbox에 포커스 있으면 키보드 driving 동작 안 함. 2026-05-30 fix: `editingFinished → clearFocus`, `mousePressEvent → focusWidget().clearFocus()`. 추가 위젯 도입 시 동일 패턴 적용.
- **depth 부적절**: 비실시간 토픽에 `depth=10` 사용하면 늦은 콜백이 누적되어 GUI 렉. 비실시간은 모두 `depth=1`로 두기 (status, joint, tracker_status, battery).
- **`/rby1/state/status` 파싱 실패**: hw-core가 bool/JSON 변경하면 GUI가 잘못 표시할 수 있음. `_cb_rby1_status`의 `bool(data.get(...))` 사용. 과거 `gripper_state` → `has_gripper` 마이그레이션 (2026-05-22) 같은 변경에 주의.
- **시뮬레이터/실 HW model 불일치**: spinbox에 표시되는 joint limit은 Connect 응답값. Connect 후에만 정확함.
- **YAML 저장 경로**: install/share/scm_gui/config/는 colcon이 매번 덮어씀. source dir로 fallback하지만 colcon symlink-install 권장.

---

## 10. 연관 패키지

- `rby1_core_msgs`, `scm_recording_msgs` — 사용하는 srv 정의
- `vive_rby1` — 본 GUI의 주요 백엔드
- `manus_inspire` — 캘리브레이션 서비스 제공자
- `teleop_bringup` — default launch에서 본 GUI 자동 기동
