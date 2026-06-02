# `vive_rby1` 개발자 가이드

> teleop 워크스페이스의 **가장 큰 패키지**. Vive Tracker 입력 → IK → RB-Y1 SDK Cartesian 명령 변환 + 녹화 상태 머신 + body tracker → torso teleop 통합.

---

## 1. 패키지 개요

이 패키지는 단일 실행 노드를 제공합니다:

| 실행 노드 | 언어 | 용도 | 기본 launch 포함? |
|---|---|---|---|
| `vive_rby1_node` | C++ (rby1-sdk dynamics FK) | **프로덕션 노드.** 트래커 → SDK Cartesian/Impedance 명령 (`/rby1/cmd/pose`). 녹화 상태 머신. body tracker → torso. | ✓ |

> FK는 `Robot::GetDynamics()`로 받는 **hw-core와 동일한 온보드 모델**에서 계산한다(pinocchio·로컬 URDF 불필요). 과거의 Python 디버그 노드(`vive_rby1_debug_node`)와 외부 IK(pink) 패키지 `rby1_ik`는 2026-06-02 제거됨 — 외부 IK를 쓰지 않고 SDK 내부 IK만 사용하기 때문.

핵심 책임:
- HTC Vive Tracker 3.0 두 개(좌/우 손) + 옵션 body tracker 입력 수신, smoothing
- 사용자의 손 trajectory를 로봇 좌표계로 변환 (`v2r_R_`), pos_scale로 스케일링
- SDK CartesianImpedance 명령(`tf2_msgs/TFMessage`)으로 `/rby1/cmd/pose` 발행 (100Hz)
- PCsensor 3-pedal 입력 처리 + 5-state 녹화 상태머신 (IDLE→ARMING→READY↔RECORDING↔PAUSED→IDLE)
- `scm_recording_msgs` 서비스 클라이언트 (외부 recording core와 통신)
- `rby1_core_msgs` 서비스 클라이언트 (hw-core 라이프사이클 호출)

---

## 2. 디렉토리 구조

```
src/core/vive_rby1/
├── CMakeLists.txt                              C++ 노드 빌드 (rby1-sdk 링크)
├── package.xml                                 ament_cmake
├── config/
│   └── vive_rby1.yaml                          (참고용 — 실제 권위는 launch dict)
└── src/
    └── vive_rby1_node.cpp                      C++ 프로덕션 노드 (단일 노드)
```

---

## 3. 빌드 / 실행

### 3.1 의존성

- ROS2 Humble + `colcon`
- `rby1-sdk` — FK용 dynamics 모델 (`find_package(rby1-sdk)`로 탐색; hw-core와 동일). pinocchio 불필요.
- 자매 패키지: `rby1_core_msgs`, `scm_recording_msgs` (teleop msgs)

### 3.2 빌드

```bash
cd teleop
source /opt/ros/humble/setup.bash
# rby1-sdk가 자동 탐색되지 않으면 빌드 디렉토리를 PREFIX로 전달 (hw-core와 동일)
colcon build --packages-select vive_rby1 \
  --cmake-args -DCMAKE_PREFIX_PATH=/path/to/rby1-sdk-*/build
source install/setup.bash
```

### 3.3 실행

```bash
# 프로덕션 (default launch에 포함됨)
ros2 run vive_rby1 vive_rby1_node --ros-args \
  -p robot_address:=192.168.30.1:50051 \
  -p robot_model:=a

# 또는 launch
ros2 launch teleop_bringup teleop.launch.py
```

---

## 4. C++ 프로덕션 노드 (`src/vive_rby1_node.cpp`)

### 4.1 클래스 구조

```cpp
namespace {
  struct SE3 { ... };                  // 경량 rigid transform (pinocchio::SE3 대체)
  class SdkFkSolver { ... };           // rby1-sdk GetDynamics() 기반 FK 제공자
  class ViveRby1Node : public rclcpp::Node {
    ...
  };
}
int main(...) { rclcpp::spin(make_shared<ViveRby1Node>()); }
```

`SdkFkSolver`는 노드 시작 시 백그라운드 스레드에서 `Robot<A|M>::Create(robot_address)` → `Connect()` → `GetDynamics()`로 hw-core와 동일한 온보드 모델을 받고, `MakeState({"base","ee_right","ee_left","link_torso_5"}, kRobotJointNames)` → `ComputeForwardKinematics`/`ComputeTransformation`으로 **FK만** 제공한다(IK는 hw-core가 수행). 로봇이 늦게 떠도 2초 간격 재시도하며, 연결 전에는 FK가 `std::nullopt`를 반환해 engage가 보류된다. 모델이 hw-core와 동일하므로 과거의 Z 오프셋 보정이 불필요하다.

### 4.2 ROS 인터페이스 (이 노드)

#### 파라미터 (launch dict가 권위)

| 파라미터 | 기본 | 단위 | 의미 |
|---|---|---|---|
| `robot_address` | `localhost:50051` | string | FK 모델용 rby1-sdk gRPC 주소 (`GetDynamics()`). hw-core가 연결하는 로봇과 동일해야 함 |
| `robot_model` | `a` | string | FK 모델 `a`(2륜) \| `m`(메카넘) |
| `topic_tracker_left` | `/teleop/tracker/left` | string | vive_ros2 출력 |
| `topic_tracker_right` | `/teleop/tracker/right` | string | 동일 |
| `topic_tracker_body` | `/teleop/tracker/body` | string | (옵션) body tracker |
| `topic_pedal` | `/teleop/pedal` | string | pedal_ros2 출력 |
| `topic_joint_state` | `/rby1/state/joint` | string | hw-core 인코더 |
| `pos_scale` | `0.5` | unitless | 트래커 → 로봇 위치 스케일 (hand trackers) |
| `torso_pos_scale` | `1.0` | unitless | body tracker → torso 위치 스케일 |
| `use_torso` | `false` | bool | body tracker → link_torso_5 활성화. **runtime toggle**: `/vive_rby1/set_use_torso` |
| `publish_rate` | `100.0` | Hz | timer 주기. hw-core RT 루프와 매치 권장 |
| `sdk_max_delta_pos` | `0.03` | m | per-frame Cartesian step clamp (SLERP/LERP) |
| `sdk_max_delta_rot_deg` | `20.0` | deg | per-frame 회전 step clamp |
| `tracker_smooth_alpha` | `0.9` | 0~1 | SLERP α. ↑ 입력 빠르게 추종 / ↓ 더 평활 (>0.8 권장) |
| `pedal_engage_index` | `0` | int | "A" 페달 — 클러치 토글 |
| `pedal_discard_index` | `1` | int | "B" 페달 — 에피소드 폐기 |
| `pedal_episode_index` | `2` | int | "C" 페달 — 에피소드 시작/종료 |

#### 구독 토픽

| 토픽 | 타입 | 콜백 |
|---|---|---|
| `/teleop/tracker/{left,right,body}` | `geometry_msgs/PoseStamped` | `onTrackerLeft/Right/Body` → smooth + 20-deep deque |
| `/teleop/pedal` | `sensor_msgs/Joy` | `onPedal` → 3개 페달 edge detection |
| `/rby1/state/joint` | `sensor_msgs/JointState` | `onJointState` → `ik_solver_->updateFromJointState` (FK용 q 갱신) |
| `/teleop/task_id` | `std_msgs/Int32` | task_id 갱신 |
| `/teleop/mirror_mode` | `std_msgs/String` | `"mirror"`/`"normal"` 토글 |

> warmup/cooldown hold는 로컬 SDK FK(`publishEeHold`)로 계산하므로 `/rby1/state/ee_pose` 구독은 제거됨(2026-06-02).

#### 발행 토픽

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/rby1/cmd/pose` | `tf2_msgs/TFMessage` | **SDK Cartesian 명령**. `transforms[].child_frame_id` ∈ {`ee_right`, `ee_left`, `link_torso_5`}. `header.frame_id="base"`. 2026-05-27 `LinkPoseCommand` → `TFMessage` 마이그레이션. |
| `/teleop/rec_state` | `std_msgs/String` | 녹화 상태: IDLE/ARMING/READY/RECORDING/PAUSED |
| `/teleop/rec_episode` | `std_msgs/Int32` | 현재 에피소드 번호 (또는 -1) |
| `/teleop/tracker_status` | `std_msgs/String` | `L:OK/JITTER/LOST R:OK/JITTER/LOST` (+ `B:` 있을 때) |
| `/teleop/clutch_state` | `std_msgs/String` | `ENGAGED`/`DISENGAGED` |

#### 서비스 서버

| 서비스 | 타입 | 동작 |
|---|---|---|
| `/vive_rby1/teleop_start` | `std_srvs/Trigger` | detached thread에서 doTeleopStart (mode→move_to_pose→stream) |
| `/vive_rby1/teleop_stop` | `std_srvs/Trigger` | detached thread에서 doTeleopStop (stream→move_to_pose) |
| `/vive_rby1/toggle_clutch` | `std_srvs/Trigger` | engage/disengage 토글 |
| `/vive_rby1/set_teleop_pose` | `scm_recording_msgs/SetTeleOpPose` | teleop pose 갱신 + hw-core `nullspace_joint_ref` 동시 전송 (2026-05-22 추가) |
| `/vive_rby1/toggle_episode` | `std_srvs/Trigger` | 페달 C와 동일 (`toggleEpisode`) |
| `/vive_rby1/set_use_torso` | `std_srvs/SetBool` | body tracker → torso 런타임 enable/disable (2026-05-22 추가) |

#### 서비스 클라이언트

| 서비스 | 타입 | 호출 시점 |
|---|---|---|
| `/scm_recording/start` | `StartRecording` | `toggleEpisode` (IDLE 시) — task_id로 새 에피소드 |
| `/scm_recording/end` | `EndRecording` | `toggleEpisode` (READY/PAUSED 시), `discardEpisode` (discard=true) |
| `/scm_recording/toggle_pause` | `TogglePause` | `engage`/`disengage` 시 |
| `/rby1/ctrl/mode` | `SetControlMode` | doTeleopStart (cartesian, impedance 고정) |
| `/rby1/stream` | `SetStream` | doTeleopStart/Stop |
| `/rby1/move_to_joint_position` | `MoveToJointPosition` | doTeleopStart/Stop (`teleop_pose_`로 이동) |
| `/rby1/set_nullspace_joint_ref` | `SetNullspaceJointRef` | `set_teleop_pose` 콜백 + doTeleopStart 후 재전송 (2026-05-22) |

### 4.3 5-state 녹화 상태머신

```
            (toggleEpisode/페달 C)
   IDLE ──────────────────────────► ARMING
    ▲                                  │
    │ (toggleEpisode at PAUSED)        │ (doTeleopStart 완료)
    │                                  ▼
    │     (페달 A engage)            READY
    │   ◄──────── PAUSED ◄─────────► RECORDING ┐
    │                  (페달 A disengage)       │
    │                                            │
    └──────────── (페달 B discard) ──────────────┘
                  rec_state = IDLE
```

상태값(`rec_state_`):
- `IDLE` ("IDLE") — 활성 세션 없음
- `ARMING` — `toggleEpisode(start)` → `/scm_recording/start` 성공 → `doTeleopStart` 진행 중 (mode + move + stream)
- `READY` — `doTeleopStart` 완료. 페달 A로 engage 대기.
- `RECORDING` — 클러치 engaged. `callTogglePause` 응답에서 `paused=false`.
- `PAUSED` — 클러치 disengaged. `callTogglePause` 응답에서 `paused=true`.

⚠ **PAUSED에서만 EndRecording 허용** — `toggleEpisode`가 RECORDING에서 호출되면 거부 + WARN.

### 4.4 페달 매핑

| 페달 인덱스 | 기본 이름 | 함수 |
|---|---|---|
| 0 | A (left) | `engage()` / `disengage()` 토글. RECORDING ↔ PAUSED 전환. |
| 1 | B (center) | `discardEpisode()` — `EndRecording(discard=true)` 호출, 즉시 IDLE로. |
| 2 | C (right) | `toggleEpisode()` — IDLE→ARMING (시작) / PAUSED→IDLE (종료). |

edge detection: 직전 tick에 안 눌려 있고 이번 tick에 눌렸을 때만 발화 (`pedal_*_prev_` 플래그).

### 4.5 좌표계 변환 (Vive → 로봇)

생성자에서 `v2r_R_` 회전 행렬 빌드. Vive base station 좌표계(보통 X=오른쪽, Y=위, Z=뒤)를 로봇 base frame(X=앞, Y=왼쪽, Z=위)으로 매핑.

```cpp
// 트래커 변위 → 로봇 base frame
Eigen::Vector3d delta_l = tracker_l_.smoothed->translation() - ref_l_->translation();
Eigen::Vector3d target_pos_l = ee_l_0_->translation() + pos_scale_ * (v2r_R_ * delta_l);

// 회전: dR을 양쪽에 v2r_R_ * dR * v2r_R_.transpose()로 적용 (similarity transform)
Eigen::Matrix3d dR_l_robot = v2r_R_ * dR_l * v2r_R_.transpose();
```

**mirror mode**: 좌우 손이 바뀌어 보이고 Y축 부호 반전. 양손 자체 교차 작업에 유용.

### 4.6 engage() 캡처 시점

페달 A press → `engage()`:
1. 트래커 smoothed 확인 (raw + smooth 모두 필요) + FK 모델 연결 확인 (`ik_solver_->connected()`)
2. `ref_l_` / `ref_r_` ← 현재 트래커 pose (이후 delta 기준점)
3. `ee_l_0_`/`ee_r_0_`, `sdk_ee_l_0_`/`sdk_ee_r_0_` ← SDK FK로 `ee_left/right` 프레임 캡처. **모델이 hw-core와 동일하므로 Z 오프셋 보정 없음**(과거 핵 제거).
4. (`use_torso_` + body tracker 있을 때) `ref_body_`/`torso5_0_` 캡처
5. `engaged_=true`, READY/PAUSED 상태면 togglepause 자동 호출

### 4.7 onTimer() 메인 루프 (100Hz)

```cpp
1. /teleop/tracker_status publish (L:/R: + 옵션 B:)
2. warmup_ticks_ > 0이면: publishEeHold() (로컬 FK로 현재 ee pose hold) 후 return (스트림 시작 직후 점프 방지)
3. cooldown_ticks_ > 0이면: publishEeHold() 후 return (disengage 직후 잔여 모션 정지)
4. 트래커 미준비면 return
5. engaged_/ref_*/ee_*_0_ 모두 OK 확인 → 미만족 시 return
6. left/right delta 계산 + mirror mode 분기
7. SDK target = ee_*_0_ + delta
8. limitSdkTarget()으로 per-frame step clamp (sdk_max_delta_pos/rot)
9. tf2_msgs/TFMessage 메시지 구성: ee_right, ee_left, (옵션) link_torso_5
10. publish
```

### 4.8 doTeleopStart / doTeleopStop (detached thread)

`doTeleopStart`:
1. `SetControlMode(source="cartesian", control="impedance")`
2. `MoveToJointPosition(target=teleop_pose_, min_time=5)` — 30s 타임아웃
3. `SetNullspaceJointRef(teleop_pose_)` — 스트림 직전에 재전송 (드롭다운 미조작 시에도 자세 반영, 2026-05-22 추가)
4. `SetStream(enable=true)` — 10s 타임아웃, `warmup_ticks_ = publish_rate_` (1초)
5. ARMING 상태면 READY로, `teleop_active_ = true`

`doTeleopStop`:
1. `SetStream(enable=false)`
2. `MoveToJointPosition(teleop_pose_, 5s)` — 안전 자세 복귀

### 4.9 body tracker → torso (2026-05-22 추가)

- `use_torso_` 플래그 (파라미터 + `/vive_rby1/set_use_torso` 런타임 토글)
- engage 시점에 body tracker 있으면 `ref_body_`/`torso5_0_` 캡처
- 늦은 재캡처: `onTrackerBody`에서 `engaged_ && use_torso_ && !ref_body_`이면 처음 받은 시점에 캡처 (engage 후에 body tracker가 켜져도 부드럽게 합류)
- onTimer에서 `transforms[].child_frame_id="link_torso_5"` 항목 추가 (CartesianImpedance 모드에서만 hw-core가 소비)
- `use_torso_=false` 시 `ref_body_`/`torso5_0_` reset → hw-core가 last torso pose에서 hold

---

## 5. (제거됨) Python 디버그 노드 / 외부 IK

과거에는 pink IK(`rby1_ik` 패키지)를 사용하는 Python 디버그 노드(`vive_rby1_debug_node`)가 4개 제어 모드(pink_position/impedance + sdk_position/impedance)를 지원했으나, 외부 IK를 쓰지 않고 SDK 내부 IK만 사용하기로 하여 디버그 노드와 `rby1_ik` 패키지 모두 2026-06-02 제거되었다. 현재 프로덕션 경로는 SDK CartesianImpedance(`/rby1/cmd/pose`) 단일 모드다.

---

## 6. 핵심 함수별 요약

| 함수 | 위치 | 역할 |
|---|---|---|
| `poseStampedToSe3` / `se3ToPose` / `poseToTransform` / `makeTransformStamped` / `se3ToTransformStamped` | 익명 ns | ROS msg ↔ 로컬 `SE3` 변환 헬퍼 |
| `SdkFkSolver::connect` | 익명 ns | `Robot::Create`+`Connect`+`GetDynamics`로 모델 빌드 (재시도 가능) |
| `SdkFkSolver::updateFromJointState` | 익명 ns | `/rby1/state/joint` 콜백 → 전체 DOF q 갱신 (이름 매핑) |
| `SdkFkSolver::framePlacement` | 익명 ns | 명명된 frame(ee_right/ee_left/link_torso_5)의 `SE3` 반환 (FK); 미연결 시 nullopt |
| `publishEeHold` | C++ 노드 | 현재 ee FK pose를 hold 명령으로 publish (warmup/cooldown) |
| `ViveRby1Node` 생성자 | — | 파라미터 선언/읽기, FK 연결 스레드, 토픽·서비스 와이어링, 타이머 시작 |
| `onTrackerLeft/Right/Body` | `:415-449` | 트래커 raw 저장 + deque 갱신 + smoothing |
| `onPedal` | `:497-526` | 3 페달 edge detection |
| `engage` / `disengage` | `:528-568` | 클러치 토글. ref/ee_0 캡처. 자동 togglepause. |
| `discardEpisode` | `:570-602` | EndRecording(discard=true) + 강제 IDLE |
| `toggleEpisode` | `:604-651` | 페달 C 또는 GUI 버튼. IDLE↔ARMING/PAUSED→IDLE. |
| `callTogglePause` | `:653-672` | TogglePause + rec_state 갱신 |
| `on_set_teleop_pose` | `:704-717` | teleop pose 갱신 + nullspace_joint_ref 동시 전송 |
| `doTeleopStart` / `doTeleopStop` | `:721-813` | detached thread 시퀀스 (mode + move + stream) |
| `limitSdkTarget` | `:825-861` | per-frame Cartesian step clamp (SLERP) |
| `smoothTracker` | `:863-885` | 50mm 위치 클램프 + SLERP 회전 평활 |
| `trackerStatus` | `:887-914` | OK/JITTER/LOST 분류 — 0.5s 타임아웃 + sliding window σ |
| `onTimer` | `:916-1014` | 메인 100Hz 루프 — tracker_status + (engaged 시) SDK target 계산 + publish |

---

## 7. 확장 / 수정 가이드

### 7.1 새 입력 디바이스 추가 (예: 새 트래커, hand controller)

1. 새 구독 토픽 추가 (`onTrackerXyz`)
2. `TrackerState` 사용 또는 유사 구조 정의
3. `trackerStatus()` 호출에 포함 (헬스 표시)
4. `engage()`에서 ref 캡처
5. `onTimer`에서 delta 계산 + target 빌드 + `transforms[]`에 push

### 7.2 새 control mode 추가 (예: hybrid position+force)

C++ 프로덕션 노드는 SDK Cartesian/Impedance 전용. 다른 모드가 필요하면 hw-core `/rby1/ctrl/mode`로 모드를 바꾸고 대응 명령 토픽(`/rby1/cmd/joint` 등)을 발행하도록 확장.

### 7.3 새 페달 동작 추가

1. `pedal_*_idx_` 파라미터 추가 (4번째 페달이라면)
2. `onPedal`에 edge detection 블록 추가
3. 대응 함수 (`doSomething`) 작성

### 7.4 녹화 상태 추가

1. `constexpr char kRecXxx[] = "XXX"` 추가
2. 전이 가이드 다이어그램 갱신
3. `toggleEpisode`/`engage`/`disengage`에 분기 추가
4. GUI 측 `scm_gui` 색상 매핑도 갱신 필요 ([`../../gui/scm_gui/DEVELOPER.ko.md`](../../gui/scm_gui/DEVELOPER.ko.md))

---

## 8. 흔한 함정

- **mode 미설정 + stream**: `doTeleopStart`가 mode를 명시 설정하지만, 외부에서 mode를 cartesian/impedance 외 다른 값으로 바꾼 후 stream만 켜면 토픽 발행은 되지만 hw-core에서 silent 무시. status JSON `ctr_type` 가드.
- **`/vive_rby1/set_use_torso`로 끄고 body tracker 데이터가 끊긴 듯 보임**: `use_torso=false`면 의도적으로 publish 안 함. hw-core가 마지막 torso pose 유지.
- **engage 시점에 body tracker 미수신**: 늦은 재캡처(2026-05-22)가 작동하므로 그냥 기다리면 됨.
- **mirror_mode 토글 후 큰 점프**: `mirror_mode_` 변경 시 `engage()`처럼 ref/ee_0 재캡처 — 이미 코드에서 처리.
- **녹화 RECORDING에서 EndRecording 호출**: 거부됨. PAUSED로 먼저 (페달 A로 disengage).
- **engage가 안 됨 / 점프**: FK 모델이 `GetDynamics()`로 로딩되기 전(`robot_address` 미도달)에는 engage가 `SDK dynamics model not ready yet`로 보류된다. 또 `robot_address`가 hw-core가 연결한 로봇과 다르면 FK 불일치로 engage 시 점프할 수 있다 — 동일 로봇을 가리켜야 한다.

---

## 9. 연관 패키지

- `rby1-sdk` — FK용 dynamics 모델 (`GetDynamics()`); hw-core와 동일 모델
- `vive_ros2` (input) — `/teleop/tracker/{left,right,body}` 발행
- `pedal_ros2` (input) — `/teleop/pedal` 발행
- `manus_ros2` (input) — Manus glove (별도; `manus_inspire`가 처리)
- `scm_gui` (gui) — 본 노드의 `/teleop/*` 토픽 구독자, 라이프사이클 서비스 호출자
- 외부: `rby1_core_node` (hw-core), `scm_recording` core (별도 외부 패키지)
