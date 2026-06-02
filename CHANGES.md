# CHANGES

## 2026-06-02

### scm_gui: Nullspace Ref Pose 드롭다운에 안내 문구 추가

- **배경:** GUI "Nullspace Ref Pose" 드롭다운의 "Apply Ref Pose"(`_on_apply_ns_ref`)는 `/rby1/set_nullspace_joint_ref`뿐 아니라 `/vive_rby1/set_teleop_pose`도 함께 호출해 `teleop_pose_`(Teleop Start / Pedal C 시 로봇이 이동하는 joint target)까지 덮어쓴다. 즉 이 드롭다운이 사실상 teleop 시작 자세 선택을 겸하지만 화면상 안내가 없었고, 드롭다운만 바꾸고 Apply를 누르지 않으면 적용되지 않는다는 점도 표시되지 않았다.
- **변경 (`src/gui/scm_gui/scm_gui/scm_gui_node.py`):** 드롭다운(`_cmb_ns_ref`) 아래, `addStretch()` 앞에 helper `QLabel` 2개 추가(`QFont('Monospace', 8)`, `color: #555555`, `setWordWrap(True)` — 기존 helper 스타일 일치).
  - 라인1: teleop 시작 자세 겸용 + nullspace 역할 설명 ("Also the Teleop Start pose: ... biases IK toward this configuration in the nullspace ... (does not move the end-effector).")
  - 라인2: Apply 안내 ("Click 'Apply Ref Pose' to apply — changing the selection alone has no effect.")
- **동작 변경 없음.** 표시 문구만 추가.
- **검증:** GUI-only(호스트 colcon 불가, Docker 전용). `scm_gui` 실행 후 드롭다운 아래 회색 안내 2줄이 컬럼 폭 내에서 줄바꿈되어 표시되는지 육안 확인.

### vive_rby1: disengage 직후 EE 잔여 모션 제거 (cooldown hold)

- **배경:** pedal A 해제(disengage) 시 `vive_rby1`는 `/rby1/cmd/pose` publish만 멈출 뿐, hw-core stream loop는 마지막 캐시 target(`T_r/T_l`)을 100Hz로 펌웨어에 계속 재전송한다. 빠른 트래커 모션 끝에서 손을 떼면 펌웨어 CartesianImpedance가 그 캐시 target까지 velocity-limited(`ee_lin_vel=0.4 m/s`)로 추종하며 ~1초간 팔이 미끄러지는 잔여 모션 발생. legacy/현재 공통 약점(전용 freeze 명령 부재, 펌웨어 hold-time에만 의존).
- **변경 (단일 노드, `src/core/vive_rby1/src/vive_rby1_node.cpp`):** 기존 `warmup_ticks_` 패턴을 disengage 측에 대칭으로 추가.
  - `disengage()`에서 `cooldown_ticks_ = round(cooldown_sec_ * publish_rate_)` 설정.
  - `onTimer()` warmup 분기 직후에 cooldown 분기 추가 — 현재 FK pose(`last_ee_pose_` = `/rby1/state/ee_pose`)를 `ee_right`/`ee_left` hold로 publish해 hw-core 캐시 target을 실제 EE 위치로 snap(`has_new=true`) → 펌웨어 target≈actual → 모션 즉시 정지.
  - `engage()`에서 `cooldown_ticks_=0`으로 진행 중 cooldown 즉시 취소(재engage 우선, 점프 없음).
  - torso(`link_torso_5`)는 warmup과 동일하게 hold에서 제외(stream 시작 시 seed된 `T_torso`를 펌웨어 impedance가 유지).
- **신규 파라미터:** `cooldown_sec` (기본 0.5, 0이면 비활성). `teleop.launch.py`의 vive_rby1_node 블록에 `'cooldown_sec': 0.5` 노출.
- **hw-core 변경 없음.** ControlHoldTime은 stream 단절 시 grace timer로 별개 — 줄여도 본 잔여 모션은 해결되지 않고 일시적 지연에 대한 안정성만 악화되므로 건드리지 않음.
- **호환성:** 신규 파라미터는 기본값 보존, 기존 동작에 영향 없음. disengage 구간(`!engaged_`)에만 작동하므로 master perceived delay 없음.
- **검증:** Docker colcon `--packages-select vive_rby1` 빌드. 실로봇에서 빠른 트래커 모션 중 pedal A 해제 시 ≤cooldown_sec 내 정지 확인. hw-core stream 로그에 cooldown 기간 `has_new=1` 유입 확인.

## 2026-06-01

### 패키지별 한국어 개발자 가이드(`DEVELOPER.ko.md`) 신설 + README 이중언어화 + 핵심 파일 헤더 주석 추가

- **배경:** teleop 워크스페이스에 13개 패키지가 있는데 패키지 내부 구조(파일 책임/함수 역할/파라미터·토픽·서비스 상세)를 코드 옆에서 바로 참고할 수 있는 문서가 부재. 외부/신규 개발자 진입장벽이 큼.
- **신규 MD (13개):**
  - `src/core/vive_rby1/DEVELOPER.ko.md` — C++ 프로덕션 + Python debug, 5-state 녹화 머신, body tracker → torso, 핵심 함수 매핑, 흔한 함정
  - `src/core/manus_inspire/DEVELOPER.ko.md` — 4-phase 16s 캘리브 절차, ergonomic → Inspire 매핑, 캘리브 파일 구조
  - `src/core/rby1_ik/DEVELOPER.ko.md` — legacy 위치 표기 (debug 노드 전용), `OneEuroFilterVec`/`Rby1IK` 요약
  - `src/gui/scm_gui/DEVELOPER.ko.md` — PySide6 + 백그라운드 노드, Signal-Slot 패턴, 5개 노드 그룹 모니터링, Mobile Base Panel
  - `src/input/pedal_ros2/DEVELOPER.ko.md` — evdev + grab() + KEY_MAP, input 그룹 권한
  - `src/input/vive_ros2/DEVELOPER.ko.md` — OpenVR 폴링, OpenVR→ROS 좌표 변환, base station 정렬
  - `src/input/manus_ros2/DEVELOPER.ko.md` — Manus SDK Integrated 모드, 5개 콜백, ManusSDK 바이너리 배치
  - `src/launch/teleop_bringup/DEVELOPER.ko.md` — launch 인자/노드별 파라미터/기동 순서
  - `src/msgs/rby1_core_msgs/DEVELOPER.ko.md` — hw-core 사본 동기화 절차 (간략 + 정본 link)
  - `src/msgs/inspire_hand_msgs/DEVELOPER.ko.md` — 동일
  - `src/msgs/manus_ros2_msgs/DEVELOPER.ko.md` — 3개 msg 필드, raw_node_count vs size() 함정
  - `src/msgs/scm_recording_msgs/DEVELOPER.ko.md` — 외부 scm_recording core와의 srv 계약, 5개 srv 의미
  - `src/msgs/interbotix_xs_msgs/DEVELOPER.ko.md` — COLCON_IGNORE 이유, system 설치 가정
- **README.md 이중언어화:** 한국어 본문(상단) + 영어 번역(하단). 패키지별 가이드 cross-link, 페달 매핑/제어 모드/녹화 워크플로/Mobile Base Panel/keyboard driving 표 보강.
- **코드 헤더 주석 추가 (CLAUDE.md "WHY 비자명한 곳에만" 정책 준수):**
  - `src/core/vive_rby1/src/vive_rby1_node.cpp` 상단 + `engage()`/`onSetUseTorso`/doTeleopStart의 nullspace 재전송 직전 — 책임 요약, 캡처 시점 의미, body tracker on/off 의도
  - `src/core/manus_inspire/manus_inspire/manus_inspire.py` 상단 — 캘리브 4-phase 의미 + 매핑 흐름
  - `src/gui/scm_gui/scm_gui/scm_gui_node.py` 상단 — 레이아웃 + Signal-Slot 패턴 + depth=1 의도(2026-05-30)
- **호환성:** 코드 변경은 주석뿐 — 빌드/런타임 영향 없음.
- **검증:** colcon 빌드 무관. cross-link 경로 grep 확인.

## 2026-05-30

### scm_gui: 키보드 드라이빙 기본값 및 포커스 수정

- Linear/Angular velocity 기본값 변경: 0.3 → 0.05 / 0.10.
- 스핀박스 입력 후 Enter를 치거나 GUI 아무 곳이나 클릭하면 포커스 해제되어 즉시 키보드 조작 가능:
  - 각 스핀박스에 `editingFinished → clearFocus()` 연결.
  - `TeleopGuiWindow.mousePressEvent` 오버라이드 → `QApplication.focusWidget().clearFocus()`.

### scm_gui: q/e 방향 수정, VLA indicator 추가, 배터리 표시, depth 최소화

- **q/e 회전 방향 수정 (`scm_gui_node.py`):**
  - `_on_drive_tick()` yaw 공식에서 Q(+)/E(-) 부호가 반전되어 있던 것을 수정.
  - 버튼 아이콘도 Q=↻, E=↺ 로 교체.

- **VLA 노드 indicator 추가:**
  - `VLA_NODES = [('rby1_vla_client', 'vla_ros2_bridge')]` 추가.
  - `MODULE_ORDER` VLA 행에 연결, `NODES_TO_WATCH` 포함 → ROS2 Node Status 패널에 자동 표시.

- **배터리 상태 표시:**
  - `sensor_msgs/BatteryState` import 추가.
  - `/rby1/state/battery` 구독 (depth=1), `_cb_battery` → `battery_changed` Signal → `_on_battery` 핸들러.
  - 상태 표시줄 Gripper 옆에 `Battery` 라벨 추가 (50%↑ 초록 / 20%↑ 노랑 / 미만 빨강).

- **비실시간 구독 depth 최소화 (10 → 1):**
  - `/rby1/state/status`, `/rby1/state/joint`, `/teleop/tracker_status`, `/rby1/state/battery`.

## 2026-05-29

### scm_gui: Servo On/Off 분리 — body only / Mobile On/Off 버튼 추가

- **배경:** 기존 Servo On/Off는 torso·arm·wheel을 모두 제어해 의도치 않게 모바일 베이스도 함께 켜졌음. 주행 활성화를 body servo와 분리해 안전하게 운용할 수 있게 함.
- **변경 (`scm_gui/scm_gui_node.py`):**
  - `ScmGuiNode.call_servo()`에 `wheel_only` 파라미터 추가 → `SetServo.Request.wheel_only` 전달.
  - **Servo On/Off** 버튼: `no_wheel=True` 로 호출 변경 — torso·right_arm·left_arm joints 만 제어.
  - **Mobile On/Off** 버튼 신규 추가 (Ctrl Enable 오른쪽, 위아래 배치): `wheel_only=True` 로 호출 — wheel joints 만 제어.
  - **Gripper Init** 버튼: 열 4 → 열 5로 이동 (Mobile On이 열 4를 점유).
  - **상태 표시줄** `_build_status_row()`: `'Control'` 레이블을 `'Ctrl'`로 변경, `Mobile` indicator 신규 추가 (Ctrl Enabled와 Gripper 사이).
  - `_on_rby1_status()`: control_state 표시 텍스트를 `'Ctrl Enabled'`/`'Ctrl FAULT'`/`'Ctrl Idle'`로 변경.

## 2026-05-28

### scm_gui: Mobile Base Panel 추가 — 키보드 수동 주행

- **배경:** GUI에서 로봇 주행을 직접 조작할 수단이 없었음. hw-core의 `/rby1/cmd/base_vel` (`geometry_msgs/Twist`) 토픽을 활용.
- **변경 (`scm_gui/scm_gui_node.py`):**
  - `geometry_msgs/Twist` 퍼블리셔 (`/rby1/cmd/base_vel`) 추가.
  - `ScmGuiNode.publish_base_vel(vx, vy, yaw)` 및 `set_mobility_accel(linear, angular)` 메서드 추가.
  - **Mobile Base Panel** 그룹을 ROS2 Node Status 오른쪽(2:1 비율)에 배치:
    - **Manual Driving (Keyboard)** 서브그룹: Enable 체크박스, QWEASD 키 버튼(눌림 시 파란색 하이라이트), Linear/Angular 속도 입력 스핀박스.
    - **Driving Parameter Manager** 서브그룹: `accel_limit_linear` / `accel_limit_angular` 입력 + Apply 버튼 (hw-core 파라미터 서비스로 런타임 변경).
  - 키 매핑: W=전진, S=후진, A=좌측 스트레이프, D=우측 스트레이프, Q=좌회전, E=우회전.
  - Model A에서 A/D 키는 SDK가 자동으로 Y 성분 무시 — GUI 필터링 불필요.

## 2026-05-27

### `/rby1/cmd/pose` 메시지 타입 교체: `rby1_core_msgs/LinkPoseCommand` → `tf2_msgs/TFMessage` (데이터용)

- **요지:** hw-core 인터페이스 변경에 맞춰 teleop publisher를 일괄 마이그레이션. `tf2_msgs/TFMessage`를 일반 데이터 메시지로 발행 — 토픽 이름이 `/tf*`가 아니므로 TF subsystem과 충돌 없음.
- **새 메시지 규약 (publisher 측):**
  - `transforms[]` 각 항목의 `child_frame_id` 가 타깃 링크: `"ee_right"`, `"ee_left"`, 옵션 `"link_torso_5"`.
  - `header.frame_id = "base"`, `header.stamp` 는 publish 시점(모든 항목 동일 stamp).
  - `transform.translation`/`rotation` 으로 6-DOF 포즈. SE3 → Transform 변환은 회전행렬→쿼터니언 1회 + 7 double 복사로 기존 SE3→Pose와 동일 비용 (100Hz 부담 없음).
  - 가변 길이: warmup/메인 루프 모두 `ee_right`, `ee_left` 먼저, `use_torso_` 시 `link_torso_5` append.
- **변경 파일:**
  - `src/msgs/rby1_core_msgs/msg/LinkPoseCommand.msg` 삭제 + `CMakeLists.txt`에서 제거 (hw-core 사본과 동기화).
  - `src/core/vive_rby1/src/vive_rby1_node.cpp` — include/publisher 타입/메시지 빌드 교체. `poseToTransform`, `makeTransformStamped`, `se3ToTransformStamped` 헬퍼 추가.
  - `src/core/vive_rby1/vive_rby1/vive_rby1_node.py` (debug 노드) — import/publisher/메시지 빌드 교체. `se3_to_transform`, `make_transform_stamped` 헬퍼 추가. 기존 `msg.header.*` 데드코드는 자연스럽게 제거(TFMessage엔 top-level header 없음).
  - `src/core/vive_rby1/CMakeLists.txt`, `package.xml` — `tf2_msgs` 의존 추가.
  - `README.md` — 토픽 표 갱신.
- **외부 통합자 영향:** `/rby1/cmd/pose`를 직접 구독하던 외부 모듈은 `tf2_msgs/TFMessage` 로 마이그레이션 필요. hw-core 측 동등 변경([hw-core CHANGES.md 2026-05-27](../hw-core/CHANGES.md)) 참고.
- **검증:** `cd teleop && colcon build --packages-up-to rby1_core_msgs vive_rby1`. 런타임: `ros2 topic echo /rby1/cmd/pose --once`로 `transforms[].child_frame_id` 가 `ee_right`/`ee_left` (+ torso 활성 시 `link_torso_5`) 확인. `ros2 topic echo /tf --once`로 본 메시지가 TF에 누설되지 않음 확인.

## 2026-05-22 (3)

### GUI — Impedance Preset에 Nullspace Ref Pose 통합, Teleop Pose dropdown 제거

- **Teleop Pose dropdown 제거**: Teleop 섹션의 "Teleop Pose:" dropdown 삭제. teleop pose는 Impedance Preset을 통해 설정.
- **Cartesian Impedance Params — Nullspace Ref Pose 행 추가**: 두 컬럼(Joint Limits, Nullspace Weights) 아래에 `Nullspace Ref Pose:` dropdown + `[Apply Nullspace Ref]` 버튼 추가.
  - dropdown 후보: joint position preset 목록(`named_poses.yaml`)에서 동적으로 채워짐.
  - Joint Position preset 신규 저장 시 Nullspace Ref dropdown에도 자동 추가.
  - `[Apply Nullspace Ref]`: 선택된 joint position preset의 값을 두 서비스에 동시 전송 — `/vive_rby1/set_teleop_pose`(teleop start/stop 이동 자세) + `/rby1/set_nullspace_joint_ref`(CartesianImpedance nullspace_ref_rad 즉시 적용). `named_poses.yaml` 수정 없음.
- **Impedance preset 구조 확장**: `nullspace_ref` 필드(joint position preset 이름) 추가. Load 시 세 값 모두 복원, Save 시 함께 저장.
- **Dirty 상태 추적**: joint limits 행 추가/제거/값변경, nullspace weight 변경, nullspace_ref dropdown 변경 시 impedance preset combobox 자동 blank.
- **ScmGuiNode**: `SetNullspaceJointRef` import + `_cli_set_ns_ref` 클라이언트 + `call_set_nullspace_joint_ref()` 메서드 추가.
- `config/impedance_presets.yaml`: default preset에 `nullspace_ref: ''` 필드 추가.

## 2026-05-22 (2)

### GUI — Cartesian Impedance Params 섹션 추가

- **신규 GUI 섹션 "Cartesian Impedance Params"**: Joint Position 섹션 아래에 추가.
  - **Joint Limits 테이블**: `[+ Add Joint]` 으로 행 추가, 드롭다운에서 body joint 선택(torso_0~5, right/left_arm_0~6), min/max spinbox, `[X]` 삭제. `[Apply Joint Limits]` 로 `/rby1/set_cartesian_joint_limits` 서비스 호출.
  - **Nullspace Weights 테이블**: right/left arm 0~6 각 14개 spinbox 고정. `[Apply Weights]` 로 `/rby1/set_nullspace_weight` 서비스 호출.
  - **Preset 저장/불러오기**: `impedance_presets.yaml`에 저장. 동일 이름 저장 시 overwrite. 기본 preset `default` 포함.
- `setup.py`: `impedance_presets.yaml` install 경로 추가.

## 2026-05-22

### torso(body tracker) teleop 개선 — on/off 토글, 헬스 표시, 늦은 재캡처, launch 파라미터 복원

- **배경:** `teleop-branch`(다른 개발자, 구 인터페이스)와 전체 코드 비교 결과, torso teleop 기능(body tracker → `link_torso_5` CartesianImpedance)은 **우리 코드에 이미 완비**되어 있었음(공통 조상 커밋 `3f45202`/hw-core `738c97c`에서 개발, 우리 리팩터가 name-keyed `LinkPoseCommand` + YAML 파라미터화로 유지·개선). 따라서 포팅이 아니라 **양쪽 코드 모두에 없던 개선점 4건**을 추가함. 변경은 teleop 단독, hw-core 변경 없음.
- `core/vive_rby1/src/vive_rby1_node.cpp`:
  - **torso on/off 토글:** `use_torso` 파라미터(노드 기본 false) + `/teleop/use_torso`(`std_msgs/Bool`) 런타임 구독 및 `/vive_rby1/set_use_torso`(`std_srvs/SetBool`) 서비스 추가. 기존 `mirror_mode`(`/teleop/mirror_mode`) 패턴을 그대로 따름. off 전환 시 `ref_body_`/`torso5_0_` reset(→ teleop가 `link_torso_5` 전송 중단 → hw-core가 마지막 torso 포즈 유지/freeze), on 재전환 시 engage 중이면 현재 torso 포즈 기준으로 재캡처. engage 캡처(`engage()`)와 스트림 전송 블록 모두 `use_torso_` 게이트 추가.
  - **늦은 body tracker 재캡처:** `onTrackerBody`에서 `engaged_ && use_torso_ && !ref_body_`이면 처음 들어온 시점에 `ref_body_`/`torso5_0_` 캡처. engage 시점에 body tracker가 없던 경우에도 재engage 없이 torso가 부드럽게 합류.
  - **body tracker 헬스 표시:** `onTimer`의 `/teleop/tracker_status` 문자열에 body tracker 수신 이력이 있을 때만 `B:OK/JITTER/LOST` 추가(`trackerStatus()` 재사용). 미설치 시 상시 `B:LOST` 노이즈 방지.
  - include `std_msgs/msg/bool.hpp`, 멤버 `sub_use_torso_`/`use_torso_` 추가.
- `teleop_bringup/launch/teleop.launch.py`: `vive_rby1_node` 파라미터에 `torso_pos_scale: 1.0`(우리 launch에서 누락되어 노드 기본값 의존하던 것 복원), `use_torso: False` 추가(기본 off — GUI `Use Torso` 체크박스로 런타임에 enable). 이전 본 changelog 초기 작성 시점에 `True`로 기록되어 있었으나 이후 적용된 launch에서는 `False`로 머지되었음.
- `core/vive_rby1/config/vive_rby1.yaml`: `torso_pos_scale`, `use_torso` 항목 문서화(주석 포함). 단, 해당 yaml은 stale하며 실제 권위는 launch dict.
- 검증: ROS 미설치 본(WSL) 환경 → colcon 빌드 미수행(빌드는 Docker/`docker/Dockerfile.teleop`). launch는 `python3 -m py_compile` 통과. 사용자 ROS2 환경에서 `cd teleop && colcon build --packages-select vive_rby1` 필요. 런타임: `ros2 param get /vive_rby1_node use_torso`, body tracker 가동 시 `/teleop/tracker_status`에 `B:` 표시, `ros2 topic pub -1 /teleop/use_torso std_msgs/Bool "{data: false}"` 후 `/rby1/cmd/pose`에서 `link_torso_5` 사라짐/`true`로 복귀 확인.

### scm_gui: /rby1/state/status 파싱을 bool/has_gripper 로 갱신

- `gui/scm_gui/scm_gui/scm_gui_node.py` `_on_rby1_status`: hw-core가 status JSON의 `power_state`/`servo_state`/`stream_state` 를 문자열 `"True"/"False"` → **JSON bool** 로, `gripper_state` → **`has_gripper`(bool)** 로 바꿈에 맞춰 파싱 수정. `== 'True'` 비교 제거 → `bool(data.get(...))`, `gripper = bool(data.get('has_gripper', False))`.
- 부수 효과: 기존엔 `gripper_state`(=no_gripper)를 그대로 써서 그리퍼 없을 때 "Gripper ✓"로 **반대로 표시**되던 버그가 해결됨(이제 has_gripper True일 때만 ✓).
- 검증: `colcon build --packages-select scm_gui`. GUI에서 power/servo/stream/gripper 라벨이 정상 표시되는지 확인.

### teleop start 시 nullspace reference 재전송 — 시작/타이밍 갭 보강

- `core/vive_rby1/src/vive_rby1_node.cpp` `doTeleopStart()`: Step 2(MoveToJointPosition `teleop_pose_`) 성공 직후·Step 3(SetStream enable) 직전에 `/rby1/set_nullspace_joint_ref` 로 `teleop_pose_` 를 재전송하는 블록 추가. 스트림 시작 전에 보내 첫 CartesianImpedance 틱부터 올바른 nullspace 가 적용됨.
  - **배경:** nullspace 전파는 기존에 GUI "Teleop Pose" 드롭다운 `currentTextChanged`(`on_set_teleop_pose`)에서만 발생했고, (1) 시작 시 `setCurrentText` 가 connect 이전이라 저장된 preset 이 자동 푸시되지 않으며 (2) hw-core 미준비 시 `service_is_ready()` false 로 조용히 스킵되어 재시도 없음. 결과적으로 드롭다운을 만지지 않으면 vive 노드의 하드코딩 `teleop_pose_` 기본값과 hw-core config `nullspace_ref_rad` 기본값이 어긋날 수 있었음.
  - teleop start 시점엔 직전 `/rby1/ctrl/mode`·`/rby1/move_to_joint_position` 응답을 받은 직후라 같은 노드의 nullspace 서비스도 사실상 ready. 스킵 시 기존의 조용한 무시 대신 WARN 로그로 가시화.
  - 기존 `on_set_teleop_pose` 전파(세션 중 드롭다운 변경 시 즉시 live 반영)는 그대로 유지(중복이나 무해).
  - include/typedef/client(`cli_nullspace_joint_ref_`)는 이미 존재 → 추가 선언 없음. 범위 외(이번 미적용): hw-core `on_set_nullspace_joint_ref` 의 `std::stoi` 방어, Python 디버그 노드 동등 구현.
- 검증: ROS 미설치 본(WSL) 환경에서 colcon 빌드 미수행 → 사용자 ROS2 머신에서 `cd teleop && colcon build --packages-select vive_rby1` 필요. 런타임: (드롭다운 미조작) teleop start → hw-core 로그 `nullspace pose updated` 확인, CartesianImpedance 중 팔이 선택 pose 쪽으로 약하게 바이어스되는지 관찰.

## 2026-05-21

### 스트리밍 지연 복원 — publish_rate / tracker_smooth_alpha

- `teleop_bringup/launch/teleop.launch.py`: `vive_rby1_node` 파라미터 `publish_rate` `20.0 → 100.0` 복원. hw-core RT 루프(100Hz)와 일치시켜 pose 명령이 매 틱 갱신되게 함. 기존 20Hz는 같은 타깃을 ~5틱 유지 후 점프 → 평균 ~25ms 지연 + 계단형 모션, per-frame clamp(`sdk_max_delta_pos=0.03`)와 곱해져 최대 EE 속도 ~0.6 m/s로 제한되던 문제 해소(100Hz면 ~3 m/s). 예전 버전(`teleop-main`)도 100Hz였음.
- `core/vive_rby1/src/vive_rby1_node.cpp`: `declare_parameter("tracker_smooth_alpha", ...)` 기본값 `0.5 → 0.9`. `q_prev.slerp(alpha, q_new)`이므로 alpha가 클수록 입력을 빨리 추종 → orientation 회전 지연 ~15-30ms 감소. (멤버 초기화는 이미 0.9였음. 예전 버전 하드코딩값도 0.9.)
- 검증: ROS 미설치 본 환경에서 colcon 빌드 미수행 → 사용자 ROS2 머신에서 `colcon build` 필요. 런타임 확인: `ros2 topic hz /rby1/cmd/pose` ~100Hz.

## 2026-05-20

### teleop GUI 레이아웃 개선 및 joint limit 표시

- `ConnectRobot.srv` 동기화: `joint_names`, `q_lower`, `q_upper` 응답 필드 추가 (hw-core 동일)
- `teleop_gui_node.py`:
  - Power On/Off 등 init 버튼을 Connect 버튼 오른쪽으로 이동 (같은 행)
  - Recording 패널: task_id와 episode를 한 행으로 합침 (20px 간격)
  - Tracker 인디케이터: L→B→R 순서, 등간격 배치 (B 중앙)
  - Joint position: 각 spinbox 오른쪽에 deg 단위 레이블 실시간 표시 (+45.3°)
  - connect 성공 시 로봇에서 수신한 joint limit을 spinbox tooltip으로 표시
  - spinbox 값이 limit 초과 시 배경 빨간색 경고

## 2026-05-19

### hw-core rby1_rt → rby1_core 이름 변경에 따른 teleop 업데이트

- `teleop_gui_node.py` NODES_TO_WATCH: `('rby1_rt_node', 'rby1_rt')` → `('rby1_core_node', 'rby1_core')`
- `teleop_gui_node.py` set_parameters 서비스 경로: `/rby1_rt_node/set_parameters` → `/rby1_core_node/set_parameters`
