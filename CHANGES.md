# CHANGES

## 2026-05-22

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
