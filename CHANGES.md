# CHANGES

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
