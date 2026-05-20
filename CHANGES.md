# CHANGES

## 2026-05-19

### hw-core rby1_rt → rby1_core 이름 변경에 따른 teleop 업데이트

- `teleop_gui_node.py` NODES_TO_WATCH: `('rby1_rt_node', 'rby1_rt')` → `('rby1_core_node', 'rby1_core')`
- `teleop_gui_node.py` set_parameters 서비스 경로: `/rby1_rt_node/set_parameters` → `/rby1_core_node/set_parameters`
