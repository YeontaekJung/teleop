# update.md — 날짜별 변경 이력

실제 적용된 변경사항만 기록. 시도 후 롤백/미적용된 내용은 제외.

---

## 2026-04-21

### 2026 repo

- **vive_rby1 Python → C++ 전환** (`src/core/vive_rby1/src/vive_rby1_node.cpp`)
  - pinocchio 기반 differential IK, 50Hz 타이머
  - sdk_position / sdk_impedance 모드: PoseArray 발행
  - clutch state 토픽 (`/teleop/clutch_state`) 추가
  - `clutch_toggle` 커맨드 GUI에서 직접 제어 가능

- **SDK EE frame 참조 수정** (`vive_rby1_node.cpp`)
  - engage 시 SDK 모드 초기 EE 참조를 `tracker_right/left` → `ee_right/ee_left` 프레임으로 교체
  - tracker_right는 URDF상 ee_right에서 [+5cm, 0, -10cm] offset → 매 engage마다 팔이 앞으로 나오던 문제 수정
  - mirror 모드도 동일하게 처리 (delta anchor만 교체, delta 계산 방식 유지)

- **pos_scale 기본값 2.0 → 1.0** (`vive_rby1_node.cpp`, `teleop.launch.py`)

- **GUI IK 라디오 버튼 rename + 기본값 변경** (`teleop_gui_node.py`)
  - SDK → Cartesian, Pink → Joint
  - 기본값: Cartesian → Joint (안전한 기본 모드)

- **End Episode 버튼 활성화 조건 수정** (`teleop_gui_node.py`)
  - `stream_on AND state in (READY, PAUSED)` → `state in (READY, RECORDING, PAUSED)`
  - stream 상태와 무관하게 에피소드 중 언제든 종료 가능

### hw-core

- **CartesianPosition 개선** (`rby1_rt_node.cpp`)
  - `SetStopJointPositionTrackingError(0)` 추가 — nullspace joint tracking error로 인한 stream 종료 방지

- **CartesianImpedance 개선** (`rby1_rt_node.cpp`)
  - `AddJointLimit("arm_3", -2.6, -0.5)` / `AddJointLimit("arm_5", 0.2, 1.9)` 추가 — workspace 초과 시 MinorFault 방지
  - VR teleop 예제 기준값 적용

- **SDK 파라미터 설정** (`rby1_rt_node.cpp`, connect 시)
  - `cartesian_command.cutoff_frequency = 5` — Cartesian 명령이 firmware에서 필터링되지 않도록 필수 설정
  - `joint_position_command.cutoff_frequency` 설정 제거 (Joint 모드 이상 원인이었음)

- **Cartesian stream 시작 구조 개선** (`rby1_rt_node.cpp`)
  - JointPosition hold phase 제거 — stream 시작부터 Cartesian 명령 사용
  - 첫 tick에 SDK FK로 초기 EE pose 계산 후 즉시 CartesianCommand 전송 (within-stream 타입 전환 제거)
  - CartesianImpedance `reset_ref` 버그 수정 — `traj_dt_cnt` 증가 전 평가하도록 수정

---

## 2026-04-08

- **Docker 구성 추가** (`docker/`)
  - `docker/Dockerfile` — osrf/ros:humble-desktop-full 기반, 이 repo에 필요한 패키지만 포함 (MoveIt/Nav2/RealSense 등 제거)
  - `docker/docker-compose.yaml` — image: scm-teleoperation:humble, 레포 루트를 `/home/ros2_ws`로 마운트
  - `docker/pip.conf` — 사내 프록시 설정
  - `docker/McAfee_Certificate.crt` — gitignore (로컬 제공 필요)
- **ManusSDK Git LFS 전환**
  - `ManusSDK/lib/*.so` (128MB, 103MB) → Git LFS로 추적
  - `.gitattributes` 추가
  - `.gitignore`에서 `/ManusSDK/` 제거, `*.7z` 추가
- **`.dockerignore` 추가** — 빌드 컨텍스트 최적화 (src/, .git/ 등 제외)
- **README 업데이트** — ManusSDK 설치 방법 → `git lfs pull`로 변경

---

## 2026-04-06

- **tracker 입력 SLERP+EMA 스무딩 추가** (`vive_rby1_node.py`)
  - position EMA + quaternion SLERP LPF (alpha=0.5) 적용
  - smoothed SE3를 IK에 사용, raw msg는 JITTER/LOST 감지용으로 유지
- **tracker JITTER 감지 로직 수정** (`vive_rby1_node.py`)
  - position std → velocity(프레임 간 차이) std로 변경
  - 이동 중 false JITTER 방지
- **pos_scale 1.0 → 1.5** (`teleop.launch.py`)
- **new_core_main.py realtime 수정** (수동 적용, 이 repo 아님)
  - `gc.disable()` 파일 상단 추가
  - `perf_counter` 기반 절대 시간 sleep으로 교체 (누적 오차 없음)
  - stream 비활성 시 `next_time` 리셋 (teleop_start 후 spurious not-realtime 경고 제거)

---

## 2026-04-03

- **pedal engage를 teleop_start 이후에만 허용** (`vive_rby1_node.py`)
  - `_teleop_active` 플래그 추가
  - `teleop_start`/`impedance_teleop_start` 전송 시 True, `teleop_stop` 시 False
  - recording init 구간(warmup + vla_pose2 + teleop_start 총 ~3s) 중 실수로 engage되는 버그 수정
- **velocity EMA 스무딩 추가** (`rby1_ik.py`)
  - IK velocity 출력에 alpha=0.6 EMA 적용
  - 빠른 회전 시 joint trembling 감소 목적

---

## 2026-04-02

- **페달 라벨 변경** (`teleop_gui_node.py`)
  - A: `Toggle/Pause` → `Resume/Pause`
  - C: `Start/Stop` → `● Rec`
- **Manus 엄지 매핑 수정** (`manus_inspire.py`)
  - `weighted_flex(MCP+PIP+DIP)` → `ThumbMCPStretch` 단독 사용 (finger 교차 오염 제거)
  - `invert=False` 양손 통일
- **Manus 4단계 캘리브레이션** (`manus_inspire.py`, `teleop_gui_node.py`)
  - Phase 1: 손 쫙 펴기 → finger min + spread min
  - Phase 2: 엄지 치켜세우기(주먹) → finger max + thumb MCPStretch max
  - Phase 3: 엄지를 검지 옆에 누르기 → spread max
  - Phase 4: 손가락 펴고 엄지만 굽히기 → thumb MCPStretch min
  - GUI 진행바 4단계로 업데이트 (총 16초)
- **orientation_cost 10.0 → 0.5** (`rby1_ik.py`)
  - 팔이 앞으로 나오는 문제 완화
- **max_teleop_dq 3.0 → 1.5 rad/s** (`rby1_ik.py`)
  - joint trembling 감소

---

## 2026-04-01

- **impedance 모드 intercept** (`vive_rby1_node.py`)
  - GUI에서 impedance 선택 시 `teleop_start` → `impedance_teleop_start`로 자동 변환
- **pos_scale 파라미터 launch 파일에 노출** (`teleop.launch.py`)

---

## 2026-03-31

- **publish_rate 20Hz → 50Hz** (`teleop.launch.py`)
- **ik_dt 0.05 → 0.1s** (`teleop.launch.py`)
- **tracker status UI 수정** (`teleop_gui_node.py`) — label/dots overlap 수정
- **impedance_teleop_start 명령 추가** (`vive_rby1_node.py`)
