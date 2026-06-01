# `rby1_ik` 개발자 가이드

> **Legacy Python IK 헬퍼.** `vive_rby1_debug_node`(Python)만 사용. 프로덕션 C++ 노드는 pinocchio DifferentialIkSolver를 자체 보유 → 본 패키지 미사용.

---

## 1. 패키지 개요

- pinocchio + pink 기반 Differential IK 솔버
- RB-Y1 24 DOF 모델 (`right_wheel`, `left_wheel`, `torso_0..5`, `right_arm_0..6`, `left_arm_0..6`, `head_0`, `head_1`)
- 20 DOF body joints (torso + 양팔)만 풀어서 반환
- One Euro Filter 헬퍼 클래스 포함 (트래커 입력 평활용)
- Self-collision barrier (`pink.barriers.SelfCollisionBarrier`)

**왜 legacy?**: 2026-05 이전엔 본 패키지가 메인 IK 경로였으나, 현재는 SDK가 펌웨어에서 IK를 풀고 hw-core가 `tf2_msgs/TFMessage`로 Cartesian target을 받는 구조로 이전됨. Python IK는 디버그/실험용으로만 유지.

---

## 2. 디렉토리 구조

```
src/core/rby1_ik/
├── package.xml                                ament_python
├── setup.py
└── rby1_ik/
    ├── __init__.py
    └── rby1_ik.py                              메인 — IK 클래스 + 헬퍼
```

---

## 3. 빌드 / 실행

이 패키지는 **실행 노드가 없습니다** — Python 라이브러리로만 사용. `vive_rby1_debug_node`가 `from rby1_ik.rby1_ik import ...` 로 import.

### 3.1 의존성

```bash
pip3 install pin pink scipy
```

### 3.2 빌드

```bash
cd teleop
colcon build --packages-select rby1_ik
source install/setup.bash
```

---

## 4. 주요 식별자

### 4.1 헬퍼 함수

| 함수 | 반환 | 용도 |
|---|---|---|
| `get_rby1_joint_name_list()` | 24개 풀네임 (wheels 2 + body 20 + head 2) | 전체 joint 순서 |
| `get_rby1_body_joint_name_list()` | 20개 (torso + 양팔) | hw-core가 사용하는 body joints |
| `get_rby1_torso_joint_name_list()` | 6개 (torso_0..5) | torso만 |

### 4.2 `OneEuroFilterVec` 클래스

n-dim 벡터에 element-wise One Euro 필터 적용. 적응형 cutoff:
- 느린/정지 신호 → 낮은 cutoff (떨림 제거)
- 빠른/의도적 움직임 → 높은 cutoff (지연 없음)

파라미터:
- `freq` — 샘플링 주파수 (Hz)
- `min_cutoff` — 최저 cutoff (Hz), 기본 1.0
- `beta` — 속도에 따른 cutoff 가산, 기본 0.1
- `d_cutoff` — 속도 추정용 cutoff, 기본 1.0

용도: 트래커 raw position을 IK solver에 넣기 전 평활화. C++ 프로덕션 노드는 자체 SLERP-기반 smoother(`smoothTracker`) 사용 — 본 필터 사용 안 함.

### 4.3 메인 IK 클래스 (`rby1_ik.py`의 후반부)

`solve_ik_to_q20(...)` 시그니처는 reference 버전과 호환 유지. 내부에서:
- pink `solve_ik(config, tasks, dt, solver='proxqp')` 호출 — solver는 2026-05 이전 `scs` → `proxqp`로 빠르게 변경
- torso joints는 zero mask 대신 velocity mask로 제외 (mid-solve drift 방지)
- `FrameTask` 양팔(`tracker_left`, `tracker_right`) + `PostureTask` body 평활 + `SelfCollisionBarrier` 안전
- 반환: 20 DOF q (body 순서)

---

## 5. ROS 인터페이스

**없음.** 라이브러리 전용 패키지. 실행 노드 없음.

---

## 6. 사용 시점

`vive_rby1_debug_node.py` 내부에서만 import:

```python
from rby1_ik.rby1_ik import (
    get_rby1_body_joint_name_list,
    Rby1IK,                                    # IK 클래스 (실제 이름은 코드 후반부 참조)
    OneEuroFilterVec,
)

ik = Rby1IK(urdf_path, srdf_path, max_teleop_dq=1.5, ik_dt=0.05)
q20 = ik.solve_ik_to_q20(left_target, right_target)
```

`max_teleop_dq = 1.5 rad/s`, `ik_dt = 0.05s` → max Δq per step = 0.075 rad. 떨림이 있으면 둘 다 낮춰서 max Δq를 줄이세요.

---

## 7. 흔한 함정

- **C++ 프로덕션 노드는 본 패키지 사용 안 함**. C++ 측은 pinocchio만 직접 사용 (pink 없음, ProxQP/SCS 없음).
- **`pink` 미설치** → 빌드는 통과하나 import 시점에 ImportError. `pip3 install pink`.
- **proxqp vs scs**: proxqp가 훨씬 빠르지만 라이선스/플랫폼 이슈가 있을 수 있음. fallback 필요 시 `solver='scs'`로.
- **head joints 포함된 24 DOF 모델**: 실제 RB-Y1엔 head가 없으므로 IK 결과의 head 값은 무시.

---

## 8. 확장 / 수정 가이드

### 8.1 새 IK task 추가
- pink `tasks.FrameTask`, `tasks.PostureTask`, `tasks.JointVelocityLimit` 등 추가
- `solve_ik(config, tasks=[task1, task2, ...], ...)` 리스트에 추가

### 8.2 다른 solver 시도
- `pink.solve_ik(solver='scs'|'proxqp'|'osqp', ...)` — pip 설치한 backend에 따라 선택

### 8.3 self-collision pair 갱신
- `pink.utils.process_collision_pairs(robot, srdf_path)` — URDF + SRDF 기반. SRDF의 `disable_collisions` 태그로 무시 페어 정의.

---

## 9. 연관 패키지

- `vive_rby1` (sibling) — Python debug 노드가 본 패키지의 유일한 클라이언트
- 외부: `pinocchio`, `pink`, `scipy`
