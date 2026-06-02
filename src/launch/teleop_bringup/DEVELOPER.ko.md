# `teleop_bringup` 개발자 가이드

> 전체 teleop 시스템(input 3 + core 2 + GUI)을 한 번에 띄우는 launch 패키지.

---

## 1. 패키지 개요

`ros2 launch teleop_bringup teleop.launch.py` 한 번으로:
- pedal_ros2 / vive_ros2 / manus_ros2 (input) — 옵션 토글로 sim 모드 시 제외
- manus_inspire (core, Manus glove → Inspire Hand)
- vive_rby1_node (core, 프로덕션 C++)
- scm_gui (GUI)

총 6개 노드를 일관된 파라미터로 기동. **debug Python 노드 `vive_rby1_debug_node`는 포함하지 않음**.

---

## 2. 디렉토리 구조

```
src/launch/teleop_bringup/
├── package.xml                                ament_python
├── setup.py
├── setup.cfg
├── resource/teleop_bringup
├── launch/
│   └── teleop.launch.py                        137줄
└── teleop_bringup/
    └── __init__.py                              (빈 패키지)
```

---

## 3. 빌드 / 실행

### 3.1 빌드

```bash
cd teleop
colcon build --packages-select teleop_bringup
source install/setup.bash
```

### 3.2 실행

```bash
# 기본 (모든 하드웨어 활성)
ros2 launch teleop_bringup teleop.launch.py

# 시뮬레이터 (하드웨어 노드 모두 비활성, core + GUI만)
ros2 launch teleop_bringup teleop.launch.py sim:=true

# 페달 없이
ros2 launch teleop_bringup teleop.launch.py use_pedal:=false

# Vive trackers 없이 (Manus는 사용)
ros2 launch teleop_bringup teleop.launch.py use_vive:=false

# 로봇 주소/모델 명시 (FK용 SDK 모델 소스)
ros2 launch teleop_bringup teleop.launch.py \
  robot_address:=192.168.30.1:50051 \
  robot_model:=a
```

---

## 4. Launch 인자 (DeclareLaunchArgument)

| 인자 | 기본 | 의미 |
|---|---|---|
| `robot_address` | `localhost:50051` | vive_rby1_node가 FK 모델을 받을 rby1-sdk gRPC 주소 (`GetDynamics()`). hw-core가 연결하는 로봇과 동일해야 FK 일치 |
| `robot_model` | `a` | FK 모델 선택: `a`(2륜) \| `m`(메카넘) |
| `sim` | `false` | true면 모든 하드웨어 입력 노드 비활성 |
| `use_manus` | `true` | Manus glove + manus_inspire 활성 |
| `use_pedal` | `true` | 페달 활성 (sim=true시 무시) |
| `use_vive` | `true` | Vive tracker 활성 (sim=true시 무시) |

조합 의미:
- `pedal_on = use_pedal AND NOT sim`
- `vive_on = use_vive AND NOT sim`
- `manus_on = use_manus AND NOT sim`

> Note: `sim:=true`는 단순히 입력 하드웨어 노드를 끌 뿐, 시뮬레이션 환경을 별도로 띄우진 않습니다. RB-Y1 시뮬레이터는 별도 컨테이너/프로세스.

---

## 5. 노드별 파라미터 (launch dict)

### 5.1 `pedal_ros2/pedal_node`
파라미터 없음 (기본값 사용). [`../../input/pedal_ros2/DEVELOPER.ko.md`](../../input/pedal_ros2/DEVELOPER.ko.md) 참조.

### 5.2 `vive_ros2/vive_tracker_node`
`trackers.yaml` 파일을 `parameters=[_trackers_yaml]`로 전달.

```python
_trackers_yaml = os.path.join(get_package_share_directory('vive_ros2'), 'config', 'trackers.yaml')
```

시리얼 변경 시: source 디렉토리 또는 install/share에서 yaml 편집 후 rebuild (`colcon build --packages-select vive_ros2`).

### 5.3 `manus_ros2/manus_data_publisher` + `manus_inspire/manus_inspire_node`
파라미터 없음. ManusSDK 바이너리가 필요.

### 5.4 `vive_rby1/vive_rby1_node`

| 파라미터 | 값 | 의미 |
|---|---|---|
| `robot_address` | launch 인자 | rby1-sdk gRPC 주소 (FK 모델 `GetDynamics()`) |
| `robot_model` | launch 인자 | FK 모델 `a`\|`m` |
| `publish_rate` | `100.0` Hz | EE 명령 publish 주기 (hw-core RT 100Hz와 매치) |
| `pos_scale` | `0.5` | tracker→robot 위치 스케일 (hand) |
| `torso_pos_scale` | `1.0` | body tracker 위치 스케일 (torso) |
| `use_torso` | `False` | body tracker → link_torso_5 비활성 (런타임 토글 `/vive_rby1/set_use_torso`) |
| `sdk_max_delta_pos` | `0.03` m | per-frame Cartesian step clamp |

> `sdk_max_delta_rot_deg`는 launch에서 미지정 → 노드 기본 20°.
> `tracker_smooth_alpha`도 미지정 → 노드 기본 0.9.
> `max_teleop_dq = 1.5 rad/s`는 vive_rby1_node.cpp에 하드코딩.

### 5.5 `scm_gui/scm_gui_node`

| 파라미터 | 값 | 의미 |
|---|---|---|
| `teleop_panel_expanded` | `True` | 시작 시 Teleop 패널 펼침 |

---

## 6. 기동 순서

ROS2 launch는 모든 `Node`를 비동기 병렬 시작. 명시적 순서가 없음. 다만:
- 모든 publisher 노드(pedal, vive, manus)는 vive_rby1보다 늦게 시작해도 무관 (vive_rby1이 구독 측이므로)
- GUI(scm_gui)는 service client. 백엔드 노드 준비 안 됐어도 GUI는 뜨고 버튼 누르면 timeout 또는 service 미가용 메시지.
- `vive_rby1_node`가 가장 빨리 준비되면 좋음 (UI에서 teleop 버튼 활성화 등)

병렬 기동 후 GUI에서 노드 상태 그룹 확인 (Core/Vision/Teleop/Recording).

---

## 7. ROS 인터페이스

이 패키지 자체는 노드를 띄울 뿐 인터페이스를 노출하지 않음. 각 노드의 토픽/서비스는 해당 패키지 가이드 참조.

---

## 8. 흔한 함정

- **`robot_address` 도달 불가**: vive_rby1_node는 FK 모델을 `GetDynamics()`로 받으므로 로봇(또는 rby1_core_node)이 떠 있어야 한다. 연결 전까지는 2초 간격 재시도 로그(`SDK dynamics connect failed ... retrying`)만 찍히고 engage 시 `SDK dynamics model not ready yet` 경고. 주소는 hw-core가 연결하는 로봇과 동일해야 FK가 일치(불일치 시 engage 점프). 로봇 모델이 메카넘이면 `robot_model:=m`.
- **`sim:=true` 의미 오해**: 시뮬레이터 자동 실행이 아니라 입력 노드 끄기. 실제 시뮬레이터는 별도 docker container 또는 host 프로세스 필요.
- **`use_vive:=false`인데 vive_rby1_node 시작**: 의도된 동작. vive_rby1은 트래커 없이도 (engage 불가) 시작. GUI에서 vive 노드 빨강 표시.
- **launch 인자 → 노드 파라미터 전달 누락**: `vive_rby1_node`의 파라미터 dict를 추가/수정한 후 source + relaunch.
- **2026-05-30 keyboard driving 기본값**: launch에는 driving 관련 파라미터 없음. `scm_gui_node` 내부 기본값(linear=0.05, angular=0.10) 사용.

---

## 9. 확장 / 수정 가이드

### 9.1 새 input 노드 추가

```python
DeclareLaunchArgument('use_xxx', default_value='true', description='...')

xxx_on = AndSubstitution(use_xxx, not_sim)

GroupAction(
    condition=IfCondition(xxx_on),
    actions=[Node(package='xxx_pkg', executable='xxx_node', name='xxx_node', output='screen')],
),
```

### 9.2 새 파라미터를 vive_rby1_node에 전달

`vive_rby1` Node의 `parameters=[{...}]` dict에 추가. 노드 측 코드(`vive_rby1_node.cpp`)에서 `declare_parameter` + `get_parameter`도 함께 갱신해야 사용 가능.

### 9.3 sim 모드에서 자동으로 시뮬레이터 띄우기

```python
GroupAction(
    condition=IfCondition(sim),
    actions=[
        ExecuteProcess(
            cmd=['docker', 'run', '--rm', 'rby1-sim:latest'],
            ...
        ),
    ],
),
```

(권장 안 함 — 시뮬레이터 옵션이 다양해 launch에 박는 것보다 별도 스크립트가 유연.)

### 9.4 디버그 Python 노드 활성화

```python
Node(
    package='vive_rby1',
    executable='vive_rby1_debug_node',
    name='vive_rby1_debug_node',
    output='screen',
    parameters=[{'control_mode': 'pink_impedance'}],
),
```

기존 `vive_rby1_node`와 토픽 충돌(`/rby1/cmd/pose`)하므로 둘 중 하나만 활성화. 보통 디버그 시엔 C++ 노드 disable.

---

## 10. 연관 패키지

이 패키지는 단순 launch이므로 모든 teleop 패키지에 의존. 핵심:
- `vive_rby1`, `scm_gui` — 항상 띄움
- `pedal_ros2`, `vive_ros2`, `manus_ros2`, `manus_inspire` — 조건부
