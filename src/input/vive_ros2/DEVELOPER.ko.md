# `vive_ros2` 개발자 가이드

> HTC Vive Tracker 3.0 → ROS2 `geometry_msgs/PoseStamped` 변환 노드. OpenVR SDK 폴링, 좌표계 변환, base station 정렬.

---

## 1. 패키지 개요

- SteamVR이 실행되어 OpenVR runtime이 활성화된 상태에서 동작
- 최대 5개 device 추적: 2개 base station (정렬 기준) + 2개 hand tracker (left/right) + 1개 optional body tracker
- **OpenVR → ROS 좌표계 변환** (Y-up → Z-up)
- **station 정렬 회전 1회 계산** — 두 base station을 잇는 벡터를 Y축에 정렬해 SteamVR 트래킹 universe의 yaw 오프셋 제거

---

## 2. 디렉토리 구조

```
src/input/vive_ros2/
├── package.xml                                ament_python
├── setup.py
├── config/
│   └── trackers.yaml                          시리얼 → 이름 매핑 (사용자 편집)
└── vive_ros2/
    ├── __init__.py
    └── vive_tracker_node.py                    246줄
```

---

## 3. 빌드 / 실행

### 3.1 의존성

- SteamVR 설치 + 실행 + Vive 디바이스 paired 상태 (녹색)
- Python: `openvr`, `scipy`, `numpy`

```bash
pip3 install openvr scipy numpy
```

### 3.2 빌드

```bash
cd teleop
colcon build --packages-select vive_ros2
source install/setup.bash
```

### 3.3 실행

```bash
# 기본
ros2 run vive_ros2 vive_tracker_node

# 또는 config 파일 명시
ros2 run vive_ros2 vive_tracker_node --ros-args \
  --params-file $(ros2 pkg prefix vive_ros2)/share/vive_ros2/config/trackers.yaml
```

default launch는 이 경로에서 자동 로드. SteamVR이 먼저 실행되고 모든 디바이스가 paired/active 상태여야 함.

---

## 4. 시리얼 번호 매핑

`config/trackers.yaml`에서 다섯 device의 SteamVR 시리얼을 본 노드의 논리 이름에 매핑합니다:

```yaml
vive_tracker_node:
  ros__parameters:
    serial_station_left:  "LHB-XXXXXXXX"   # base station (left)
    serial_station_right: "LHB-XXXXXXXX"   # base station (right)
    serial_tracker_left:  "LHR-XXXXXXXX"   # hand tracker (left)
    serial_tracker_right: "LHR-XXXXXXXX"   # hand tracker (right)
    serial_tracker_body:  "LHR-XXXXXXXX"   # body tracker (optional; ""이면 비활성)
    publish_rate: 100.0
    topic_tracker_left:   "/teleop/tracker/left"
    topic_tracker_right:  "/teleop/tracker/right"
    topic_tracker_body:   "/teleop/tracker/body"
```

### 4.1 시리얼 확인 방법

```bash
# 노드 실행 시 시리얼이 로그에 찍힘 (paired된 모든 device)
ros2 run vive_ros2 vive_tracker_node
# 또는 SteamVR > Devices 메뉴
# 또는:
python3 -c "import openvr; vr=openvr.init(openvr.VRApplication_Other); \
  [print(i, vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)) \
   for i in range(openvr.k_unMaxTrackedDeviceCount)]; openvr.shutdown()"
```

---

## 5. 좌표계 변환

### 5.1 OpenVR → ROS

| 축 | OpenVR (Y-up) | ROS (Z-up) |
|---|---|---|
| x | right | forward |
| y | up | left |
| z | back | up |

매핑:
```python
ros.x = -vr.z
ros.y = -vr.x
ros.z =  vr.y
```

회전 행렬에도 동일하게 적용 (각 column 변환).

### 5.2 Station 정렬

SteamVR의 tracking universe yaw는 임의로 정해집니다 (room setup 시점에 의존). 본 노드는:
1. 첫 tick에 두 base station의 ROS 좌표 `p1`, `p2` 획득
2. `vec = p2 - p1` → `angle = atan2(vec.y, vec.x)`
3. `_station_rot = R_z(-angle)` → 이후 모든 tracker pose에 곱
4. 결과: 양 base station이 Y축을 따라 놓이도록 정렬 → 로봇 좌표계와 일관된 매핑

`_initialized = False` 상태에선 publish 안 함. 두 base station + 두 hand tracker가 모두 alive (`bPoseIsValid`) 한 첫 tick에 정렬 + 발행 시작.

---

## 6. 함수별 요약

| 함수 | 역할 |
|---|---|
| `openvr_to_ros_pos(p)` | 3-vector OpenVR → ROS 좌표 변환 |
| `openvr_col_to_ros(col)` | 회전 행렬 column 변환 |
| `openvr_mat34_to_ros(mat34)` | OpenVR 3×4 행렬 → (pos_ros, rot_ros_3×3) |
| `rot_matrix_to_quat_xyzw(R)` | scipy `Rotation` → 쿼터니언 [x,y,z,w] |
| `get_serial(vr, idx)` | device 시리얼 문자열 |
| `get_rotation_to_align_stations(p1, p2)` | 두 station을 Y축에 정렬하는 Z-회전 행렬 |
| `_poll_devices` | 매 timer tick. 5개 device 폴링 + connect/disconnect 로그 |
| `_make_pose_stamped(idx)` | `_T[idx]` → 좌표 변환 + station_rot 적용 → `PoseStamped` |
| `_timer_cb` | poll → (첫 tick) 정렬 → tracker publish |

---

## 7. ROS 인터페이스

### 7.1 발행 토픽

| 토픽 | 타입 | 발행 조건 |
|---|---|---|
| `/teleop/tracker/left` | `geometry_msgs/PoseStamped` | tracker_left alive |
| `/teleop/tracker/right` | 동일 | tracker_right alive |
| `/teleop/tracker/body` | 동일 | tracker_body alive (없으면 publish 안 함) |

QoS: `KeepLast(10)` 기본. `frame_id="world"`.

### 7.2 파라미터

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `serial_station_left` | (config) | 왼쪽 base station 시리얼 |
| `serial_station_right` | (config) | 오른쪽 base station 시리얼 |
| `serial_tracker_left` | (config) | 왼쪽 hand tracker 시리얼 |
| `serial_tracker_right` | (config) | 오른쪽 hand tracker 시리얼 |
| `serial_tracker_body` | `""` | body tracker 시리얼 (비어있으면 비활성) |
| `publish_rate` | `100.0` Hz | timer 주기 |
| `topic_tracker_left/right/body` | `/teleop/tracker/...` | 발행 토픽 이름 |

---

## 8. 흔한 함정

- **`Unknown device serial: ...`** WARN: paired 시리얼이 `trackers.yaml`에 없음. 시리얼 갱신.
- **두 base station이 안 보임**: SteamVR 실행 확인. station LED 녹색? room setup 완료?
- **첫 tick 후에도 publish 없음**: `bPoseIsValid` false 상태 = SteamVR이 추적 못함. 카메라 시야 차폐 확인.
- **로봇 동작 방향이 90° 회전**: station 정렬 오작동. 두 station의 ROS 좌표(`p1`, `p2`)를 디버그 로그로 확인. base station을 둘로 두고 사이 거리 ≥1m 권장.
- **`openvr.init` 실패**: SteamVR이 OpenVR 호환 모드 아님. SteamVR 재시작.
- **`destroy_node`에서 `openvr.shutdown()` 누락**: SteamVR이 다음 실행 시 자원 해제 못함. 본 노드는 처리하지만 시그널로 강제 종료 시 누락될 수 있음.

---

## 9. 확장 / 수정 가이드

### 9.1 새 device 추가 (예: 두 번째 body tracker)
1. `DEVICE_NAMES`에 `'tracker_body_2'` 추가
2. 파라미터 `serial_tracker_body_2` + 토픽 파라미터 추가
3. `_T`/`_alive`/`_alive_prev` 크기 갱신
4. `_timer_cb`에서 publish 분기 추가

### 9.2 좌표계 매핑 변경
- OpenVR 좌표계가 다른 환경(예: 차량 천장에 station)이면 `openvr_to_ros_pos`/`openvr_col_to_ros` 수정. 위·앞 정의를 명확히 한 뒤 변환식 재도출.

### 9.3 정렬 방식 변경
- yaw 정렬 외에 roll/pitch 정렬이 필요하면 `get_rotation_to_align_stations`를 3D 정렬로 확장 (예: SVD로 두 station의 normal을 -Z에 정렬).

### 9.4 QoS 변경
- 100Hz 실시간이라 `BestEffort` + `KeepLast(1)`이 더 적절할 수 있음. vive_rby1 측 QoS와 매치.

---

## 10. 연관 패키지

- `vive_rby1` (core) — 본 노드의 출력 토픽 주요 구독자
- `scm_gui` — `/teleop/tracker_status` 시각화 (직접 구독 아님, vive_rby1이 계산해서 발행)
