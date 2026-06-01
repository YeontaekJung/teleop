# `pedal_ros2` 개발자 가이드

> PCsensor FootSwitch (3-pedal USB keyboard) → `sensor_msgs/Joy` 변환 ROS2 노드. evdev로 raw key 이벤트를 읽어 OS에 키 이벤트 누설을 막고 ROS 토픽으로 변환.

---

## 1. 패키지 개요

PCsensor 3-pedal 발판은 USB HID 키보드처럼 동작합니다 (Left=A, Middle=B, Right=C 키 송신). 본 노드는:
1. evdev로 디바이스 발견 + `grab()`으로 단독 점유 (OS/터미널에 a/b/c 키스트로크 누설 방지)
2. 백그라운드 thread에서 `read_loop()` 블로킹 read
3. press/release 시 `_state[0..2]` 갱신 + `/teleop/pedal` (`sensor_msgs/Joy`) publish

`vive_rby1_node`가 이 토픽을 구독하여 페달 A/B/C로 클러치/discard/episode 토글을 수행합니다.

---

## 2. 디렉토리 구조

```
src/input/pedal_ros2/
├── package.xml                                ament_python
├── setup.py
└── pedal_ros2/
    ├── __init__.py
    └── pedal_node.py                           105줄
```

---

## 3. 빌드 / 실행

### 3.1 시스템 권한 설정 (1회, 재로그인 필요)

```bash
sudo usermod -aG input $USER
# 완전히 로그아웃 → 재로그인 (터미널 닫기로는 부족)
groups | grep input    # 확인
```

`/dev/input/*` device를 읽으려면 `input` 그룹이 필요합니다. 미설정 시 `Permission denied`. 임시로 `sudo ros2 run ...`도 가능하지만 권장 안 함 (ROS env vars가 깨질 수 있음).

### 3.2 의존성

```bash
pip3 install evdev
```

### 3.3 빌드

```bash
cd teleop
colcon build --packages-select pedal_ros2
source install/setup.bash
```

### 3.4 실행

```bash
ros2 run pedal_ros2 pedal_node

# 토픽 또는 device 이름 오버라이드
ros2 run pedal_ros2 pedal_node --ros-args \
  -p device_name:='PCsensor FootSwitch Keyboard' \
  -p topic:=/teleop/pedal
```

성공 시:
```
[INFO] Found device: /dev/input/eventXX — PCsensor FootSwitch Keyboard
[INFO] Publishing to /teleop/pedal
```

---

## 4. 동작 흐름

```
[Hardware] PCsensor 3-pedal USB
              │
              ▼
       /dev/input/eventXX  (input 그룹 권한 필요)
              │
              ▼
    evdev.InputDevice.grab() → 단독 점유, OS에 키 누설 차단
              │
              ▼
    threading.Thread → read_loop() 블로킹 read
              │
              ▼ (EV_KEY + KEY_A/B/C)
    _state[KEY_MAP[code]] = 1 (press) | 0 (release) | 1 (repeat→press 취급)
              │
              ▼
    /teleop/pedal (sensor_msgs/Joy)
       buttons=[a,b,c], axes=[]
```

이벤트 타입:
- `EV_KEY` only 처리 (다른 타입 무시)
- KEY_A/B/C만 처리 (KEY_MAP에 있는 코드)
- value: `1`=press, `0`=release, `2`=repeat(hold) → repeat은 press 취급 (계속 누름)

---

## 5. 페달 매핑 (KEY_MAP)

| 페달 위치 | evdev 코드 | buttons[idx] | vive_rby1 측 의미 |
|---|---|---|---|
| Left  | `KEY_A` (30) | `buttons[0]` | 클러치 engage/disengage 토글 (페달 A) |
| Middle | `KEY_B` (48) | `buttons[1]` | 에피소드 폐기 (`discard=true`, 페달 B) |
| Right | `KEY_C` (46) | `buttons[2]` | 에피소드 시작/종료 (페달 C) |

`/teleop/pedal` 구독자는 `vive_rby1_node`. 자세한 행동은 [`../../core/vive_rby1/DEVELOPER.ko.md`](../../core/vive_rby1/DEVELOPER.ko.md) §4.4 참조.

---

## 6. ROS 인터페이스

### 6.1 발행 토픽

| 토픽 | 타입 | depth | 의미 |
|---|---|---|---|
| `/teleop/pedal` | `sensor_msgs/Joy` | 10 | `buttons=[a,b,c]` (0 또는 1), `axes=[]`. 매 이벤트마다 발행. |

### 6.2 ROS 파라미터

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `device_name` | `"PCsensor FootSwitch Keyboard"` | 매칭할 evdev device name (`evdev.InputDevice.name`) |
| `topic` | `/teleop/pedal` | 발행 토픽 이름 |

---

## 7. 함수별 요약

| 함수 | 역할 |
|---|---|
| `find_device(name)` | `evdev.list_devices()` 순회하며 `dev.name == name`인 첫 디바이스 반환 |
| `PedalNode.__init__` | 파라미터 선언 + device 찾기 + grab + thread 시작 |
| `_read_loop` | 백그라운드 thread. `read_loop()` 블로킹. EV_KEY 이벤트 → `_state` 갱신 → publish |
| `_publish` | `Joy` 메시지 빌드 + publish |

---

## 8. 흔한 함정

- **`Device '...' not found`**: USB 연결 확인, `lsusb`로 PCsensor 보임 확인. evdev device 이름이 정확히 일치해야 함:
  ```bash
  python3 -c "import evdev; [print(d.name) for d in [evdev.InputDevice(p) for p in evdev.list_devices()]]"
  ```
- **`Permission denied` (`/dev/input/eventXX`)**: `input` 그룹 미설정. §3.1 참조. **재로그인 필수.**
- **터미널에 a/b/c가 입력됨**: `grab()` 실패 또는 미수행. evdev 0.7.0+ 필요.
- **release 이벤트 누락**: `grab()` 직후 디바이스가 unplug되면 thread가 멈춤. 노드 재시작.
- **다른 PCsensor 모델**: 일부 PCsensor 펌웨어는 다른 키 코드 송신. `evtest /dev/input/eventXX`로 실제 코드 확인 후 `KEY_MAP` 갱신.

---

## 9. 확장 / 수정 가이드

### 9.1 새 페달 모델 지원
1. `evtest`로 키 코드 확인
2. `KEY_MAP`에 새 매핑 추가 (예: `ecodes.KEY_D: 3`)
3. `_state` 크기 늘림 (예: `[0]*4`)
4. `topic` 파라미터 변경 또는 그대로

### 9.2 다른 입력 디바이스 (조이스틱 등)
- `evdev.list_devices()` + `dev.capabilities()`로 EV_ABS(축) 지원 확인
- `EV_ABS` 이벤트 처리 추가
- `msg.axes` 채우기

### 9.3 멀티 페달 (둘 이상)
- `find_device`를 모든 매치 반환으로 변경
- 디바이스별 별도 thread + 별도 토픽

---

## 10. 연관 패키지

- `vive_rby1` (core) — `/teleop/pedal` 유일한 구독자
- `scm_gui` (gui) — `/teleop/pedal` 시각화
