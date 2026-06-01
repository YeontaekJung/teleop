# `manus_ros2` 개발자 가이드

> Manus Prime X Haptic glove SDK (Integrated 모드) → ROS2 변환 노드. C++ 클라이언트 + Manus SDK 콜백 → `ManusGlove` 발행.

---

## 1. 패키지 개요

- C++17 ROS2 노드. Manus SDK의 5개 콜백 (skeleton/raw device data/ergonomics/landscape)을 구독해 메시지화
- **`ConnectionType_Integrated`** 모드 — Manus Core 없이 단독 사용 (gRPC dep 불필요)
- 손에 연결된 모든 glove를 발견 후 각 glove마다 `/manus_glove_<idx>` 토픽으로 발행
- ManusSDK 바이너리(`libManusSDK_Integrated.so`)가 **워크스페이스 루트의 `ManusSDK/`에 수동 복사** 필요

---

## 2. 디렉토리 구조

```
src/input/manus_ros2/
├── CMakeLists.txt                              ManusSDK 경로 + ncurses link
├── package.xml
├── config/
│   └── gloves.yaml                             glove ID → side 매핑
├── client_scripts/
│   └── manus_data_viz.py                       (디버그 시각화, optional)
└── src/
    ├── ClientLogging.hpp                         로깅 매크로
    ├── ClientPlatformSpecific.{hpp,cpp}          OS별 init/shutdown 헬퍼
    ├── ClientPlatformSpecificTypes.hpp           플랫폼별 타입
    ├── ManusDataPublisher.{hpp,cpp}              메인 클래스 (~673줄)
    └── manus_data_publisher.cpp                  엔트리포인트 (13줄)
```

---

## 3. 빌드 / 실행

### 3.1 ManusSDK 바이너리 배치 (필수)

ManusSDK는 git에 없는 바이너리. 워크스페이스 루트에:

```
teleop/
└── ManusSDK/
    ├── include/
    │   ├── ManusSDK.h
    │   ├── ManusSDKTypeInitializers.h
    │   └── ManusSDKTypes.h
    └── lib/
        ├── libManusSDK_Integrated.so       (Integrated 모드용 — 본 패키지 기본)
        └── libManusSDK.so                  (Remote 모드용 — gRPC 필요, 미사용)
```

Manus 개발자 포털에서 다운로드 또는 다른 머신에서 복사.

### 3.2 시스템 의존성

```bash
sudo apt install libncurses-dev    # CMake가 ncurses link
```

### 3.3 빌드

```bash
cd teleop
colcon build --packages-select manus_ros2
source install/setup.bash
```

빌드 후 `libManusSDK_Integrated.so`도 `install/manus_ros2/lib/manus_ros2/`에 복사됨 (RPATH `$ORIGIN`).

### 3.4 실행

```bash
ros2 run manus_ros2 manus_data_publisher

# config 명시
ros2 run manus_ros2 manus_data_publisher --ros-args \
  --params-file $(ros2 pkg prefix manus_ros2)/share/manus_ros2/config/gloves.yaml
```

---

## 4. 동작 흐름

```
[Hardware] Manus Prime X Haptic glove (USB dongle)
              │
              ▼
       ManusSDK_Integrated.so  (C ABI)
              │ (5개 callback)
              ▼
    ManusDataPublisher (rclcpp::Node + SDKClientPlatformSpecific)
       ├─ OnRawSkeletonStreamCallback     (skeleton 노드 데이터)
       ├─ OnRawDeviceDataStreamCallback   (raw 센서, 미사용)
       ├─ OnErgonomicsStreamCallback      (각도 → /manus_glove_<idx>의 ergonomics)
       └─ OnLandscapeCallback             (glove 발견/연결 상태)
              │
              ▼
       m_PublishTimer (rclcpp timer)
              │ tick:
              ▼
     PublishCallback() → ManusGlove 메시지 빌드 + publish
```

---

## 5. 주요 클래스 멤버

### 5.1 `ManusDataPublisher`

생성자에서:
1. ROS 파라미터 선언 (`glove_id_left`, `glove_id_right`)
2. `SDKClientPlatformSpecific::InitializeWindow` (콘솔 ncurses)
3. `InitializeSDK` — Manus SDK 초기화 + Integrated 모드
4. `RegisterAllCallbacks` — 5개 콜백 등록
5. `Connect` — host 발견 + 연결
6. `m_PublishTimer` 생성

### 5.2 5개 콜백 (static)

| 콜백 | 데이터 | 처리 |
|---|---|---|
| `OnRawSkeletonStreamCallback` | skeleton nodes + info | `m_GloveDataMap[id]`에 저장 (mutex) |
| `OnRawDeviceDataStreamCallback` | raw 센서값 (IMU 등) | `m_RawSensorDataMap[id]` (현재 미사용) |
| `OnErgonomicsStreamCallback` | 각 손가락 MCP/PIP/DIP 각도 (string-keyed) | `m_ErgonomicsDataMap[id]` |
| `OnLandscapeCallback` | glove 발견/topology | `m_NewLandscape` 갱신 (glove publisher 동적 추가) |

`s_Instance`(singleton)로 콜백 → instance 멤버 접근.

### 5.3 `PublishCallback`

타이머 콜백. 매 tick:
1. mutex 잡고 `m_GloveDataMap`, `m_ErgonomicsDataMap` 스냅샷
2. glove마다 `manus_ros2_msgs/ManusGlove` 메시지 빌드:
   - `glove_id` (uint32)
   - `side` ("L"/"R") — landscape data 기반
   - `nodes[]` (skeleton 노드 위치/회전)
   - `ergonomics` (key→value 맵 또는 별도 메시지 구조)
3. `m_GlovePublisher[id]->publish(msg)`

`m_PublishCountMap`으로 glove별 발행 카운트 + 10s마다 로그.

### 5.4 좌표계 설정

```cpp
m_WorldSpace = true;
m_CoordinateSystem = {
    AxisView::AxisView_XFromViewer,
    AxisPolarity::AxisPolarity_PositiveZ,
    Side::Side_Right,
    1.0f
};
```

Manus → ROS 좌표 변환은 SDK가 처리. 본 노드는 이 설정만 전달.

---

## 6. ROS 인터페이스

### 6.1 발행 토픽 (glove마다 동적)

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/manus_glove_0` | `manus_ros2_msgs/ManusGlove` | 첫 번째 발견된 glove |
| `/manus_glove_1` | 동일 | 두 번째 |
| ... | | 더 많은 glove 연결 시 |

`manus_inspire` 노드는 `0`과 `1` 둘 다 구독. `msg.side`로 좌/우 식별.

### 6.2 파라미터

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `glove_id_left` | `"101FD009"` | 왼손 glove 하드웨어 ID (Manus dongle에 인쇄) |
| `glove_id_right` | `"8C1794FD"` | 오른손 ID |

ID는 단순 매핑용 (`msg.side` 결정). Manus SDK는 자체적으로 glove 발견하므로 ID 불일치해도 발행은 됨.

---

## 7. ManusGlove 메시지

자세한 필드는 [`../../msgs/manus_ros2_msgs/DEVELOPER.ko.md`](../../msgs/manus_ros2_msgs/DEVELOPER.ko.md) 참조.

핵심:
- `glove_id` — uint32
- `side` — "L" 또는 "R"
- `nodes` — `ManusRawNode[]` (skeleton 노드 위치/회전)
- `ergonomics` — `ManusErgonomics` 또는 key→value 맵

---

## 8. 흔한 함정

- **ManusSDK 누락**: `ManusSDK/lib/libManusSDK_Integrated.so` 또는 `ManusSDK/include/ManusSDK.h` 없으면 CMake error. §3.1 절차로 복사.
- **빌드는 되는데 런타임에 SDK 못 찾음**: RPATH `$ORIGIN`이 설정되어 있으나 install/share/lib 구조가 깨졌으면 fail. `colcon build`를 source 디렉토리에서 한 번 더 시도.
- **gRPC missing 에러**: Remote 모드 SDK를 사용 중. CMakeLists에서 `ManusSDK_Integrated` 확인 (기본).
- **glove 발견 안 됨**: dongle USB 연결 + Manus glove pairing 확인. ncurses 화면에서 자체 진단 로그 확인.
- **glove ID 불일치**: `gloves.yaml`의 ID는 시각적 매핑용. 다른 dongle 사용 시 갱신.
- **`s_Instance` static**: 단일 노드 인스턴스만 가능 (콜백이 static). 한 프로세스에 여러 ManusDataPublisher 불가.

---

## 9. 확장 / 수정 가이드

### 9.1 raw sensor data 발행 (현재 미사용)
1. `OnRawDeviceDataStreamCallback`이 이미 데이터를 `m_RawSensorDataMap`에 저장
2. `PublishCallback`에서 새 publisher 생성 (`/manus_raw_<id>`)
3. `manus_ros2_msgs/ManusRawData` 같은 새 메시지 정의 (msgs 패키지)

### 9.2 좌표계 변경
- `m_CoordinateSystem` 필드 수정. 예: `AxisView::AxisView_ZFromViewer`.
- Manus 문서의 좌표계 옵션 참고.

### 9.3 Remote 모드 (Manus Core 사용)
1. ManusCore 설치 + 실행
2. `CMakeLists.txt`에서 `ManusSDK_Integrated` → `ManusSDK`로 변경
3. `m_ConnectionType = ConnectionType_Local` 또는 `Remote` (IP 설정)
4. gRPC dependency 설치

---

## 10. 연관 패키지

- `manus_ros2_msgs` (msgs) — `ManusGlove`, `ManusErgonomics`, `ManusRawNode` 정의
- `manus_inspire` (core) — `/manus_glove_*` 유일한 구독자
- 외부: ManusSDK 바이너리, ncurses
