# `scm_recording_msgs` 개발자 가이드

> 외부 `scm_recording` core(녹화 백엔드)와의 서비스 계약. **scm_recording core 자체는 본 워크스페이스에 없는 별도 패키지**입니다.

---

## 1. 패키지 개요

5개 srv 정의:
- `StartRecording` / `EndRecording` / `TogglePause` — 에피소드 라이프사이클
- `SetTeleOpPose` — teleop pose 갱신 (vive_rby1 측에서 처리)
- `GetStatus` — 녹화 상태 조회

teleop 워크스페이스에 정의되어 있지만 **서비스 서버는 외부**(`scm_recording` 패키지)에서 구현. teleop의 노드들은 클라이언트.

---

## 2. 디렉토리 구조

```
src/msgs/scm_recording_msgs/
├── CMakeLists.txt
├── package.xml
└── srv/
    ├── StartRecording.srv
    ├── EndRecording.srv
    ├── TogglePause.srv
    ├── SetTeleOpPose.srv
    └── GetStatus.srv
```

---

## 3. 빌드

```bash
cd teleop
colcon build --packages-select scm_recording_msgs
source install/setup.bash
```

---

## 4. 서비스별 상세

### 4.1 `StartRecording.srv`

```
int32 task_id
---
bool result
string message
int32 task_id
int32 episode_id
string vla_path
string others_path
```

새 에피소드 시작. `task_id`는 GUI Recording 패널에서 설정. 응답은 외부 `scm_recording`이 할당한 `episode_id` + 데이터가 저장될 경로.

호출자: `vive_rby1_node::toggleEpisode` (IDLE 시).

### 4.2 `EndRecording.srv`

```
bool discard         # true면 폐기 (현재 에피소드 데이터 삭제), false면 정상 종료/저장
---
bool result
string message
```

호출자:
- `vive_rby1_node::toggleEpisode` (PAUSED 시, `discard=false`) — 페달 C
- `vive_rby1_node::discardEpisode` (`discard=true`) — 페달 B

### 4.3 `TogglePause.srv`

```
(빈 요청)
---
bool result
bool paused          # true면 일시정지 상태로 들어감, false면 재개
string message
```

호출자: `vive_rby1_node::callTogglePause` — 클러치 engage/disengage 시.

응답의 `paused`로 RECORDING ↔ PAUSED 상태 결정.

### 4.4 `SetTeleOpPose.srv`

```
sensor_msgs/JointState pose
---
bool success
string message
```

⚠ 이 서비스는 외부 `scm_recording`이 아닌 **`vive_rby1_node`가 서버**입니다 (`/vive_rby1/set_teleop_pose`). teleop 시작/종료 시 이동할 자세 갱신용. GUI dropdown에서 호출.

호출자: `scm_gui_node` ("[Apply Nullspace Ref]" 버튼).

### 4.5 `GetStatus.srv`

```
(빈 요청)
---
string state                # 녹화 상태 문자열 (예: "RECORDING", "PAUSED")
int32 task_id
int32 episode_id
int64 step_index
string vla_path
string others_path
bool recording_active
bool paused
```

녹화 상태 폴링. 현재 teleop 측 코드(`vive_rby1`/`scm_gui`)는 본 서비스를 호출하지 않음 — 외부 시스템 통합용으로 정의됨.

---

## 5. 외부 `scm_recording` core 가정

- `/scm_recording/start` (`StartRecording`)
- `/scm_recording/end` (`EndRecording`)
- `/scm_recording/toggle_pause` (`TogglePause`)
- (optional) `/scm_recording/get_status` (`GetStatus`)

서비스 서버를 외부 패키지가 제공해야 정상 녹화 가능. 외부 패키지 없으면:
- `cli_*->service_is_ready()` 가 false
- teleop은 동작하지만 에피소드 저장 불가 (GUI 버튼 누르면 "service not available" 경고)

---

## 6. 흔한 함정

- **EndRecording 차단** (PAUSED 상태에서만 허용): RECORDING 상태에서 toggleEpisode 호출하면 거부. 페달 A로 먼저 disengage(PAUSED) 후 페달 C.
- **discard 의미**: `discard=true`는 **데이터 삭제**. 실수로 호출하면 복구 불가. 페달 B에 매핑되어 있어 신중히 사용.
- **`SetTeleOpPose` 위치**: 본 srv는 외부 scm_recording이 아닌 vive_rby1 서버. 이름이 헷갈리지만 의도된 위치.
- **`GetStatus` 미사용**: 현재 코드는 ROS 토픽(`/teleop/rec_state`, `/teleop/rec_episode`)으로 polling. 외부 시스템에서 한 번에 조회하려면 GetStatus 호출 가능.

---

## 7. 확장 / 수정 가이드

### 7.1 메타데이터 필드 추가 (예: operator name, environment ID)
1. `StartRecording.srv`의 request에 `string operator_name` 등 추가
2. 외부 scm_recording core 측 핸들러 갱신
3. GUI에서 입력 필드 추가
4. teleop `vive_rby1_node::toggleEpisode`에서 req에 채우기

### 7.2 새 라이프사이클 이벤트 추가 (예: AddBookmark)
- 새 srv 정의 후 CMakeLists.txt에 등록
- 외부 core와 양쪽에서 합의해 구현

---

## 8. 연관 패키지

- `vive_rby1` (core) — 본 서비스의 주요 클라이언트
- `scm_gui` — `SetTeleOpPose` 클라이언트
- 외부 — `scm_recording` (별도 워크스페이스/패키지) — 4개 서비스의 서버
