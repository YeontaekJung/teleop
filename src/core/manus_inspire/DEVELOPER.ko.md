# `manus_inspire` 개발자 가이드

> Manus glove ergonomic 데이터를 Inspire Hand 6-슬롯 명령으로 매핑하는 ROS2 노드. 4-phase 16초 사용자 캘리브레이션 포함.

---

## 1. 패키지 개요

- Manus glove의 `ManusErgonomics`(15+ ergonomic 각도) → `InspireHandCtrl`(6 손가락 슬롯, mode=2 angle)
- **4-phase 16초 캘리브레이션** — 각 손가락의 실제 가동 범위를 사용자별로 측정
- 캘리브레이션 결과 `~/.ros/manus_inspire_calib.yaml`에 저장. 다음 부팅 시 자동 로드.
- mirror mode (`/teleop/mirror_mode`) — 좌우 손 데이터 교환

---

## 2. 디렉토리 구조

```
src/core/manus_inspire/
├── package.xml                                ament_python
├── setup.py                                   entry_point: manus_inspire = manus_inspire.manus_inspire:main
└── manus_inspire/
    ├── __init__.py
    └── manus_inspire.py                        모든 코드 (279줄)
```

---

## 3. 빌드 / 실행

### 3.1 의존성

- `manus_ros2_msgs` (teleop msgs)
- `inspire_hand_msgs` (teleop msgs — hw-core 사본과 동기화 필수)
- `pyyaml`

### 3.2 빌드

```bash
cd teleop
colcon build --packages-select manus_inspire
source install/setup.bash
```

### 3.3 실행

```bash
ros2 run manus_inspire manus_inspire
# 또는 launch에서 자동 실행
```

전제: Manus glove 노드(`manus_data_publisher`)가 `/manus_glove_0`/`/manus_glove_1` 발행 중.

---

## 4. 데이터 흐름

```
manus_ros2 ── /manus_glove_{0,1} ──► ManusInspire
                  (ManusGlove.side="L"/"R")    │
                                               ▼
                                         cb_glove(msg):
                                           1) ergonomic 추출
                                           2) (calib 모드면) sample 누적
                                           3) (정상 모드면) flex_to_inspire 매핑
                                               → /rt/inspire_hand/ctrl/{l,r}
```

---

## 5. 4-Phase 캘리브레이션 (16초)

| Phase | 시간 | 자세 | 측정 |
|---|---|---|---|
| 0 (`1/4`) | 0~4s | **Open hands fully** | 4개 손가락의 굽힘 최솟값 (`index/middle/ring/pinky.min`) + 엄지 spread 최솟값 (`spread.min`) |
| 1 (`2/4`) | 4~8s | **Thumbs up (주먹 + 엄지 펴기)** | 4개 손가락의 굽힘 최댓값 + 엄지 MCPStretch 최댓값 (`thumb.max`) |
| 2 (`3/4`) | 8~12s | **Press thumb to side of index finger** | 엄지 spread 최댓값 (`spread.max`) |
| 3 (`4/4`) | 12~16s | **Open fingers, bend thumb only** | 엄지 MCPStretch 최솟값 (`thumb.min`) |

각 phase 4초 동안 ergonomic 샘플을 누적, phase 종료 시 min/max 추출. 단계 안내는 `get_logger().warn`으로 표시. GUI는 같은 텍스트를 보여줍니다.

### 5.1 캘리브레이션 트리거

- **첫 실행** — `~/.ros/manus_inspire_calib.yaml` 없으면 자동 시작
- **재캘리브레이션** — `~/manus_inspire/calibrate` (`std_srvs/Trigger`) 호출 또는 파일 삭제
- GUI의 "Recalibrate" 버튼이 이 서비스 호출

```bash
# 수동 트리거
ros2 service call /manus_inspire/calibrate std_srvs/srv/Trigger "{}"

# 강제 재캘리브레이션
rm ~/.ros/manus_inspire_calib.yaml
ros2 run manus_inspire manus_inspire
```

### 5.2 캘리브레이션 데이터 구조

```yaml
left:
  index:  {min: 5.2, max: 78.3}      # degrees, weighted flex
  middle: {min: 4.8, max: 82.1}
  ring:   {min: 3.5, max: 79.5}
  pinky:  {min: 2.1, max: 65.0}
  thumb:  {min: -10.0, max: 50.0}    # MCPStretch
  spread: {min: -25.0, max: 35.0}    # MCPSpread
right:
  ... (same structure)
```

---

## 6. 함수별 상세

### 6.1 핵심 상수

| 상수 | 값 | 의미 |
|---|---|---|
| `MAX_INSPIRE` | 1000 | Inspire Hand 슬롯 최댓값 (펌웨어 기준) |
| `CALIB_DURATION` | 4.0s | phase당 시간 |
| `FINGERS` | 6개 슬롯 | `index`, `middle`, `ring`, `pinky`, `thumb`, `spread` |
| `ERGO_KEYS` | dict | 손가락별 (MCP, PIP, DIP) ergonomic 키 이름 |
| `DEFAULT_CALIB` | dict | 캘리브레이션 없을 때 fallback |

### 6.2 매핑 함수

```python
def weighted_flex(mcp, pip, dip):
    # MCP/PIP/DIP 각도를 단일 굽힘 값으로 가중 평균
    return 0.25 * mcp + 0.55 * pip + 0.20 * dip

def flex_to_inspire(flex, calib_min, calib_max, invert=True):
    # 캘리브 범위 [min, max]를 Inspire [0, MAX_INSPIRE]로 선형 매핑
    # invert=True: open=MAX_INSPIRE, closed=0 (Inspire 펌웨어 기본 부호 일치)
    rng = max(calib_max - calib_min, 1.0)
    normalized = (flex - calib_min) / rng     # 0~1
    value = MAX_INSPIRE * (1.0 - normalized) if invert else MAX_INSPIRE * normalized
    return int(clamp(value, 0, MAX_INSPIRE))
```

### 6.3 `cb_glove(msg)` 콜백 흐름

1. `msg.side` (`"L"` 또는 `"R"`) 결정. mirror_mode이면 swap.
2. `msg.ergonomics`에서 손가락별 MCP/PIP/DIP/Spread 추출.
3. **calib_mode** 시: `_collect_sample` → `_advance_calib` (phase 종료 시 min/max 추출).
4. **정상 모드** 시: 각 손가락에 `weighted_flex` → `flex_to_inspire` → `InspireHandCtrl.angle_set[i]` 채우기. `mode = 0b0001` (angle only).
5. side에 따라 `pub_l` 또는 `pub_r`로 publish.

### 6.4 `ManusGlove` 메시지 슬롯 ↔ Inspire 슬롯 매핑

| Inspire 슬롯 | 의미 | Manus ergonomic key |
|---|---|---|
| 0 (검지) | index | `weighted_flex(IndexMCPStretch, IndexPIPStretch, IndexDIPStretch)` |
| 1 (중지) | middle | `weighted_flex(MiddleMCPStretch, ...)` |
| 2 (약지) | ring | `weighted_flex(RingMCPStretch, ...)` |
| 3 (새끼) | pinky | `weighted_flex(PinkyMCPStretch, ...)` |
| 4 (엄지 MCP 굽힘) | thumb | `ThumbMCPStretch` (단일값) |
| 5 (엄지 spread) | spread | `ThumbMCPSpread` |

---

## 7. ROS 인터페이스

### 7.1 구독 토픽

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/manus_glove_0` | `manus_ros2_msgs/ManusGlove` | Manus glove 0번 (side는 msg.side로 식별) |
| `/manus_glove_1` | 동일 | 1번 |
| `/teleop/mirror_mode` | `std_msgs/String` | `"mirror"`/`"normal"` |

### 7.2 발행 토픽

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/rt/inspire_hand/ctrl/l` | `inspire_hand_msgs/InspireHandCtrl` | 왼손 명령 (mode=1=angle, `angle_set[6]`) |
| `/rt/inspire_hand/ctrl/r` | 동일 | 오른손 |

### 7.3 서비스

| 서비스 | 타입 | 의미 |
|---|---|---|
| `~/calibrate` (= `/manus_inspire/calibrate`) | `std_srvs/Trigger` | 캘리브레이션 시작 |

### 7.4 파라미터

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `calib_file` | `~/.ros/manus_inspire_calib.yaml` | 캘리브레이션 저장 경로 |

---

## 8. 흔한 함정

- **첫 실행 시 자동 캘리브레이션**: glove를 정자세로 두지 않으면 잘못된 범위 캐시. 콘솔 안내 따라 명확히 자세 잡기. 잘못 캘리브 시 `~/.ros/manus_inspire_calib.yaml` 삭제 후 재시작.
- **side 불일치**: `msg.side`가 `"L"`/`"R"`. Manus SDK 측 매핑이 어긋나면 좌/우 손이 swap된 채로 보임. mirror_mode 토글로 임시 해결 가능.
- **MAX_INSPIRE 범위**: Inspire 펌웨어가 0~1000을 기대. 다른 펌웨어 버전이면 상수 조정.
- **invert 부호**: open=1000인지 closed=1000인지 펌웨어 의존. `flex_to_inspire(invert=...)`로 토글.

---

## 9. 확장 / 수정 가이드

### 9.1 새 손가락/슬롯 추가
1. `FINGERS`, `ERGO_KEYS`, `DEFAULT_CALIB`에 슬롯 추가
2. `_collect_sample`, `_advance_calib`에 phase별 처리 추가 (또는 새 phase)
3. `cb_glove`의 매핑 루프 갱신
4. `InspireHandCtrl.msg`에 슬롯 확장 시 양쪽 워크스페이스 동기화 + 빌드

### 9.2 캘리브레이션 단계 추가
1. `CALIB_DURATION × N` 늘리기 (4초 × 5단계 = 20초 등)
2. `_advance_calib`에 새 phase 분기 추가
3. GUI `scm_gui_node.py`의 단계 텍스트 + 진행 바 길이 갱신

### 9.3 매핑 함수 변경
- 단순 가중 평균(`weighted_flex`) 대신 비선형 매핑이 필요하면 `flex_to_inspire`나 새 함수를 도입. 캘리브 범위는 그대로 사용.

---

## 10. 연관 패키지

- `manus_ros2` (input) — Manus glove SDK 클라이언트, `/manus_glove_*` 발행
- `inspire_hand_msgs` (msgs) — `InspireHandCtrl` 정의. hw-core 사본과 동기화 필수.
- `manus_ros2_msgs` (msgs) — `ManusGlove`, `ManusErgonomics` 정의
- `inspire_hand_driver` (hw-core) — 본 노드가 발행하는 토픽의 구독자. Modbus로 Inspire Hand에 write.
- `scm_gui` — `Recalibrate` 버튼이 본 노드의 calibrate 서비스 호출
