# `inspire_hand_msgs` 개발자 가이드 (teleop 사본)

> ⚠ **이 패키지는 `hw-core/src/inspire_hand/src/inspire_hand_msgs/`의 사본입니다.** 두 사본의 msg 정의는 비트 단위로 동일해야 합니다.

자세한 메시지 필드 의미와 사용처는 정본 가이드 참조:
- **정본**: [`<hw-core>/src/inspire_hand/src/inspire_hand_msgs/DEVELOPER.ko.md`](../../../../hw-core/src/inspire_hand/src/inspire_hand_msgs/DEVELOPER.ko.md)

---

## 1. 왜 사본이 두 개?

- hw-core 워크스페이스에 `inspire_hand_driver`(메시지를 생산·소비), teleop 워크스페이스에 `manus_inspire`(메시지를 생산), `scm_gui`(메시지를 소비)
- 두 워크스페이스가 독립 빌드되므로 각자 install/share에 자체 사본 필요
- 패키지명 + 메시지명으로 식별되므로 두 사본이 동일하면 동일 메시지로 인식

## 2. 동기화 절차

hw-core 측이 정본:
```bash
cd hw-core/src/inspire_hand/src/inspire_hand_msgs
vim msg/InspireHandCtrl.msg
```

teleop 사본 갱신:
```bash
cp hw-core/src/inspire_hand/src/inspire_hand_msgs/msg/*.msg teleop/src/msgs/inspire_hand_msgs/msg/
```

두 워크스페이스 재빌드 + CHANGES.md 양쪽 기록.

## 3. 점검 명령

```bash
diff -ur hw-core/src/inspire_hand/src/inspire_hand_msgs/msg/ teleop/src/msgs/inspire_hand_msgs/msg/
```

## 4. 디렉토리 구조

```
teleop/src/msgs/inspire_hand_msgs/
├── CMakeLists.txt
├── msg/
│   ├── InspireHandCtrl.msg
│   ├── InspireHandState.msg
│   └── InspireHandTouch.msg
└── package.xml
```

## 5. 소비자 (teleop 워크스페이스 측)

- `manus_inspire` (`manus_inspire_node`) — `InspireHandCtrl` publisher (`/rt/inspire_hand/ctrl/{l,r}`)
- `scm_gui` — `InspireHandState` 구독 가능 (현재 코드는 미사용; 상태 표시 확장 가능)
