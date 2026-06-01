# `rby1_core_msgs` 개발자 가이드 (teleop 사본)

> ⚠ **이 패키지는 `hw-core/src/rby1_core_msgs/`의 사본입니다.** 두 사본의 srv 정의는 비트 단위로 동일해야 합니다.

자세한 srv 필드 의미와 사용처는 정본 가이드 참조:
- **정본**: [`<hw-core>/src/rby1_core_msgs/DEVELOPER.ko.md`](../../../../hw-core/src/rby1_core_msgs/DEVELOPER.ko.md)

---

## 1. 왜 사본이 두 개?

- hw-core 워크스페이스와 teleop 워크스페이스는 독립적으로 빌드되므로 각자의 install/share에 자체 사본이 필요
- ROS2 인터페이스는 패키지명 + 메시지명으로 식별 — 두 사본이 동일한 한 양쪽 워크스페이스에서 동일 srv로 인식

## 2. 동기화 절차

hw-core 측에서 먼저 편집:
```bash
cd hw-core/src/rby1_core_msgs
vim srv/MyService.srv
```

teleop 측 동기화:
```bash
cp hw-core/src/rby1_core_msgs/srv/*.srv teleop/src/msgs/rby1_core_msgs/srv/
# CMakeLists.txt도 새 srv 추가 시 양쪽 동일하게 갱신
```

두 워크스페이스 모두 재빌드:
```bash
cd hw-core && colcon build --packages-select rby1_core_msgs && source install/setup.bash
cd ../teleop && colcon build --packages-select rby1_core_msgs && source install/setup.bash
```

CHANGES.md 양쪽에 동일 날짜로 기록.

## 3. 점검 명령

```bash
diff -ur hw-core/src/rby1_core_msgs/srv/ teleop/src/msgs/rby1_core_msgs/srv/
```

차이가 있다면 즉시 동기화. CI에서 이 diff를 검증하면 좋습니다.

## 4. 디렉토리 구조

```
teleop/src/msgs/rby1_core_msgs/
├── CMakeLists.txt                              hw-core 사본과 동일
├── msg/                                          (현재 비어있음 — 포즈는 tf2_msgs/TFMessage)
├── package.xml
└── srv/                                          9개 srv (hw-core와 동일)
    ├── ConnectRobot.srv
    ├── MoveToJointPosition.srv
    ├── SetCartesianJointLimits.srv
    ├── SetControlMode.srv
    ├── SetNullspaceJointRef.srv
    ├── SetNullspaceWeight.srv
    ├── SetPower.srv
    ├── SetServo.srv
    └── SetStream.srv
```

## 5. 소비자 (teleop 워크스페이스 측)

- `vive_rby1` (`vive_rby1_node`, `vive_rby1_debug_node`) — SetControlMode, MoveToJointPosition, SetStream, SetNullspaceJointRef 클라이언트
- `scm_gui` (`scm_gui_node`) — 모든 13개 서비스 클라이언트
