# `interbotix_xs_msgs` 개발자 가이드

> Interbotix X-Series 메시지/서비스 정의. **본 워크스페이스에서는 빌드하지 않습니다** (`COLCON_IGNORE` 존재). 시스템 설치된 사본을 사용.

---

## 1. 왜 본 워크스페이스에 있는가?

- 코드 참고용으로 포함 (소스 트리에서 .msg/.srv 정의 확인 가능)
- `manus_ros2` 등 일부 노드의 빌드 의존성에 들어가지만, 실제 패키지는 system-wide 설치 (`ros-humble-interbotix-xs-msgs` 또는 유사) 가정

---

## 2. 디렉토리 구조

```
src/msgs/interbotix_xs_msgs/
├── COLCON_IGNORE                              ← colcon이 본 패키지를 건너뜀
├── CMakeLists.txt
├── README.md                                    Interbotix 공식 README
├── package.xml
├── msg/
│   ├── ArmJoy.msg
│   ├── HexJoy.msg
│   ├── JointGroupCommand.msg
│   ├── JointSingleCommand.msg
│   ├── JointTemps.msg
│   ├── JointTrajectoryCommand.msg
│   ├── LocobotJoy.msg
│   └── TurretJoy.msg
└── srv/
    ├── MotorGains.srv
    ├── OperatingModes.srv
    ├── Reboot.srv
    ├── RegisterValues.srv
    ├── RobotInfo.srv
    └── TorqueEnable.srv
```

---

## 3. 시스템 설치 절차

```bash
sudo apt install ros-humble-interbotix-xs-msgs   # 또는 source build
```

system-wide로 설치되면 `find_package(interbotix_xs_msgs REQUIRED)`가 system 패키지를 발견.

> 만약 본 워크스페이스 사본을 사용하려면 `COLCON_IGNORE` 파일 삭제. **권장 안 함** — system 패키지와 충돌 가능.

---

## 4. 사용처

현재 teleop 측 코드(`vive_rby1`, `manus_inspire` 등)는 본 메시지를 **직접 사용하지 않음**. 일부 디버그 스크립트나 외부 모듈이 사용할 수 있어 정의를 유지.

---

## 5. 메시지/서비스 개요

자세한 내용은 [Interbotix 공식 문서](https://github.com/Interbotix/interbotix_ros_core/tree/main/interbotix_ros_xseries/interbotix_xs_msgs) 참조. 본 사본도 README.md 포함.

핵심:
- `JointGroupCommand`, `JointSingleCommand` — 단일/그룹 joint 명령
- `RobotInfo.srv` — 로봇 정보 조회
- `OperatingModes.srv` — 모터 모드 전환 (`position`, `velocity`, `pwm` 등)
- `TorqueEnable.srv` — 토크 on/off

---

## 6. 흔한 함정

- **`COLCON_IGNORE` 삭제 후 빌드 시도**: system 사본과 충돌 위험. 두 사본이 같은 패키지명을 등록하면 마지막 source된 쪽이 ABI 결정. 권장 안 함.
- **system 사본 미설치**: `find_package` 실패. apt 설치 또는 다른 워크스페이스에서 source.

---

## 7. 연관 패키지

본 패키지는 빌드되지 않으므로 의존성 그래프에서 분리됨. system 패키지로 대체.
