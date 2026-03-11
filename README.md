# Samsung
teleoperation/
├── inputs/                  # 1. 입력 디바이스 (Sensors & Controllers)
│   ├── base_input/          # 입력 디바이스 공통 인터페이스 (추상 클래스)
│   ├── manus_vive/          # Manus Glove + Vive Tracker 관련 패키지
│   └── xr_hmd/              # XR HMD 관련 패키지
│       ├── openxr/          # Meta Quest, Galaxy XR 등을 위한 OpenXR 공통 클라이언트
│       └── visionos/        # Apple Vision Pro (AVP) 전용 앱/클라이언트
│
├── robots/                  # 2. 제어 대상 로봇 (Targets)
│   ├── base_humanoid/       # 휴머노이드 공통 인터페이스 (관절 상태, 제어 명령 구조체 등)
│   ├── rby1/                # 현재 타겟인 RBY1 전용 제어 및 통신 모듈
│   └── dummy_robot/         # 테스트용 가상 로봇 (물리 로봇 없이 테스트할 때 유용)
│
├── core/                    # 3. 핵심 로직 (Brain)
│   ├── retargeting/         # 사람의 모션 데이터를 로봇의 Kinematics(역기구학 등)에 맞게 변환
│   ├── filtering/           # 센서 노이즈 제거 및 모션 스무딩 (Kalman filter, Low-pass 등)
│   └── safety/              # 충돌 방지, 특이점(Singularity) 회피 등 안전 로직
│
├── comms/                   # 4. 통신 프로토콜 (Network & Middleware)
│   ├── ros2/                # ROS2 노드 및 메시지 타입
│   └── webrtc_udp/          # 저지연 고속 통신을 위한 네트워크 래퍼 (필요 시)
│
├── launch/                  # 5. 실행 및 통합 설정
│   ├── scripts/             # 실행 스크립트 (ex: run_quest_to_rby1.sh)
│   └── configs/             # YAML 파라미터 설정 (장치별 오프셋, 매핑 파라미터 등)
│
├── docker/                  # 환경 설정 (의존성 충돌 방지용)
├── docs/                    # 위키 및 사용 설명서
└── README.md
