**CoCooker**는 AI 비전과 6축 협동로봇을 연동한 지능형 주방 보조 로봇 시스템입니다.
> 본 프로젝트는 단순한 시스템 구현을 넘어, 엔지니어링 관점의 **'프로세스 최적화 및 데이터 기반 신뢰성 검증'**에 핵심 역량을 집중했습니다.

# 주요기능
- 음성기반 통합제어 인터페이스
- 고확장성 플러그앤플레이 아키텍쳐(Highly Scalable & Modular Architecture)
- 6-DoF 정밀 모션제어
- 다중객체 인식 파이프라인
- 데이터 기반 시스템 신뢰성 검증(Data-Driven System Verification)

# 시스템 설계 및 플로우차트
- [시스템 설계](docs/system_architecture.png)
- [플로우차트](docs/flowchart.png)
- [워크스페이스](docs/workspace.png)
- tree
```
cocooker
├── analysis
│   ├── data
│   │   ├── pot_pick_data2.csv
│   │   └── pot_pick_data.csv
│   ├── notebooks
│   │   ├── pick_and_place_pot_yolo_validation.py
│   │   └── validataion.py
│   └── results
│       ├── confusion_matrix.png
│       ├── cosine correlation.png
│       ├── regression.png
│       └── system_performance_analysis.jpeg
├── docs
│   ├── flowchart.png
│   ├── system_architecture.png
│   └── workspace.png
├── README.md
└── src
    ├── kitchen_assistant
    │   ├── kitchen_assistant
    │   │   ├── core
    │   │   ├── drivers
    │   │   ├── __init__.py
    │   │   ├── legacy
    │   │   │   ├── fridge_leftdown_close.py
    │   │   │   ├── fridge_leftdown_open.py
    │   │   │   ├── fridge_leftup_close.py
    │   │   │   ├── fridge_leftup_open.py
    │   │   │   ├── fridge_open_then_apple_pick.py
    │   │   │   ├── fridge_rightdown_close.py
    │   │   │   ├── fridge_rightdown_open.py
    │   │   │   ├── fridge_rightup_close.py
    │   │   │   ├── fridge_rightup_open.py
    │   │   │   ├── __init__.py
    │   │   │   ├── pick_and_place_apple_yolo.py
    │   │   │   ├── pick_and_place_bottle_yolo.py
    │   │   │   ├── pick_and_place_cup_yolo.py
    │   │   │   ├── pick_and_place_knife.py
    │   │   │   ├── pick_and_place_orange_yolo.py
    │   │   │   ├── pick_and_place_pan_yolo.py
    │   │   │   ├── pick_and_place_pot_yolo.py
    │   │   │   ├── pick_and_place_ramen_yolo.py
    │   │   │   ├── pick_and_place_scissors_yolo.py
    │   │   │   ├── pick_and_place_spatula_yolo.py
    │   │   │   ├── pick_and_place_spoon_yolo.py
    │   │   │   └── pytorch_version.py
    │   │   ├── models
    │   │   │   ├── kitchen.pt
    │   │   │   ├── only_ramen_best.pt
    │   │   │   ├── potandhandle.pt
    │   │   │   ├── T_gripper2camera.npy
    │   │   │   └── yolo11n.pt
    │   │   ├── nodes
    │   │   │   ├── __init__.py
    │   │   │   ├── kitchen_cli.py
    │   │   │   └── kitchen_voice.py
    │   │   ├── onrobot.py
    │   │   ├── plot_result.py
    │   │   ├── realsense.py
    │   │   ├── skills
    │   │   ├── tools
    │   │   │   ├── __init__.py
    │   │   │   └── robot_movej.py
    │   │   ├── utils.py
    │   │   └── voice
    │   │       ├── command_parser.py
    │   │       ├── __init__.py
    │   │       ├── runner.py
    │   │       ├── stt_openai.py
    │   │       └── wakeup_word.py
    │   ├── launch
    │   │   ├── kitchen_cli.launch.py
    │   │   └── kitchen_voice.launch.py
    │   ├── package.xml
    │   ├── resource
    │   │   ├── hello_rokey_8332_32.tflite
    │   │   └── kitchen_assistant
    │   ├── setup.cfg
    │   └── setup.py
    └── ui_node
        ├── package.xml
        ├── resource
        │   └── ui_node
        ├── setup.cfg
        ├── setup.py
        ├── test
        │   ├── test_copyright.py
        │   ├── test_flake8.py
        │   └── test_pep257.py
        └── ui_node
            ├── firebase_ros2_bridge_fixed.py
            ├── __init__.py
            └── ui_clean.html
```

# 운영체제 및 개발환경
- OS : Ubuntu 22.04.5 LTS
- Middleware : ROS2 Humble
- Language : Python3
- AI : yolo11n
- Database/Network : HTML5

# 사용장비 목록
- Manipulator : Doosan Robotics M0609 (6-DoF, Payload 6kg)
- Gripper : modbus gripper DA_v1
- Camera : Intel RealSense D435i(RGB-D Depth Sensing)

# 의존성
[requirements.txt](requirements.txt)

# 실행순서
```
# step1. ros2 패키지 설치
colcon build --packages-select kitchen_assistant ui_node

# step2. 노드실행
ros2 run kitchen_assistant kitchen_voice
ros2 run ui_node ui_node

# step3. ui 페이지 로드 및 실행
cd ./src/ui_node/ui_node
xdg-open ui_clean.html

# step4. 검증.
cp ./analysis/notebooks/pick_and_place_pot_yolo_validation.py ./src/kitchen_assistant/kitchen_assistant/legacy/pick_and_place_pot_yolo.py
ros2 run kitchen_assistant kitchen_cli
ros2 run ui_node ui_node
# command> 냄비
```