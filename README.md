# URRC Monza 레이스 워크스페이스


## 환경

기본 환경은 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic입니다. 팀 연습 PC와 대회 PC에서 같은 버전 및 물리 설정을 사용해 주세요.

## 저장소 내려받기

~~~bash
cd ~
git clone https://github.com/nahee2345/urrc_2.git urrc_f1_ws
cd ~/urrc_f1_ws
~~~

## Monza 

기본 화면으로 실행:

~~~bash
cd ~/urrc_f1_ws
bash run_monza.sh
~~~

출발 그리드가 보이는 화면으로 실행:

~~~bash
bash run_monza.sh --view grid
~~~

Gazebo 서버 확인:

~~~bash
bash test_gazebo.sh
~~~

연습 월드는 맵만 실행합니다. 참가팀은 자신의 차량 모델과 LiDAR 주행 알고리즘을 별도로 준비해 연습해야 합니다.

## 차량 규격

대회 참가 차량의 최대 외형은 다음과 같습니다. 바퀴·센서·안테나 등 차량 외곽으로 돌출되는 부분도 크기에 포함합니다.

| 항목 | 최대 크기 |
|---|---:|
| 길이 | 0.55 m |
| 폭 | 0.25 m |
| 높이 | 0.18 m |

차량 좌표계는 +X 전방, +Y 좌측, +Z 위쪽을 기준으로 합니다. 차량 모델의 원점, 바퀴 접촉면, collision 형상을 함께 점검하고, 스폰 시 바닥에 묻히거나 뜨지 않도록 조정해 주세요.

## Monza 출발·스폰 기준

좌표는 Gazebo 월드 좌표이며 각도 단위는 radian입니다.

- 출발/결승 계측선: x=0.00000, y=0.00000, z=1.22071, yaw=1.46663
- 예선 출발 및 본선 1번 칸 중심: x=0.37867, y=-2.05604, 노면 z=1.20353, yaw=1.45738
- 기본 차량 스폰 높이: 노면 높이에 0.04 m를 더한 값 (z=1.24353)
- 본선 그리드는 예선 순위대로 배치하며 최대 참가 인원은 12팀입니다.

나머지 그리드 좌표는 src/urrc_track_gazebo/tracks/processed/monza.json을 기준으로 합니다. 맵이나 좌표를 참가팀별로 수정하지 마세요.

## 참가팀 제출물

참가팀은 **차량 모델(URDF 또는 SDF), 센서 설정, LiDAR 주행 알고리즘, ROS 2 패키지**를 한 폴더에 넣어 제출합니다. 폴더 전체를 복사하거나 같은 구조의 ZIP으로 만들어 운영자에게 전달해 주세요.

~~~text
team01/
├── race_entry.json
└── src/
    └── team01_race/
        ├── package.xml
        ├── CMakeLists.txt 또는 setup.py
        ├── launch/
        │   └── race.launch.py
        ├── models/
        │   └── race_car/
        │       └── model.sdf 또는 vehicle.urdf
        └── 알고리즘 및 필요한 설정 파일
~~~

race_entry.json은 race/entry_template.json을 복사해 작성합니다. 참가 폴더 이름과 entry_id를 같게 하고 다음 항목을 빠짐없이 지정해 주세요.

- 참가 ID와 표시 이름
- ROS 패키지 이름 및 실행할 launch 파일
- 차량 모델 파일의 참가 폴더 기준 상대 경로와 형식
- 차량별 속도·조향·LiDAR 토픽 이름
- 필요한 launch 인자

참가 launch는 race_start_topic, speed_topic, steering_topic, lidar_topic, use_sim_time 인자를 받아 알고리즘에 전달해야 합니다. /race/start/참가ID가 true가 되기 전까지 차량은 정지 상태를 유지해야 합니다. 차량은 대회 실행기가 스폰하므로 참가 launch에서 차량을 중복 스폰하지 마세요.

토픽, 노드, TF 프레임, Gazebo 모델 이름은 참가 ID별로 겹치지 않게 설정합니다. 패키지는 제출 폴더 안의 파일만 사용하고, 특정 사용자의 홈 경로나 외부 인터넷 다운로드에 의존하지 않도록 구성해 주세요.

**src/ 또는 ROS 패키지만 단독으로 보내면 참가 메타데이터가 빠집니다.** race_entry.json과 src/가 함께 있는 참가 폴더 전체를 제출해야 합니다.

## 운영자: 참가 접수와 경기 실행

참가 폴더 또는 ZIP을 race/inbox/에 넣은 뒤 대회 PC에서 아래 순서로 실행합니다.

~~~bash
cd ~/urrc_f1_ws
bash competition.sh prepare
bash competition.sh new-event
bash competition.sh qualifying
bash competition.sh final
~~~

- prepare: 참가물을 접수하고 ROS 패키지를 빌드·검사합니다.
- new-event: 새 대회 기록을 시작합니다. 예선 전에 한 번 실행합니다.
- qualifying: 참가 차량을 한 대씩 출발시켜 2랩 기록을 측정합니다.
- final: 예선 순위로 그리드에 배치해 10랩 본선을 진행하고 결과를 저장합니다.

예선과 본선 사이에는 new-event를 다시 실행하지 마세요. 대회 트랙은 Monza로 고정되어 있습니다. 결과는 race/results/에 저장됩니다. 참가자가 0명이면 연습은 run_monza.sh로 하고, 예선·본선 명령은 실행하지 않아도 됩니다.

## LiDAR 주행 및 제작 주의사항

- 주행 판단은 자기 차량의 LiDAR 센서 데이터로 수행합니다.
- 카메라·GPS·IMU·월드 좌표·다른 참가 차량의 센서 데이터에 의존하는 알고리즘은 사용하지 않습니다.
- 차량 collision은 실제 차체 크기에 맞게 구성하고, 트랙 관통이나 비정상적으로 큰 보이지 않는 collision을 만들지 않습니다.
- 센서의 시야가 차체에 가려지지 않도록 확인합니다.
- 관성값은 양수이며 유한한 값으로 설정하고, 바퀴가 노면에 닿은 상태로 안정적으로 정지하는지 시험합니다.
- 예선과 본선에서 차량 소스, 알고리즘, 설정값을 바꾸지 않습니다.
- 실제 대회 전에 참가 차량 한 대로 스폰, 출발 신호, LiDAR 토픽, 제어 토픽, 랩 계측을 먼저 확인합니다.

상세 실행 규정은 race/README_KO.md, 제출 설정 예시는 race/entry_template.json을 참고해 주세요.
