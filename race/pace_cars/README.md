# Monza / Hungaroring 시연 차량 2대

## 실행

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic

```bash
sudo apt update
sudo apt install -y cmake g++ libgz-sim8-dev
cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
bash run_pace_cars.sh monza
# 또는 bash run_pace_cars.sh hungaroring
```

관전 카메라 (다른 터미널):

```bash
cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
bash watch_pace_cars.sh monza
# 또는 bash watch_pace_cars.sh hungaroring
```

맵/차량 데이터 생성 및 정적 확인만:

```bash
bash run_pace_cars.sh monza --prepare-only
bash run_pace_cars.sh hungaroring --prepare-only
```

| 차량 | 그리드 | 목표 속도 | 시작 조건 |
| --- | ---: | ---: | --- |
| 빨간색 | 2 | 10 m/s | 빨간불 5개 **전체 소등 후** |
| 파란색 | 9 | 20 m/s | 빨간불 5개 **전체 소등 후** |

- 두 차량은 원본 그리드의 X/Y/Z/방향에서 출발합니다. 초기 출발 시 트랙 중앙으로 옮기지 않습니다.
- 각자의 원래 그리드와 동일한 **도로 측면 간격**을 유지하며 연속 루프를 주행합니다.
- 기존 `run_v10.sh`, 관전 카메라, 원본 맵, 빨간불 시퀀스 코드는 수정하지 않았습니다. 실행 중 별도의 생성 폴더에 맵 복사본을 생성합니다.
- 속도는 Gazebo 시뮬레이션 시간에 대한 경로 이동 속도이며, 차량은 **Gazebo 시스템 플러그인에서 경로를 따라 위치 갱신하는 시연 차량**입니다. 센서 기반 자율주행, 실제 조향·구동 물리, 충돌 회피 기능이 아닙니다.
- 신호등의 경기 규정에 따라 **5개가 순차 점등되고 모두 동시에 꺼진 뒤에만 출발**합니다. 신호등 종료 서비스가 성공했다는 출력이 없으면 주행하지 않습니다.
- 계속 회전하는 차량 간 추월·접촉에 대한 대응은 구현하지 않았습니다. 20 m/s는 급커브에서 물리 기반 구동 차량에는 비현실적일 수 있습니다.
- 실제 Gazebo GUI/물리 주행 검증은 Ubuntu/Gazebo 설치 컴퓨터에서 확인해야 합니다.
- Gazebo 로그: `generated/pace_cars/<track>/gazebo.log`.

### 종료

맵 실행 터미널에서 `Ctrl+C`.
