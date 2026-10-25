# URRC 몬자 자율주행 연습 환경

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic용입니다. **몬자 맵 1개**를 사용하며 참가자는 자신의 차량·알고리즘을 직접 제작하고 스폰합니다.

## 설치

```bash
git clone https://github.com/nahee2345/urrc_2.git
cd urrc_2
sudo apt update
sudo apt install -y ros-jazzy-ros-gz python3-colcon-common-extensions cmake g++ libgz-sim8-dev
```

## 실행 (4가지 모드)

| 모드 | 명령어 |
|---|---|
| **연습차 2대 + F1 출발 신호** (기본) | `bash run_practice.sh` |
| 연습차 2대 + 신호등 미작동 | `bash run_practice.sh --no-lights` |
| 연습차 없음 + F1 출발 신호 | `bash run_practice.sh --cars 0` |
| 연습차 없음 + 신호등 미작동 | `bash run_practice.sh --cars 0 --no-lights` |

- **신호등 작동**: 연습차 생성 후 빨간불 5개가 모두 켜지고, 불규칙한 간격으로 **하나씩 꺼집니다**. 모두 꺼지면 연습차량이 출발하며 시퀀스는 시작 시 **한 번만** 실행됩니다. 서비스 지연 시 실제 소등시간은 6초를 넘을 수 있습니다.
- **신호등 미작동**: 출발 빨간불 시퀀스가 실행되지 않으며 연습차가 바로 주행합니다. 신호등 구조물은 보일 수 있습니다.
- 연습차 A: **2번 그리드**, 파란색, **0.7 m/s**. 연습차 B: **20번 그리드**, 주황색, **1.4 m/s**.
- 선택: `--cars 0|1|2`, `--speed-a 0.7`, `--speed-b 1.4`, `--view overview`, `--dry-run`.
- 종료: 실행 터미널에서 `Ctrl+C`.

## 내 자율주행 차량 개발

1. 저장소의 `race/README_KO.md`를 읽고, **별도 ROS 2 패키지**에서 차량 모델(`model.sdf` 또는 URDF), LiDAR, 구동·조향 제어기, 자율주행 알고리즘을 만드세요. `race/entry_template.json`을 복사해 `race_entry.json`을 작성하세요.
2. **최대 차량 크기: 길이 0.55 × 폭 0.25 × 높이 0.18 m** (바퀴·센서·collision 등 돌출부 포함). 전륜 조향각 ±35° 이내. +X 전방·+Y 왼쪽·+Z 위쪽.
3. 참가 차량 스폰은 본인이 직접 진행합니다. **몬자 1번 그리드 중앙**: `x=0.37867`, `y=-2.05604`, 노면 `z=1.20353`, `yaw=1.45738 rad`. 모델 원점 높이 보정 기본값은 `0.04 m`이지만 실제 바퀴 접촉을 확인해 조정하세요.
4. 참가 차량은 자체 LiDAR로 주행하며, 카메라는 현행 대회 규정상 출발 신호 확인용입니다. 속도 `std_msgs/msg/Float32`(m/s), 조향 `std_msgs/msg/Float32`(rad, **양수 우회전**), LiDAR `sensor_msgs/msg/LaserScan`, 출발·정지 `/race/start/<entry_id>`(`std_msgs/msg/Bool`), `use_sim_time=true`. 팀별 고유 토픽 이름을 사용하세요.
5. 연습용 차량은 연습 환경의 장애물이며, **참가자 차량의 자동 스폰 및 알고리즘 실행은 하지 않습니다**. 패키지 제작 후 본인이 Gazebo에 차량을 생성하고 ROS launch를 실행해 회피를 시험하세요.

### 참고

- `bash run_monza.sh --view grid`: 기존 몬자 맵 실행.
- `race/README_KO.md`: 참가 제출물 형식·실행 계약·세부 규칙.
- `python3 verify_release.py`: 기존 배포 원본 검증. README 및 연습 기능 추가 후에는 체크섬/검사 규칙의 갱신이 필요할 수 있습니다.
- 연습용 두 차량은 **시뮬레이터 내부에서 위치를 연속 갱신하는 운동학적 모델**입니다. 바퀴 힘·제동·LiDAR 기반 자체 우회 알고리즘은 구현되어 있지 않습니다. 따라서 실제 물리 충돌·회피 성능 검증을 대체하지 못합니다.
