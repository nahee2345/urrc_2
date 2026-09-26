# URRC 몬자 대회 워크스페이스

이 저장소는 **1/5 축척 몬자(Monza) 맵만** 포함합니다. 다른 서킷의 world, model, 좌표, launch 파일은 배포하지 않습니다. 연습용 `monza.sdf`와 신호등·랩 계측용 `monza_race.sdf`가 들어 있습니다.

> 공개 좌표와 SRTM 지형을 사용한 제작본이며 측량 인증본이 아닙니다. 도로 폭·출발 위치·고도는 근사값입니다. 정적 검사는 수행했지만 이 제작 환경에서는 Gazebo/ROS 2 동적 주행을 검증하지 못했습니다.

## 실행과 대회 운영

```bash
cd ~/urrc_f1_ws
bash run_monza.sh                 # 연습
bash run_monza.sh --view grid     # 그리드 화면
bash test_gazebo.sh               # Gazebo 서버 검사

# 팀 폴더/ZIP을 race/inbox/에 넣은 뒤
bash competition.sh prepare
bash competition.sh new-event
bash competition.sh qualifying
bash competition.sh final
```

ROS 2 워크스페이스 빌드:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/urrc_f1_ws
bash build.sh
source install/setup.bash
ros2 launch urrc_track_gazebo monza.launch.py
```

## 차량 규격

| 항목 | 규정(Gazebo 미터 단위) |
|---|---|
| 최대 외형 | 길이 1.20 m × 폭 0.50 m × 높이 0.35 m |
| 권장 외형 | 길이 1.10 m 이하 × 폭 0.46 m 이하 |
| 차량 좌표 | `+X` 전방, `+Y` 좌측, `+Z` 위쪽 |
| 모델 원점 | 차량 중심의 지면 투영점 권장 |
| 기본 스폰 높이 | `road_z + 0.04 m` |
| 센서 | 외형 한계 안에 LiDAR/거리 센서 1개 이상 |

외형은 collision, 바퀴, 센서, 윙 등 모든 돌출부를 포함합니다. 그리드 표시가 1.20 × 0.50 m이므로 한계치 차량은 오차 여유가 없습니다. 모델 원점에서 바퀴 최저점까지 높이가 0.04 m가 아니면 `race_entry.json`의 `spawn_z_offset_m`만 조정하십시오. 맵 좌표를 팀별로 수정하면 안 됩니다.

## 몬자 스폰 위치

- 출발/결승선: `x=0.00000`, `y=0.00000`, `z=1.22071`, `yaw=1.46663 rad`(약 84.03°)
- 예선 및 1번 그리드: `x=0.37867`, `y=-2.05604`, `road_z=1.20353`, `yaw=1.45738 rad`
- 기본 1번 스폰 Z: `1.24353 m` (`road_z + 0.04`)
- 총 20칸, 연속 칸 종방향 간격 1.6 m, 같은 열 간격 3.2 m, 지그재그 2열
- 전체 좌표의 단일 기준: `src/urrc_track_gazebo/tracks/processed/monza.json`

스폰 pose는 차량 기준 프레임 원점에 적용됩니다. SDF/URDF 내부에 월드 초기 pose를 넣지 마십시오. 본선은 예선 순위대로 1~12번 칸에 배치합니다.

## 팀 제출 폴더

차량, 센서 설정, 주행 알고리즘을 **하나의 독립 폴더**로 제출합니다. 운영자는 폴더 그대로 또는 같은 구조의 ZIP을 `race/inbox/`에 넣습니다.

```text
race/inbox/team01/
  race_entry.json                 필수; 폴더명과 entry_id 동일
  src/
    team01_race/
      package.xml
      CMakeLists.txt 또는 setup.py
      launch/race.launch.py
      models/race_car/model.sdf   또는 URDF
      ...알고리즘·센서 코드...
```

루트 `src/`에 ROS 패키지만 복사하면 팀 메타데이터와 격리가 사라지므로 공식 접수로 인정하지 않습니다. 팀 폴더의 `src/`만 다른 장소로 옮겨 빌드해도 동작하도록 내부 경로는 모두 상대 경로 또는 ROS package share 경로로 작성하십시오. 절대 경로, 홈 디렉터리 경로, 대회 중 외부 다운로드를 금지합니다.

필수 계약:

- [race/entry_template.json](race/entry_template.json)을 복사해 `race_entry.json`을 작성합니다.
- launch는 `race_start_topic`, `speed_topic`, `steering_topic`, `lidar_topic`, `use_sim_time` 인자를 선언하고 사용합니다.
- `/race/start/<entry_id>`가 `true`이기 전에는 구동 명령을 내지 않습니다.
- 중앙 실행기가 차량을 스폰하므로 참가 launch는 차량을 다시 스폰하지 않습니다.
- 노드명·토픽·TF frame·entity를 팀 ID로 격리하고 의존성을 `package.xml`에 선언합니다.
- `use_sim_time=true`를 사용합니다.

상세 제출 계약, 예선/본선, 실격·재주행 규정은 [race/README_KO.md](race/README_KO.md)를 따릅니다.

## 제작 시 주의사항

- 실제 몬자 공표 길이 5,793 m를 1/5로 구성했지만 공개 좌표 해상도 때문에 생성 중심선은 정확히 1,158.6 m와 같지 않을 수 있습니다.
- 도로 폭은 실차 기준 10~12 m의 1/5입니다. 벽 높이 0.8 m는 안전을 위해 다시 축소하지 않았습니다.
- 차량 collision은 복잡한 concave mesh 대신 primitive 또는 단순 convex 형상을 권장합니다.
- 검은 배경, 회색 노면, 빨강/흰색 벽에서도 센서가 동작해야 합니다. 절대 월드 좌표나 맵 색을 외운 주행은 금지합니다.
- `model://` 또는 package share URI를 사용하고 팀 PC의 절대 경로를 넣지 마십시오.
- `/clock`, 중앙 대회 토픽, 다른 팀 토픽을 변조하면 실격입니다.

```text
urrc_f1_ws/
  run_monza.sh
  competition.sh
  race/                         접수·경기 실행기와 규정
  src/urrc_track_gazebo/
    worlds/                     몬자 연습/대회 world
    models/urrc_monza/          도로·벽·표시 메시
    tracks/{raw,config,processed}/
    launch/monza.launch.py
    scripts/                    실행·생성·검증
```

Gazebo Harmonic + ROS 2 Jazzy(Ubuntu 24.04)를 기본 대회 환경으로 고정하는 것을 권장합니다. 예선과 본선 사이에 운영체제, ROS/Gazebo, GPU 드라이버, 물리 설정을 바꾸지 마십시오.
