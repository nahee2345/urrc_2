# URRC 참가 차량·알고리즘 제출 규격

## 1. 공통 환경과 차량

- Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic. 대회 PC의 사용자명·폴더명·IP와 무관하게 실행되어야 합니다.
- 차량 최대 외형: **길이 0.55 m × 폭 0.25 m × 높이 0.18 m**. 바퀴·LiDAR·모든 돌출부와 collision을 포함합니다.
- 전륜 조향, 최대 조향각 35° 이하. 모델 좌표는 +X 전방, +Y 왼쪽, +Z 위쪽입니다.
- 원점은 차량 중심의 지면 투영점을 권장합니다. `spawn_z_offset_m`는 노면에서 모델 원점까지의 스폰 높이이며 기본 0.04 m입니다. 바퀴가 안정적으로 접촉하도록 확인해 주세요.
- 주행 판단은 자신의 LiDAR만 사용합니다. 카메라·GPS·IMU·월드 pose·운영자 계측 토픽·저장된 트랙 좌표로 주행하지 않습니다.
- 렌즈가 1초 간격으로 5개 켜졌다가 모두 꺼지면 출발합니다. 초록등은 사용하지 않습니다. 알고리즘은 아래 Bool 신호를 기준으로 출발하며 카메라로 신호등을 읽지 않습니다.

## 2. 제출 폴더

```text
team01/
  race_entry.json
  src/
    team01_race/
      package.xml
      CMakeLists.txt 또는 setup.py + setup.cfg + resource/
      race_entry.json             # src/패키지만 제출할 때 필수
      launch/race.launch.py
      models/team01_car/vehicle.urdf  # 또는 model.sdf
      meshes/                    # 차량이 참조하는 모든 메시
      config/                    # 센서·제어·bridge·알고리즘 설정
      ... 알고리즘과 차량 제어 어댑터 소스 ...
    team01_support/              # 필요하면 보조 패키지도 함께 제출
```

`race/entry_template.json`을 복사하여 실제 값으로 작성합니다. 팀 폴더명과 `entry_id`를 같게 합니다. 대표 ROS 패키지에도 같은 `race_entry.json`을 넣으면 **src만 또는 단일 패키지만 전달**해도 접수기가 표준 팀 폴더로 정리합니다. 이때도 `vehicle_model_file`은 정리된 팀 폴더 기준 `src/패키지명/...` 경로입니다. 메타데이터 없이 URDF·src만 제출하면 실행할 패키지와 토픽을 알 수 없으므로 접수되지 않습니다.

- 단일 패키지 제출은 모든 코드·메시·설정이 그 패키지 안에 있을 때만 가능합니다.
- 보조 패키지가 있으면 전부 포함된 src 또는 팀 폴더를 제출합니다.
- 모든 ROS 패키지 이름과 model://로 참조하는 모델 폴더 이름은 팀별로 고유하게 작성합니다. `team01_` 등 접두어를 사용해 주세요.
- `build`, `install`, `log`, `.git`, 캐시·기록·대회 맵·시스템 Python 환경은 제출하지 않습니다.
- URDF를 사용하면 Gazebo의 바퀴 조인트·제어 플러그인·LiDAR 설정까지 포함합니다. xacro는 확장된 URDF로 제출합니다. URDF 파일만으로 주행 알고리즘이 실행되지는 않습니다.

## 3. 필수 launch 및 토픽 계약

참가 launch는 다음 인자를 선언하고 실제 노드·bridge 설정에 전달해야 합니다.

| launch 인자 | 의미/형식 |
|---|---|
| `race_start_topic` | `/race/start/<entry_id>`, `std_msgs/msg/Bool`, reliable/volatile, depth 10 |
| `speed_topic` | 차량별 `std_msgs/msg/Float32`, 목표 속도 m/s, +전진/-후진 |
| `steering_topic` | 차량별 `std_msgs/msg/Float32`, 목표 조향 rad, **+우회전/-좌회전** |
| `lidar_topic` | 차량별 `sensor_msgs/msg/LaserScan`, 센서 QoS(best effort 수신 지원) |
| `use_sim_time` | `true`; 중앙 `/clock` 사용 |

차량 플러그인이 Twist·Ackermann·다른 타입을 요구하면 참가 패키지의 어댑터에서 변환합니다. Gazebo 표준 `angular.z`는 +좌회전이므로 상위 조향 규칙과 부호를 맞춰 변환해야 합니다. 운영자는 참가 차량마다 제어 코드나 토픽을 수동 수정하지 않습니다.

- 참가 launch가 알고리즘, LiDAR ROS bridge, 구동·조향 어댑터 및 필요한 컨트롤러를 실행합니다.
- **중앙 실행기가 world와 차량을 스폰합니다. 참가 launch에서는 world/차량을 스폰하지 않습니다.**
- `/clock` bridge와 대회 계측 bridge는 중앙에서 실행합니다. 참가자는 중복 실행하지 않습니다.
- 출발 Bool을 받기 전 및 false일 때는 속도 명령을 계속 0으로 유지합니다. true 수신 후 주행을 시작합니다. 노드 재시작 시 다시 정지 상태로 시작해야 합니다.
- 노드, ROS/Gazebo 토픽, service, action, TF frame 및 모델 내부 센서/제어 이름을 팀별로 분리합니다.
- `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `GZ_PARTITION`은 운영 환경에서 상속받습니다. 참가 코드에서 덮어쓰지 않습니다.
- 중앙 `/race/*`, `/world/*`, `/clock`에 임의 발행·호출하거나 타 팀의 정보를 사용하지 않습니다.

## 4. 경로와 의존성

- `/home/ww/...`, `/home/qor/...`, 특정 IP·네트워크 인터페이스, 개인 Downloads 경로를 코드·URDF·launch에 넣지 않습니다.
- Python launch에서는 `get_package_share_directory()` 또는 `FindPackageShare`로 설치 리소스를 찾습니다.
- `models`, `meshes`, `config`, `launch`가 package share에 설치되도록 CMake/setup.py를 작성합니다.
- Gazebo 리소스는 `model://` 또는 올바른 package URI로 참조합니다. 필요한 파일은 모두 제출 src 안에 포함합니다.
- 심볼릭 링크는 제출하지 않습니다. 외부 파일을 링크한 경우 원본 파일로 복사해 주세요.
- ROS 의존성을 `package.xml`에, 추가 Python 의존성을 `requirements.txt`에 명시합니다. 운영자가 사전에 설치·검토합니다. 경기 중 다운로드하거나 pip/apt를 실행하지 않습니다.
- 가상환경, 컴파일된 다른 PC 바이너리, 외부 사용자 설정에 의존하지 않습니다. qor PC에서 소스로 다시 빌드합니다.

## 5. 몬자 1번 그리드

| 항목 | 값 |
|---|---:|
| x | 0.37867 m |
| y | -2.05604 m |
| 노면 z | 1.20353 m |
| 기본 스폰 z | 1.24353 m |
| yaw | 1.45738 rad |

실제 적용은 `src/urrc_track_gazebo/tracks/processed/monza.json`의 첫 grid 좌표와 `spawn_z_offset_m` 합을 사용합니다. 다른 대회 맵 좌표는 운영자가 자동 적용하므로 차량에 몬자 스폰 pose를 고정하지 않습니다.

## 6. 제출 파일 생성

저장소 루트에서 실제 참가 폴더 경로를 지정합니다.

```bash
python3 race/scripts/pack_entry.py /경로/team01 --output /경로/team01.zip
# src만 전달할 경우(대표 패키지에 race_entry.json 필요)
python3 race/scripts/pack_entry.py /경로/팀워크스페이스/src --output /경로/team01-src.zip
# 독립 패키지 하나인 경우
python3 race/scripts/pack_entry.py /경로/team01_race --output /경로/team01-package.zip
```

출력 ZIP 최상위는 `race_entry.json`과 `src/`입니다. 폴더 자체 또는 ZIP을 운영자에게 전달합니다. 운영자는 `race/inbox/`에 넣고 `bash competition.sh prepare`만 실행합니다. 준비가 성공하면 `race/entrants/<entry_id>/`에 정리됩니다. 중복 ID는 덮어쓰지 않습니다.

## 7. 제출 전 실제 검증

1. 다른 이름의 새 폴더에 소스만 복사하고 빌드합니다.
2. 차량과 메시가 모두 표시되고 collision·관성·바퀴 접촉이 정상인지 확인합니다.
3. LiDAR가 실제 센서 데이터를 발행하고 어댑터를 통해 차량이 움직이는지 확인합니다.
4. 출발 전 정지, 출발 Bool true 이후 주행, false 이후 정지를 확인합니다.
5. 몬자에서 아래 연습용 예선을 완주합니다. 예선은 그리드에서 출발하여 첫 출발선 통과 후 **2바퀴**를 완료합니다. 기록은 출발 신호 시점부터 측정합니다.
6. 두 팀을 동시에 실행해 이름·토픽·제어 간섭이 없는지 확인합니다.
7. 최종 제출 후 예선~본선 사이 소스·파라미터를 바꾸지 않습니다.

자동 접수 검사는 외형 크기, 물리 안정성, LiDAR 전용 사용 여부나 모든 런타임 의존성을 인증하지 않습니다. 실제 차량 검수와 주행 시험은 운영자가 진행합니다.

## 8. 예선·본선 규정

- 1~12팀 출전. 0팀이면 실행기가 주행을 시작하지 않습니다.
- 새 이벤트에서 트랙 하나를 선택하며 같은 이벤트의 예선과 본선은 같은 트랙을 사용합니다.
- 예선: 한 팀씩 1번 그리드, 2바퀴, 완주 시간 순.
- 본선: 예선 순위대로 1~12번 그리드, 10바퀴 동시 주행.
- 5개 빨간 LED를 1초 간격으로 점등하고 0.2~3초 후 소등·출발합니다.
- 기본 제한시간: 예선 팀당 600초, 본선 전체 2400초(프로세스 감시의 실시간 제한). 랩 기록은 Gazebo 시뮬레이션 시간입니다.
- 완주자는 기록 순, DNF는 완료 랩 수 우선입니다. 동률은 운영자가 마지막 계측 기록과 예선 순위로 판정합니다.
- 첫 출발선 통과는 그리드 접근 구간이며 한 바퀴로 세지 않습니다. 정상 방향으로 전체 코스를 주행해야 합니다. 코스 단축·반복 선 횡단은 영상/로그로 운영자가 판정합니다.
- 규격 초과, 출발 전 이동, 다른 팀·중앙 시스템 간섭, 코스/physics 변조는 실격입니다.
- 참가 코드/의존성 오류·시간 초과는 DNF, 운영 장비/Gazebo 공통 장애는 해당 주행 무효 후 동일 조건 재주행입니다.
- 예선 후 참가자 명단을 바꾸거나 새 이벤트를 생성하면 기존 결과로 본선을 실행할 수 없습니다.
- 결과는 `race/results/`에 저장됩니다. 새 이벤트 생성 시 기존 JSON/CSV는 `race/results/archive/`로 보관됩니다.
