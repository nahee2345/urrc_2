# URRC F1 자율주행 연습 환경 — urrc_2

`urrc_2`는 참가자에게 제공하는 ROS 2·Gazebo 기반 **연습용 워크스페이스**입니다. 공식 대회는 별도의 운영 환경에서 진행됩니다.

- **지원 환경:** Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic
- **제공 맵:** Monza, Hungaroring, Monaco (3개)
- **차량 센서:** 차량에 장착된 카메라·LiDAR 등 참가팀 센서 사용. 맵 외부 관전용 카메라는 제공하지 않습니다.
- **출발 절차:** 맵 준비 확인 → ROS 브리지·제어 노드 준비 → 차량 스폰 → 차량 카메라 영상 확인 → 빨간불 5개 순차 점등·전체 소등 → 신호 인식 후 출발
- **주의:** 맵 미리보기는 차량 없이 실행됩니다. `src/team/` 폴더가 있다고 자동으로 차량을 스폰하지 않습니다.

## 1. 설치 및 맵 실행

**지원 PC 환경:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, 그래픽 출력 가능한 PC.
Windows, macOS, 다른 ROS 2 배포판에서의 호환성은 보장하지 않습니다.

처음 사용하는 PC에서는 [ROS 2 Jazzy 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)에 따라 ROS 2를 설치한 후, 다음 패키지와 저장소를 준비하십시오.

```bash
sudo apt update
sudo apt install -y git python3 python3-colcon-common-extensions python3-rosdep ros-jazzy-ros-gz

cd ~
git clone https://github.com/nahee2345/urrc_2.git
cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
mkdir -p src/team/team_{01..12}
```

프로젝트에 추가 ROS 의존성이 필요한 경우(ROS 2 환경이 올바르게 설치되어 있다면):

```bash
cd ~/urrc_2
rosdep install --from-paths src --ignore-src -r -y
```

**다운로드가 제대로 되었는지 먼저 확인하십시오.**

```bash
cd ~/urrc_2
test -f race_v24.sh && test -f verify_integrated_v24.sh && echo 'v24 필수 파일 확인 PASS'
bash verify_integrated_v24.sh
```

두 명령이 실패하면 배포 파일 누락 또는 PC 환경 문제이므로 다음 단계로 진행하지 마십시오.

다음 명령은 **차량 없이 맵·그리드·신호등을 확인하는 미리보기**입니다.

| 맵 | 실행 명령어 |
|---|---|
| Monza | `bash race_v24.sh preview qualifying --track monza` |
| Hungaroring | `bash race_v24.sh preview qualifying --track hungaroring` |
| Monaco | `bash race_v24.sh preview qualifying --track monaco` |

맵마다 **한 개씩 실행**하십시오. Gazebo 화면에서 도로·그리드·신호등이 모두 표시되면 터미널에 `READY`를 정확히 입력합니다. 이후 빨간불 5개가 순차 점등되고 동시에 꺼지는지 확인한 다음 Gazebo 창을 닫습니다.

차량 없는 환경의 자동 통합검증:

```bash
cd ~/urrc_2
bash verify_integrated_v24.sh
```

GUI와 신호등을 직접 확인하는 통합검증:

```bash
cd ~/urrc_2
bash verify_integrated_v24.sh --gui
```

## 2. 1번 출발 그리드 좌표

좌표 기준은 Gazebo 월드 좌표계이며, 단위는 m입니다. `yaw`는 rad입니다.

| 맵 | X (m) | Y (m) | 노면 Z (m) | Yaw (rad) |
|---|---:|---:|---:|---:|
| Monza | 0.378672 | -2.056035 | 1.203527 | 1.457384 |
| Hungaroring | 2.032524 | -0.658216 | 6.397033 | 2.469082 |
| Monaco | 0.454020 | -2.007622 | 2.046087 | 1.554473 |

위 Z는 **노면 높이**로, 차량 모델 원점의 최종 스폰 Z와 같다는 뜻이 아닙니다. 실제 스폰 높이는 차량 모델 원점과 노면 접촉 위치를 고려하여 운영 시스템이 처리합니다. 원본 데이터는 `src/urrc_track_gazebo/tracks/processed/<맵>.json`의 `grid` 중 `position: 1`을 참조하십시오.

## 3. 차량 최대 규격 (센서 포함)

| 항목 | 허용 최대치 |
|---|---:|
| 전후 길이 | **0.55 m (550 mm)** |
| 좌우 너비 | **0.25 m (250 mm)** |
| 상하 높이 | **0.18 m (180 mm)** |

- 차체뿐 아니라 **바퀴, 카메라, LiDAR, 센서 브래킷, 하우징, 안테나 및 모든 돌출 부품**을 포함한 최종 외접상자가 위 세 치수를 **각각** 넘으면 안 됩니다.
- `visual`과 `collision` 형상 모두 차량 규격을 준수해야 합니다. 외형보다 부당하게 작은 충돌체를 사용해서는 안 됩니다.
- 차체 크기만 맞추고 센서를 규격 밖으로 돌출시키는 방식은 허용되지 않습니다.

## 4. `base_link` 위치 규정

`base_link`는 **차량의 3차원 정중앙**에 위치해야 합니다. 평면상 가운데(x, y)만 맞추는 것이 아니라 높이(z) 방향도 중앙이어야 합니다.

차량 전체 충돌체의 3차원 외접상자(AABB)를 기준으로 하면 원점은 다음 위치와 일치해야 합니다.

```text
base_link.x = (x_min + x_max) / 2
base_link.y = (y_min + y_max) / 2
base_link.z = (z_min + z_max) / 2
```

- `base_link`는 실제 URDF/SDF 링크로 존재해야 합니다.
- `base_link`와 차량 충돌체 중심을 임의로 어긋나게 설정해서는 안 됩니다.
- 이는 무게중심을 뜻하는 것이 아니라 **기하학적인 외접상자 중심**입니다.
- 모델 변환(URDF → SDF) 후에도 링크 및 위치가 유지되는지 참가팀이 직접 확인해야 합니다.

## 5. 참가팀 폴더 — 독립 실행·제출 필수

참가팀은 자신의 모든 제작물을 **`src/team/team_01/` 내부**에 넣어야 합니다. `team_01`은 예시이며, 실제 배정받은 `team_XX` 이름을 사용합니다.

```text
urrc_2/
└── src/
    └── team/
        └── team_01/
            ├── race_entry.json
            └── src/
                └── team01_race/
                    ├── package.xml
                    ├── setup.py (Python 패키지인 경우)
                    ├── launch/
                    │   └── race.launch.py
                    ├── models/
                    │   └── car/
                    │       └── model.sdf (또는 URDF)
                    └── ... (제어·인지·센서 설정·필요한 리소스)
```

**필수 제출 조건**

1. 차량 모델, 메시, 카메라·LiDAR 설정, ROS 2 패키지, 주행 알고리즘, Launch 파일, 설정 파일 등 **팀 자체 파일을 모두 `team_01` 안에 포함**해야 합니다.
2. **`team_01` 폴더 하나만 대회 운영 환경의 `src/team/`에 복사해도** 외부 팀별 소스 파일 없이 실행할 수 있어야 합니다. 표준 ROS 2·Gazebo 시스템 패키지 같은 공통 의존성은 별도 설치 대상이며, 필요한 의존성을 명확히 선언해야 합니다.
3. `race_entry.json`의 `entry_id`는 폴더명(`team_01`)과 일치해야 하며, ROS 패키지명·모델 경로·Launch 파일·제어 및 센서 토픽을 실제 파일과 일치하게 작성해야 합니다.
4. **팀마다 ROS 패키지명과 토픽을 고유하게 사용**해야 합니다. 예: `/team_01/scan`, `/team_01/camera/image_raw`, `/team_01/speed`, `/team_01/steering`. 실제 이름은 팀 설정과 일치해야 합니다.
5. 차량 장착 카메라로 **빨간불 5개 전체 소등을 인식한 뒤 출발**하도록 구현해야 합니다. 카메라·LiDAR 토픽의 QoS도 실제 Publisher/Subscriber와 호환되어야 합니다.
6. 팀 제어 노드는 자체 월드 실행이나 **중복 차량 스폰을 하지 않아야 합니다.** 차량 스폰과 출발 그리드 배치는 운영 시스템이 담당합니다.
7. 특정 PC의 절대경로, 다른 팀 폴더, 운영자 전용 파일 또는 외부에 저장된 개인 맵 파일에 의존하지 않아야 합니다.

**참가팀이 직접 검증해야 하는 항목:** `base_link`의 3차원 중심, 외접 규격, ROS 빌드, 독립 Launch 실행, 센서 수신과 QoS, 속도·조향 제어, 신호등 인식, 맵별 정상 주행. 운영자 환경에서는 차량 알고리즘의 정상 완주를 보장하지 않습니다.

## 6. 참가 차량 연습주행

`race_entry.json`을 작성한 후 운영 스크립트의 팀 검사·동기화·빌드를 실행합니다.

```bash
cd ~/urrc_2
bash race_v24.sh check
bash race_v24.sh sync
bash race_v24.sh prepare
```

`team_01` 차량의 단독 연습주행 예시:

```bash
bash race_v24.sh rehearse qualifying --team team_01 --track monza --seconds 300
bash race_v24.sh rehearse qualifying --team team_01 --track hungaroring --seconds 300
bash race_v24.sh rehearse qualifying --team team_01 --track monaco --seconds 300
```

맵 화면 로딩·제어 노드·차량 스폰·카메라 영상 수신을 확인한 뒤 운영자가 `READY`를 입력해야 신호등이 작동합니다. **차량이 없는 상태에서 연습주행 명령을 실행하면 정상 주행할 수 없습니다.**

## 7. 대회 환경과의 호환성

연습용 `urrc_2`는 3개 맵만 제공합니다. 공식 운영용 `urrc_22`에는 6개 맵이 있습니다. 차량·센서·제어·출발 인터페이스는 동일하게 설계되어 있으므로 참가팀은 **맵별로 운영 코드나 차량 제출 폴더를 변경하지 않는 것**을 목표로 개발해야 합니다.

연습용 3개 맵의 정상 주행이 미공개 맵의 완주를 보장하는 것은 아닙니다. 제출 전에 `team_01`의 독립성과 공통 인터페이스 준수 여부를 참가팀이 직접 검증하십시오.
