# URRC 몬자 연습 환경

참가자용 저장소입니다. **몬자 맵만 포함**하며 신호등이 있는 월드를 기본 실행합니다.
대회 운영용 6개 맵은 [urrc_f1_ws](https://github.com/nahee2345/urrc_f1_ws)에 있습니다.

## 설치 및 다운로드

Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic이 설치된 PC에서 실행합니다.

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz python3-colcon-common-extensions python3-rosdep cmake
git clone https://github.com/nahee2345/urrc_2.git
cd urrc_2
python3 verify_release.py
```

GitHub의 Download ZIP으로 받은 경우 압축 해제된 폴더에서 실행하시면 됩니다. 폴더명이나 사용자명이 달라도 됩니다. `build/`, `install/`, `log/`를 다른 PC에서 복사하지 않습니다.

## 신호등 포함 몬자 실행

```bash
bash run_monza.sh --view grid
```

맵을 실행하면 Gazebo 준비 후 신호등이 **자동으로 5개 순차 점등 → 전체 일괄 소등**됩니다. 수동 재실행은 자동 시퀀스가 종료된 뒤 다른 터미널에서 진행합니다.

```bash
bash test_lights.sh sequence  # 5개 순차 점등 → 단일 명령으로 전체 소등
bash test_lights.sh on        # 5개 LED를 계속 켜 놓기
bash test_lights.sh off       # 모두 소등
```

맵 자동 점등을 끄고 수동 테스트만 하려면 `bash run_monza.sh --no-auto-start`를 사용하세요. 5개 임시 발광 렌즈가 원래 렌즈 위에 생성되며, 일괄 소등할 때 임시 모델 하나를 제거합니다. 차량 출발 허가와는 별개로 수동 신호등 테스트할 수 있습니다.
두 터미널에서 `GZ_PARTITION`을 별도로 지정했다면 같은 값이어야 합니다. 지정하지 않으면 실행기와 테스트가 같은 사용자별 값을 사용합니다.

전체 맵 보기: `bash run_monza.sh`. 신호등 없는 월드가 필요하면 `bash run_monza.sh --no-lights`입니다. 차량은 맵 실행만으로 자동 생성되지 않습니다.

## 차량 크기와 1번 스폰

- 최대 크기: **0.55 × 0.25 × 0.18 m (길이 × 폭 × 높이)**, 모든 돌출부 포함.
- 1번 스폰: **x=0.37867, y=-2.05604, z=1.24353 m, yaw=1.45738 rad**.
- 기본 z는 노면 1.20353 m + `spawn_z_offset_m=0.04 m`입니다.
- 기준 데이터: `src/urrc_track_gazebo/tracks/processed/monza.json`의 첫 grid.
- 차량은 +X 전방/+Y 왼쪽/+Z 위쪽, LiDAR 전용 주행입니다.

## 제출물로 연습 예선 확인

팀 폴더 또는 ZIP을 `race/inbox/`에 넣고 실행합니다. 앞서 실행한 연습 Gazebo는 먼저 종료합니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12
bash competition.sh prepare
bash competition.sh new-event --track monza
bash competition.sh qualifying
```

실행기가 차량을 1번 그리드에 스폰하고 팀 launch를 실행한 뒤 신호등·출발 신호·2랩 계측을 진행합니다. 참가 launch가 LiDAR bridge와 제어 어댑터를 포함해야 합니다. 팀 패키지별 추가 의존성은 운영자와 사전에 설치합니다.

## 무엇을 제출해야 하나요?

**차량 URDF/SDF + 모든 메시·센서/제어 설정 + LiDAR 주행 알고리즘 + ROS 패키지 + race_entry.json**을 제출합니다.

[제출 구조·토픽 타입·경로 규칙·주의사항·경기 규정](race/README_KO.md)을 반드시 따라 주세요.
[설정 예제](race/entry_template.json)를 대표 패키지에도 넣으면 src 또는 패키지만 옮겨 접수할 수 있습니다.
[pack_entry.py](race/scripts/pack_entry.py)로 제출 ZIP을 만들 수 있습니다.

## 새로 다운로드한 파일 확인

```bash
python3 verify_release.py
bash run_monza.sh --view grid
# 별도 터미널
bash test_lights.sh sequence
```

`verify_release.py`는 파일 체크섬, 몬자 전용 구성, 스폰·LED·리소스 참조와 PC 절대 경로를 검사합니다. 실제 Gazebo 렌더링과 참가 차량 주행은 실행 PC에서 확인해야 합니다.

### 출발 신호 v10.8 동기화 (urrc_f1_ws와 동일 제어 코드)

신호등 제어 프로그램 `src/urrc_track_gazebo/scripts/traffic_lights_v10.py`는 `urrc_f1_ws`의 동일 파일을 그대로 사용합니다. 1번부터 5번까지 **한 개씩 임의 간격 점등**한 뒤 **5개 발광 렌즈가 속한 단일 모델**을 제거하여 한꺼번에 소등합니다. 자동 신호등은 월드 및 신호 제어 서비스 준비 후 시작합니다. 기본 연습 차량 2대는 `bash run_practice.sh --cars 2 --lights`로 실행하고, 차량 없는 월드는 `bash run_monza.sh --view grid`로 실행합니다. `urrc_2`에는 v10.8 관전 카메라 센서가 없으므로 카메라 준비 검사만 생략합니다.

정상 소스 확인: `git rev-parse HEAD`, `python3 -m unittest discover -s race -p 'test_signal_v108.py' -v`, `python3 verify_release.py`. 이전 설치 폴더를 별도로 종료/삭제하고 새 GitHub clone에서 실행해야 구형 신호 제어 코드가 섞이지 않습니다. 실제 Gazebo에서의 동시 화면 소등과 3~6초 준수는 렌더러 및 명령 응답 지연에 영향을 받으므로 로그를 확인하세요.
