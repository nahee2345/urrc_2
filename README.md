# URRC_2 연습맵

## 1. GitHub에서 워크스페이스 다운로드

터미널을 열고 아래 명령어를 실행합니다.

```bash
cd ~
git clone https://github.com/nahee2345/urrc_2.git
cd ~/urrc_2
```

이미 한 번 다운로드한 경우에는 다시 `git clone`하지 않고 아래처럼 최신 내용을 받아오면 됩니다.

```bash
cd ~/urrc_2
git pull origin main
```

## 2. ROS 2 Jazzy 환경 불러오기

```bash
source /opt/ros/jazzy/setup.bash
```

## 3. 워크스페이스 빌드

처음 다운로드한 뒤에는 반드시 한 번 빌드합니다.

```bash
cd ~/urrc_2
colcon build --symlink-install
```

빌드가 정상적으로 끝나면 워크스페이스 환경을 불러옵니다.

```bash
source ~/urrc_2/install/setup.bash
```

## 4. 연습맵 실행

```bash
cd ~/urrc_2
./run_gazebo.sh
```

`run_gazebo.sh`를 실행하면 연습용 Monza 서킷이 Gazebo에서 실행됩니다.

스크립트 내부에서도 필요한 ROS 2 환경을 불러오고 `urrc_track_gazebo` 패키지를 빌드한 뒤 Gazebo를 실행하도록 구성되어 있습니다.

## 5. 다음 실행부터

최초 빌드가 끝난 뒤에는 일반적으로 아래 명령어만 실행하면 됩니다.

```bash
cd ~/urrc_2
./run_gazebo.sh
```

## 6. 코드가 업데이트된 경우

GitHub의 최신 내용을 다시 받아오려면 아래 순서로 실행합니다.

```bash
cd ~/urrc_2
git pull origin main
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/urrc_2/install/setup.bash
```

그다음 연습맵을 실행합니다.

```bash
cd ~/urrc_2
./run_gazebo.sh
```

## 7. 차량 제작 기준

차량은 1/10 스케일을 기준으로 제작합니다.

최대 차량 크기는 다음을 넘지 않도록 합니다.

```text
길이: 0.55 m
폭:   0.25 m
높이: 0.18 m
```

즉, 최대 크기는 `0.55 m × 0.25 m × 0.18 m (길이 × 폭 × 높이)`입니다.

## 8. 연습경기 차량 스폰 위치

연습맵인 Monza를 실행한 뒤 제작한 차량은 아래 위치와 방향에 스폰합니다.

```text
x   = 39.59260
y   = -27.38488
z   = 0.120
yaw = 1.5707963 rad
```

`yaw = 1.5707963 rad`는 약 `90°`입니다.

차량의 기준 프레임 또는 모델 원점이 차체 중심에 있다고 가정한 스폰 기준입니다.
