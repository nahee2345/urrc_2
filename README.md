# URRC_2 연습맵

## 실행 방법

```bash
git clone https://github.com/nahee2345/urrc_2.git ~/urrc_2
cd ~/urrc_2

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

./run_gazebo.sh
