#!/usr/bin/env bash
set -eo pipefail
set +u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"
mkdir -p "$WS_DIR/log/ros" "$WS_DIR/log/gz"
export ROS_LOG_DIR="$WS_DIR/log/ros"
export GZ_LOG_PATH="$WS_DIR/log/gz"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "[ERROR] ROS 2 Jazzy not found: /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "[ERROR] ros_gz_sim not found. Install ros-jazzy-ros-gz first." >&2
  exit 1
fi

echo "[urrc_2] Incremental build..."
colcon build --symlink-install --packages-select urrc_track_gazebo

set +u
source "$WS_DIR/install/setup.bash"
exec ros2 launch urrc_track_gazebo competition_random.launch.py
