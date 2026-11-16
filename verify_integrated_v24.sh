#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Supplied Gazebo Harmonic/ROS 2 Jazzy installation is user-local, not bundled.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then source /opt/ros/jazzy/setup.bash; fi
if [[ -f "$ROOT/race/install/setup.bash" ]]; then source "$ROOT/race/install/setup.bash"; fi
exec python3 "$ROOT/race/scripts/integration_v24.py" --root "$ROOT" "$@"
