#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -ne 1 ]]; then echo "Usage: bash watch_v10.sh monaco" >&2; exit 2; fi
case "$1" in monza|spa|monaco|cota|hungaroring|baku) ;; *) echo "Unknown track: $1" >&2; exit 2;; esac
if ! command -v ros2 >/dev/null 2>&1; then
  for setup in /opt/ros/*/setup.bash; do
    if [[ -f "$setup" ]]; then source "$setup"; fi
    if command -v ros2 >/dev/null 2>&1; then break; fi
  done
fi
if ! command -v ros2 >/dev/null 2>&1; then echo "ROS2 not found" >&2; exit 3; fi
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
exec python3 "$root/src/urrc_track_gazebo/scripts/spectator_switch_v10.py" "$1"
