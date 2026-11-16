#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$root"
python3 -m compileall -q src/urrc_track_gazebo/urrc_tracks/v10_assets.py src/urrc_track_gazebo/scripts/traffic_lights_v10.py src/urrc_track_gazebo/scripts/spectator_switch_v10.py
python3 src/urrc_track_gazebo/scripts/traffic_lights_v10.py --track monaco --dry-run
python3 src/urrc_track_gazebo/scripts/check_v10.py --package src/urrc_track_gazebo --source-only
git diff --check
echo "[PASS] Static v10.8 checks; Gazebo runtime not verified here."
