#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 ]]; then echo "Usage: bash start_v10_lights.sh monaco" >&2; exit 2; fi
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
exec python3 "$root/src/urrc_track_gazebo/scripts/traffic_lights_v10.py" --track "$1"
