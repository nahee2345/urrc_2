#!/usr/bin/env bash
set -euo pipefail
URRC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$URRC_ROOT/src/urrc_track_gazebo/scripts/run_world.py" monza "$@"
