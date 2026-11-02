#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$ROOT/src/urrc_track_gazebo/scripts/run_world.py" "$@"
