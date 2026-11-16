#!/usr/bin/env bash
set -euo pipefail
URRC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
URRC_PYTHON=python3
if [[ -x "$URRC_ROOT/.venv/bin/python" ]]; then URRC_PYTHON="$URRC_ROOT/.venv/bin/python"; fi
"$URRC_PYTHON" "$URRC_ROOT/src/urrc_track_gazebo/scripts/validate_tracks.py" --output "$URRC_ROOT/validation_results/geometry.json"
"$URRC_PYTHON" -m pytest "$URRC_ROOT/src/urrc_track_gazebo/test" -q
