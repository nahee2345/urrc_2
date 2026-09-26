#!/usr/bin/env bash
set -euo pipefail
URRC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$URRC_ROOT"
if command -v colcon >/dev/null 2>&1 && [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  colcon build --packages-select urrc_track_gazebo --symlink-install
else
  if ! command -v cmake >/dev/null 2>&1; then
    printf '%s\n' 'CMake is not installed. Install cmake, or run run_monza.sh without building.' >&2
    exit 2
  fi
  cmake -S src/urrc_track_gazebo -B build_standalone/urrc_track_gazebo -DCMAKE_INSTALL_PREFIX="$URRC_ROOT/install_standalone"
  cmake --build build_standalone/urrc_track_gazebo
  cmake --install build_standalone/urrc_track_gazebo
fi
