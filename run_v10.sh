#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 ]]; then
  echo "Usage: bash run_v10.sh monaco|monza|spa|cota|hungaroring|baku" >&2
  exit 2
fi
if [[ $# -eq 1 ]]; then set -- "$1" --view grid; fi
exec python3 "$root/src/urrc_track_gazebo/scripts/run_world.py" "$@"
