#!/usr/bin/env bash
# v17: operator-only map previews and isolated running rehearsals.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  preview|rehearse)
    MODE="$1"; shift
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
      set +u; source /opt/ros/jazzy/setup.bash; set -u
    fi
    if [[ -f "$ROOT/race/install/setup.bash" ]]; then
      set +u; source "$ROOT/race/install/setup.bash"; set -u
    fi
    export PYTHONDONTWRITEBYTECODE=1
    exec python3 "$ROOT/race/scripts/stage_preview_v17.py" "$MODE" --root "$ROOT" "$@"
    ;;
  *) exec bash "$ROOT/race_v16.sh" "$@" ;;
esac
