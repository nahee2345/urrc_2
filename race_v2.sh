#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [[ -f "$ROOT/race/install/setup.bash" ]]; then
  set +u; source "$ROOT/race/install/setup.bash"; set -u
else
  echo '참가자 패키지를 먼저 빌드하십시오: bash competition.sh prepare' >&2
  exit 2
fi
exec python3 "$ROOT/race/scripts/stage_v2.py" "$@" --root "$ROOT"
