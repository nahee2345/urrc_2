#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [[ -f "$ROOT/install/setup.bash" ]]; then
  set +u
  source "$ROOT/install/setup.bash"
  set -u
fi
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$ROOT/race/scripts/environment_diagnostic_v15.py" "$@"
