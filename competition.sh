#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_setup() {
  local setup_file="$1"
  # ROS setup scripts read optional variables that nounset treats as errors.
  set +u
  source "$setup_file"
  local source_status=$?
  set -u
  return "$source_status"
}
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_setup /opt/ros/jazzy/setup.bash
else
  echo "Ubuntu에 설치된 ROS 2 환경을 먼저 source하세요." >&2; exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
case "${1:-}" in
  new-event)
    exec python3 "$ROOT/race/scripts/race_manager.py" new-event --root "$ROOT"
    ;;
  status)
    exec python3 "$ROOT/race/scripts/race_manager.py" status --root "$ROOT"
    ;;
  prepare|qualifying|final)
    python3 "$ROOT/race/scripts/race_manager.py" intake --root "$ROOT"
    mapfile -t BASE_PATHS < <(find "$ROOT/race/entrants" -mindepth 2 -maxdepth 2 -type d -name src -print | sort)
    if ((${#BASE_PATHS[@]} == 0)); then
      if [[ "$1" == prepare ]]; then exec python3 "$ROOT/race/scripts/race_manager.py" validate --root "$ROOT"; fi
      exec python3 "$ROOT/race/scripts/race_manager.py" "$1" --root "$ROOT"
    fi
    colcon build --base-paths "${BASE_PATHS[@]}" --build-base "$ROOT/race/build" --install-base "$ROOT/race/install" --log-base "$ROOT/race/log" --merge-install
    source_setup "$ROOT/race/install/setup.bash"
    if [[ "$1" == prepare ]]; then exec python3 "$ROOT/race/scripts/race_manager.py" validate --root "$ROOT"; fi
    exec python3 "$ROOT/race/scripts/race_manager.py" "$1" --root "$ROOT"
    ;;
  *)
    echo "사용법: bash competition.sh {prepare|new-event|qualifying|final|status}" >&2; exit 2
    ;;
esac
