#!/usr/bin/env bash
# v16: offline/live vehicle inspection and explicit first qualifying entrant.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  vehicles)
    shift
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
      set +u; source /opt/ros/jazzy/setup.bash; set -u
    fi
    if [[ -f "$ROOT/race/install/setup.bash" ]]; then
      set +u; source "$ROOT/race/install/setup.bash"; set -u
    fi
    export PYTHONDONTWRITEBYTECODE=1
    exec python3 "$ROOT/race/scripts/vehicle_preflight_v16.py" --root "$ROOT" "$@"
    ;;
  qualifying)
    shift
    FIRST=''
    if [[ "${1:-}" == --first-team ]]; then
      if (( $# < 2 )); then echo '[v16] --first-team 뒤에 team_01 형식의 팀 ID가 필요합니다.' >&2; exit 2; fi
      FIRST="$2"; shift 2
    elif [[ "${1:-}" =~ ^team_ ]]; then
      FIRST="$1"; shift
    fi
    if [[ -n "$FIRST" ]]; then
      if [[ ! "$FIRST" =~ ^team_(0[1-9]|1[0-2])$ ]]; then
        echo "[v16] 잘못된 참가팀: $FIRST (team_01~team_12)" >&2; exit 2
      fi
      export URRC_QUALIFYING_FIRST="$FIRST"
    else
      unset URRC_QUALIFYING_FIRST
    fi
    exec bash "$ROOT/race_v13.sh" qualifying "$@"
    ;;
  check|sync|prepare|semi_a|semi_b|final|refresh-maps|show-maps|follow|import-downloads|signal-check)
    exec bash "$ROOT/race_v13.sh" "$@"
    ;;
  *)
    echo '사용법: bash race_v16.sh {vehicles [--team team_01] [--live --track monza --seconds 20]|qualifying [team_01|--first-team team_01] [--track monza]|semi_a|semi_b|final|check|sync|prepare}' >&2
    exit 2;;
esac
