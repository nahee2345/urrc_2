#!/usr/bin/env bash
# v18: automatic compact TXT results; never alter official scoring.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPORTER="$ROOT/race/scripts/summary_txt_v18.py"
case "${1:-}" in
  results|summary)
    shift
    exec python3 "$REPORTER" --root "$ROOT" "$@"
    ;;
  vehicles|qualifying|semi_a|semi_b|final|rehearse)
    verb="$1"; shift
    finish_report() {
      local code=$?
      trap - EXIT
      python3 "$REPORTER" --root "$ROOT" || echo '[v18] TXT 보고서 갱신 실패. 기존 공식 기록은 보존됩니다.' >&2
      exit "$code"
    }
    trap finish_report EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    if [[ "$verb" == vehicles ]]; then
      has_report=0
      team_id=all
      arg_prev=''
      for arg in "$@"; do
        if [[ "$arg_prev" == --team ]]; then team_id="$arg"; fi
        if [[ "$arg" == --report ]]; then has_report=1; fi
        arg_prev="$arg"
      done
      if (( ! has_report )); then
        # Keep raw source JSON outside the single user-facing TXT folder.
        dir="$ROOT/race/results/inspections"
        mkdir -p "$dir"
        bash "$ROOT/race_v17.sh" vehicles "$@" --report "$dir/current_${team_id}.json"
      else
        bash "$ROOT/race_v17.sh" vehicles "$@"
      fi
    else
      bash "$ROOT/race_v17.sh" "$verb" "$@"
    fi
    ;;
  *) exec bash "$ROOT/race_v17.sh" "$@" ;;
esac
