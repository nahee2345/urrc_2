#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  vehicles)
    shift;exec python3 "$ROOT/race/scripts/base_link_check_v6.py" --root "$ROOT" "$@";;
  monitor)
    shift;exec python3 "$ROOT/race/scripts/race_monitor_v6.py" "${1:?예선 qualifying / semi_a / semi_b / final}" --root "$ROOT" "${@:2}";;
  qualifying|semi_a|semi_b|final)
    exec bash "$ROOT/race_v3.sh" "$@";;
  *)
    echo 'Usage: bash race_v6.sh vehicles [--report FILE] | monitor {qualifying|semi_a|semi_b|final} [--once] | {qualifying|semi_a|semi_b|final}' >&2;exit 2;;
esac
