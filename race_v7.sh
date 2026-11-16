#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  sync|check)
    cmd="$1";shift
    exec python3 "$ROOT/race/scripts/team_sync_v7.py" "$cmd" --root "$ROOT" "$@";;
  prepare)
    shift
    python3 "$ROOT/race/scripts/team_sync_v7.py" sync --root "$ROOT"
    exec bash "$ROOT/competition.sh" prepare "$@";;
  qualifying|semi_a|semi_b|final)
    exec bash "$ROOT/race_v3.sh" "$@";;
  refresh-maps|show-maps)
    exec bash "$ROOT/race_v3.sh" "$@";;
  vehicles)
    exec bash "$ROOT/race_v6.sh" "$@";;
  *)
    echo 'Usage: bash race_v7.sh {sync|check|prepare|qualifying|semi_a|semi_b|final|refresh-maps|show-maps|vehicles}' >&2
    exit 2;;
esac
