#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  qualifying|semi_a|semi_b|final|refresh-maps|show-maps)
    exec bash "$ROOT/race_v7.sh" "$@";;
  signal-check)
    shift
    exec python3 "$ROOT/race/scripts/signal_check_v8.py" --root "$ROOT" "$@";;
  prepare|sync|check|vehicles)
    exec bash "$ROOT/race_v7.sh" "$@";;
  *)
    echo 'Usage: bash race_v8.sh {refresh-maps|show-maps|qualifying|semi_a|semi_b|final|sync|check|prepare|vehicles}' >&2
    exit 2;;
esac
