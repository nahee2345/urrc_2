#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
case "${1:-}" in
  vehicles) shift; exec python3 "$ROOT/race/scripts/inspection_v4.py" vehicles --root "$ROOT" "$@";;
  cameras) shift; exec python3 "$ROOT/race/scripts/inspection_v4.py" cameras --root "$ROOT" "$@";;
  watch) shift; track="${1:?맵 이름이 필요합니다}"; exec bash "$ROOT/watch_v10.sh" "$track";;
  *) echo '사용법: bash race_v4.sh {vehicles|cameras --track spa [--sample]|watch spa}' >&2; exit 2;;
esac
