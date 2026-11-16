#!/usr/bin/env bash
# Operator compatibility guard. Only delegates approved official commands.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  compatibility)
    shift
    exec python3 "$ROOT/race/scripts/compatibility_v23.py" --root "$ROOT" "$@"
    ;;
  qualifying|semi_a|semi_b|final)
    STAGE="$1"
    echo "[v23] 공식 $STAGE 실행 전 6개 맵·팀별 토픽 정적 호환성 점검" 
    python3 "$ROOT/race/scripts/compatibility_v23.py" --root "$ROOT" --require-synced --tracks all
    exec bash "$ROOT/race_v22.sh" "$@"
    ;;
  *)
    exec bash "$ROOT/race_v22.sh" "$@"
    ;;
esac
