#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if (($# == 0)); then set -- sequence; fi
exec python3 "$ROOT/race/scripts/lights.py" "$@"
