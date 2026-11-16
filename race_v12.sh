#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  import-downloads)
    shift
    exec python3 "$ROOT/race/scripts/import_downloads_v12.py" --root "$ROOT" "$@";;
  *) exec bash "$ROOT/race_v11.sh" "$@";;
esac
