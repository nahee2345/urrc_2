#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GZ_PARTITION="urrc_pace_$(id -u)"
exec bash "$ROOT/watch_v10.sh" "$@"
