#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  follow) shift; exec bash "$ROOT/race_v5.sh" follow "$@";;
  *) exec bash "$ROOT/race_v8.sh" "$@";;
esac
