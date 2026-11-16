#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Both practice map and Gazebo GUI client must share a Gazebo partition.
export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
case "${1:-}" in
 follow) echo "[v22] 관전·차량추적 카메라가 제거되었습니다. 차량 장착 카메라만 사용합니다." >&2; exit 2;;
 map) shift; exec bash "$ROOT/race_v3.sh" "$@";;
 *) echo '사용법: bash race_v5.sh follow --order race/grids/qualifying.txt [--view rear|front]' >&2; exit 2;;
esac
