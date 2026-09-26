#!/usr/bin/env bash
set -euo pipefail
URRC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -m venv "$URRC_ROOT/.venv"
"$URRC_ROOT/.venv/bin/python" -m pip install -r "$URRC_ROOT/requirements-generation.txt"
printf '%s\n' 'Generator environment ready. This is only needed when regenerating or validating meshes.'
