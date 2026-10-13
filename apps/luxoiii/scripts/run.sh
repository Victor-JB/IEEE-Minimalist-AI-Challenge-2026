#!/bin/bash
# ON THE BOARD, for development: stop the boot service and run luxoiii in the foreground.
#   bash apps/luxoiii/scripts/run.sh                 # real arm, HUD on :8080
#   bash apps/luxoiii/scripts/run.sh --fake-arm      # camera + AI only, arm untouched
#   PERF=1 bash apps/luxoiii/scripts/run.sh          # timing report on Ctrl+C
set -euo pipefail
cd "$(dirname "$0")/.."
systemctl --user stop luxoiii luxoiii-desk lumen lumen-desk 2>/dev/null || true
PYTHON="${LUXOIII_PYTHON:-.venv/bin/python}"
[[ -x "$PYTHON" ]] || PYTHON=../../.venv/bin/python
exec "$PYTHON" -m luxoiii --headless --stream 8080 "$@"
