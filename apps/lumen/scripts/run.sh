#!/bin/bash
# ON THE BOARD, for development: stop the boot service and run Lumen in the foreground.
#   bash apps/lumen/scripts/run.sh                 # real arm, HUD on :8080
#   bash apps/lumen/scripts/run.sh --fake-arm      # camera + AI only, arm untouched
#   PERF=1 bash apps/lumen/scripts/run.sh          # timing report on Ctrl+C
set -euo pipefail
cd "$(dirname "$0")/.."
systemctl --user stop lumen 2>/dev/null || true
exec /home/arduino/IEEE-Minimalist-AI-Challenge-2026/.venv/bin/python -m lumen --headless --stream 8080 "$@"
