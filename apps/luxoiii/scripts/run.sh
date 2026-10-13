#!/bin/bash
# Start the main web app at http://127.0.0.1:8082/ on the Mac or board.
# The board observer supplies AI/tracking; this launcher never starts the legacy arm controller.
set -euo pipefail
cd "$(dirname "$0")/.."
LUXOIII_RUN_PYTHON="${LUXOIII_PYTHON:-.venv/bin/python}"
if [[ ! -x "$LUXOIII_RUN_PYTHON" ]]; then
  LUXOIII_RUN_PYTHON="$(command -v python3)"
fi
exec "$LUXOIII_RUN_PYTHON" -m luxoiii "$@"
