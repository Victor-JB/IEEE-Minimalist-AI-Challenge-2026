#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ADB="$(command -v adb || true)"
if [[ -z "$ADB" ]]; then
  for candidate in /opt/homebrew/bin/adb /usr/local/bin/adb; do
    if [[ -x "$candidate" ]]; then ADB="$candidate"; break; fi
  done
fi
[[ -n "$ADB" ]] || { echo 'Install android-platform-tools with Homebrew.' >&2; exit 1; }
"$ADB" -s 2709356903 forward tcp:2222 tcp:22
SSH=(ssh -p 2222 -o BatchMode=yes -o ConnectTimeout=5 arduino@localhost)
"${SSH[@]}" 'mkdir -p /home/arduino/stepper-desk-q/{sketch,python,scripts}'
scp -P 2222 -o BatchMode=yes "$ROOT"/sketch/* arduino@localhost:/home/arduino/stepper-desk-q/sketch/
scp -P 2222 -o BatchMode=yes "$ROOT"/python/*.py "$ROOT"/python/requirements.txt arduino@localhost:/home/arduino/stepper-desk-q/python/
scp -P 2222 -o BatchMode=yes "$ROOT/scripts/flash-local.sh" arduino@localhost:/home/arduino/stepper-desk-q/scripts/
"${SSH[@]}" 'cd /home/arduino/stepper-desk-q && python3 -m venv .venv && .venv/bin/pip install -r python/requirements.txt && arduino-cli compile --profile default --build-path /home/arduino/stepper-desk-q/build sketch && bash scripts/flash-local.sh'
