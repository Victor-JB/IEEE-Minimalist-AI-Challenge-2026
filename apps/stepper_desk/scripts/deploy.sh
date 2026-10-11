#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ADB="$(command -v adb || true)"
if [[ -z "$ADB" ]]; then
  for candidate in /opt/homebrew/bin/adb /usr/local/bin/adb; do
    if [[ -x "$candidate" ]]; then ADB="$candidate"; break; fi
  done
fi
# STEPPER_LINK=auto (default): USB via ADB when plugged in, else Tailscale (STEPPER_HOST, default daboard).
LINK="${STEPPER_LINK:-auto}"
if [[ "$LINK" != tailscale && -n "$ADB" ]] && "$ADB" -s 2709356903 forward tcp:2222 tcp:22 >/dev/null 2>&1; then
  PORT=2222; HOST=arduino@localhost; echo 'Link: USB'
elif [[ "$LINK" == usb ]]; then
  echo 'Arduino Q not found over USB (install android-platform-tools or check the cable).' >&2; exit 1
else
  PORT=22; HOST="arduino@${STEPPER_HOST:-daboard}"; echo "Link: Tailscale ($HOST)"
fi
SSH=(ssh -p "$PORT" -o BatchMode=yes -o ConnectTimeout=5 "$HOST")
"${SSH[@]}" 'mkdir -p /home/arduino/stepper-desk-q/{sketch,python,scripts}'
scp -P "$PORT" -o BatchMode=yes "$ROOT"/sketch/* "$HOST":/home/arduino/stepper-desk-q/sketch/
scp -P "$PORT" -o BatchMode=yes "$ROOT"/python/*.py "$ROOT"/python/requirements.txt "$HOST":/home/arduino/stepper-desk-q/python/
scp -P "$PORT" -o BatchMode=yes "$ROOT/scripts/flash-local.sh" "$HOST":/home/arduino/stepper-desk-q/scripts/
"${SSH[@]}" 'cd /home/arduino/stepper-desk-q && python3 -m venv .venv && .venv/bin/pip install -r python/requirements.txt && arduino-cli compile --profile default --build-path /home/arduino/stepper-desk-q/build sketch && bash scripts/flash-local.sh'
