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
# The board runs from its clone of this repo; deploy copies the working tree over it, so
# uncommitted Mac changes deploy too (commit and git pull there to keep the clone clean).
REPO=/home/arduino/IEEE-Minimalist-AI-Challenge-2026
BOARD=$REPO/apps/stepper_desk
"${SSH[@]}" "test -d $REPO/.git || git clone -q https://github.com/Victor-JB/IEEE-Minimalist-AI-Challenge-2026 $REPO"
"${SSH[@]}" "mkdir -p $BOARD/{sketch,python,scripts,calibration,clips,recordings} $REPO/motion_lib/clips"
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/sketch/* "$HOST":$BOARD/sketch/
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/python/*.py "$ROOT"/python/requirements.txt "$HOST":$BOARD/python/
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/scripts/*.sh "$ROOT"/scripts/*.py "$HOST":$BOARD/scripts/
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/calibration/*.json "$HOST":$BOARD/calibration/
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/clips/*.json "$HOST":$BOARD/clips/
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/../../motion_lib/player.py "$HOST":$REPO/motion_lib/
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/../../motion_lib/clips/*.json "$HOST":$REPO/motion_lib/clips/
"${SSH[@]}" "cd $BOARD && python3 -m venv .venv && .venv/bin/pip install -q -r python/requirements.txt && arduino-cli compile --profile default --build-path $BOARD/build sketch && bash scripts/flash-local.sh"
