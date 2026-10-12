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
# uncommitted Mac changes deploy too.
# 1) The shared arm module: firmware (compile + flash), Python API, calibration, clips, venv.
STEPPER_LINK="$LINK" bash "$ROOT/../../arm/scripts/deploy.sh"
# 2) This app's manual tools (emote.py, armctl.py, ...). They run with arm/.venv/bin/python.
REPO=/home/arduino/IEEE-Minimalist-AI-Challenge-2026
BOARD=$REPO/apps/stepper_desk
"${SSH[@]}" "mkdir -p $BOARD/{scripts,recordings}"
scp -q -P "$PORT" -o BatchMode=yes "$ROOT"/scripts/*.sh "$ROOT"/scripts/*.py "$HOST":$BOARD/scripts/
echo "Manual tools: ssh $HOST; cd $BOARD && ../../arm/.venv/bin/python scripts/emote.py lamp_happy/arm --execute"
