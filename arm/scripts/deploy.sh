#!/bin/bash
# From the Mac: copy the arm module to the board, install its venv, compile and flash the
# STM32 firmware. Run once, then again only when arm/sketch or arm/python changes.
#   bash arm/scripts/deploy.sh                 # USB if plugged in, else Tailscale (daboard)
#   ARM_NO_FLASH=1 bash arm/scripts/deploy.sh  # copy + venv only (no firmware flash)
set -euo pipefail
REPO_LOCAL="$(cd "$(dirname "$0")/../.." && pwd)"
ADB="$(command -v adb || true)"
[[ -z "$ADB" && -x /opt/homebrew/bin/adb ]] && ADB=/opt/homebrew/bin/adb
LINK="${STEPPER_LINK:-auto}"
if [[ "$LINK" != tailscale && -n "$ADB" ]] && "$ADB" -s 2709356903 forward tcp:2222 tcp:22 >/dev/null 2>&1; then
  PORT=2222; HOST=arduino@localhost; echo 'Link: USB'
elif [[ "$LINK" == usb ]]; then
  echo 'Arduino Q not found over USB.' >&2; exit 1
else
  PORT=22; HOST="arduino@${STEPPER_HOST:-daboard}"; echo "Link: Tailscale ($HOST)"
fi
SSH=(ssh -p "$PORT" -o BatchMode=yes -o ConnectTimeout=5 "$HOST")
REPO=/home/arduino/IEEE-Minimalist-AI-Challenge-2026

"${SSH[@]}" "test -d $REPO/.git || git clone -q https://github.com/Victor-JB/IEEE-Minimalist-AI-Challenge-2026 $REPO"
# Working tree -> board clone (uncommitted changes deploy too).
( cd "$REPO_LOCAL" && COPYFILE_DISABLE=1 tar czf - \
    --exclude='.venv' --exclude='__pycache__' --exclude='arm/build*' --exclude='arm/recordings' \
    arm ) | "${SSH[@]}" "tar xzf - -C $REPO"
"${SSH[@]}" "set -e; cd $REPO/arm && python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt"
if [[ -z "${ARM_NO_FLASH:-}" ]]; then
  "${SSH[@]}" "set -e; cd $REPO/arm && arduino-cli compile --profile default --build-path $REPO/arm/build sketch && bash scripts/flash-local.sh"
fi
