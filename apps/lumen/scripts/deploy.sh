#!/bin/bash
# From the Mac: copy Lumen + the shared code it uses to the board, install its venv,
# install/restart the boot service. Does NOT flash the MCU: the arm firmware comes from
# apps/stepper_desk/scripts/deploy.sh (run that once, and again only when firmware changes).
#   bash apps/lumen/scripts/deploy.sh            # USB if plugged in, else Tailscale (daboard)
#   LUMEN_NO_START=1 bash apps/lumen/scripts/deploy.sh   # install but don't (re)start
set -euo pipefail
REPO_LOCAL="$(cd "$(dirname "$0")/../../.." && pwd)"
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
# Working tree -> board clone (uncommitted changes deploy too, like stepper_desk's deploy).
( cd "$REPO_LOCAL" && COPYFILE_DISABLE=1 tar czf - \
    --exclude='.venv' --exclude='__pycache__' --exclude='apps/lumen/logs' \
    --exclude='motion_lib/review' --exclude='motion_lib/gamma_comparison' \
    apps/lumen inference perf motion_lib \
    apps/stepper_desk/python apps/stepper_desk/scripts/emote.py \
    apps/stepper_desk/calibration apps/stepper_desk/clips ) \
  | "${SSH[@]}" "tar xzf - -C $REPO"

"${SSH[@]}" "set -e; cd $REPO/apps/lumen
  python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt
  mkdir -p ~/.config/systemd/user logs && cp lumen.service ~/.config/systemd/user/
  systemctl --user daemon-reload && systemctl --user enable lumen >/dev/null
  if [ -z '${LUMEN_NO_START:-}' ]; then systemctl --user restart lumen; fi
  sleep 2; systemctl --user --no-pager status lumen | head -5 || true"
echo "HUD: http://${STEPPER_HOST:-daboard}:8080/   logs: ssh $HOST journalctl --user -u lumen -f"
