#!/bin/bash
# From the Mac: copy Lumen (apps/lumen) and the shared arm module (arm/) to the board, install its venv,
# install/restart the boot service. Does NOT flash the MCU: the firmware comes from
# arm/scripts/deploy.sh (run that once, and again only when arm/sketch changes).
#   bash apps/lumen/scripts/deploy.sh            # USB if plugged in, else Tailscale (luxo3); on the board itself it runs locally
#   LUMEN_NO_START=1 bash apps/lumen/scripts/deploy.sh   # install but don't (re)start
set -euo pipefail
REPO_LOCAL="$(cd "$(dirname "$0")/../../.." && pwd)"
ADB="$(command -v adb || true)"
[[ -z "$ADB" && -x /opt/homebrew/bin/adb ]] && ADB=/opt/homebrew/bin/adb
LINK="${STEPPER_LINK:-auto}"
REPO=/home/arduino/IEEE-Minimalist-AI-Challenge-2026
if [[ "$LINK" == local || ( "$LINK" == auto && -S /var/run/arduino-router.sock ) ]]; then
  # Running on the Arduino Q itself: no SSH hop (its Tailscale is userspace-only, so it can't reach its own 100.x IP).
  LOCAL=1; echo 'Link: local'
elif [[ "$LINK" != tailscale && -n "$ADB" ]] && "$ADB" -s 2709356903 forward tcp:2222 tcp:22 >/dev/null 2>&1; then
  PORT=2222; HOST=arduino@localhost; echo 'Link: USB'
elif [[ "$LINK" == usb ]]; then
  echo 'Arduino Q not found over USB.' >&2; exit 1
else
  PORT=22; HOST="arduino@${STEPPER_HOST:-100.78.199.105}"; echo "Link: Tailscale ($HOST)"
fi
if [[ -n "${LOCAL:-}" ]]; then SSH=(bash -c); HOST=localhost
else SSH=(ssh -p "$PORT" -o BatchMode=yes -o ConnectTimeout=5 "$HOST"); fi

[[ -n "${LOCAL:-}" ]] || "${SSH[@]}" "test -d $REPO/.git || git clone -q https://github.com/Victor-JB/IEEE-Minimalist-AI-Challenge-2026 $REPO"
# Working tree -> board clone (uncommitted changes deploy too; skipped when deploying from that clone): the app + the shared arm module.
[[ "$REPO_LOCAL" -ef "$REPO" ]] || ( cd "$REPO_LOCAL" && COPYFILE_DISABLE=1 tar czf - \
    --exclude='.venv' --exclude='__pycache__' --exclude='apps/lumen/logs' \
    --exclude='arm/build*' --exclude='arm/recordings' \
    apps/lumen arm ) \
  | "${SSH[@]}" "tar xzf - -C $REPO"

"${SSH[@]}" "set -e; cd $REPO/apps/lumen
  python3 -m venv \"\$(readlink -f .venv)\" && .venv/bin/pip install -q -r requirements.txt
  mkdir -p ~/.config/systemd/user logs && cp lumen.service ~/.config/systemd/user/
  systemctl --user daemon-reload && systemctl --user enable lumen >/dev/null
  if [ -z '${LUMEN_NO_START:-}' ]; then systemctl --user restart lumen; fi
  sleep 2; systemctl --user --no-pager status lumen | head -5 || true"
echo "HUD: http://${STEPPER_HOST:-100.78.199.105}:8080/   logs: ssh $HOST journalctl --user -u lumen -f"
