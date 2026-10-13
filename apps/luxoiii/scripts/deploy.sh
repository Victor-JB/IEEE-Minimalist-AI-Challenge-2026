#!/bin/bash
# From the Mac: copy luxoiii (apps/luxoiii) and the shared arm module (arm/) to the board, install its venv,
# install/restart the boot service. Does NOT flash the MCU: the firmware comes from
# arm/scripts/deploy.sh (run that once, and again only when arm/sketch changes).
#   bash apps/luxoiii/scripts/deploy.sh            # USB if plugged in, else Tailscale (luxo3); on the board itself it runs locally
#   LUXOIII_NO_START=1 bash apps/luxoiii/scripts/deploy.sh   # install but don't (re)start
set -euo pipefail
REPO_LOCAL="$(cd "$(dirname "$0")/../../.." && pwd)"
ADB="$(command -v adb || true)"
[[ -z "$ADB" && -x /opt/homebrew/bin/adb ]] && ADB=/opt/homebrew/bin/adb
LINK="${LUXOIII_LINK:-${STEPPER_LINK:-auto}}"
REPO=/home/arduino/IEEE-Minimalist-AI-Challenge-2026
if [[ "$LINK" == local || ( "$LINK" == auto && -S /var/run/arduino-router.sock ) ]]; then
  # Running on the Arduino Q itself: no SSH hop (its Tailscale is userspace-only, so it can't reach its own 100.x IP).
  LOCAL=1; echo 'Link: local'
elif [[ "$LINK" != tailscale && -n "$ADB" ]] && "$ADB" -s "${ARDUINO_Q_SERIAL:-2709356903}" forward tcp:2222 tcp:22 >/dev/null 2>&1; then
  PORT=2222; HOST=arduino@localhost; echo 'Link: USB'
elif [[ "$LINK" == usb ]]; then
  echo 'Arduino Q not found over USB.' >&2; exit 1
else
  PORT=22; HOST="arduino@${LUXOIII_HOST:-${STEPPER_HOST:-100.78.199.105}}"; echo "Link: Tailscale ($HOST)"
fi
if [[ -n "${LOCAL:-}" ]]; then SSH=(bash -c); HOST=localhost
else SSH=(ssh -p "$PORT" -o BatchMode=yes -o ConnectTimeout=5 "$HOST"); fi

[[ -n "${LOCAL:-}" ]] || "${SSH[@]}" "test -d $REPO/.git || git clone -q https://github.com/Victor-JB/IEEE-Minimalist-AI-Challenge-2026 $REPO"
# Working tree -> board clone (uncommitted changes deploy too; skipped when deploying from that clone): the app + the shared arm module.
[[ "$REPO_LOCAL" -ef "$REPO" ]] || ( cd "$REPO_LOCAL" && COPYFILE_DISABLE=1 tar czf - \
    --exclude='.venv' --exclude='__pycache__' --exclude='apps/luxoiii/logs' \
    --exclude='arm/build*' --exclude='arm/recordings' \
    apps/luxoiii arm tools/spatial ) \
  | "${SSH[@]}" "tar xzf - -C $REPO"

"${SSH[@]}" "set -e; cd $REPO/apps/luxoiii
  python3 -m venv \"\$(readlink -f .venv)\" && .venv/bin/pip install -q -r requirements.txt
  mkdir -p ~/.config/systemd/user logs && cp luxoiii.service luxoiii-observer.service luxoiii-desk.service ~/.config/systemd/user/
  systemctl --user daemon-reload
  if [ -z '${LUXOIII_NO_START:-${LUMEN_NO_START:-}}' ]; then
    systemctl --user disable --now lumen lumen-desk 2>/dev/null || true
    systemctl --user stop luxoiii luxoiii-observer luxoiii-desk 2>/dev/null || true
    systemctl --user enable luxoiii >/dev/null
    systemctl --user restart luxoiii
  fi
  sleep 2; systemctl --user --no-pager status luxoiii | head -5 || true"
echo "Main app: http://127.0.0.1:8082/ (board loopback; forward with SSH). Logs: ssh $HOST journalctl --user -u luxoiii -f"
