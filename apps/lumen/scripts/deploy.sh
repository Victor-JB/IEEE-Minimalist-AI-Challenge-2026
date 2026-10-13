#!/bin/bash
# From the Mac: copy Lumen (apps/lumen) and the shared arm module (arm/) to the board, install its venv,
# install/restart the boot service. Does NOT flash the MCU: the firmware comes from
# arm/scripts/deploy.sh (run that once, and again only when arm/sketch changes).
#   bash apps/lumen/scripts/deploy.sh            # USB if plugged in, else Tailscale (luxo3)
#   LUMEN_NO_START=1 bash apps/lumen/scripts/deploy.sh   # install but don't (re)start
#   LUMEN_BOOT=lumen bash apps/lumen/scripts/deploy.sh   # boot into Lumen + HUD (:8080) instead of
#                                                        # the desk page (:8081, the default on luxoIII)
# Lumen and the desk page both need the arm and camera (lumen-desk.service Conflicts= lumen),
# so exactly one is enabled at boot.
set -euo pipefail
REPO_LOCAL="$(cd "$(dirname "$0")/../../.." && pwd)"
ADB="$(command -v adb || true)"
[[ -z "$ADB" && -x /opt/homebrew/bin/adb ]] && ADB=/opt/homebrew/bin/adb
LINK="${STEPPER_LINK:-auto}"
if [[ "$LINK" != tailscale && -n "$ADB" ]] && "$ADB" -s "${ARDUINO_Q_SERIAL:-634018433}" forward tcp:2222 tcp:22 >/dev/null 2>&1; then
  PORT=2222; HOST=arduino@localhost; echo 'Link: USB'
elif [[ "$LINK" == usb ]]; then
  echo 'Arduino Q not found over USB.' >&2; exit 1
else
  PORT=22; HOST="arduino@${STEPPER_HOST:-luxo3}"; echo "Link: Tailscale ($HOST)"
fi
SSH=(ssh -p "$PORT" -o BatchMode=yes -o ConnectTimeout=5 "$HOST")
REPO=/home/arduino/IEEE-Minimalist-AI-Challenge-2026
BOOT="${LUMEN_BOOT:-desk}"
[[ "$BOOT" == desk || "$BOOT" == lumen ]] || { echo "LUMEN_BOOT must be desk or lumen" >&2; exit 1; }

"${SSH[@]}" "test -d $REPO/.git || git clone -q https://github.com/Victor-JB/IEEE-Minimalist-AI-Challenge-2026 $REPO"
# Working tree -> board clone (uncommitted changes deploy too): the app + the shared arm module.
( cd "$REPO_LOCAL" && COPYFILE_DISABLE=1 tar czf - \
    --exclude='.venv' --exclude='__pycache__' --exclude='apps/lumen/logs' \
    --exclude='arm/build*' --exclude='arm/recordings' \
    apps/lumen arm ) \
  | "${SSH[@]}" "tar xzf - -C $REPO"

"${SSH[@]}" "set -e; cd $REPO/apps/lumen
  python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt
  mkdir -p ~/.config/systemd/user logs && cp lumen.service lumen-desk.service ~/.config/systemd/user/
  systemctl --user daemon-reload
  if [ '${BOOT}' = lumen ]; then ON=lumen OFF=lumen-desk; else ON=lumen-desk OFF=lumen; fi
  systemctl --user disable \$OFF >/dev/null 2>&1 || true
  systemctl --user enable \$ON >/dev/null
  if [ -z '${LUMEN_NO_START:-}' ]; then systemctl --user restart \$ON; fi
  sleep 2; systemctl --user --no-pager status \$ON | head -5 || true"
if [[ "$BOOT" == lumen ]]; then
  echo "Boot: Lumen. HUD: http://${STEPPER_HOST:-luxo3}:8080/   logs: ssh $HOST journalctl --user -u lumen -f"
else
  echo "Boot: desk page http://${STEPPER_HOST:-luxo3}:8081/ (Lumen installed, off: LUMEN_BOOT=lumen to switch)"
fi
