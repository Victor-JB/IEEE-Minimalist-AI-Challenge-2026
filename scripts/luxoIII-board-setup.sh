#!/bin/bash
# luxoIII: set up a fresh Arduino UNO Q the way the lamp's board was, over USB (adb) from the Mac.
#   bash scripts/luxoIII-board-setup.sh root      # once, interactive: Wi-Fi, python3-venv, SSH, linger, clock (asks for
#                                                 # the board's sudo password and the Wi-Fi password)
#   bash scripts/luxoIII-board-setup.sh restore   # everything else; safe to re-run
#   bash scripts/luxoIII-board-setup.sh tailscale # join the tailnet as luxo3 (prints a login link)
# ARDUINO_Q_SERIAL picks the board (adb devices), BRANCH the branch the board clone checks out
# (default: the Mac's current branch).
#
# Layout on the board (same as the board that died 2026-10-12):
#   ~/IEEE-Minimalist-AI-Challenge-2026     repo clone; Lumen runs from here at boot (lumen.service)
#     .venv                                 one venv; apps/lumen/.venv and arm/.venv link to it
#   ~/IEEE-tongtong-worktree                git worktree on branch tongtong, for development
#   ~/tailscale                             rootless Tailscale (tailscaled.service, user unit)
#   ~/CHANGES.md                            notes for teammates (copy of docs/CHANGES-board-*.md)
#   ~/luxoIII/README.md                     this layout, written by this script
set -euo pipefail
SERIAL="${ARDUINO_Q_SERIAL:-634018433}"
REPO_LOCAL="$(cd "$(dirname "$0")/.." && pwd)"
BRANCH="${BRANCH:-$(git -C "$REPO_LOCAL" branch --show-current)}"
ADB="$(command -v adb || echo /opt/homebrew/bin/adb)"
A=("$ADB" -s "$SERIAL")
REPO=/home/arduino/IEEE-Minimalist-AI-Challenge-2026
WT=/home/arduino/IEEE-tongtong-worktree
GITHUB=https://github.com/Victor-JB/IEEE-Minimalist-AI-Challenge-2026
TS_VERSION=1.104.1
# Run on the board: over SSH (USB forward) once key login works, since `systemctl --user` needs
# the login session that adb shell lacks; over adb shell before that (fresh board).
board() {
  if [[ -z "${VIA:-}" ]]; then
    "${A[@]}" forward tcp:2222 tcp:22 >/dev/null 2>&1 || true
    if ssh -p 2222 -o BatchMode=yes -o ConnectTimeout=5 arduino@localhost true 2>/dev/null; then VIA=ssh; else VIA=adb; fi
  fi
  if [[ $VIA == ssh ]]; then ssh -p 2222 -o BatchMode=yes arduino@localhost "$1"; else "${A[@]}" shell "$1"; fi
}

root() {
  cat > /tmp/luxoIII-root.sh <<'EOF'
set -e
if ! nmcli -t -f DEVICE,STATE dev | grep -q '^wlan0:connected$'; then
  nmcli dev wifi list | head -15
  read -rp 'Wi-Fi network name: ' ssid
  nmcli --ask dev wifi connect "$ssid"
fi
dpkg -s python3-venv >/dev/null 2>&1 || { apt-get update -qq && apt-get install -y -qq python3-venv; }
ssh-keygen -A                      # the stock image ships without SSH host keys
systemctl reset-failed ssh 2>/dev/null || true
systemctl enable ssh && systemctl restart ssh
loginctl enable-linger arduino
timedatectl set-ntp true
sleep 3; date; nmcli -t -f DEVICE,STATE dev | grep wlan0; systemctl is-active ssh
loginctl show-user arduino -p Linger
EOF
  "${A[@]}" push /tmp/luxoIII-root.sh /tmp/luxoIII-root.sh >/dev/null
  "${A[@]}" shell -t 'sudo bash /tmp/luxoIII-root.sh; rm -f /tmp/luxoIII-root.sh'
}

restore() {
  board 'ping -c1 -W3 github.com >/dev/null && ping -c1 -W3 pypi.org >/dev/null' \
    || { echo 'The board is offline. Run: bash scripts/luxoIII-board-setup.sh root' >&2; exit 1; }

  echo '== SSH key and USB forward'
  "${A[@]}" push ~/.ssh/id_ed25519.pub /tmp/mac.pub >/dev/null
  board 'mkdir -p ~/.ssh && chmod 700 ~/.ssh && touch ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys
         grep -qxF "$(cat /tmp/mac.pub)" ~/.ssh/authorized_keys || cat /tmp/mac.pub >> ~/.ssh/authorized_keys; rm /tmp/mac.pub'
  "${A[@]}" forward tcp:2222 tcp:22

  echo "== Repo ($BRANCH) and worktree"
  git -C "$REPO_LOCAL" bundle create /tmp/luxoIII.bundle --branches 2>/dev/null
  "${A[@]}" push /tmp/luxoIII.bundle /tmp/luxoIII.bundle >/dev/null
  board "set -e
    git config --global user.name >/dev/null || git config --global user.name '$(git -C "$REPO_LOCAL" config user.name)'
    git config --global user.email >/dev/null || git config --global user.email '$(git -C "$REPO_LOCAL" config user.email)'
    [ -d $REPO/.git ] || git clone -q /tmp/luxoIII.bundle $REPO
    cd $REPO && git remote set-url origin $GITHUB && git fetch -q /tmp/luxoIII.bundle '+refs/heads/*:refs/remotes/mac/*'
    git fetch -q origin || echo 'GitHub fetch failed; using the Mac bundle only'
    git checkout -q -B $BRANCH mac/$BRANCH 2>/dev/null || git checkout -q $BRANCH
    git branch -q -u origin/$BRANCH 2>/dev/null || true
    [ -d $WT ] || git worktree add -q $WT -B tongtong $BRANCH
    rm -f /tmp/luxoIII.bundle; git log --oneline -1; git worktree list"
  rm -f /tmp/luxoIII.bundle

  echo '== Python venv (one for Lumen, the desk and the arm tools)'
  board "set -e; cd $REPO
    [ -x .venv/bin/python ] || python3 -m venv .venv
    .venv/bin/pip install -q -r apps/lumen/requirements.txt
    for d in apps/lumen arm; do [ -e \$d/.venv ] || ln -s $REPO/.venv \$d/.venv; done
    .venv/bin/python -c 'import numpy, cv2, msgpack; from ai_edge_litert import interpreter; print(\"venv ok\")'"

  echo '== Firmware: pinned core, compile, flash'
  board "set -e; cd $REPO/arm
    { arduino-cli core update-index && arduino-cli lib update-index; } >/dev/null || echo 'index update failed; compiling with what is cached'
    arduino-cli compile --profile default --build-path $REPO/arm/build sketch | tail -2
    bash scripts/flash-local.sh 2>&1 | grep -iE 'error|verified|wrote' | tail -4"

  echo '== Boot services: the desk web page (on at boot); Lumen installed but off (they conflict)'
  board "set -e; cd $REPO/apps/lumen; mkdir -p ~/.config/systemd/user logs
    cp lumen.service lumen-desk.service ~/.config/systemd/user/
    systemctl --user daemon-reload && systemctl --user enable --now lumen-desk >/dev/null
    echo \"desk: \$(systemctl --user is-enabled lumen-desk); lumen: \$(systemctl --user is-enabled lumen)\""

  echo '== Notes'
  "${A[@]}" push "$(ls "$REPO_LOCAL"/docs/CHANGES-board-*.md | tail -1)" /home/arduino/CHANGES.md >/dev/null
  board "mkdir -p ~/luxoIII && cat > ~/luxoIII/README.md" <<EOF
# luxoIII board

This Arduino UNO Q (USB serial $SERIAL) runs luxoIII, the expressive lamp arm. It replaced the
board that died on 2026-10-12. Set up by \`scripts/luxoIII-board-setup.sh\` in the repo.

| Path | What |
|---|---|
| \`~/IEEE-Minimalist-AI-Challenge-2026\` | repo clone on \`$BRANCH\`; Lumen runs from here at boot |
| \`~/IEEE-Minimalist-AI-Challenge-2026/.venv\` | the one venv (\`apps/lumen/.venv\`, \`arm/.venv\` link to it) |
| \`~/IEEE-tongtong-worktree\` | git worktree on branch \`tongtong\`, for development |
| \`~/tailscale\` | rootless Tailscale, user unit \`tailscaled.service\`, tailnet name \`luxo3\` |
| \`~/CHANGES.md\` | what changed in the 2026-10-12 work, for teammates |

- Lumen: \`systemctl --user start|stop lumen\`, logs \`journalctl --user -u lumen -f\`, HUD on :8080.
- Desk page: \`cd ~/IEEE-tongtong-worktree/apps/lumen && ~/IEEE-Minimalist-AI-Challenge-2026/.venv/bin/python -m lumen.desk --port 8081\`.
- Firmware: \`cd arm && arduino-cli compile --profile default --build-path build sketch && bash scripts/flash-local.sh\`.
- From the Mac over USB: \`adb forward tcp:2222 tcp:22\` then \`ssh -p 2222 arduino@localhost\`.
EOF
  echo 'restore done'
}

tailscale_up() {
  board "set -e; mkdir -p ~/tailscale/state ~/.config/systemd/user; cd ~/tailscale
    if [ ! -x tailscaled ]; then
      curl -sS -m 120 -o ts.tgz https://pkgs.tailscale.com/stable/tailscale_${TS_VERSION}_arm64.tgz
      echo \"\$(curl -sS -m 20 https://pkgs.tailscale.com/stable/tailscale_${TS_VERSION}_arm64.tgz.sha256)  ts.tgz\" | sha256sum -c -
      tar xzf ts.tgz --strip-components=1 && rm ts.tgz
    fi
    cat > ~/.config/systemd/user/tailscaled.service <<'UNIT'
[Unit]
Description=Tailscale (userspace, no root)
[Service]
ExecStart=%h/tailscale/tailscaled --tun=userspace-networking --statedir=%h/tailscale/state --socket=%h/tailscale/tailscaled.sock --port=41641
Restart=on-failure
[Install]
WantedBy=default.target
UNIT
    systemctl --user daemon-reload && systemctl --user enable --now tailscaled >/dev/null; sleep 3
    (timeout 15 ./tailscale --socket=\$HOME/tailscale/tailscaled.sock up --hostname=luxo3 2>&1 || true) \
      | grep -o 'https://login.tailscale.com/[^ ]*' | head -1
    ./tailscale --socket=\$HOME/tailscale/tailscaled.sock status 2>&1 | head -2"
}

case "${1:-restore}" in
  root) root ;;
  restore) restore ;;
  tailscale) tailscale_up ;;
  *) echo "usage: $0 root|restore|tailscale" >&2; exit 2 ;;
esac
