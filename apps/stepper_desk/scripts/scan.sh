#!/bin/bash
set -euo pipefail
ADB="$(command -v adb || true)"
if [[ -z "$ADB" ]]; then
  for candidate in /opt/homebrew/bin/adb /usr/local/bin/adb; do
    if [[ -x "$candidate" ]]; then ADB="$candidate"; break; fi
  done
fi
[[ -n "$ADB" ]] || { echo 'Install android-platform-tools with Homebrew.' >&2; exit 1; }
# Use the existing USB shell directly; it does not need the SSH forward.
"$ADB" -s 2709356903 shell '/home/arduino/IEEE-Minimalist-AI-Challenge-2026/arm/.venv/bin/python /home/arduino/IEEE-Minimalist-AI-Challenge-2026/arm/python/scanner.py'
