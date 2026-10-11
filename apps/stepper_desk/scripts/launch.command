#!/bin/zsh
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ ! -d "$ROOT/mac/Stepper Desk Q.app" ]]; then zsh "$ROOT/mac/build.sh"; fi
open "$ROOT/mac/Stepper Desk Q.app"
