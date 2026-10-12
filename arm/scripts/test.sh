#!/bin/bash
# Firmware logic (portable C++ against the sketch headers) + the Python arm API. No board needed.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TEMP="$(mktemp -d /tmp/arm-tests.XXXXXX)"
trap 'rm -rf "$TEMP"' EXIT
for t in motion sensors limits power trajectory counted_jog roll; do
  c++ -std=c++17 "$ROOT/tests/${t}_test.cpp" -o "$TEMP/$t" && "$TEMP/$t"
done
c++ -std=c++17 -O1 "$ROOT/tests/servo_test.cpp" -o "$TEMP/servo" && "$TEMP/servo"
python3 -m unittest discover -s "$ROOT/tests" -p 'test_*.py'
