#!/bin/bash
# Run on Arduino Q after building. Use the recipe shipped with the pinned core;
# the older /opt/openocd/bin/arduino-flash.sh uses a different sketch address.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CORE=/home/arduino/.arduino15/packages/arduino/hardware/zephyr/1.0.0
LOADER="$CORE/firmwares/zephyr-arduino_uno_q_stm32u585xx.elf"
BUILD_DIR="${STEPPER_BUILD_DIR:-$ROOT/build}"
SKETCH="$BUILD_DIR/sketch.ino.elf-zsk.bin"
RECIPE="$CORE/variants/arduino_uno_q_stm32u585xx/flash_sketch.cfg"
[[ -f "$LOADER" && -f "$SKETCH" && -f "$RECIPE" ]] || { echo 'Build artifacts or pinned core missing.' >&2; exit 1; }
/opt/openocd/bin/openocd -d2 -s /opt/openocd -f openocd_gpiod.cfg \
  -c "set filename0 {$LOADER}; set filename1 {$SKETCH}" -f "$RECIPE"
# Verify actual flash contents after programming, then resume the stopped sketch.
/opt/openocd/bin/openocd -d2 -s /opt/openocd -f openocd_gpiod.cfg \
  -c "init; halt; flash verify_image {$LOADER}; flash verify_image {$SKETCH} 0x08100000 bin; resume; shutdown"
