#!/bin/bash
set -euo pipefail
ROOT=/home/arduino/stepper-desk-q
CORE=/home/arduino/.arduino15/packages/arduino/hardware/zephyr/1.0.0
LOADER="$CORE/firmwares/zephyr-arduino_uno_q_stm32u585xx.elf"
SKETCH="$ROOT/diagnostics/build-standard-scanner/i2c_scanner.ino.elf-zsk.bin"
RECIPE="$CORE/variants/arduino_uno_q_stm32u585xx/flash_sketch.cfg"
test -f "$SKETCH"
/opt/openocd/bin/openocd -d2 -s /opt/openocd -f openocd_gpiod.cfg \
 -c "set filename0 {$LOADER}; set filename1 {$SKETCH}" -f "$RECIPE"
/opt/openocd/bin/openocd -d2 -s /opt/openocd -f openocd_gpiod.cfg \
 -c "init; halt; flash verify_image {$SKETCH} 0x08100000 bin; resume; shutdown"
