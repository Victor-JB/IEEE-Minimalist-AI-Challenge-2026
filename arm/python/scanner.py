#!/usr/bin/env python3
"""Run on Arduino Q; read-address scan of STM32 header SDA/SCL, not Linux I2C."""
import sys
import time
from arduino.router_bridge import Bridge


def main():
    bridge = Bridge()
    try:
        if not bridge.connect(timeout=3):
            raise RuntimeError('Arduino router not available')
        state = bridge.call('stepper_scan', 1, timeout=1)
        if len(state) < 8 or state[0] != 1:
            raise RuntimeError('Scan unavailable (release/disarm all motors first): %r' % state)
        deadline = time.monotonic() + 8
        while state[1] != 3 and time.monotonic() < deadline:
            time.sleep(0.1)
            state = bridge.call('stepper_scan', 0, timeout=1)
        if state[1] != 3:
            raise TimeoutError('Scanner did not finish')
        _, _, generation, error, last, sda, scl, ready, *addresses = state
        print('STM32 I2C2: SDA header (D20/PB11), SCL header (D21/PB10)')
        print('Bus ready: %s; sampled SDA=%s, SCL=%s' % (bool(ready), sda, scl))
        print('Read-address scan: 0x08 through 0x%02X; generation %d' % (last, generation))
        print('Responding addresses: ' + (', '.join('0x%02X' % a for a in addresses) or 'none'))
        muxes = [a for a in addresses if 0x70 <= a <= 0x77]
        if muxes:
            print('Possible PCA9548A/TCA9548A: ' + ', '.join('0x%02X' % a for a in muxes))
        if error:
            raise RuntimeError('Scan incomplete, MCU error %d' % error)
        print('No motor-start command sent. Read-only probes can miss write-only devices.')
        return 0
    except Exception as exc:
        print('Scanner error: %s' % exc, file=sys.stderr)
        return 1
    finally:
        bridge.disconnect()

if __name__ == '__main__':
    sys.exit(main())
