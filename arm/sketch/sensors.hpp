// SPDX-License-Identifier: MIT
#pragma once
#include <cstdint>
namespace encoder {
enum Error { OK=0, STARTING=1, MUX=2, SENSOR=3, BUS=4 };
struct Sample { int raw=0, magnet=0, error=STARTING; uint32_t sampled=0; };
// One selected channel at a time: all three AS5600s share address 0x36.
// Only reads sensor registers; never programs configuration or OTP memory.
template<class Bus> Sample read(Bus &bus, unsigned channel) {
    Sample out;
    if (channel > 2) { out.error=MUX; return out; }
    if (!bus.select(uint8_t(1u << channel))) {
        out.error=MUX; bus.select(0); return out;
    }
    uint8_t status=0, angle[2]={0,0};
    if (!bus.read(0x0b, &status, 1) || !bus.read(0x0c, angle, 2)) out.error=SENSOR;
    else {
        out.raw=((unsigned(angle[0]) << 8) | angle[1]) & 0x0fff;
        out.magnet=status & 0x38;
        out.error=OK;
    }
    if (!bus.select(0)) out.error=MUX;
    return out;
}
}
