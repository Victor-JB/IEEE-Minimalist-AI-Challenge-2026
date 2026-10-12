#include "../sketch/sensors.hpp"
#include <vector>
#include <cassert>
#include <iostream>
struct Bus {
    std::vector<int> masks;
    bool muxOK=true, sensorOK=true, deselectOK=true;
    int reads=0;
    uint8_t status=0x20, hi=0x08, lo=0x00;
    bool select(uint8_t mask) { masks.push_back(mask); return mask ? muxOK : deselectOK; }
    bool read(uint8_t reg, uint8_t *bytes, size_t len) {
        reads++; if (!sensorOK) return false;
        if(reg==0x0b) { assert(len==1); bytes[0]=status; }
        else { assert(reg==0x0c && len==2); bytes[0]=hi; bytes[1]=lo; }
        return true;
    }
};
int main() {
    for(unsigned i=0;i<3;i++) {
        Bus bus; auto result=encoder::read(bus,i);
        assert((bus.masks==std::vector<int>{1<<i,0}));
        assert(result.raw==2048 && result.magnet==0x20 && result.error==0);
    }
    Bus missing; missing.sensorOK=false;
    assert(encoder::read(missing,0).error==encoder::SENSOR);
    assert(missing.masks.back()==0);
    Bus mux; mux.muxOK=false;
    assert(encoder::read(mux,1).error==encoder::MUX && !mux.reads);
    Bus magnet; magnet.status=0x18; magnet.hi=0xff; magnet.lo=0xff;
    auto result=encoder::read(magnet,2);
    assert(result.magnet==0x18 && result.raw==4095);
    Bus stuck; stuck.deselectOK=false;
    assert(encoder::read(stuck,0).error==encoder::MUX);
    Bus invalid; assert(encoder::read(invalid,3).error==encoder::MUX && invalid.masks.empty());
    std::cout << "Sensor checks passed: channel isolation, 12-bit angle, magnet bits, bus failures\n";
}
