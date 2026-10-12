#include <cassert>
#include <iostream>
#include "../sketch/limits.hpp"
using namespace joint_limit;
int main() {
    encoder::Sample s; s.error=0; s.raw=0; s.magnet=32; s.sampled=100;
    Guard g;
    assert(g.state(1,s,100)==UNCONFIGURED && !g.permits(1,s,100,0));
    assert(!g.configure(180000001,0,1) && !g.configure(0,2,1) && !g.configure(0,0,0));
    // Exact boundaries independent of encoder quantization: zero shifts raw=0.
    for(unsigned motor=0; motor<3; ++motor) {
        int forward=motor==1 ? 1 : -1;
        assert(g.configure(-MIN_MDEG[motor]*1000,0,forward));
        assert(g.state(motor,s,100)==LOWER);
        assert(g.permits(motor,s,100,forward>0 ? 0 : 1));
        assert(!g.permits(motor,s,100,forward>0 ? 1 : 0));
        assert(g.configure(-MAX_MDEG[motor]*1000,0,forward));
        assert(g.state(motor,s,100)==UPPER);
        assert(g.permits(motor,s,100,forward<0 ? 0 : 1));
        assert(!g.permits(motor,s,100,forward<0 ? 1 : 0));
    }
    assert(g.configure(0,0,1));
    assert(g.permits(1,s,100,0) && g.permits(1,s,100,1));
    s.magnet=48; assert(g.permits(1,s,100,0)); // Low field with MD stays usable.
    s.magnet=16; assert(!g.permits(1,s,100,0));
    s.magnet=40; assert(!g.permits(1,s,100,0));
    s.magnet=32; assert(!g.permits(1,s,201,0));
    s.error=3; assert(!g.permits(1,s,100,0));
    s.error=0; s.raw=4096; assert(!g.permits(1,s,100,0));
    s.raw=0; s.sampled=UINT32_MAX-10; assert(g.permits(1,s,5,0));
    s.sampled=100; s.raw=4095;
    assert(std::abs(g.angle(s)+0.087890625)<0.000001);
    assert(g.configure(22500000,0,1)); s.raw=256; assert(std::abs(g.angle(s))<0.000001);
    assert(g.configure(53349609,1,-1)); s.raw=607; assert(std::abs(g.angle(s))<0.000001);
    assert(SENSOR_FOR_MOTOR[0]==1 && SENSOR_FOR_MOTOR[1]==0 && SENSOR_FOR_MOTOR[2]==2);
    std::cout<<"PASS: limits, motor/sensor swap, inward escape, calibration, signed directions, weak field, sensor failure, stale data and clock wrap\n";
}
