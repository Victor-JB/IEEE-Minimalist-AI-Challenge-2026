// Head-tilt servo command: clamping, slew, pulse mapping, limp.
#include "../sketch/roll.hpp"
#include <cassert>
#include <cstdio>
using namespace roll;

int main(){
    Roll r;
    assert(r.pulseUs()==0);                                   // limp until the first command
    assert(r.set(10,0,1000)==0&&r.enabled&&r.current==0);     // wakes at the last angle (0)...
    assert(r.pulseUs()==1500);
    for(uint32_t t=1001;t<=1100;++t)r.tick(t);                // ...and slews there at 180 deg/s
    assert(std::abs(r.current-10)<1e-6&&r.pulseUs()==unsigned(std::lround(1500+10*2000.0/270)));
    r.release();assert(r.pulseUs()==0);
    assert(r.set(-30,60,1100)==0&&r.current==10);             // after limp: resumes from 10, no snap
    r.last=1000;
    for(uint32_t t=1001;t<=1500;++t)r.tick(t);                // 0.5 s at 60 deg/s = 30 deg
    assert(std::abs(r.current-(-20))<1e-6);
    for(uint32_t t=1501;t<=2000;++t)r.tick(t);
    assert(std::abs(r.current-(-30))<1e-6);                   // arrives, no overshoot
    assert(r.set(200,0,2000)==0&&r.target==45);               // clamped, not refused
    assert(r.set(NAN,0,2000)==-EINVAL&&r.set(0,-1,2000)==-EINVAL);
    assert(r.configure(1500,2000.0/270,-60,60,-1,90)==0);
    r.current=20;assert(r.pulseUs()==unsigned(std::lround(1500-20*2000.0/270)));  // sign flips
    assert(r.configure(1500,7,10,-10,1,90)==-EINVAL&&r.configure(3000,7,-10,10,1,90)==-EINVAL);
    assert(r.configure(1500,100,-135,135,1,90)==-EINVAL);
    assert(r.configure(1500,2000.0/270,-135,135,1,90)==0);
    r.current=135;assert(r.pulseUs()==unsigned(std::lround(1500+135*2000.0/270)));
    r.centerUs=2400;assert(r.pulseUs()==2500);                // never outside 500-2500 us
    r.release();assert(r.pulseUs()==0);
    std::puts("roll tests passed");
}
