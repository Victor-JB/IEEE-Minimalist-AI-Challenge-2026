#include "../sketch/motion.hpp"
#include <cassert>
#include <iostream>
using namespace stepper;
int main() {
    Point curve[]={{0,0},{60,500},{3000,1000}};
    Motor m;
    assert(m.start(1,1,0,200,curve,3,0)==-EACCES);
    assert(m.claim(123)==0);
    assert(m.start(123,1,1,200,curve,3,0)==0);
    m.tick(59); assert(!m.direction && !m.hz);
    m.tick(100); assert(m.direction && !m.hz);
    m.tick(119); assert(!m.hz);
    m.tick(150); assert(m.hz==250);
    assert(m.hold(123,1,200)==0);
    m.tick(250); assert(m.hz>500);
    assert(m.hold(123,1,500)==-ESTALE); assert(!m.running && !m.hz);
    assert(m.start(123,1,0,200,curve,3,501)==-EINVAL);
    assert(m.start(123,2,0,200,curve,3,501)==0);
    assert(m.claim(456)==-EBUSY);
    m.stop(); assert(!m.hz && !m.running);
    Point bad[]={{0,0},{10,2500}};
    assert(m.start(123,3,0,200,bad,2,502)==-EINVAL);
    assert(m.last_id==2);
    Motor axes[3];
    for(unsigned a=0;a<3;a++) {
        axes[a].claim(a+1); curve[2].hz=1000+a*500;
        assert(axes[a].start(a+1,1,a%2,200,curve,3,0)==0);
    }
    for(unsigned t=0;t<=4000;t+=5) {
        for(unsigned a=0;a<3;a++) { if(t%100==0) axes[a].hold(a+1,1,t); axes[a].tick(t); }
    }
    assert(axes[0].hz==1000 && axes[1].hz==1500 && axes[2].hz==2000);
    axes[1].stop(); assert(axes[0].running && !axes[1].running && axes[2].running);
    m.stop(); assert(m.start(123,3,1,200,curve,3,UINT32_MAX-99)==0);
    m.tick(UINT32_MAX-20); m.tick(40); assert(m.hz>0);
    m.tick(201); assert(!m.running);
    for(unsigned hz=MIN_HZ;hz<=MAX_HZ;hz++) { unsigned period=(1000000+hz/2)/hz; assert(period<=65536 && period/2>=2); }
    std::cout << "Motion checks passed: 3 axes, direction dwell, expiry, replay, wrap, PWM range\n";
}
