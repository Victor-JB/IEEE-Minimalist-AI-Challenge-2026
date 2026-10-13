// Lamp LED: clamping, fades, compare mapping.
#include "../sketch/light.hpp"
#include <cassert>
#include <cstdio>
using namespace light;

int main(){
    Light l;
    assert(l.current==0&&l.compare(8000)==0);                 // off at power-up
    assert(l.set(1000,0,100)==0&&l.current==1000);            // fade 0 = jump
    assert(l.compare(8000)==8000&&l.remaining(100)==0);
    assert(l.set(0,500,1000)==0&&l.current==1000);            // fades down over 500 ms
    l.tick(1250);assert(l.current==500&&l.remaining(1250)==250);
    l.tick(1500);assert(l.current==0&&l.remaining(1500)==0);
    l.tick(1600);assert(l.current==0);                        // stays there
    assert(l.set(400,100,2000)==0);l.tick(2050);assert(l.current==200);
    assert(l.set(1000,100,2050)==0);                          // retarget mid-fade: from 200, no jump
    assert(l.current==200);l.tick(2100);assert(l.current==600);l.tick(2150);assert(l.current==1000);
    assert(l.set(5000,0,3000)==0&&l.current==1000);           // clamped, not refused
    assert(l.set(-5,0,3000)==0&&l.current==0);
    assert(l.set(500,-1,3000)==-EINVAL&&l.set(500,MAX_FADE_MS+1,3000)==-EINVAL&&l.current==0);
    assert(l.set(1,0,3000)==0&&l.compare(8000)==8);           // 0.1 % steps: 8 counts at 20 kHz/160 MHz
    assert(l.set(333,0,3000)==0&&l.compare(1000)==333);
    Light w;w.set(1000,100,0xFFFFFFF0u);w.tick(0x00000022u);  // millis() wraps mid-fade (50 ms in)
    assert(w.current==500);w.tick(0x00000060u);assert(w.current==1000);
    std::puts("light tests passed");
}
