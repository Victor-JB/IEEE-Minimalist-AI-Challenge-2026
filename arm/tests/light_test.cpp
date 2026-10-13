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
    Light automatic;
    assert(automatic.set(400,0,100,800)==0&&automatic.current==400);
    automatic.tick(899);assert(automatic.current==400);
    automatic.tick(900);assert(automatic.current==0&&automatic.to==0); // expires even at steady brightness
    automatic.set(400,0,1000,800);automatic.set(400,0,1700,800);
    automatic.tick(1800);assert(automatic.current==400);             // renewal extends the lease
    automatic.tick(2500);assert(automatic.current==0);
    automatic.set(400,1000,3000,100);automatic.tick(3100);
    assert(automatic.current==0&&automatic.remaining(3100)==0);      // expiry interrupts a fade
    automatic.set(400,0,0xFFFFFFF0u,100);automatic.tick(0x00000054u);
    assert(automatic.current==0);                                  // expiry across millis wrap
    assert(automatic.set(400,0,4000,-1)==-EINVAL);
    assert(automatic.set(400,0,4000,MAX_LEASE_MS+1)==-EINVAL);
    automatic.set(400,0,4000,800);automatic.set(0,0,4010);
    assert(automatic.current==0&&automatic.lease==0);               // immediate off cancels the lease
    automatic.set(400,0,5000);automatic.tick(10000);
    assert(automatic.current==400);                                // existing manual API is unchanged
    std::puts("light tests passed");
}
