// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <cerrno>
#include <cstdint>
// Lamp LED brightness: 0-1000 (per mille duty), with an optional linear fade so the 10-25 Hz
// host updates don't step visibly. The host already applies the perceptual (gamma) curve and
// the ease in/out ramps (apps/luxoiii/luxoiii/io/light.py); this only clamps, fades and maps to a
// compare value. Portable: no hardware access, tested on the host (tests/light_test.cpp).
//
// Unlike motion, the light needs no owner and survives the session ending: a lamp that goes
// dark whenever the app restarts would be worse than one that stays at its last level.
namespace light {
constexpr int MAX=1000;
constexpr int MAX_FADE_MS=60000;

struct Light {
    int from=0,to=0,current=0;
    uint32_t start=0,fade=0;

    // Out-of-range levels are clamped, not refused (a refused command would end the host
    // session). A bad fade time is refused. fade 0 = jump.
    int set(int level,int fadeMs,uint32_t now){
        if(fadeMs<0||fadeMs>MAX_FADE_MS)return -EINVAL;
        tick(now);                                   // fade from wherever it is right now
        from=current;to=std::max(0,std::min(MAX,level));start=now;fade=uint32_t(fadeMs);
        if(!fade)current=to;
        return 0;
    }
    void tick(uint32_t now){
        if(current==to)return;
        const uint32_t elapsed=now-start;
        if(elapsed>=fade){current=to;return;}
        const int64_t span=int64_t(to)-from;
        current=from+int((span*int64_t(elapsed)+(span>=0?1:-1)*int64_t(fade/2))/int64_t(fade));
    }
    uint32_t remaining(uint32_t now)const{
        if(current==to)return 0;
        const uint32_t elapsed=now-start;
        return elapsed>=fade?0:fade-elapsed;
    }
    // Timer compare value for a PWM period in counts; 1000 = always on, 0 = off.
    uint32_t compare(uint32_t period)const{
        return uint32_t((uint64_t(period)*uint64_t(current)+MAX/2)/MAX);
    }
};
}
