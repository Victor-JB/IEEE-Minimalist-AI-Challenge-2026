// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <cerrno>
#include <cstdint>
// Portable light state and expiry. D7 uses setSwitch(): any positive level means
// fully on, with no PWM or fade. The general set/fade/compare helpers remain for
// other backends and existing host tests. No hardware access here.
//
// Manual levels survive the session ending. An optional renewable lease is for automatic
// lighting: missing host updates force it off in the MCU's independent watchdog thread.
namespace light {
constexpr int MAX=1000;
constexpr int MAX_FADE_MS=60000;
constexpr int MAX_LEASE_MS=2000;

struct Light {
    int from=0,to=0,current=0;
    uint32_t start=0,fade=0;
    uint32_t leasedAt=0,lease=0;

    // Out-of-range levels are clamped, not refused (a refused command would end the host
    // session). A bad fade time is refused. fade 0 = jump.
    int set(int level,int fadeMs,uint32_t now,int leaseMs=0){
        if(fadeMs<0||fadeMs>MAX_FADE_MS||leaseMs<0||leaseMs>MAX_LEASE_MS)return -EINVAL;
        tick(now);                                   // fade from wherever it is right now
        from=current;to=std::max(0,std::min(MAX,level));start=now;fade=uint32_t(fadeMs);
        leasedAt=now;lease=uint32_t(leaseMs);
        if(!fade)current=to;
        return 0;
    }
    // D7 is an on/off output. Keep the legacy request shape, but report the
    // actual binary level and never ramp. Invalid requests cannot renew a lease.
    int setSwitch(int level,int fadeMs,uint32_t now,int leaseMs=0){
        if(fadeMs<0||fadeMs>MAX_FADE_MS)return -EINVAL;
        return set(level>0?MAX:0,0,now,leaseMs);
    }
    void tick(uint32_t now){
        if(lease&&uint32_t(now-leasedAt)>=lease){from=to=current=0;fade=lease=0;return;}
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
