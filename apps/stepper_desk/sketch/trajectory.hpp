// SPDX-License-Identifier: MIT
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cerrno>
#include <algorithm>
namespace coordinated {
// All arrays use physical motor order. Keep this first commissioning mode slow.
constexpr double LOWER[3]={-72,-34,-80}, UPPER[3]={79,38,87};
constexpr unsigned MAX_RATE=200, MAX_JOG_RATE=2400, JOG_RATE=100;
enum State { IDLE, PREPARING, MOVING, MEASURING, COMPLETE, RELEASED, LEASE_LOST, SENSOR_FAULT, LIMIT, TRACKING, TIMING, TRAVEL, SETTLING };
struct Motion {
    State state=IDLE;
    uint32_t id=0,lastID=0,renewed=0,began=0,duration=0,lastTick=0,badSince=0,settling=0,pausedAt=0;
    bool bad=false,settlingStarted=false,externalJog=false;
    int jog=-1;
    unsigned jogPeriodMs=10,jogRateHz=JOG_RATE,prepareMs=80;
    std::array<double,3> start{},target{},ppd{},reference{};
    std::array<unsigned,3> direction{},total{},count{},lastPulse{};
    bool running()const{return state==PREPARING||state==MOVING||state==MEASURING||state==SETTLING;}
    void stop(State why=RELEASED){if(why==RELEASED&&!running())return;state=why;}
    static double blend(double t){t=std::max(0.0,std::min(1.0,t));return t*t*t*(10+t*(-15+6*t));}
    int begin(uint32_t next,const std::array<double,3>& measured,const std::array<double,3>& goal,
              const std::array<double,3>& conversion,const std::array<int,3>& forward,uint32_t now,
              int jogAxis=-1,unsigned jogDirection=0,unsigned jogRate=JOG_RATE,bool hardwareJog=false,unsigned preparationMs=80){
        if(running())return -EBUSY;
        if(!next||next<=lastID||jogAxis < -1||jogAxis>2||jogDirection>1)return -EINVAL;
        if(jogAxis>=0&&(jogRate<20||jogRate>(hardwareJog?MAX_JOG_RATE:MAX_RATE)))return -ERANGE;
        if(preparationMs<80||preparationMs>500||(preparationMs!=80&&!(hardwareJog&&jogAxis>=0)))return -ERANGE;
        double seconds=5;
        for(unsigned a=0;a<3;++a){
            if(!std::isfinite(measured[a])||!std::isfinite(goal[a])||std::abs(measured[a])>180)return -EINVAL;
            if(jogAxis<0){
                if(!std::isfinite(conversion[a])||conversion[a]<1||conversion[a]>10000 ||
                    measured[a]<LOWER[a]||measured[a]>UPPER[a] || goal[a]<LOWER[a]+2||goal[a]>UPPER[a]-2 ||
                    std::abs(goal[a]-measured[a])>5.0)return -ERANGE;
                // Min-jerk peak speed = 1.875 * distance / duration, with headroom.
                seconds=std::max(seconds,std::abs(goal[a]-measured[a])*conversion[a]*1.875/150.0);
            }
        }
        if(seconds>120)return -ERANGE;
        id=lastID=next;renewed=began=lastTick=now;jog=jogAxis;start=reference=measured;target=goal;ppd=conversion;
        externalJog=hardwareJog&&jogAxis>=0;jogRateHz=jogRate;prepareMs=preparationMs;
        jogPeriodMs=jogAxis>=0?(1000+jogRate-1)/jogRate:10;
        duration=uint32_t(std::ceil(seconds*1000));bad=false;settlingStarted=false;
        count.fill(0);lastPulse.fill(now);
        for(unsigned a=0;a<3;++a){
            direction[a]=jogAxis>=0?jogDirection:((goal[a]-measured[a])*forward[a]<0?1:0);
            total[a]=jogAxis>=0?0:unsigned(std::lround(std::abs(goal[a]-measured[a])*ppd[a]));
        }
        state=PREPARING;return 0;
    }
    int hold(uint32_t next,uint32_t now){
        if(running()&&uint32_t(now-renewed)>=300)stop(LEASE_LOST);
        if(!running()||next!=id)return -ESTALE;
        renewed=now;return 0;
    }
    // Explicit bounded powered settle; never resumes or emits any more steps.
    // Repeating PAUSE cannot extend the fixed deadline. HOLD is still required.
    int pause(uint32_t next,uint32_t now){
        if(running()&&uint32_t(now-renewed)>=300)stop(LEASE_LOST);
        if(state!=MEASURING||!externalJog||next!=id)return -ESTALE;
        state=SETTLING;pausedAt=now;return 0;
    }
    // Returns a pulse mask; caller emits exactly one >=4 us pulse for each bit.
    // No free-running PWM and no catch-up bursts. count is issued edges, not proof of motion.
    unsigned tick(uint32_t now,const std::array<double,3>& measured,bool healthy,bool permitted,bool awake){
        if(!running())return 0;
        if(uint32_t(now-renewed)>=300){stop(LEASE_LOST);return 0;}
        if(!healthy){stop(SENSOR_FAULT);return 0;}
        if(state!=SETTLING&&!permitted){stop(LIMIT);return 0;}
        if(uint32_t(now-lastTick)>50){stop(TIMING);return 0;}
        lastTick=now;
        if(state==SETTLING){
            if(uint32_t(now-pausedAt)>=500||!awake)stop(RELEASED);
            return 0;
        }
        if(!awake||uint32_t(now-began)<prepareMs)return 0;
        unsigned elapsed=now-began-prepareMs,mask=0;
        state=jog<0?MOVING:MEASURING;
        if(jog>=0){
            if(elapsed>=60000||count[jog]>=3600||std::abs(measured[jog]-start[jog])>=12){stop(TRAVEL);return 0;}
            if(!externalJog&&uint32_t(now-lastPulse[jog])>=jogPeriodMs){mask=1u<<jog;++count[jog];lastPulse[jog]=now;}
            return mask;
        }
        bool off=false,finished=true;
        double t=blend(double(elapsed)/duration);
        for(unsigned a=0;a<3;++a){
            reference[a]=start[a]+(target[a]-start[a])*t;
            double issued=start[a]+(target[a]>=start[a]?1.0:-1.0)*count[a]/ppd[a];
            off|=std::abs(issued-measured[a])>2.0;
            unsigned budget=unsigned(std::lround(total[a]*t));
            if(count[a]<budget && uint32_t(now-lastPulse[a])>=5){mask|=1u<<a;}
            finished&=count[a]+((mask>>a)&1)>=total[a];
        }
        if(off){if(!bad){bad=true;badSince=now;}else if(uint32_t(now-badSince)>=150){stop(TRACKING);return 0;}}
        else bad=false;
        if(elapsed>duration+2000){stop(TIMING);return 0;}
        if(elapsed>=duration&&finished){
            if(!settlingStarted){settlingStarted=true;settling=now;}
            if(uint32_t(now-settling)>=200){
                bool close=true;for(unsigned a=0;a<3;++a)close&=std::abs(measured[a]-target[a])<=1.0;
                if(close)stop(COMPLETE);
                else if(uint32_t(now-settling)>=1000)stop(TRACKING);
            }
        }
        for(unsigned a=0;a<3;++a)if(mask&(1u<<a)){++count[a];lastPulse[a]=now;}
        return mask;
    }
};
}
