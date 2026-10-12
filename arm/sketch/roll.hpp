// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
// Head tilt (J5, "head_roll" in the clips): a 270 deg hobby servo (Hiwonder HPS-2027) on D2
// (TIM2 CH2, the only timer no STEP pin uses; 50 Hz). The servo closes its own
// position loop, so this only clamps the angle, slews the command so moves stay
// smooth between 25 Hz host setpoints, and maps degrees to a pulse width.
// Portable: no hardware access, tested on the host.
namespace roll {
constexpr uint32_t PERIOD_US=20000;
constexpr double MIN_US=500,MAX_US=2500;     // 270 deg over 500-2500 us (checked on the arm 2026-10-12)

struct Roll {
    double centerUs=1500,usPerDeg=2000.0/270,lower=-45,upper=45,maxSpeed=180; // deg, deg/s
    int sign=1;
    bool enabled=false;                      // false = no pulses, the servo goes limp
    double target=0,current=0,speed=0;uint32_t last=0;

    int configure(double center,double perDeg,double lo,double hi,int s,double vmax){
        if(!std::isfinite(center)||!std::isfinite(perDeg)||!std::isfinite(lo)||!std::isfinite(hi)||!std::isfinite(vmax))return -EINVAL;
        if(center<MIN_US||center>MAX_US||perDeg<1||perDeg>20||lo>=hi||lo<-135||hi>135||(s!=1&&s!=-1)||vmax<1||vmax>1000)return -EINVAL;
        centerUs=center;usPerDeg=perDeg;lower=lo;upper=hi;sign=s;maxSpeed=vmax;
        target=clamp(target);current=clamp(current);
        return 0;
    }
    double clamp(double a)const{return std::max(lower,std::min(upper,a));}
    // Out-of-range angles are clamped, not refused: a refused command would end the host session.
    // speed 0 = maxSpeed. After limp, pulses resume at the last commanded angle (where an unloaded
    // servo still is) and slew from there, so waking it never snaps the head.
    int set(double angle,double spd,uint32_t now){
        if(!std::isfinite(angle)||!std::isfinite(spd)||spd<0)return -EINVAL;
        target=clamp(angle);speed=spd>0?std::min(spd,maxSpeed):maxSpeed;
        enabled=true;
        last=now;return 0;
    }
    void release(){enabled=false;}
    void tick(uint32_t now){
        double dt=uint32_t(now-last)/1000.0;last=now;
        if(!enabled)return;
        double step=speed*std::min(dt,0.1);
        current+=std::max(-step,std::min(step,target-current));
    }
    unsigned pulseUs()const{
        if(!enabled)return 0;
        double us=centerUs+sign*current*usPerDeg;
        return unsigned(std::lround(std::max(MIN_US,std::min(MAX_US,us))));
    }
};
}
