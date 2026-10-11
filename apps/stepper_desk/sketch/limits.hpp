// SPDX-License-Identifier: MIT
#pragma once
#include <cmath>
#include <cstdint>
#include "sensors.hpp"
namespace joint_limit {
constexpr unsigned SENSOR_FOR_MOTOR[3] = {1,0,2};
constexpr int MIN_MDEG[3] = {-72000,-45000,-80000};
constexpr int MAX_MDEG[3] = {79000,38000,87000};
constexpr uint32_t MAX_SENSOR_AGE_MS = 100;
enum State { READY=0, UNCONFIGURED=1, SENSOR_UNAVAILABLE=2, LOWER=3, UPPER=4 };
struct Guard {
    bool configured=false, reversed=false;
    int zeroUdeg=0, forwardSign=0;
    bool configure(int zero, int reverse, int forward) {
        if (zero < -180000000 || zero > 180000000 || reverse < 0 || reverse > 1 || (forward != -1 && forward != 1)) return false;
        zeroUdeg=zero; reversed=reverse; forwardSign=forward; configured=true; return true;
    }
    static bool usable(const encoder::Sample &s, uint32_t now) {
        return s.error==encoder::OK && s.raw>=0 && s.raw<=4095 &&
               (s.magnet & 0x28)==0x20 && uint32_t(now-s.sampled)<=MAX_SENSOR_AGE_MS;
    }
    double angle(const encoder::Sample &s) const {
        double value=(double(s.raw)*360.0/4096.0-double(zeroUdeg)/1000000.0)*(reversed ? -1 : 1);
        value=std::fmod(value+180.0,360.0);
        if (value<0) value+=360.0;
        return value-180.0;
    }
    State state(unsigned motor, const encoder::Sample &s, uint32_t now) const {
        if (!configured) return UNCONFIGURED;
        if (!usable(s,now)) return SENSOR_UNAVAILABLE;
        const double degrees=angle(s);
        if (degrees<=double(MIN_MDEG[motor])/1000.0) return LOWER;
        if (degrees>=double(MAX_MDEG[motor])/1000.0) return UPPER;
        return READY;
    }
    bool permits(unsigned motor, const encoder::Sample &s, uint32_t now, unsigned direction) const {
        if (direction>1) return false;
        const State condition=state(motor,s,now);
        if (condition==UNCONFIGURED || condition==SENSOR_UNAVAILABLE) return false;
        const int sign=direction ? -forwardSign : forwardSign;
        return condition==READY || (condition==LOWER && sign>0) || (condition==UPPER && sign<0);
    }
};
}
