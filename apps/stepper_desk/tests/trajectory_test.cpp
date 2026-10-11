#include "../sketch/trajectory.hpp"
#include "../sketch/limits.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using coordinated::Motion;
using namespace coordinated;
using Angles=std::array<double,3>;
static const Angles zero{0,0,0}, goal{5,-3,1}, ppd{20,32,40};
static const std::array<int,3> signs{-1,1,-1};
static Motion pose(uint32_t now=0) {
    Motion m; assert(m.begin(1,zero,goal,ppd,signs,now)==0); return m;
}
static Angles actual(const Motion &m) {
    Angles a=m.start;
    for(unsigned i=0;i<3;i++) a[i]+=(m.target[i]>=m.start[i]?1.0:-1.0)*m.count[i]/m.ppd[i];
    return a;
}
static void validation() {
    assert(Motion::blend(-1)==0 && Motion::blend(0)==0 && Motion::blend(1)==1 && Motion::blend(2)==1);
    assert(std::abs(Motion::blend(.5)-.5)<1e-12);
    double prior=0; for(int i=0;i<=1000;i++){ double b=Motion::blend(i/1000.0);assert(b>=prior);prior=b; }
    for(int axis=0;axis<3;axis++) {
        Motion m; auto bad=goal;bad[axis]=5.001;
        assert(m.begin(1,zero,bad,ppd,signs,0)==-ERANGE && m.lastID==0);
        bad=zero;bad[axis]=std::numeric_limits<double>::quiet_NaN();
        assert(m.begin(1,bad,goal,ppd,signs,0)==-EINVAL);
        bad=ppd;bad[axis]=0;assert(m.begin(1,zero,goal,bad,signs,0)==-ERANGE);
        bad[axis]=std::numeric_limits<double>::infinity();assert(m.begin(1,zero,goal,bad,signs,0)==-ERANGE);
        auto from=zero,to=zero;from[axis]=LOWER[axis];to[axis]=LOWER[axis]+2;
        assert(m.begin(1,from,to,ppd,signs,0)==0);m.stop();
        to[axis]=LOWER[axis]+1.999;assert(m.begin(2,from,to,ppd,signs,1)==-ERANGE);
        from[axis]=UPPER[axis];to[axis]=UPPER[axis]-2;
        assert(m.begin(2,from,to,ppd,signs,1)==0);m.stop();
        from[axis]=UPPER[axis]+.001;assert(m.begin(3,from,to,ppd,signs,2)==-ERANGE);
    }
    Motion m=pose();assert(m.direction[0]==1 && m.direction[1]==1 && m.direction[2]==1);
    assert(m.begin(2,zero,goal,ppd,signs,0)==-EBUSY);m.stop();
    assert(m.begin(1,zero,goal,ppd,signs,0)==-EINVAL);
    assert(m.begin(0,zero,goal,ppd,signs,0)==-EINVAL);
    assert(m.begin(2,zero,goal,Angles{10000,10000,10000},signs,0)==-ERANGE);
}
static void successfulTrajectory(uint32_t start) {
    Motion m=pose(start);std::array<unsigned,3> seen{};std::array<uint32_t,3> previous{};
    for(uint32_t dt=0;dt<8000 && m.running();dt++) {
        uint32_t now=start+dt;
        if(dt%100==0) assert(m.hold(1,now)==0);
        unsigned mask=m.tick(now,actual(m),true,true,true);
        assert(mask<=7);
        for(unsigned a=0;a<3;a++) if(mask&(1u<<a)) {
            assert(dt>=80);if(seen[a])assert(uint32_t(now-previous[a])>=5);
            previous[a]=now;++seen[a];assert(seen[a]<=m.total[a]);
        }
        assert(seen==m.count);
    }
    assert(m.state==COMPLETE && !m.running());assert(m.count==m.total);
    assert((m.total==std::array<unsigned,3>{100,96,40}));
    auto count=m.count;assert(m.tick(start+9000,goal,true,true,true)==0 && m.count==count);
    assert(m.hold(1,start+9000)==-ESTALE);
}
static void watchdogAndGates() {
    Motion m=pose();for(unsigned t=0;t<80;t++)assert(m.tick(t,zero,true,true,true)==0);
    for(unsigned t=80;t<299;t++)assert(m.tick(t,zero,true,true,false)==0);
    assert(m.hold(2,299)==-ESTALE && m.renewed==0);
    assert(m.hold(1,300)==-ESTALE && m.state==LEASE_LOST);
    assert(m.tick(301,zero,true,true,true)==0);
    m=pose();for(unsigned t=0;t<300;t+=10)m.tick(t,zero,true,true,true);
    assert(m.tick(300,zero,true,true,true)==0 && m.state==LEASE_LOST);
    m=pose();assert(m.tick(50,zero,true,true,true)==0 && m.running());
    assert(m.tick(101,zero,true,true,true)==0 && m.state==TIMING);
    m=pose();assert(m.tick(1,zero,false,true,true)==0 && m.state==SENSOR_FAULT);
    m=pose();assert(m.tick(1,zero,true,false,true)==0 && m.state==LIMIT);
    m=pose(UINT32_MAX-99);assert(m.hold(1,199)==0); // 299 ms through wrap.
    assert(m.hold(1,499)==-ESTALE && m.state==LEASE_LOST);
}
static void tracking() {
    Motion m=pose();Angles far{3,0,0};
    for(unsigned t=0;t<230;t++) {
        if(t%100==0)assert(m.hold(1,t)==0);
        assert(m.tick(t,far,true,true,true)==0);
    }
    assert(m.bad && m.running());
    assert(m.tick(230,far,true,true,true)==0 && m.state==TRACKING);
    m=pose();
    for(unsigned t=0;t<500;t++) {
        if(t%100==0)assert(m.hold(1,t)==0);
        m.tick(t,t>=80&&t<150?far:actual(m),true,true,true);
    }
    assert(m.running() && !m.bad);
    // Final error below the 2-degree running threshold still prevents success.
    m=pose();
    for(unsigned t=0;t<8000&&m.running();t++) {
        if(t%100==0)assert(m.hold(1,t)==0);
        auto measured=actual(m); measured[2]-=1.5;
        m.tick(t,measured,true,true,true);
    }
    assert(m.state==TRACKING);
}
static void terminalStateSurvivesStop() {
    for(State terminal : {IDLE,COMPLETE,RELEASED,LEASE_LOST,SENSOR_FAULT,LIMIT,TRACKING,TIMING,TRAVEL}) {
        Motion m;m.state=terminal;m.count={12,34,56};
        m.stop();m.stop();
        assert(m.state==terminal && (m.count==std::array<unsigned,3>{12,34,56}));
    }
    auto m=pose();m.stop();assert(m.state==RELEASED);
    m=pose();for(unsigned t=0;t<300;t+=10)m.tick(t,zero,true,true,true);
    m.tick(300,zero,true,true,true);assert(m.state==LEASE_LOST);
    m.stop();assert(m.state==LEASE_LOST);
    // A new accepted action, not STOP, clears the latched fault state.
    assert(m.begin(2,zero,goal,ppd,signs,301)==0 && m.state==PREPARING);
}
static void rateAndBacklog() {
    Motion m;const Angles many{500,500,500};
    assert(m.begin(1,zero,Angles{5,5,5},many,signs,0)==0);
    // Slower servicing must emit at most one pulse per axis; never catch up in bursts.
    for(unsigned t=0;t<40000&&m.running();t+=20) {
        if(t%100==0)assert(m.hold(1,t)==0);
        auto before=m.count;unsigned mask=m.tick(t,actual(m),true,true,true);
        for(unsigned a=0;a<3;a++)assert(m.count[a]-before[a]==((mask>>a)&1));
        auto after=m.count;
        assert(m.tick(t,actual(m),true,true,true)==0 && after==m.count);
    }
    assert(m.state==TIMING); // Bound the lag instead of extending a move indefinitely.
    assert(m.count[0]<m.total[0]);
    Motion stationary;assert(stationary.begin(1,zero,zero,ppd,signs,0)==0);
    for(unsigned t=0;t<6000&&stationary.running();t++) {
        if(t%100==0)assert(stationary.hold(1,t)==0);
        assert(stationary.tick(t,zero,true,true,true)==0);
    }
    assert(stationary.state==COMPLETE);
}
static void jogAndSensors() {
    Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,1,0)==0);
    unsigned count=0,last=0;
    for(unsigned t=0;t<=1000;t++) {
        if(t%100==0)assert(m.hold(1,t)==0);
        auto mask=m.tick(t,zero,true,true,true);assert(mask==0||mask==2);
        if(mask){if(count)assert(t-last>=10);last=t;count++;}
    }
    assert(m.count[1]==count && m.count[0]==0 && m.count[2]==0);
    auto moved=zero;moved[1]=12;
    assert(m.tick(1001,moved,true,true,true)==0 && m.state==TRAVEL);
    m=Motion{};assert(m.begin(1,zero,zero,Angles{},signs,0,0,1)==0);
    for(unsigned t=0;t<=60080 && m.running();t++) {
        if(t%100==0)assert(m.hold(1,t)==0);
        m.tick(t,zero,true,true,true);
    }
    assert(m.state==TRAVEL && m.count[0]<=3600);
    m=Motion{};assert(m.begin(1,zero,zero,Angles{},signs,0,3,0)==-EINVAL);
    assert(m.begin(1,zero,zero,Angles{},signs,0,0,2)==-EINVAL);
    joint_limit::Guard guard;assert(guard.configure(0,0,1));
    encoder::Sample sensor;sensor.raw=0;sensor.magnet=0x30;sensor.error=encoder::OK;sensor.sampled=UINT32_MAX-49;
    assert(guard.usable(sensor,50));assert(!guard.usable(sensor,51));
    sensor.sampled=0;sensor.magnet=0x20;assert(guard.usable(sensor,0));
    sensor.magnet=0;assert(!guard.usable(sensor,0));sensor.magnet=0x28;assert(!guard.usable(sensor,0));
}
static void jogRates() {
    for(unsigned rate : {20u,60u,100u,200u}) {
        Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,rate)==0);
        unsigned prior=0,seen=0,period=(1000+rate-1)/rate;
        for(unsigned t=0;t<=1000;++t) {
            if(t%100==0)assert(m.hold(1,t)==0);
            if(m.tick(t,zero,true,true,true)){if(seen)assert(t-prior>=period);prior=t;++seen;}
        }
        assert(seen==1+920/period);
    }
    for(unsigned rate : {0u,19u,201u,UINT32_MAX}) {
        Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,rate)==-ERANGE);
        assert(!m.running()&&m.lastID==0);
    }
    Motion m;assert(m.begin(1,zero,goal,ppd,signs,0,-1,0,0)==0);
}
static void hardwareJogRates() {
    for(unsigned rate : {20u,100u,1333u,2400u}) {
        Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,rate,true)==0);
        for(unsigned t=0;t<200;++t){if(t%50==0)m.hold(1,t);assert(m.tick(t,zero,true,true,true)==0);}
        assert(m.count[0]==0&&m.state==MEASURING&&m.jogRateHz==rate);
        m.count[0]=3600;assert(m.tick(200,zero,true,true,true)==0&&m.state==TRAVEL);
    }
    for(unsigned rate : {0u,19u,2401u,UINT32_MAX}) {
        Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,rate,true)==-ERANGE);
    }
    Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,2400,true)==0);
    assert(m.tick(80,zero,true,true,true)==0&&m.state==TIMING);
    m=Motion{};assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,1333,true)==0);
    for(unsigned t=0;t<300;++t)m.tick(t,zero,true,true,true);
    m.tick(300,zero,true,true,true);assert(m.state==LEASE_LOST);
}
static void poweredSettle() {
    for(unsigned base : {0u,UINT32_MAX-250}) {
        Motion m;assert(m.begin(1,zero,zero,Angles{},signs,base,0,0,1333,true)==0);
        for(unsigned t=0;t<=100;++t)m.tick(base+t,zero,true,true,true);
        m.count[0]=123;
        assert(m.pause(2,base+100)==-ESTALE&&m.state==MEASURING);
        assert(m.pause(1,base+100)==0&&m.state==SETTLING&&m.running());
        assert(m.pause(1,base+101)==-ESTALE&&m.pausedAt==base+100);
        assert(m.begin(2,zero,zero,Angles{},signs,base+101,0,0,1333,true)==-EBUSY);
        for(unsigned t=101;t<600;++t){
            if(t%50==0)assert(m.hold(1,base+t)==0);
            assert(m.tick(base+t,zero,true,false,true)==0);
            assert(m.running()&&m.count[0]==123);
        }
        assert(m.tick(base+600,zero,true,true,true)==0&&m.state==RELEASED&&!m.running());
        assert(m.count[0]==123&&m.hold(1,base+600)==-ESTALE);
    }
    for(unsigned failure=0;failure<4;++failure){
        Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,1333,true)==0);
        for(unsigned t=0;t<=100;++t)m.tick(t,zero,true,true,true);
        assert(m.pause(1,100)==0);
        if(failure==0){m.stop();assert(!m.running());}
        if(failure==1){m.tick(101,zero,false,true,true);assert(m.state==SENSOR_FAULT);}
        if(failure==2){for(unsigned t=101;t<=300;++t)m.tick(t,zero,true,true,true);assert(m.state==LEASE_LOST);}
        if(failure==3){m.tick(151,zero,true,true,true);assert(m.state==TIMING);}
    }
    Motion idle;assert(idle.pause(1,0)==-ESTALE);
}
static void calibrationPreparation() {
    for(unsigned base : {0u,UINT32_MAX-200}) {
        Motion m;assert(m.begin(1,zero,zero,Angles{},signs,base,0,0,1333,true,400)==0);
        for(unsigned t=0;t<400;++t){
            if(t%50==0)assert(m.hold(1,base+t)==0);
            assert(m.tick(base+t,zero,true,true,true)==0);
            assert(m.state==PREPARING&&m.count[0]==0);
        }
        assert(m.tick(base+400,zero,true,true,true)==0&&m.state==MEASURING);
    }
    Motion m;assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,1333,true,400)==0);
    for(unsigned t=0;t<=300;++t)m.tick(t,zero,true,true,true);
    assert(m.state==LEASE_LOST&&m.count[0]==0);
    for(unsigned prep : {0u,79u,501u,UINT32_MAX}) {
        m=Motion{};assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,1333,true,prep)==-ERANGE);
        assert(!m.running());
    }
    m=Motion{};assert(m.begin(1,zero,goal,ppd,signs,0,-1,0,100,false,400)==-ERANGE);
    assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,100,false,400)==-ERANGE);
    assert(m.begin(1,zero,zero,Angles{},signs,0,0,0,1333,true,500)==0);
    m.tick(0,zero,false,true,true);assert(m.state==SENSOR_FAULT);
}
int main() {
    calibrationPreparation();
    poweredSettle();
    hardwareJogRates();
    jogRates();
    validation();successfulTrajectory(0);successfulTrajectory(UINT32_MAX-2000);
    watchdogAndGates();tracking();terminalStateSurvivesStop();rateAndBacklog();jogAndSensors();
    std::cout<<"Coordinated motion: boundaries, counted pulses, lease, timing, wrap, tracking, sensors and jog caps passed\n";
}
