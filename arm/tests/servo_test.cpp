// Closed-loop controller against a simulated arm: position-dependent ratios,
// backlash, lost steps, slips, obstructions, quantized/late encoder samples.
#include "../sketch/servo.hpp"
#include <cassert>
#include <cstdio>
#include <iostream>
#include <functional>
#include <limits>
using namespace servo;

struct SimJoint {
    double q=0,m=0,backlash=1.8,base=142.2,swing=0,knot=0,loss=0,lossAcc=0,obstacle=NAN;
    double tightFrom=NAN,tightTo=NAN;unsigned tightRate=0;   // tight spot: passable only below tightRate
    int forwardSign=1;
    uint32_t slipFrom=0,slipTo=0;
    double rateAcc=0;unsigned count=0;
    double ppd()const{return base*(1+swing*std::tanh((q-knot)/5));}
    void place(double angle){q=m=angle;}
    void step(unsigned rate,unsigned dir,uint32_t now){
        rateAcc+=rate/1000.0;unsigned n=unsigned(rateAcc);rateAcc-=n;count+=n;
        for(unsigned i=0;i<n;++i){
            if(now>=slipFrom&&now<slipTo)continue;           // rotor slipping: no motion
            double s=forwardSign*(dir?-1.0:1.0);
            if(loss>0&&s>0){lossAcc+=loss;if(lossAcc>=1){lossAcc-=1;continue;}} // load-dependent lost steps
            double nm=m+s/ppd(),nq=q;
            if(nm-nq>backlash/2)nq=nm-backlash/2;else if(nq-nm>backlash/2)nq=nm+backlash/2;
            // Too fast for the torque needed here; inside the gear slack the motor turns freely.
            if(nq!=q&&q>=tightFrom&&q<=tightTo&&rate>tightRate)continue;
            if(std::isfinite(obstacle)&&((s>0&&q<=obstacle&&nq>obstacle)||(s<0&&q>=obstacle&&nq<obstacle))){
                nq=obstacle;continue;                          // blocked output stalls the motor
            }
            m=nm;q=nq;
        }
    }
};

struct Rig {
    Controller c;std::array<SimJoint,3> s;uint32_t now=1000;
    std::array<uint32_t,3> sampled{};Vec reading{};
    bool powered=false,ready=false,healthy=true;uint32_t wokeAt=0;unsigned permit=0x3f,noise=1;
    unsigned maxRate=0;Output last;std::function<void(Rig&)> each;
    Rig(){
        int fs[3]={-1,1,-1};
        for(unsigned a=0;a<3;++a){s[a].forwardSign=fs[a];JointConfig cfg;cfg.forwardSign=fs[a];c.configure(a,cfg);}
        // Motor order: elbow, shoulder, head. Last documented pose.
        s[0].place(63.281);s[1].place(-12.041);s[2].place(4.307);
        // Ratios measured in commissioning: elbow 131.6-148.5, shoulder 119-173.
        s[0].base=140;s[0].swing=0.06;s[0].knot=61;s[1].base=146;s[1].swing=0.18;s[1].knot=-7;s[2].base=143.1;
        for(unsigned a=0;a<3;++a){sampled[a]=now-a*3;reading[a]=sense(a);}
    }
    double sense(unsigned a){
        double raw=std::round(s[a].q*4096/360.0);
        noise=noise*1103515245u+12345u;if((noise>>16)%7==0)raw+=((noise>>8)&1)?1:-1;
        return raw*360/4096.0;
    }
    Vec angles()const{return Vec{s[0].q,s[1].q,s[2].q};}
    void tick(bool renew=true){
        if(each)each(*this);
        for(unsigned a=0;a<3;++a)if(uint32_t(now-sampled[a])>=9){sampled[a]=now;reading[a]=sense(a);}
        std::array<unsigned,3> counts{s[0].count,s[1].count,s[2].count};
        last=c.tick(now,reading,sampled,counts,healthy,permit,ready);
        for(unsigned a=0;a<3;++a){s[a].step(last.rate[a],last.dir[a],now);maxRate=std::max(maxRate,last.rate[a]);}
        if(last.power&&!powered){powered=true;wokeAt=now;}
        if(!last.power)powered=false;
        ready=powered&&uint32_t(now-wokeAt)>=3;
        if(renew&&now%25==0&&c.moving())c.renew(c.id,now);
        ++now;
    }
    void run(uint32_t ms,bool renew=true){for(uint32_t i=0;i<ms;++i)tick(renew);}
    // Joint-order convenience: shoulder, elbow, head -> motor order.
    static Vec motor(double sh,double el,double hd){return Vec{el,sh,hd};}
    int move(const Vec &goal,uint32_t duration=0,double speed=0){
        static uint32_t next=0;return c.move(++next,goal,duration,speed,reading,now);
    }
    uint32_t untilDone(uint32_t limit=60000){
        uint32_t began=now;
        while(c.moving()&&now-began<limit)tick();
        return now-began;
    }
    double worst(const Vec &goal)const{double w=0;for(unsigned a=0;a<3;++a)w=std::max(w,std::abs(s[a].q-goal[a]));return w;}
};

static void quinticBoundaries(){
    Quintic q;q.plan(1,2,3,10,-1,2);double p,v,a;
    q.eval(0,p,v,a);assert(std::abs(p-1)<1e-9&&std::abs(v-2)<1e-9&&std::abs(a-3)<1e-9);
    q.eval(2-1e-9,p,v,a);assert(std::abs(p-10)<1e-6&&std::abs(v+1)<1e-5&&std::abs(a)<1e-4);
    q.eval(5,p,v,a);assert(p==10&&v==0&&a==0);
}

static void validation(){
    Rig r;
    assert(r.move(Rig::motor(37,0,0))==-ERANGE);           // inside the 2 degree margin
    assert(r.move(Rig::motor(0,-71,0))==-ERANGE);
    assert(r.move(Rig::motor(NAN,0,0))==-ERANGE);
    assert(r.c.move(0,Rig::motor(0,60,0),0,0,r.reading,r.now)==-EINVAL);
    assert(r.c.move(5,Rig::motor(-12,63,4),0,-1,r.reading,r.now)==-EINVAL);
    assert(r.c.move(5,Rig::motor(-12,63,4),0,0,r.reading,r.now)==0);
    assert(r.c.move(5,Rig::motor(-12,63,4),0,0,r.reading,r.now)==-EINVAL); // ids must increase
    Vec bad=r.reading;bad[1]=NAN;assert(r.c.move(6,Rig::motor(-12,63,4),0,0,bad,r.now)==-EINVAL);
    assert(r.c.stream(7,Rig::motor(-12,63,4),Vec{0,0,0},10,r.reading,r.now)==-EINVAL);
    assert(r.c.stream(7,Rig::motor(-12,63,4),Vec{0,50,0},100,r.reading,r.now)==-EINVAL);
    assert(r.c.scale(0)==-ERANGE&&r.c.scale(1.5)==-ERANGE&&r.c.scale(0.5)==0);
}

// Every joint reaches its target within 0.3 degrees at several speeds, with
// the controller's 142.2 pulses/degree guess wrong by up to about 20%.
static void accuracyAcrossSpeeds(){
    const Vec goals[]={Rig::motor(-2,50,15),Rig::motor(-25,70,-10),Rig::motor(10,30,40),Rig::motor(-12,63,4)};
    for(double speed:{3.0,8.0,12.0}){
        Rig r;double worst=0;
        for(const auto &g:goals){
            assert(r.move(g,0,speed)==0);
            uint32_t took=r.untilDone();
            assert(r.c.state==HOLDING&&r.c.reason==COMPLETE);
            assert(took<=uint32_t(r.c.segT*1000)+1500);
            r.run(300);
            worst=std::max(worst,r.worst(g));
            assert(r.worst(g)<=0.3);
        }
        assert(r.maxRate<=MAX_RATE);
        std::printf("  speed %4.1f deg/s: worst endpoint error %.3f deg\n",speed,worst);
    }
}

static void durationIsHonoredAndSynchronized(){
    Rig r;Vec g=Rig::motor(-6,58,8);
    assert(r.move(g,4000)==0);assert(std::abs(r.c.segT-4.0)<1e-9);
    // Halfway, every joint is roughly halfway: joints move together.
    Vec start=r.angles();r.run(2000);
    for(unsigned a=0;a<3;++a){double frac=(r.s[a].q-start[a])/(g[a]-start[a]);assert(frac>0.3&&frac<0.7);}
    uint32_t rest=r.untilDone();assert(r.c.reason==COMPLETE&&rest<3500);
    // A move faster than the joints allow is stretched, never sped up.
    assert(r.move(Rig::motor(30,-30,60),500)==0);assert(r.c.segT>500/1000.0*4);
}

static void reversalBacklash(){
    Rig r;r.s[2].backlash=1.82;r.s[1].backlash=0.8;r.s[0].backlash=0.5;
    for(int i=0;i<6;++i){
        Vec g=i%2?Rig::motor(-12,63,4):Rig::motor(-8,59,9);
        assert(r.move(g,0,12)==0);r.untilDone();r.run(200);
        assert(r.c.reason==COMPLETE&&r.worst(g)<=0.3);
    }
}

// Head nods at 30 deg/s through 1 deg of gear slack: without compensation the
// reversal lag stays high; compensating 0.6 deg cuts it and keeps endpoints exact.
static double nodLag(double slackDeg){
    Rig r;r.s[2].backlash=1.0;r.c.set.slackDeg[2]=slackDeg;
    JointConfig cfg;cfg.forwardSign=-1;cfg.vmax=60;cfg.amax=500;r.c.configure(2,cfg);
    uint32_t id=200;double worst=0;
    for(int k=0;k<300;++k){
        double t=k*0.02,w=2*M_PI*0.5,amp=9.5,ahead=t+0.12;   // 30 deg/s peak
        Vec goal=Rig::motor(-12.041,63.281,4.307+amp*std::sin(w*ahead));
        Vec vel{0,0,amp*w*std::cos(w*ahead)};
        assert(r.c.stream(++id,goal,vel,120,r.reading,r.now)==0);
        r.run(20,false);
        if(k>50)worst=std::max(worst,std::abs(r.s[2].q-(4.307+amp*std::sin(w*(t+0.02)))));
    }
    assert(r.c.state!=STALLED);
    return worst;
}
static void headSlackCompensation(){
    double off=nodLag(0),on=nodLag(0.6);
    std::printf("  head nod 30 deg/s, 1 deg slack: worst lag %.2f deg uncompensated, %.2f compensated\n",off,on);
    assert(on<off*0.8);
    Rig r;r.s[2].backlash=1.0;r.c.set.slackDeg[2]=0.6;     // compensated reversals still land exactly
    for(int i=0;i<6;++i){Vec g=i%2?Rig::motor(-12,63,4):Rig::motor(-12,63,12);
        assert(r.move(g,0,30)==0);r.untilDone();r.run(200);assert(r.c.reason==COMPLETE&&r.worst(g)<=0.3);}
}

static void holdsAgainstDisturbance(){
    Rig r;Vec g=Rig::motor(0,55,0);assert(r.move(g)==0);r.untilDone();
    assert(r.c.state==HOLDING&&r.last.power);
    r.s[1].q-=1.2;r.s[1].m-=1.2;                           // shoulder knocked/sagged 1.2 degrees
    r.run(1000);
    assert(r.c.state==HOLDING&&std::abs(r.s[1].q-g[1])<=0.3);
    r.c.set.holdTimeoutMs=5000;r.run(5000);
    assert(r.c.state==IDLE&&r.c.reason==HOLD_TIMEOUT&&!r.last.power);
}

static void leaseLossHoldsInPlace(){
    Rig r;assert(r.move(Rig::motor(15,30,30),0,12)==0);
    r.run(500);r.run(400,false);
    assert(r.c.state==HOLDING&&r.c.reason==LEASE&&r.last.power);
    Vec where=r.angles();r.run(800,false);
    for(unsigned a=0;a<3;++a)assert(std::abs(r.s[a].q-where[a])<1.0);
    assert(r.c.renew(r.c.id,r.now)==0&&r.c.state==HOLDING); // acknowledged, not resumed
    assert(r.c.renew(r.c.id+1,r.now)==-ESTALE);
}

static void loadLossStillReachesTarget(){
    Rig r;r.s[1].loss=0.35;                                // 35% of shoulder lifting steps lost
    Vec g=Rig::motor(10,63,4);assert(r.move(g,0,8)==0);r.untilDone();r.run(300);
    assert(r.c.reason==COMPLETE&&r.worst(g)<=0.3&&r.c.stalls==0);
}

static void transientSlipRecovers(){
    Rig r;Vec g=Rig::motor(8,63,4);assert(r.move(g,0,10)==0);
    r.s[1].slipFrom=r.now+1500;r.s[1].slipTo=r.now+2300;
    r.untilDone();r.run(300);
    assert(r.c.stalls==1&&r.c.retries==1&&r.c.stallJoint==1);
    assert(r.c.reason==COMPLETE&&r.worst(g)<=0.3);
    std::printf("  transient slip: stalled at %.2f deg, recovered with %u retry\n",r.c.stallAngle,r.c.retries);
}

// Shoulder too weak with the elbow extended: recovery folds the elbow to its
// limit, lifts the shoulder, then unfolds the elbow to the commanded pose.
static void shoulderRecoversByTuckingElbow(){
    Rig r;double folded=0;
    r.each=[&](Rig &x){x.s[1].loss=x.s[0].q<70?1.0:0.1;folded=std::max(folded,x.s[0].q);};
    Vec g=Rig::motor(8,63,4);assert(r.move(g,0,10)==0);
    r.untilDone(60000);r.each=nullptr;r.run(300);
    std::printf("  shoulder tuck: %u stall, %u retry, elbow folded to %.1f deg, final error %.3f deg\n",r.c.stalls,r.c.retries,folded,r.worst(g));
    assert(r.c.reason==COMPLETE&&r.c.stalls==1&&r.c.retries==1&&folded>=75&&r.worst(g)<=0.3);
    // Disabled: an unrecoverable shoulder is reported instead.
    Rig n;n.c.set.tuck=false;n.each=[](Rig &x){x.s[1].loss=x.s[0].q<70?1.0:0.1;};
    assert(n.move(g,0,10)==0);n.untilDone(60000);
    assert(n.c.state==STALLED&&n.c.stallJoint==1);
}

static void obstructionStopsPushing(){
    Rig r;r.s[1].obstacle=0;Vec g=Rig::motor(15,63,4);assert(r.move(g,0,10)==0);
    r.untilDone(30000);
    assert(r.c.state==STALLED&&r.c.reason==OBSTRUCTION&&r.c.stallJoint==1);
    assert(r.c.retries<=r.c.set.maxRetries&&r.c.stalls<=3);
    assert(std::abs(r.c.stallAngle-0)<=2);
    // Holding where it stopped, not pushing into the obstacle.
    unsigned before=r.s[1].count;r.run(2000,false);
    assert(r.c.state==STALLED&&r.s[1].count-before<200);
    std::printf("  obstruction: %u stalls, %u retries, held at %.2f deg\n",r.c.stalls,r.c.retries,r.s[1].q);
    // A fresh MOVE away from the obstacle is accepted and succeeds.
    r.s[1].obstacle=NAN;Vec back=Rig::motor(-10,63,4);assert(r.move(back)==0);r.untilDone();
    assert(r.c.reason==COMPLETE);
}

static void streamingTracksAndStops(){
    Rig r;uint32_t id=100;double worst=0;
    // Head sweeps +/-10 degrees at 0.15 Hz (9.4 deg/s peak) from 50 Hz setpoints, 100 ms horizon.
    for(int k=0;k<500;++k){
        // Setpoint = where the head should be one horizon (100 ms) from now.
        double t=k*0.02,w=2*M_PI*0.15,ahead=t+0.1;
        double hd=4.307+10*std::sin(w*ahead),vel=10*w*std::cos(w*ahead);
        Vec goal=r.reading;goal[2]=hd;Vec vel3{0,0,-0.0};vel3[2]=std::max(-11.9,std::min(11.9,vel));
        goal[0]=63.281;goal[1]=-12.041;
        assert(r.c.stream(++id,goal,vel3,100,r.reading,r.now)==0);
        r.run(20,false);
        if(k>50){double target=4.307+10*std::sin(w*(t+0.02));worst=std::max(worst,std::abs(r.s[2].q-target));}
    }
    std::printf("  streaming sine: worst lag error %.2f deg\n",worst);
    assert(worst<1.5);
    r.run(400,false);
    assert(r.c.state==HOLDING&&r.last.power);
    // A joint that won't move at all: grinds maxGrinds times, then stops and reports.
    r.s[2].slipFrom=r.now;r.s[2].slipTo=r.now+100000;
    for(int k=0;k<400&&r.c.state!=STALLED;++k){
        // Capped well inside the head's range: the grinds take ~4.5 s, and a goal past the limit
        // would be refused, letting the lease lapse to HOLDING before the last stall.
        Vec goal=Rig::motor(-12.041,63.281,std::min(40.0,4.307+0.4*k));
        r.c.stream(++id,goal,Vec{0,0,0},100,r.reading,r.now);r.run(20,false);
    }
    assert(r.c.state==STALLED&&r.c.reason==STALL&&r.c.retries==0&&r.c.stalls==r.c.set.maxGrinds+1);
    assert(r.c.stream(++id,Rig::motor(-12,63,4),Vec{0,0,0},100,r.reading,r.now)==-EBUSY);
}

static void streamingGrindsThroughTightSpot(){
    // Head tight spot (like the real one near +14 deg): passable only below ~4 deg/s.
    Rig r;r.s[2].tightFrom=11;r.s[2].tightTo=15;r.s[2].tightRate=600;
    uint32_t id=0;double head=4.307;
    for(int k=0;k<400;++k){
        head=std::min(20.0,head+0.16);                     // ~8 deg/s up through the spot
        assert(r.c.stream(++id,Rig::motor(-12.041,63.281,head),Vec{0,0,0},100,r.reading,r.now)==0);
        r.run(20,false);
    }
    r.run(400,false);
    std::printf("  tight spot: %u stall(s), ground through to %.2f deg\n",r.c.stalls,r.s[2].q);
    assert(r.c.stalls>=1&&r.c.state==HOLDING&&std::abs(r.s[2].q-20)<1);
}

// Lumen's hold drift on the real head: 25 Hz setpoints wandering +/-1.5 deg around +13 deg,
// a new random target every 2-5 s, smoothed over 0.6 s. The head has ~2.5 deg of gear slack
// and a tight spot near +14 deg that loses steps above ~4 deg/s. Each slow reversal winds the
// loop up while the motor crosses the slack, then slams the gear in fast and loses steps: on
// the arm (2026-10-12) a FAULT every ~30 s with drift on. jam = an obstacle the drift pushes
// into (a real stall). worst = worst tracking error.
struct DriftResult { unsigned stalls; bool stalled; double worst; };
static DriftResult holdDrift(double band,unsigned maxGrinds,bool jam=false){
    Rig r;r.s[2].backlash=2.5;r.s[2].tightFrom=11;r.s[2].tightTo=15;r.s[2].tightRate=600;
    r.c.set.backlashDeg[2]=band;r.c.set.maxGrinds=maxGrinds;
    const double base=13;uint32_t id=0,seed=7;
    auto rnd=[&seed]{seed=seed*1103515245u+12345u;return double((seed>>8)&0xffff)/65535.0;};
    for(int k=0;k<100;++k){                                 // stream up to the hold pose
        double h=4.307+(base-4.307)*k/99.0;
        assert(r.c.stream(++id,Rig::motor(-12.041,63.281,h),Vec{0,0,0},120,r.reading,r.now)==0);r.run(40,false);
    }
    unsigned before=r.c.stalls;
    if(jam)r.s[2].obstacle=r.s[2].q-0.3;                   // something wedged just below the head
    double target=0,s1=0,s2=0,next=0,worst=0,last=base;bool stalled=false;
    for(int k=0;k<1500;++k){                                // 60 s
        double t=k*0.04;
        if(t>=next){target=1.5*(2*rnd()-1);next=t+2+3*rnd();}
        s1+=(target-s1)*0.04/0.6;s2+=(s1-s2)*0.04/0.6;     // HOLD_DRIFT_SMOOTH_S, two stages
        int e=r.c.stream(++id,Rig::motor(-12.041,63.281,base+s2),Vec{0,0,0},120,r.reading,r.now);
        if(e==-EBUSY){stalled=true;break;}                  // STALLED: refused until the host HALTs
        assert(e==0);
        r.run(40,false);
        if(k>25&&!jam)worst=std::max(worst,std::abs(r.s[2].q-last));
        last=base+s2;
    }
    return {r.c.stalls-before,stalled||r.c.state==STALLED,worst};
}
static void holdDriftThroughBacklash(){
    // Before: no slack band, no gentle engagement. Without grinds (the firmware that was on the
    // arm) a reversal stall is a STALLED; with grinds it still stalls every few seconds.
    DriftResult old0=holdDrift(0,0),old=holdDrift(0,3);
    assert(old0.stalled&&old.stalls>=3);
    // Now (default 2.5 deg band on the head): reversals cross the slack, engage gently, no stall.
    Settings defaults;assert(defaults.backlashDeg[2]>=2.0);
    DriftResult now0=holdDrift(defaults.backlashDeg[2],0),now=holdDrift(defaults.backlashDeg[2],3);
    assert(!now0.stalled&&now0.stalls==0&&!now.stalled&&now.stalls==0);
    assert(now.worst<1.5);
    // A real jam during the same hold is still detected: every push into it stalls (grinds,
    // which reset when the drift turns away), and without grinds it is STALLED.
    DriftResult jam=holdDrift(defaults.backlashDeg[2],3,true),jam0=holdDrift(defaults.backlashDeg[2],0,true);
    assert(jam.stalls>=3&&jam0.stalled);
    std::printf("  hold drift, 2.5 deg slack: %u stalls before (STALLED without grinds: %s), %u now, "
                "worst error %.2f deg; jam: %u stalls (STALLED without grinds: %s)\n",old.stalls,
                old0.stalled?"yes":"no",now.stalls,now.worst,jam.stalls,jam0.stalled?"yes":"no");
}

static void liveScaleSlowsPath(){
    Rig a,b;Vec g=Rig::motor(5,45,20);
    assert(a.move(g)==0);uint32_t fast=a.untilDone();
    assert(b.move(g)==0);b.c.scale(0.5);uint32_t slow=b.untilDone();
    assert(slow>fast*1.7&&b.c.reason==COMPLETE&&b.worst(g)<=0.3);
}

static void faultsSleep(){
    Rig r;assert(r.move(Rig::motor(5,45,20))==0);r.run(300);
    r.healthy=false;r.tick();assert(r.c.state==FAULT&&r.c.reason==SENSOR&&!r.last.power);
    Rig l;assert(l.move(Rig::motor(5,45,20))==0);l.run(100);l.permit=0;l.run(5);
    assert(l.c.state==FAULT&&l.c.reason==LIMIT&&!l.last.power);
    Rig t;assert(t.move(Rig::motor(5,45,20))==0);t.run(100);t.now+=80;t.tick();
    assert(t.c.state==FAULT&&t.c.reason==TIMING);
    Rig h;assert(h.move(Rig::motor(5,45,20))==0);h.run(100);assert(h.c.halt(h.now)==0&&h.c.state==HOLDING&&h.c.reason==HALTED);
    h.c.relax();h.tick();assert(h.c.state==IDLE&&!h.last.power);
}

static void reversalWaitsForZeroRate(){
    Rig r;assert(r.move(Rig::motor(-12,63,14),0,12)==0);
    std::array<unsigned,3> dir{},rate{};bool flipped=false;
    r.run(600);dir=r.last.dir;rate=r.last.rate;assert(rate[2]>0);
    assert(r.move(Rig::motor(-12,63,-5),0,12)==0);
    for(int i=0;i<3000;++i){
        r.tick();
        if(r.last.dir[2]!=dir[2]){assert(rate[2]==0&&r.last.rate[2]==0);flipped=true;}
        dir=r.last.dir;rate=r.last.rate;
    }
    assert(flipped);
}

int main(){setvbuf(stdout,nullptr,_IONBF,0);
    quinticBoundaries();validation();
    std::puts("accuracy:");accuracyAcrossSpeeds();
    durationIsHonoredAndSynchronized();reversalBacklash();holdsAgainstDisturbance();
    leaseLossHoldsInPlace();loadLossStillReachesTarget();
    std::puts("recovery:");transientSlipRecovers();shoulderRecoversByTuckingElbow();obstructionStopsPushing();
    std::puts("streaming:");streamingTracksAndStops();streamingGrindsThroughTightSpot();headSlackCompensation();holdDriftThroughBacklash();
    liveScaleSlowsPath();faultsSleep();reversalWaitsForZeroRate();
    std::puts("servo tests passed");
}
