// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
// Closed-loop joint position control. Arrays use physical motor order.
// The AS5600 angle closes the loop; pulses/degree is only a feedforward hint,
// so position-dependent ratios, reversal backlash and lost steps cost time,
// not endpoint accuracy. Portable: no hardware access, tested on the host.
namespace servo {
using Vec=std::array<double,3>;
constexpr double LOWER[3]={-72,-34,-80}, UPPER[3]={79,38,87};
constexpr double MARGIN=2;                   // targets stay this far inside hard limits
constexpr unsigned MAX_RATE=12000, MIN_RATE=20; // STEP pulses/s per joint (closed loop only)
constexpr double MAX_SECONDS=60;
// State values continue the coordinated::State numbering in arm snapshots.
enum State:int { IDLE=0, MOVING=13, HOLDING=14, RECOVERING=15, STALLED=16, STREAMING=17, FAULT=18 };
enum Reason:int { NONE=0, COMPLETE=1, UNSETTLED=2, LEASE=3, SENSOR=4, LIMIT=5, TIMING=6,
                  STALL=7, OBSTRUCTION=8, HALTED=9, RELAXED=10, HOLD_TIMEOUT=11 };

// Quintic p(t) from (p0,v0,a0) to (p1,v1,0) over T seconds.
struct Quintic {
    double c[6]={0,0,0,0,0,0},T=0,end=0;
    void plan(double p0,double v0,double a0,double p1,double v1,double duration){
        T=duration;end=p1;
        const double h=p1-p0,T2=T*T,T3=T2*T,T4=T3*T,T5=T4*T;
        c[0]=p0;c[1]=v0;c[2]=a0/2;
        c[3]=(20*h-(8*v1+12*v0)*T-3*a0*T2)/(2*T3);
        c[4]=(-30*h+(14*v1+16*v0)*T+3*a0*T2)/(2*T4);
        c[5]=(12*h-6*(v1+v0)*T-a0*T2)/(2*T5);
    }
    void eval(double t,double &p,double &v,double &a)const{
        if(t>=T){p=end;v=0;a=0;return;}
        t=std::max(0.0,t);
        p=c[0]+t*(c[1]+t*(c[2]+t*(c[3]+t*(c[4]+t*c[5]))));
        v=c[1]+t*(2*c[2]+t*(3*c[3]+t*(4*c[4]+t*5*c[5])));
        a=2*c[2]+t*(6*c[3]+t*(12*c[4]+t*20*c[5]));
    }
};

struct Settings {
    double kp=8;                          // 1/s: correction speed per degree of error
    double settleTol=0.3,engage=0.25,release=0.1; // deg; hysteresis prevents hunting
    double startSpeed=1.0;                // deg/s a joint may start or stop at instantly
    double accelGain=3;                   // command accel headroom over the profile accel
    double speedGain=1.5;                 // command speed headroom over the profile speed
    uint32_t dirSetupMs=2,settleMs=150,settleTimeoutMs=3000,holdTimeoutMs=60000,leaseMs=300;
    // Stall: commanded pulses that the encoder does not see move the joint.
    uint32_t stallWindowMs=150;unsigned stallMinPulses=80,backlashPulses=400,stallWindows=2;
    double stallEfficiency=0.3,maxFollow=5;uint32_t followMs=300;
    // Recovery: pause powered, back off, retry slower; repeated stall = obstruction.
    double backoffDeg=1,sameStallDeg=2,retryScale=0.5;unsigned maxRetries=2;uint32_t pauseMs=150;
    // Shoulder recovery: fold the elbow to a limit to pull the centre of mass
    // over the shoulder, lift the shoulder, then unfold. NAN = fold the elbow
    // opposite the shoulder's lean; set an angle once the right side is known.
    bool tuck=true;double tuckElbow=NAN;
    // Gear-slack compensation (motor order elbow, shoulder, head): on a direction
    // reversal after a real run, issue this much extra travel at slackRate without
    // counting it as joint motion. Head measured about 0.7-0.8 deg on 2026-10-11;
    // set under that, the loop corrects the rest. 0 disables. Off on all joints
    // until the head's tight spot near +14 deg is fixed and an A/B test shows it helps.
    double slackDeg[3]={0,0,0};unsigned slackRate=2000;double slackMinRunDeg=1.0;
};
constexpr unsigned SHOULDER=1,ELBOW=0;      // motor-order indices
struct JointConfig { int forwardSign=1; double ppd=142.2,vmax=12,amax=40; };

struct Output {
    std::array<unsigned,3> rate{},dir{};
    bool power=false;
};

struct Joint {
    JointConfig cfg;
    Quintic seg;
    double ref=0,refV=0,refA=0,goal=0,meas=0,vCmd=0;
    double ppdEst[2]={142.2,142.2};       // learned per driver direction
    long pulses=0,pulsesAtSample=0;       // issued, signed in joint-angle sense
    unsigned lastCount=0,dir=0,rate=0,sinceReverse=0,slackLeft=0;
    uint32_t sampleMs=0,dirReadyAt=0;
    bool engaged=false,hasSample=false;
    // Stall window
    unsigned windowPulses=0,badWindows=0;double windowStart=0;uint32_t windowMs=0;
    bool following=false;uint32_t followSince=0;
    double angleSign(unsigned d)const{return cfg.forwardSign*(d?-1.0:1.0);}
    double ppd()const{return ppdEst[dir];}
    double estimate()const{
        double moved=double(pulses-pulsesAtSample)/ppd();
        return meas+std::max(-1.0,std::min(1.0,moved));
    }
};

struct Controller {
    Settings set;
    std::array<Joint,3> j;
    State state=IDLE;Reason reason=NONE;
    uint32_t id=0,lastID=0,renewed=0,lastTick=0,segStart=0,holdSince=0,settleAt=0,phaseAt=0;
    double segT=0,speedScale=1,requestSpeed=0,live=1;
    double tau=0;                         // warped segment time; live scale slows the path
    bool segActive=false,settling=false,ticked=false,everStarted=false;
    unsigned retries=0,stalls=0;int stallJoint=-1,lastStallJoint=-1;double stallAngle=0,lastStallAngle=0;
    enum Phase { NO_PHASE, PAUSE, BACKOFF, TUCK, LIFT } phase=NO_PHASE;
    Vec moveGoal{};

    bool moving()const{return state==MOVING||state==STREAMING||state==RECOVERING;}
    bool active()const{return state!=IDLE&&state!=FAULT;}
    bool wantsPower()const{return active();}

    void configure(unsigned a,const JointConfig &c){
        j[a].cfg=c;j[a].ppdEst[0]=j[a].ppdEst[1]=c.ppd;
    }
    static bool validGoal(unsigned a,double g){
        return std::isfinite(g)&&g>=LOWER[a]+MARGIN&&g<=UPPER[a]-MARGIN;
    }
    double clampGoal(unsigned a,double g)const{return std::max(LOWER[a]+MARGIN,std::min(UPPER[a]-MARGIN,g));}

    // Adopt live measurements as the starting reference when not already active.
    void seed(const Vec &measured,uint32_t now){
        for(unsigned a=0;a<3;++a){auto &q=j[a];
            if(!active()){q.meas=measured[a];q.pulsesAtSample=q.pulses;q.ref=measured[a];q.refV=q.refA=0;q.vCmd=0;q.engaged=false;}
        }
        if(!active()){lastTick=now;ticked=true;}
    }
    // Plans one synchronized segment from the current reference state.
    // Duration grows until each joint stays within vmax*vScale and amax*aScale.
    // aScale 0 = streaming: keep the host's horizon and bound only speed.
    int plan(const Vec &goal,const Vec &endVel,double minSeconds,double vScale,double aScale,const Vec &vCap){
        double T=std::max(0.02,minSeconds);
        for(unsigned a=0;a<3&&aScale>0;++a){
            double d=std::abs(goal[a]-j[a].ref),v=vCap[a]*vScale,acc=j[a].cfg.amax*aScale;
            if(v<=0||acc<=0)return -ERANGE;
            T=std::max({T,1.875*d/v,std::sqrt(5.7735*d/acc),std::abs(j[a].refV)/acc});
        }
        for(int attempt=0;;++attempt){
            if(T>MAX_SECONDS)return -ERANGE;
            bool ok=true;
            for(unsigned a=0;a<3;++a){
                Quintic q;q.plan(j[a].ref,j[a].refV,j[a].refA,goal[a],endVel[a],T);
                double v=vCap[a]*vScale*1.05,acc=aScale>0?j[a].cfg.amax*aScale*1.05:INFINITY;
                for(int i=1;i<=48&&ok;++i){double p,qv,qa;q.eval(T*i/48.0-1e-9,p,qv,qa);ok=std::abs(qv)<=v+1e-9&&std::abs(qa)<=acc+1e-9;}
            }
            if(ok||attempt>=12)break;
            T*=1.2;
        }
        for(unsigned a=0;a<3;++a){j[a].seg.plan(j[a].ref,j[a].refV,j[a].refA,goal[a],endVel[a],T);j[a].goal=goal[a];}
        segT=T;tau=0;segActive=true;settling=false;
        return 0;
    }
    Vec caps(double speed)const{
        Vec v;for(unsigned a=0;a<3;++a)v[a]=speed>0?std::min(speed,j[a].cfg.vmax):j[a].cfg.vmax;return v;
    }
    // Synchronized move: every joint arrives together and stops.
    // durationMs 0 = as fast as speed/accel allow; speed 0 = configured vmax.
    int move(uint32_t next,const Vec &goal,uint32_t durationMs,double speed,const Vec &measured,uint32_t now){
        if(!next||next<=lastID)return -EINVAL;
        if(durationMs>uint32_t(MAX_SECONDS*1000)||!std::isfinite(speed)||speed<0||speed>1000)return -EINVAL;
        for(unsigned a=0;a<3;++a)if(!validGoal(a,goal[a]))return -ERANGE;
        for(unsigned a=0;a<3;++a)if(!std::isfinite(measured[a]))return -EINVAL;
        if(state==FAULT)state=IDLE;
        seed(measured,now);
        if(state==RECOVERING){for(auto &q:j){q.refV=q.refA=0;}}
        moveGoal=goal;requestSpeed=speed;speedScale=1;retries=0;stalls=0;stallJoint=lastStallJoint=-1;phase=NO_PHASE;
        int err=plan(goal,Vec{0,0,0},durationMs/1000.0,1,1,caps(speed));
        if(err)return err;
        id=lastID=next;renewed=segStart=now;state=MOVING;reason=NONE;everStarted=true;
        for(unsigned a=0;a<3;++a){resetWindow(a,now);j[a].following=false;}
        return 0;
    }
    // One short-horizon setpoint from a higher-level behavior model. Each one
    // replaces the previous segment and renews the lease. No automatic retry.
    int stream(uint32_t next,const Vec &goal,const Vec &vel,uint32_t horizonMs,const Vec &measured,uint32_t now){
        if(!next||next<=lastID||horizonMs<20||horizonMs>2000)return -EINVAL;
        for(unsigned a=0;a<3;++a){
            if(!validGoal(a,goal[a]))return -ERANGE;
            if(!std::isfinite(vel[a])||std::abs(vel[a])>j[a].cfg.vmax||!std::isfinite(measured[a]))return -EINVAL;
        }
        if(state==FAULT)state=IDLE;
        if(state==STALLED||state==RECOVERING)return -EBUSY; // host must HALT or MOVE to acknowledge
        seed(measured,now);
        // Re-anchor at the current warped time so a new point continues smoothly.
        if(segActive)for(auto &q:j)q.seg.eval(tau,q.ref,q.refV,q.refA);
        int err=plan(goal,vel,horizonMs/1000.0,set.speedGain,0,caps(0));
        if(err)return err;
        if(state!=STREAMING)for(unsigned a=0;a<3;++a){resetWindow(a,now);j[a].following=false;}
        id=lastID=next;renewed=segStart=now;state=STREAMING;reason=NONE;everStarted=true;
        return 0;
    }
    // HOLD from the host. Late renewals halt (-ESTALE); once the move has
    // finished, stalled or faulted, a HOLD for it is acknowledged harmlessly
    // so the host can read the outcome from the status that comes back.
    int renew(uint32_t current,uint32_t now){
        if(current!=id||!everStarted)return -ESTALE;
        if(moving()&&uint32_t(now-renewed)>=set.leaseMs){haltAt(now,LEASE);return -ESTALE;}
        if(moving())renewed=now;
        return 0;
    }
    // Live shared slowdown in (0,1]; the path is unchanged, only its timing.
    int scale(double s){if(!std::isfinite(s)||s<0.05||s>1)return -ERANGE;live=s;return 0;}
    int halt(uint32_t now){if(!active())return -ESTALE;haltAt(now,HALTED);return 0;}
    void relax(Reason why=RELAXED){
        for(auto &q:j){q.vCmd=0;q.rate=0;q.engaged=false;}
        segActive=false;phase=NO_PHASE;
        if(active())reason=why;
        state=IDLE;
    }
    void fail(Reason why){for(auto &q:j){q.vCmd=0;q.rate=0;}segActive=false;phase=NO_PHASE;state=FAULT;reason=why;}

    void holdAt(const Vec &pose){
        for(unsigned a=0;a<3;++a){auto &q=j[a];q.ref=clampGoal(a,pose[a]);q.refV=q.refA=0;q.engaged=false;q.vCmd=0;}
        segActive=false;settling=false;
    }
    void haltAt(uint32_t now,Reason why){
        Vec est;for(unsigned a=0;a<3;++a)est[a]=j[a].estimate();
        holdAt(est);state=HOLDING;reason=why;holdSince=now;phase=NO_PHASE;
    }

    void onSample(unsigned a,double q,uint32_t ms,uint32_t now){
        auto &x=j[a];
        x.meas=q;x.sampleMs=ms;x.pulsesAtSample=x.pulses;x.hasSample=true;
        if(uint32_t(now-x.windowMs)<set.stallWindowMs)return;
        bool judged=moving()&&phase!=PAUSE&&x.windowPulses>=set.stallMinPulses&&x.sinceReverse>=set.backlashPulses;
        if(judged){
            double progress=(q-x.windowStart)*x.angleSign(x.dir);
            double eff=std::max(0.0,progress)*x.cfg.ppd/x.windowPulses;
            x.badWindows=eff<set.stallEfficiency?x.badWindows+1:0;
            if(eff>0.5&&progress>=0.5){
                double r=x.windowPulses/progress,lo=x.cfg.ppd*0.6,hi=x.cfg.ppd*1.6;
                x.ppdEst[x.dir]=0.8*x.ppdEst[x.dir]+0.2*std::max(lo,std::min(hi,r));
            }
        }else if(x.windowPulses<set.stallMinPulses)x.badWindows=0;
        x.windowPulses=0;x.windowStart=q;x.windowMs=now;
    }
    void resetWindow(unsigned a,uint32_t now){auto &x=j[a];x.windowPulses=0;x.windowStart=x.meas;x.windowMs=now;x.badWindows=0;}

    void onStall(unsigned a,uint32_t now){
        Vec est;for(unsigned i=0;i<3;++i)est[i]=j[i].estimate();
        ++stalls;stallJoint=int(a);stallAngle=est[a];
        bool repeat=retries>0&&int(a)==lastStallJoint&&std::abs(est[a]-lastStallAngle)<=set.sameStallDeg;
        if(state==STREAMING||state==RECOVERING||repeat||retries>=set.maxRetries){
            holdAt(est);state=STALLED;reason=repeat?OBSTRUCTION:STALL;
            holdSince=now;phase=NO_PHASE;return;
        }
        lastStallJoint=int(a);lastStallAngle=est[a];
        holdAt(est);state=RECOVERING;phase=PAUSE;phaseAt=now;
        for(unsigned i=0;i<3;++i)resetWindow(i,now);
    }
    double tuckAngle()const{
        double t=set.tuckElbow;
        if(!std::isfinite(t))t=j[SHOULDER].ref>=0?LOWER[ELBOW]+MARGIN:UPPER[ELBOW]-MARGIN;
        return clampGoal(ELBOW,t);
    }
    bool settled(uint32_t now)const{
        if(segActive)return false;
        bool close=true;for(auto &q:j)close&=std::abs(q.ref-q.meas)<=set.settleTol;
        return close||uint32_t(now-phaseAt)>=uint32_t(segT*1000)+1000;
    }
    // Moves only the listed joints toward target; others hold their reference.
    bool stage(Phase next,unsigned joint,double target,double scale,uint32_t now){
        Vec goal;for(unsigned a=0;a<3;++a)goal[a]=j[a].ref;
        goal[joint]=clampGoal(joint,target);
        if(plan(goal,Vec{0,0,0},0,scale,scale,caps(requestSpeed))){fail(TIMING);return false;}
        segStart=phaseAt=now;phase=next;
        for(unsigned i=0;i<3;++i){resetWindow(i,now);j[i].following=false;}
        return true;
    }
    void resume(uint32_t now){
        if(plan(moveGoal,Vec{0,0,0},0,speedScale,speedScale,caps(requestSpeed))){fail(TIMING);return;}
        segStart=now;phase=NO_PHASE;state=MOVING;
        for(unsigned i=0;i<3;++i){resetWindow(i,now);j[i].following=false;}
    }
    void recover(uint32_t now){
        unsigned a=unsigned(stallJoint);
        if(phase==PAUSE&&uint32_t(now-phaseAt)>=set.pauseMs){
            double push=moveGoal[a]-j[a].ref;
            if(std::abs(push)<1e-6)push=j[a].vCmd;
            stage(BACKOFF,a,j[a].ref-(push>=0?1:-1)*set.backoffDeg,set.retryScale*speedScale,now);
        }else if(phase==BACKOFF&&settled(now)){
            ++retries;speedScale*=set.retryScale;
            if(a==SHOULDER&&set.tuck&&std::abs(j[ELBOW].ref-tuckAngle())>set.sameStallDeg)
                stage(TUCK,ELBOW,tuckAngle(),speedScale,now);
            else resume(now);
        }else if(phase==TUCK&&settled(now))stage(LIFT,SHOULDER,moveGoal[SHOULDER],speedScale,now);
        else if(phase==LIFT&&settled(now))resume(now);
    }

    // Called every control tick (about 1 ms). counts are cumulative issued
    // STEP edges per axis; permit bit 2*axis+dir says the limit guard allows
    // that driver direction; ready means drivers are awake past wake delay.
    Output tick(uint32_t now,const Vec &measured,const std::array<uint32_t,3> &sampled,
                const std::array<unsigned,3> &counts,bool healthy,unsigned permit,bool ready){
        Output out;
        double dt=ticked?std::max(0.001,double(uint32_t(now-lastTick))/1000.0):0.001;
        bool late=ticked&&uint32_t(now-lastTick)>50;
        lastTick=now;ticked=true;
        // Count bookkeeping runs in every state so a new move starts in sync.
        for(unsigned a=0;a<3;++a){auto &x=j[a];
            unsigned delta=counts[a]>=x.lastCount?counts[a]-x.lastCount:0;x.lastCount=counts[a];
            unsigned slack=std::min(delta,x.slackLeft);x.slackLeft-=slack;   // takes up gear slack, moves nothing
            x.pulses+=long(delta-slack)*long(x.angleSign(x.dir));x.windowPulses+=delta-slack;x.sinceReverse+=delta;
        }
        if(!active()){
            for(unsigned a=0;a<3;++a)if(std::isfinite(measured[a])){j[a].meas=measured[a];j[a].pulsesAtSample=j[a].pulses;j[a].sampleMs=sampled[a];}
            return out;
        }
        if(late){fail(TIMING);return out;}
        if(!healthy){fail(SENSOR);return out;}
        for(unsigned a=0;a<3;++a)if(sampled[a]!=j[a].sampleMs||!j[a].hasSample)onSample(a,measured[a],sampled[a],now);
        if(moving()&&uint32_t(now-renewed)>=set.leaseMs)haltAt(now,LEASE);
        if((state==HOLDING||state==STALLED)&&uint32_t(now-holdSince)>=set.holdTimeoutMs){relax(HOLD_TIMEOUT);return out;}
        // Reference. The live scale stretches time without changing the path.
        if(segActive){
            tau+=dt*(state==STREAMING?1.0:live);
            for(auto &q:j)q.seg.eval(tau,q.ref,q.refV,q.refA);
            for(auto &q:j){q.refV*=state==STREAMING?1.0:live;}
            if(tau>=segT){segActive=false;settleAt=now;settling=false;}
        }
        if(state==RECOVERING){recover(now);if(state==FAULT)return out;}
        if(state==MOVING&&!segActive){
            bool close=true;for(auto &q:j)close&=std::abs(q.ref-q.meas)<=set.settleTol;
            if(close){if(!settling){settling=true;settleAt=now;}
                else if(uint32_t(now-settleAt)>=set.settleMs){state=HOLDING;reason=COMPLETE;holdSince=now;}}
            else{settling=false;
                if(uint32_t(now-segStart)>=uint32_t(segT*1000/live)+set.settleTimeoutMs){state=HOLDING;reason=UNSETTLED;holdSince=now;}}
        }
        if(state==STREAMING&&!segActive){state=HOLDING;reason=COMPLETE;holdSince=now;}
        // Stall and following-error detection while a path is active.
        if(moving()&&phase!=PAUSE){
            for(unsigned a=0;a<3;++a){auto &x=j[a];
                bool far=std::abs(x.ref-x.estimate())>set.maxFollow;
                if(far&&!x.following){x.following=true;x.followSince=now;}else if(!far)x.following=false;
                if(x.badWindows>=set.stallWindows||(x.following&&uint32_t(now-x.followSince)>=set.followMs)){
                    x.badWindows=0;x.following=false;onStall(a,now);break;
                }
            }
        }
        // Velocity command: feedforward + proportional correction, then
        // speed/accel limits, then reversal through zero with DIR setup.
        for(unsigned a=0;a<3;++a){auto &x=j[a];
            double e=x.ref-x.estimate();
            bool tracking=segActive&&phase!=PAUSE;
            if(tracking||std::abs(e)>set.engage)x.engaged=true;
            else if(std::abs(e)<set.release)x.engaged=false;
            double vt=x.engaged?(tracking?x.refV:0)+set.kp*e:0;
            double vLimit=std::min(MAX_RATE/std::max(x.ppdEst[0],x.ppdEst[1]),x.cfg.vmax*set.speedGain);
            vt=std::max(-vLimit,std::min(vLimit,vt));
            double dv=x.cfg.amax*set.accelGain*dt,v=x.vCmd,next=v+std::max(-dv,std::min(dv,vt-v));
            if(std::abs(v)<=set.startSpeed){
                if(vt==0||v*vt<0)next=0;
                else if(std::abs(next)<set.startSpeed)next=(vt>0?1:-1)*std::min(std::abs(vt),set.startSpeed);
            }
            if(phase==PAUSE)next=0;
            if(!ready)next=0;
            unsigned rate=0;
            if(next!=0){
                unsigned want=next*x.cfg.forwardSign>0?0:1;
                if(want!=x.dir){
                    if(x.rate){next=0;}
                    else{
                        bool ran=x.sinceReverse>=unsigned(set.slackMinRunDeg*x.ppd());
                        x.slackLeft=ran?unsigned(std::lround(set.slackDeg[a]*x.cfg.ppd)):0;
                        x.dir=want;x.dirReadyAt=now+set.dirSetupMs;x.sinceReverse=0;resetWindow(a,now);
                    }
                }
                if(next!=0&&int32_t(now-x.dirReadyAt)>=0){
                    double r=std::abs(next)*x.ppd();
                    rate=r<MIN_RATE?0:unsigned(std::min<double>(MAX_RATE,std::lround(r)));
                    if(x.slackLeft)rate=std::max(rate,std::min(set.slackRate,MAX_RATE));
                }
            }
            if(rate&&!(permit&(1u<<(2*a+x.dir)))){fail(LIMIT);return Output{};}
            x.vCmd=next;x.rate=rate;
            out.rate[a]=rate;out.dir[a]=x.dir;
        }
        out.power=true;
        return out;
    }
    double elapsedMs(uint32_t now)const{return double(uint32_t(now-segStart));}
};
}
