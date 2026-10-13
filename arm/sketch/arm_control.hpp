// Included after legacy output/power helpers. Mutex held by every entry point.
static uint32_t channel(unsigned a){return a==1?LL_TIM_CHANNEL_CH4:LL_TIM_CHANNEL_CH3;}
#include "counted_jog.hpp"
static std::array<double,3> measuredArm(uint32_t now){
    std::array<double,3> result{};
    for(unsigned a=0;a<3;++a){auto &s=sensors[joint_limit::SENSOR_FOR_MOTOR[a]];
        result[a]=limits[a].configured&&limits[a].usable(s,now)?limits[a].angle(s):NAN;}
    return result;
}
#include "pulse_train.hpp"
// Runs inside armTick every control-loop pass (about 1 ms) while owned.
static void servoTick(uint32_t now){
    auto angles=measuredArm(now);bool healthy=true;
    std::array<uint32_t,3> sampled{};unsigned permit=0;
    for(unsigned a=0;a<3;++a){
        const auto &s=sensors[joint_limit::SENSOR_FOR_MOTOR[a]];
        healthy&=std::isfinite(angles[a]);sampled[a]=s.sampled;
        for(unsigned d=0;d<2;++d)if(limits[a].permits(a,s,now,d))permit|=1u<<(2*a+d);
    }
    auto out=servoCtl.tick(now,angles,sampled,trainCounts(),healthy,permit,power.ready(now));
    if(!out.power){releaseTrains();return;}
    if(!trainsOwnTimers)return; // claimTrains() failed; servo commands report it
    for(unsigned a=0;a<3;++a){
        unsigned rate=out.rate[a];
        if(out.dir[a]!=appliedDirection[a]){
            // DIR only changes between pulses; the controller already ramped
            // this axis to zero and waits its DIR setup time.
            if(trains.running[a])rate=0;
            else{digitalWrite(DIR_PINS[a],out.dir[a]?HIGH:LOW);appliedDirection[a]=out.dir[a];}
        }
        if(trainSet(a,rate,now)){servoCtl.fail(servo::TIMING);trainStopAll();return;}
    }
}
static std::vector<int> servoStatus(){
    uint32_t now=millis();auto counts=trainCounts();const auto &c=servoCtl;
    std::vector<int> r={2,int(c.state),int(c.reason),int(c.id),int(c.retries),int(c.stalls),c.stallJoint,
        int(std::lround(c.stallAngle*1000)),int(std::lround(c.segT*1000)),int(now-c.segStart),int(std::lround(c.live*1000))};
    for(unsigned a=0;a<3;++a){const auto &q=c.j[a];
        r.insert(r.end(),{int(std::lround(q.estimate()*1000)),int(std::lround(q.ref*1000)),int(std::lround(q.goal*1000)),
            int(std::lround(q.vCmd*1000)),int(q.rate),int(std::lround(q.ppdEst[0]*1000)),int(std::lround(q.ppdEst[1]*1000)),
            int(q.dir),int(counts[a])});}
    return r;
}
static void countedMode(bool enabled){
    stopCountedJog();
    for(unsigned a=0;a<3;++a){
        LL_TIM_DisableCounter(timers[a]);
        LL_TIM_OC_SetMode(timers[a],channel(a),LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_SetOnePulseMode(timers[a],LL_TIM_ONEPULSEMODE_REPETITIVE);
        LL_TIM_OC_SetMode(timers[a],channel(a),enabled?LL_TIM_OCMODE_FORCED_INACTIVE:LL_TIM_OCMODE_PWM1);
        if(a==1)LL_TIM_OC_SetCompareCH4(timers[a],0);else LL_TIM_OC_SetCompareCH3(timers[a],0);
        LL_TIM_GenerateEvent_UPDATE(timers[a]);LL_TIM_ClearFlag_UPDATE(timers[a]);
        LL_TIM_EnableCounter(timers[a]);applied[a]=0;
    }
}
static void releaseArm(){
    headRoll.release();armMotion.stop();servoCtl.relax();releaseTrains();armOwner=0;armConfigured=0;armVerified=0;
    countedMode(false);syncPower();
}
static void armTick(uint32_t now){
    if(!armOwner){stopCountedJog();return;}
    updateCountedJogCount();
    if(uint32_t(now-armSeen)>2000){releaseArm();return;}
    servoTick(now);
    auto angles=measuredArm(now);bool healthy=true,permitted=true;
    for(unsigned a=0;a<3;++a){
        healthy&=std::isfinite(angles[a]);
        if(armMotion.running()&&armMotion.state!=coordinated::SETTLING&&(armMotion.jog<0?armMotion.total[a]>0:int(a)==armMotion.jog))
            permitted&=limits[a].permits(a,sensors[joint_limit::SENSOR_FOR_MOTOR[a]],now,armMotion.direction[a]);
    }
    unsigned mask=armMotion.tick(now,angles,healthy,permitted,power.ready(now));
    if(armMotion.externalJog){
        if(armMotion.running()&&armMotion.state==coordinated::MEASURING){
            int error=runCountedJog(unsigned(armMotion.jog),armMotion.jogRateHz,now,armMotion.renewed);
            if(error){armMotion.stop(coordinated::TIMING);stopCountedJog();}
        }else stopCountedJog();
    }
    // At most 200 edges/s/joint in this commissioning backend. Delays slow it
    // down; they never create extra edges. IRQ lock bounds each high pulse.
    if(mask){
        unsigned key=irq_lock();
        for(unsigned a=0;a<3;++a)if(mask&(1u<<a))LL_TIM_OC_SetMode(timers[a],channel(a),LL_TIM_OCMODE_FORCED_ACTIVE);
        k_busy_wait(4);
        for(unsigned a=0;a<3;++a)if(mask&(1u<<a))LL_TIM_OC_SetMode(timers[a],channel(a),LL_TIM_OCMODE_FORCED_INACTIVE);
        irq_unlock(key);
    }
}
static std::vector<int> armStatus(int error){
    uint32_t now=millis();auto angles=measuredArm(now);
    // Once a closed-loop move has run, the same 38 fields describe it:
    // servo states 13-18, its reference/goal, issued train edges and DIR.
    const bool servoView=servoCtl.everStarted&&!armMotion.running();
    auto counts=trainCounts();
    std::vector<int> r={1,error,int(armOwner),servoView?int(servoCtl.state):int(armMotion.state),
        servoView?int(servoCtl.id):int(armMotion.id),int(now),
        servoView?int(servoCtl.moving()):int(armMotion.running()),int(power.awake),int(power.enabled),int(power.generation),
        int(armConfigured),int(armVerified),servoView?int(std::lround(servoCtl.segT*1000)):int(armMotion.duration),
        servoView?int(now-servoCtl.segStart):int(now-armMotion.began)};
    for(unsigned a=0;a<3;++a){const auto &s=sensors[joint_limit::SENSOR_FOR_MOTOR[a]];const auto &q=servoCtl.j[a];
        r.insert(r.end(),{std::isfinite(angles[a])?int(std::lround(angles[a]*1000)):INT32_MIN,
            int(std::lround((servoView?q.ref:armMotion.reference[a])*1000)),int(std::lround((servoView?q.goal:armMotion.target[a])*1000)),
            servoView?int(counts[a]):int(armMotion.count[a]),servoView?int(q.dir):int(armMotion.direction[a]),int(now-s.sampled),s.magnet,s.error});}
    return r;
}
static std::vector<int> armLock(){
    k_mutex_lock(&lock,K_FOREVER);
    power.change(0,power.generation);headRoll.release();armMotion.stop();servoCtl.relax();releaseTrains();
    for(unsigned a=0;a<3;++a){motors[a].stop();applyOrFault(a);}
    syncPower();auto result=armStatus(0);k_mutex_unlock(&lock);return result;
}

static std::vector<int> armObserve(){
    k_mutex_lock(&lock,K_FOREVER);auto result=armStatus(0);k_mutex_unlock(&lock);return result;
}
// Read-only head-tilt telemetry (version 3): enabled, target/current mdeg, pulse us, PWM error.
static std::vector<int> armRollStatus(){
    k_mutex_lock(&lock,K_FOREVER);
    std::vector<int> r={3,int(headRoll.enabled),int(std::lround(headRoll.target*1000)),
        int(std::lround(headRoll.current*1000)),int(headRoll.pulseUs()),rollFault};
    k_mutex_unlock(&lock);return r;
}
// Read-only closed-loop telemetry (38 integers, version 2). See servoStatus().
static std::vector<int> armServoStatus(){
    k_mutex_lock(&lock,K_FOREVER);auto result=servoStatus();k_mutex_unlock(&lock);return result;
}

// Owner exclusively reserves all motors. STATUS renews ownership, never motion.
// 0 claim; 1 configure axis; 2 pose; 3 jog; 4 status; 5 hold; 6 stop; 7 release; 8 power; 9 bounded powered settle (motion ID).
// Closed loop: 10 move; 11 stream setpoint; 12 halt and hold; 13 servo axis config;
// 14 servo hold/tuck config; 15 live speed scale. Angles are millidegrees, motor order.
// Head tilt: 16 roll to angle (mdeg) at speed (mdeg/s, 0 = max); 17 roll config (center us,
// us/deg x1000, lower/upper mdeg, sign, max mdeg/s); 18 roll limp. Release and lock also go limp.
// Light: 19 lamp brightness 0-1000 (0 off). Independent of motor power; it stays as set on release.
static std::vector<int> armCommand(std::vector<int> args){
    k_mutex_lock(&lock,K_FOREVER);
    int err=0;uint32_t now=millis();
    if(args.size()<2||args[1]<=0)err=-EINVAL;
    else {
        int op=args[0];uint32_t owner=args[1];
        if(op==0&&args.size()==2){
            bool busy=armOwner!=0;for(unsigned a=0;a<3;++a)busy|=motors[a].running||motors[a].fault;
            if(busy)err=-EBUSY;
            else {for(unsigned a=0;a<3;++a){motors[a].stop();applyOrFault(a);limits[a]=joint_limit::Guard{};}
                armOwner=owner;armSeen=now;armConfigured=armVerified=0;armMotion=coordinated::Motion{};servoCtl=servo::Controller{};countedMode(true);}
        }else if(armOwner!=owner)err=-EACCES;
        else {
            armSeen=now;
            if(op==1&&args.size()==8){
                int a=args[2];
                if(armMotion.running())err=-EBUSY;
                else if(a<0||a>2||args[6]<0||args[6]>10000000||args[7]<0||args[7]>1||
                        (args[7]&&args[6]<1000))err=-EINVAL;
                else if(!limits[a].configure(args[3],args[4],args[5]))err=-EINVAL;
                else {armPPD[a]=args[6]/1000.0;armConfigured|=1u<<a;
                    armVerified=(armVerified&~(1u<<a))|(unsigned(args[7])<<a);}
            }else if((op==2&&args.size()==7)||(op==3&&(args.size()==6||args.size()==7||args.size()==8))){
                bool faulty=false;for(unsigned a=0;a<3;++a)faulty|=motors[a].fault!=0;
                if(faulty)err=-EIO;
                else if(!power.permits(args.back())||armConfigured!=7||(op==2&&armVerified!=7))err=-EPERM;
                else if(scanState==1||scanState==2||servoCtl.active())err=-EBUSY;
                else {
                    releaseTrains();
                    auto angles=measuredArm(now),goals=angles;std::array<int,3> signs{};
                    for(unsigned a=0;a<3;++a){signs[a]=limits[a].forwardSign;if(op==2)goals[a]=args[3+a]/1000.0;}
                    if(op==3&&(args[3]<0||args[3]>2||args[4]<0||args[4]>1))err=-EINVAL;
                    else err=armMotion.begin(args[2],angles,goals,armPPD,signs,now,op==3?args[3]:-1,op==3?args[4]:0,op==3&&args.size()>=7?args[5]:coordinated::JOG_RATE,op==3,op==3&&args.size()==8?args[6]:80);
                    if(!err)for(unsigned a=0;a<3;++a){digitalWrite(DIR_PINS[a],armMotion.direction[a]?HIGH:LOW);appliedDirection[a]=armMotion.direction[a];}
                }
            }else if((op==10&&args.size()==9)||(op==11&&args.size()==11)){
                bool faulty=false;for(unsigned a=0;a<3;++a)faulty|=motors[a].fault!=0;
                if(faulty)err=-EIO;
                else if(!power.permits(args.back())||armConfigured!=7)err=-EPERM;
                else if(scanState==1||scanState==2||armMotion.running())err=-EBUSY;
                else if(!(err=claimTrains())){
                    auto angles=measuredArm(now);servo::Vec goal{},vel{};
                    for(unsigned a=0;a<3;++a){goal[a]=args[3+a]/1000.0;servoCtl.j[a].cfg.forwardSign=limits[a].forwardSign;}
                    if(op==10)err=(args[6]<0||args[7]<0)?-EINVAL:servoCtl.move(args[2],goal,uint32_t(args[6]),args[7]/1000.0,angles,now);
                    else{for(unsigned a=0;a<3;++a)vel[a]=args[6+a]/1000.0;
                        err=args[9]<0?-EINVAL:servoCtl.stream(args[2],goal,vel,uint32_t(args[9]),angles,now);}
                }
            }else if(op==12&&args.size()==2)err=servoCtl.halt(now);
            else if(op==13&&args.size()==6){
                int a=args[2];
                if(servoCtl.moving())err=-EBUSY;
                else if(a<0||a>2||args[3]<20000||args[3]>1000000||args[4]<500||args[4]>60000||args[5]<1000||args[5]>500000)err=-EINVAL;
                else{servo::JointConfig c;c.forwardSign=limits[a].forwardSign?limits[a].forwardSign:1;
                    c.ppd=args[3]/1000.0;c.vmax=args[4]/1000.0;c.amax=args[5]/1000.0;servoCtl.configure(a,c);}
            }else if(op==14&&args.size()==5){
                if(servoCtl.moving())err=-EBUSY;
                // args: hold timeout ms, tuck mode (0 off, 1 auto side, 2 fixed), elbow tuck mdeg.
                else if(args[2]<1000||args[2]>600000||args[3]<0||args[3]>2||args[4]<-70000||args[4]>77000)err=-EINVAL;
                else{servoCtl.set.holdTimeoutMs=args[2];servoCtl.set.tuck=args[3]!=0;
                    servoCtl.set.tuckElbow=args[3]==2?args[4]/1000.0:NAN;}
            }else if(op==15&&args.size()==3)err=servoCtl.scale(args[2]/1000.0);
            else if(op==16&&args.size()==4){
                if(rollFault)err=-EIO;
                else if(!power.enabled)err=-EPERM;
                else err=headRoll.set(args[2]/1000.0,args[3]/1000.0,now);
            }else if(op==17&&args.size()==8)
                err=headRoll.configure(args[2],args[3]/1000.0,args[4]/1000.0,args[5]/1000.0,args[6],args[7]/1000.0);
            else if(op==18&&args.size()==2)headRoll.release();
            else if(op==19&&args.size()==3)err=args[2]<0||args[2]>1000?-EINVAL:setLight(args[2]);
            else if(op==5&&args.size()==3)err=servoCtl.everStarted&&!armMotion.running()?servoCtl.renew(args[2],now):armMotion.hold(args[2],now);
            else if(op==9&&args.size()==3){
                err=armMotion.pause(args[2],now);
                if(!err)stopCountedJog();
            }
            else if(op==6&&args.size()==2){armMotion.stop();servoCtl.relax();releaseTrains();}
            else if(op==7&&args.size()==2)releaseArm();
            else if(op==8&&args.size()==4){err=power.change(args[2],args[3]);if(!err){armMotion.stop();servoCtl.relax();releaseTrains();}}
            else if(op!=4||args.size()!=2)err=-EINVAL;
        }
    }
    syncPower();armTick(now);syncPower();auto r=armStatus(err);k_mutex_unlock(&lock);return r;
}

// Commissioning diagnostic: checks actual input pad levels in alternate-function
// mode. Only available to the exclusive owner with SLEEP physically LOW/locked.
static unsigned stepPadMask(){return ((GPIOB->IDR>>0)&1)|(((GPIOA->IDR>>11)&1)<<1)|(((GPIOB->IDR>>8)&1)<<2);}
static std::vector<int> armSelftest(int token){
    k_mutex_lock(&lock,K_FOREVER);
    if(token<=0||uint32_t(token)!=armOwner||power.enabled||power.awake||armMotion.running()||(GPIOB->IDR&(1u<<4))){
        k_mutex_unlock(&lock);return {1,-EPERM};
    }
    std::vector<int> r={1,0,int(stepPadMask())};
    for(unsigned a=0;a<3;++a){
        unsigned key=irq_lock();
        LL_TIM_OC_SetMode(timers[a],channel(a),LL_TIM_OCMODE_FORCED_ACTIVE);
        k_busy_wait(4);unsigned high=stepPadMask();
        LL_TIM_OC_SetMode(timers[a],channel(a),LL_TIM_OCMODE_FORCED_INACTIVE);
        k_busy_wait(4);unsigned low=stepPadMask();irq_unlock(key);
        r.push_back(high);r.push_back(low);
        if(high!=(1u<<a)||low)r[1]=-EIO;
    }
    if(r[2])r[1]=-EIO;
    if(r[1])for(unsigned a=0;a<3;++a)motors[a].fault=-EIO;
    armSeen=millis();k_mutex_unlock(&lock);return r;
}

// Independent input-pad edge counter while drivers are physically asleep.
// Run before commissioning after any timer backend change. Never moves motors.
static std::vector<int> armPulseSelftest(std::vector<int> args){
    k_mutex_lock(&lock,K_FOREVER);
    if(args.size()!=3||args[0]<=0||uint32_t(args[0])!=armOwner||power.enabled||power.awake||
        armMotion.running()||(GPIOB->IDR&(1u<<4))||args[1]<20||args[1]>2400||args[2]<1||args[2]>200){
        k_mutex_unlock(&lock);return {1,-EPERM};
    }
    std::vector<int> result={1,0,args[1]};
    for(unsigned a=0;a<3;++a){
        unsigned observed=0,previous=stepPadMask(),began=millis();
        int error=runCountedJog(a,args[1],began,began);
        while(!error&&countedJog.count<unsigned(args[2])){
            unsigned pads=stepPadMask(),now=millis();
            if((pads&~previous)&(1u<<a))++observed;
            if(pads&~(1u<<a))error=-EIO;
            previous=pads;
            countedJog.serviced=now;countedJog.renewed=now;
            if(now-began>12000||countedJog.fault)error=-ETIMEDOUT;
        }
        // Freeze before final pad/count reconciliation. Any in-flight rising
        // edge is already present in the input pad and accounted by STOP.
        unsigned key=irq_lock();
        LL_TIM_DisableCounter(timers[a]);
        unsigned pads=stepPadMask();
        if((pads&~previous)&(1u<<a))++observed;
        stopCountedJog();
        unsigned issued=armMotion.count[a];
        irq_unlock(key);
        if(issued!=observed||issued<unsigned(args[2])||stepPadMask())error=-EIO;
        result.insert(result.end(),{int(issued),int(observed),error});
        if(error){result[1]=error;break;}
    }
    countedMode(true);armMotion.count.fill(0);armSeen=millis();
    k_mutex_unlock(&lock);return result;
}
