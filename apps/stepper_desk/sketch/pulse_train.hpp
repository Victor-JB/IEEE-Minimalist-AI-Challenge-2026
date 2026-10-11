// Closed-loop STEP trains on all three axes at once. Same one-pulse PWM2
// scheme as counted_jog.hpp: the update ISR counts each completed rising edge
// and is the only thing that restarts the timer, so a late ISR slows output
// and never adds pulses. Rate changes take effect from the next pulse (ARR
// preload). Included after counted_jog.hpp; mutex held by every caller.
struct PulseTrains {
    volatile unsigned period[3]={0,0,0},count[3]={0,0,0};
    volatile bool running[3]={false,false,false};
    volatile uint32_t serviced=0;
};
static PulseTrains trains;
static void setStepCompare(unsigned a,unsigned value){
    if(a==1)LL_TIM_OC_SetCompareCH4(timers[a],value);else LL_TIM_OC_SetCompareCH3(timers[a],value);
}
static void trainInterrupt(unsigned a){
    ++trains.count[a]; // Rollover completed exactly one physical rising edge.
    unsigned period=trains.period[a];
    if(!period||uint32_t(k_uptime_get_32()-trains.serviced)>50){
        trains.running[a]=false;LL_TIM_DisableIT_UPDATE(timers[a]);return;
    }
    LL_TIM_SetAutoReload(timers[a],period-1);setStepCompare(a,period/2);
    LL_TIM_EnableCounter(timers[a]);
}
// rate 0 lets an in-flight pulse finish, then the axis stays stopped.
static int trainSet(unsigned a,unsigned rate,uint32_t now){
    unsigned period=0;
    if(rate){period=(tickHz[a]+rate/2)/rate;if(period<8||period>65536)return -ERANGE;}
    unsigned key=irq_lock();
    trains.serviced=now;trains.period[a]=period;
    if(period&&!trains.running[a]){
        auto *timer=timers[a];
        LL_TIM_DisableCounter(timer);LL_TIM_DisableIT_UPDATE(timer);
        LL_TIM_OC_SetMode(timer,channel(a),LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_SetCounterMode(timer,LL_TIM_COUNTERMODE_UP);
        LL_TIM_SetAutoReload(timer,period-1);setStepCompare(a,period/2);
        LL_TIM_SetCounter(timer,0);
        LL_TIM_SetOnePulseMode(timer,LL_TIM_ONEPULSEMODE_SINGLE);
        LL_TIM_GenerateEvent_UPDATE(timer); // Load preloads before enabling IRQ.
        LL_TIM_ClearFlag_UPDATE(timer);
        LL_TIM_OC_SetMode(timer,channel(a),LL_TIM_OCMODE_PWM2);
        trains.running[a]=true;
        LL_TIM_EnableIT_UPDATE(timer);LL_TIM_EnableCounter(timer);
    }
    irq_unlock(key);return 0;
}
// Immediate stop. Freezes first and counts an already-risen, not yet
// ISR-counted edge, exactly like stopCountedJog().
static void trainStopAll(){
    unsigned key=irq_lock();
    for(unsigned a=0;a<3;++a){
        trains.period[a]=0;
        if(!trains.running[a])continue;
        auto *timer=timers[a];
        LL_TIM_DisableCounter(timer);LL_TIM_DisableIT_UPDATE(timer);
        unsigned compare=a==1?LL_TIM_OC_GetCompareCH4(timer):LL_TIM_OC_GetCompareCH3(timer);
        if(LL_TIM_IsActiveFlag_UPDATE(timer)||LL_TIM_GetCounter(timer)>=compare)++trains.count[a];
        LL_TIM_OC_SetMode(timer,channel(a),LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_ClearFlag_UPDATE(timer);LL_TIM_SetCounter(timer,0);
        trains.running[a]=false;
    }
    irq_unlock(key);
}
static void releaseTrains(){
    if(!trainsOwnTimers)return;
    trainStopAll();trainsOwnTimers=false;
}
static int claimTrains(){
    if(trainsOwnTimers)return 0;
    stopCountedJog();
    unsigned key=irq_lock();int err=connectStepIRQs();
    if(!err)trainsOwnTimers=true;
    irq_unlock(key);return err;
}
static std::array<unsigned,3> trainCounts(){
    unsigned key=irq_lock();
    std::array<unsigned,3> c{trains.count[0],trains.count[1],trains.count[2]};
    irq_unlock(key);return c;
}
