// Hardware-counted commissioning jogs. Each timer is in one-pulse PWM2 mode:
// LOW -> HIGH at compare -> LOW at rollover, then hardware stops the timer.
// Only the update ISR may start another pulse. A delayed/missed ISR therefore
// cannot conceal any extra pulses. Counts mean STEP rising edges, not motor steps.
static const unsigned jogIRQs[3]={TIM3_IRQn,TIM1_UP_IRQn,TIM4_IRQn};
struct CountedJog {
    volatile unsigned axis=3,count=0;
    volatile bool active=false,fault=false;
    volatile uint32_t serviced=0,renewed=0;
    bool connected=false;
};
static CountedJog countedJog;
// Closed-loop servo trains (pulse_train.hpp) share these timers and IRQs.
static volatile bool trainsOwnTimers=false;
static void trainInterrupt(unsigned a);
static void jogInterrupt(const void *arg){
    unsigned a=unsigned(reinterpret_cast<uintptr_t>(arg));
    auto *timer=timers[a];
    if(!LL_TIM_IsActiveFlag_UPDATE(timer))return;
    LL_TIM_ClearFlag_UPDATE(timer);
    if(trainsOwnTimers){trainInterrupt(a);return;}
    if(!countedJog.active||countedJog.axis!=a)return;
    ++countedJog.count; // Rollover completed exactly one physical rising edge.
    uint32_t now=k_uptime_get_32();
    if(countedJog.count>=3600||uint32_t(now-countedJog.serviced)>50||uint32_t(now-countedJog.renewed)>=300){
        countedJog.active=false;
        countedJog.fault=countedJog.count<3600;
        LL_TIM_DisableIT_UPDATE(timer);
        return;
    }
    LL_TIM_EnableCounter(timer);
}
static void updateCountedJogCount(){
    unsigned key=irq_lock();
    if(countedJog.axis<3){
        armMotion.count[countedJog.axis]=countedJog.count;
        if(countedJog.fault)armMotion.stop(coordinated::TIMING);
    }
    irq_unlock(key);
}
static void stopCountedJog(){
    unsigned key=irq_lock();
    unsigned a=countedJog.axis;
    if(a<3){
        auto *timer=timers[a];
        // Freeze first; capture an already emitted but not ISR-counted edge.
        LL_TIM_DisableCounter(timer);
        LL_TIM_DisableIT_UPDATE(timer);
        unsigned compare=a==1?LL_TIM_OC_GetCompareCH4(timer):LL_TIM_OC_GetCompareCH3(timer);
        if(LL_TIM_IsActiveFlag_UPDATE(timer)||LL_TIM_GetCounter(timer)>=compare){
            ++countedJog.count;
        }
        LL_TIM_OC_SetMode(timer,channel(a),LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_ClearFlag_UPDATE(timer);
        LL_TIM_SetCounter(timer,0);
        armMotion.count[a]=countedJog.count;
        countedJog.axis=3;countedJog.active=false;
    }
    irq_unlock(key);
}
// Caller holds irq_lock. These three timers are exclusively owned by STEP
// PWM. The Arduino build has CONFIG_DYNAMIC_INTERRUPTS; no PWM capture driver.
static int connectStepIRQs(){
    if(countedJog.connected)return 0;
    for(unsigned i=0;i<3;++i){
        int irq=irq_connect_dynamic(jogIRQs[i],2,jogInterrupt,reinterpret_cast<void*>(uintptr_t(i)),0);
        if(irq<0)return irq;
        irq_enable(jogIRQs[i]);
    }
    countedJog.connected=true;return 0;
}
static int runCountedJog(unsigned a,unsigned rate,uint32_t now,uint32_t renewed){
    if(a>2||rate<20||rate>coordinated::MAX_JOG_RATE)return -EINVAL;
    unsigned key=irq_lock();
    if(countedJog.axis<3){
        int result=countedJog.axis!=a||countedJog.fault?-EIO:0;
        countedJog.serviced=now;countedJog.renewed=renewed;
        irq_unlock(key);return result;
    }
    if(trainsOwnTimers){irq_unlock(key);return -EBUSY;}
    {int irq=connectStepIRQs();if(irq<0){irq_unlock(key);return irq;}}
    auto *timer=timers[a];
    LL_TIM_DisableCounter(timer);
    LL_TIM_DisableIT_UPDATE(timer);
    LL_TIM_OC_SetMode(timer,channel(a),LL_TIM_OCMODE_FORCED_INACTIVE);
    unsigned period=(tickHz[a]+rate/2)/rate;
    if(period<8||period>65536){irq_unlock(key);return -ERANGE;}
    LL_TIM_SetCounterMode(timer,LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(timer,period-1);
    if(a==1)LL_TIM_OC_SetCompareCH4(timer,period/2);else LL_TIM_OC_SetCompareCH3(timer,period/2);
    LL_TIM_SetCounter(timer,0);
    LL_TIM_SetOnePulseMode(timer,LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_GenerateEvent_UPDATE(timer); // Load preloads before enabling IRQ.
    LL_TIM_ClearFlag_UPDATE(timer);
    LL_TIM_OC_SetMode(timer,channel(a),LL_TIM_OCMODE_PWM2);
    countedJog.count=0;countedJog.axis=a;countedJog.active=true;countedJog.fault=false;
    countedJog.serviced=now;countedJog.renewed=renewed;
    LL_TIM_EnableIT_UPDATE(timer);
    LL_TIM_EnableCounter(timer);
    irq_unlock(key);return 0;
}
