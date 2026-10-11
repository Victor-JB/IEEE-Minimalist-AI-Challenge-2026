#include <cassert>
#include <cstdint>
#include <iostream>
#include "../sketch/trajectory.hpp"
struct Timer {unsigned counter=0,arr=999,compare=500,mode=0;bool enabled=false,update=false,interrupt=false;};
static Timer storage[3];static Timer* timers[]={&storage[0],&storage[1],&storage[2]};
static unsigned tickHz[]={1000000,1000000,1000000};
static coordinated::Motion armMotion;
static uint32_t nowMs;
static unsigned channel(unsigned a){return a;}
constexpr unsigned TIM3_IRQn=0,TIM1_UP_IRQn=1,TIM4_IRQn=2;
constexpr unsigned LL_TIM_OCMODE_FORCED_INACTIVE=0,LL_TIM_OCMODE_PWM2=1;
constexpr unsigned LL_TIM_COUNTERMODE_UP=0,LL_TIM_ONEPULSEMODE_SINGLE=1;
static unsigned irq_lock(){return 0;}static void irq_unlock(unsigned){}
static uint32_t k_uptime_get_32(){return nowMs;}
static int irq_connect_dynamic(unsigned irq,unsigned,void(*)(const void*),const void*,unsigned){return irq;}
static void irq_enable(unsigned){}
static bool LL_TIM_IsActiveFlag_UPDATE(Timer*t){return t->update;}
static void LL_TIM_ClearFlag_UPDATE(Timer*t){t->update=false;}
static void LL_TIM_EnableCounter(Timer*t){t->enabled=true;}
static void LL_TIM_DisableCounter(Timer*t){t->enabled=false;}
static void LL_TIM_EnableIT_UPDATE(Timer*t){t->interrupt=true;}
static void LL_TIM_DisableIT_UPDATE(Timer*t){t->interrupt=false;}
static unsigned LL_TIM_OC_GetCompareCH4(Timer*t){return t->compare;}
static unsigned LL_TIM_OC_GetCompareCH3(Timer*t){return t->compare;}
static void LL_TIM_OC_SetCompareCH4(Timer*t,unsigned v){t->compare=v;}
static void LL_TIM_OC_SetCompareCH3(Timer*t,unsigned v){t->compare=v;}
static unsigned LL_TIM_GetCounter(Timer*t){return t->counter;}
static void LL_TIM_SetCounter(Timer*t,unsigned v){t->counter=v;}
static void LL_TIM_OC_SetMode(Timer*t,unsigned,unsigned v){t->mode=v;}
static void LL_TIM_SetCounterMode(Timer*,unsigned){}
static void LL_TIM_SetAutoReload(Timer*t,unsigned v){t->arr=v;}
static void LL_TIM_SetOnePulseMode(Timer*,unsigned){}
static void LL_TIM_GenerateEvent_UPDATE(Timer*t){t->counter=0;t->update=true;}
#include "../sketch/counted_jog.hpp"
#include "../sketch/pulse_train.hpp"
static unsigned edges;
static void reset(){stopCountedJog();for(auto &t:storage)t=Timer{};countedJog=CountedJog{};armMotion=coordinated::Motion{};edges=nowMs=0;}
// Model the peripheral, not the algorithm: one-pulse PWM2 generates a rise at
// CCR and a fall + update + hardware CEN clear at rollover. ISR can be delayed.
static void clockTicks(unsigned a,unsigned ticks,bool dispatch=true){
    auto *t=timers[a];
    while(ticks--){
        if(t->enabled){
            ++t->counter;
            if(t->counter==t->compare&&t->mode==LL_TIM_OCMODE_PWM2)++edges;
            if(t->counter>t->arr){t->counter=0;t->update=true;t->enabled=false;}
        }
        if(dispatch&&t->update&&t->interrupt)jogInterrupt(reinterpret_cast<void*>(uintptr_t(a)));
    }
}
// Closed-loop trains: all axes, rate changes, zero-rate finish, watchdog.
static void pulseTrains(){
    for(unsigned a=0;a<3;++a){
        reset();trains=PulseTrains{};trainsOwnTimers=false;
        assert(claimTrains()==0&&trainsOwnTimers);
        assert(runCountedJog(a,100,0,0)==-EBUSY);           // jog cannot share the timers
        assert(trainSet(a,1000,0)==0&&trains.running[a]);
        clockTicks(a,1000*50);assert(trains.count[a]>=49&&edges>=50);
        assert(trainSet(a,2000,0)==0);clockTicks(a,1000*2);assert(timers[a]->arr==499);
        clockTicks(a,500*20);
        assert(trainSet(a,0,0)==0);clockTicks(a,1000*5);    // in-flight pulse finishes, then stops
        assert(!trains.running[a]&&!timers[a]->enabled&&trains.count[a]==edges);
        unsigned before=edges;clockTicks(a,10000);assert(edges==before);
        assert(trainSet(a,1000,0)==0);clockTicks(a,1700);trainStopAll();
        assert(trains.count[a]==edges&&!trains.running[a]); // STOP reconciles a risen edge
        clockTicks(a,10000,false);assert(trains.count[a]==edges);
        assert(trainSet(a,1000,0)==0);clockTicks(a,1000*5,false); // missing ISR: one edge only
        trainStopAll();assert(trains.count[a]==edges);
        nowMs=0;assert(trainSet(a,2400,0)==0);nowMs=51;     // control loop stopped servicing
        clockTicks(a,100000);assert(!trains.running[a]&&trains.count[a]==edges);
        nowMs=0;assert(trainSet(a,10,0)==-ERANGE&&trainSet(a,1000000,0)==-ERANGE);
        releaseTrains();assert(!trainsOwnTimers&&runCountedJog(a,100,0,0)==0);stopCountedJog();
    }
}
int main(){
    pulseTrains();
    for(unsigned a=0;a<3;++a)for(unsigned rate:{20u,100u,1333u,2400u}){
        reset();assert(runCountedJog(a,rate,0,0)==0);
        unsigned period=timers[a]->arr+1,half=timers[a]->compare;
        clockTicks(a,half-1);stopCountedJog();assert(edges==0&&armMotion.count[a]==0);
        reset();assert(runCountedJog(a,rate,0,0)==0);
        clockTicks(a,half);stopCountedJog();assert(edges==1&&armMotion.count[a]==1);
        stopCountedJog();assert(armMotion.count[a]==1); // Idempotent STOP.
        reset();assert(runCountedJog(a,rate,0,0)==0);
        clockTicks(a,period*10,false); // Missing ISR cannot produce extra pulses.
        assert(edges==1&&countedJog.count==0);
        stopCountedJog();assert(armMotion.count[a]==1&&!timers[a]->interrupt);
        jogInterrupt(reinterpret_cast<void*>(uintptr_t(a))); // Stale ISR cannot restart.
        assert(!timers[a]->enabled);
        assert(runCountedJog(a,rate,0,0)==0);clockTicks(a,period*100);
        assert(countedJog.count==100&&edges==101);
        stopCountedJog();assert(armMotion.count[a]==100); // New session resets count.
    }
    reset();assert(runCountedJog(0,2400,0,0)==0);nowMs=51;
    clockTicks(0,10000);assert(edges==1&&countedJog.fault&&!countedJog.active);
    stopCountedJog();assert(armMotion.count[0]==1);
    reset();assert(runCountedJog(0,2400,299,0)==0);nowMs=300;
    clockTicks(0,10000);assert(edges==1&&countedJog.fault);
    reset();assert(runCountedJog(0,2400,0,0)==0);
    clockTicks(0,(timers[0]->arr+1)*3700);assert(edges==3600&&!countedJog.active&&!countedJog.fault);
    stopCountedJog();assert(armMotion.count[0]==3600);
    for(unsigned bad:{0u,19u,2401u,UINT32_MAX}){reset();assert(runCountedJog(0,bad,0,0)==-EINVAL);assert(edges==0);}
    std::cout<<"Pulse trains and counted timer: pad-edge model, pending IRQ, partial pulse, delayed ISR, repeat STOP, restart, watchdog and budget passed\n";
}
