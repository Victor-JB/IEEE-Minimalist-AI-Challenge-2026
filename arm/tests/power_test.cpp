#include "../sketch/power.hpp"
#include "../sketch/motion.hpp"
#include <cassert>
#include <cstdio>
int main() {
    stepper::Power power;
    assert(!power.awake && power.enabled && power.permits(1));
    assert(!power.ready(100) && !power.update(false,100)); // Idle/arming do not wake.
    assert(power.change(2,1)==-EINVAL && power.change(1,2)==-ESTALE);
    stepper::Motor motors[3];
    const stepper::Point points[]={{0,0},{60,500}};
    auto sync = [&](uint32_t now) {
        bool moving=false; for (const auto &m: motors) moving |= m.running;
        return power.update(moving,now);
    };
    for (auto &m: motors) assert(m.claim(42)==0);
    assert(motors[0].start(42,1,0,400,points,2,100)==0);
    assert(sync(100) && power.awake && power.generation==1);
    assert(!power.ready(102) && power.ready(103));
    assert(motors[1].start(42,1,0,3200,points,2,105)==0);
    motors[0].stop(); assert(!sync(106) && power.awake); // Another motor still active.
    motors[1].stop(); assert(sync(107) && !power.awake);
    assert(motors[1].hold(42,1,108)==-ESTALE && !sync(108));
    assert(power.change(0,1)==0 && !power.permits(2));
    assert(!power.update(true,200) && !power.awake); // Locked cannot wake.
    assert(power.change(1,2)==0 && power.permits(3) && !power.permits(1));
    assert(!power.update(false,201) && !power.awake); // Enable movement does not wake.
    assert(power.update(true,0xfffffffe));
    assert(!power.ready(0) && power.ready(1));
    assert(power.update(false,2) && !power.awake);
    std::puts("PASS: idle sleep, accepted-start wake, 3 ms delay, last-motor sleep, stale hold, explicit lock, generation and clock wrap");
}
