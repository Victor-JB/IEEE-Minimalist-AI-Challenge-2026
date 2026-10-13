// SPDX-License-Identifier: MIT
// Arduino UNO Q: three DRV8825 drivers. No motion until a valid CURVE + HOLD.
#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <stm32_ll_tim.h>
#include <vector>
#include "motion.hpp"
#include "power.hpp"
#include "trajectory.hpp"
#include "servo.hpp"
#include "roll.hpp"
#include "light.hpp"
#include <zephyr/irq.h>
#include "sensors.hpp"
#include "limits.hpp"
#include <Wire.h>
#include <zephyr/drivers/i2c.h>

using namespace stepper;
constexpr uint8_t SLEEP_PIN = 8;
static Power power;
static coordinated::Motion armMotion;
static servo::Controller servoCtl; // closed-loop moves; see servo.hpp
static uint32_t armOwner=0,armSeen=0;
static unsigned armConfigured=0,armVerified=0;
static std::array<double,3> armPPD{};
constexpr uint8_t STEP_PINS[AXES] = {3, 5, 9};
// Lamp LED (light.hpp). LIGHT_ENABLED 1 puts the LED's MOSFET gate on D7 and moves the head
// DIR from D7 to D12, so it needs a rewire; 0 (default) keeps today's wiring and the arm_light
// RPC reports "not enabled". Why D7: it is TIM8 CH4N, and TIM8 drives no other header pin.
// Every other PWM-capable header pin shares a timer that is already taken: D3/D6/D8 (TIM3),
// D5/D11/D12/D13 (TIM1) and D9/D10 (TIM4) belong to the STEP pulse trains, which rewrite their
// timer's prescaler/period and switch to forced/one-pulse modes; D2/D20/D21 (TIM2) are the
// roll servo's fixed 50 Hz (visible flicker and camera banding) plus the encoder I2C.
// D12 (PB14) only ever runs as a plain GPIO for DIR, like D6 already does on TIM3.
#ifndef LIGHT_ENABLED
#define LIGHT_ENABLED 0
#endif
constexpr uint8_t HEAD_DIR_PIN = LIGHT_ENABLED ? 12 : 7;
constexpr uint8_t DIR_PINS[AXES] = {6, 4, HEAD_DIR_PIN}; // elbow DIR moved D2 -> D6 for the roll servo
// These are PWM spec indices, not Arduino digital pin numbers.
static const pwm_dt_spec outputs[AXES] = {
    PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1), // D3 TIM3 CH3
    PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2), // D5 TIM1 CH4
    PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6), // D9 TIM4 CH3
};
static TIM_TypeDef *const timers[AXES] = {TIM3, TIM1, TIM4};
// Head-tilt servo (roll.hpp). TIM2 has no STEP pin, so its 50 Hz period stays put.
constexpr uint8_t ROLL_PIN = 2;
static const pwm_dt_spec rollOutput = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0); // D2 TIM2 CH2
static roll::Roll headRoll;
static unsigned rollApplied = ~0u;
static int rollFault = 0;
constexpr uint8_t LIGHT_PIN = 7;
constexpr uint32_t LIGHT_PWM_HZ = 20000;   // above hearing and far above camera/eye flicker
#if LIGHT_ENABLED
static const pwm_dt_spec lightOutput = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4); // D7 TIM8 CH4N
#endif
static light::Light lamp;
static uint32_t lightPeriod = 0, lightApplied = ~0u;   // TIM8 counts per PWM period, compare sent
static int lightFault = LIGHT_ENABLED ? 0 : -ENODEV;
static Motor motors[AXES];
static joint_limit::Guard limits[AXES];
static uint32_t tickHz[AXES], applied[AXES];
static bool appliedDirection[AXES];
static k_mutex lock;
static k_thread motionThread;
K_THREAD_STACK_DEFINE(motionStack, 2048);
static encoder::Sample sensors[AXES];
static k_thread sensorThread;
K_THREAD_STACK_DEFINE(sensorStack, 2048);
static const device *const sensorBus = DEVICE_DT_GET(DT_NODELABEL(i2c2));
// Explicit diagnostics: only probe normal 7-bit addresses, with one-byte reads.
// This never writes sensor configuration or OTP. I2C runs only in sensorThread.
static int scanState=0, scanGeneration=0, scanError=0, scanLast=0;
static std::vector<int> scanAddresses;
static std::vector<int> scanBus(int request) {
    k_mutex_lock(&lock, K_FOREVER);
    if (request) {
        if(armOwner){ k_mutex_unlock(&lock); return {1,-EBUSY}; }
        for (unsigned a=0; a<AXES; ++a) {
            if (motors[a].running) { k_mutex_unlock(&lock); return {1,-EBUSY}; }
        }
        if (scanState != 1 && scanState != 2) {
            scanAddresses.clear(); scanError=0; scanLast=0; scanState=1;
        }
    }
    // Wire's actual I2C2 pin levels: PB11 SDA and PB10 SCL.
    auto pins = GPIOB->IDR;
    std::vector<int> result = {1,scanState,scanGeneration,scanError,scanLast,
                              int((pins>>11)&1),int((pins>>10)&1),int(device_is_ready(sensorBus))};
    result.insert(result.end(),scanAddresses.begin(),scanAddresses.end());
    k_mutex_unlock(&lock);
    return result;
}
struct SensorBus {
    bool select(uint8_t mask) { return i2c_write(sensorBus, &mask, 1, 0x70) == 0; }
    bool read(uint8_t reg, uint8_t *bytes, size_t len) {
        return i2c_write_read(sensorBus, 0x36, &reg, 1, bytes, len) == 0;
    }
};
static void readSensors(void *, void *, void *) {
    Wire.begin(); // UNO Q Wire = I2C2, SDA D20/PB11, SCL D21/PB10.
    const bool ready = device_is_ready(sensorBus) &&
        i2c_configure(sensorBus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD)) == 0;
    SensorBus bus;
    for (;;) {
        k_mutex_lock(&lock, K_FOREVER);
        const bool scan = scanState == 1;
        if (scan) scanState=2;
        k_mutex_unlock(&lock);
        if (scan) {
            std::vector<int> addresses;
            int error=ready ? 0 : -ENODEV, last=0;
            const uint32_t began=millis();
            if (ready) {
                // Normally all channels are deselected between sensor samples.
                // If 0x70 is present, ensure this is an upstream-only scan.
                bus.select(0);
                for (unsigned address=0x08; address<=0x77; ++address) {
                    uint8_t byte;
                    if (i2c_read(sensorBus,&byte,1,address)==0) addresses.push_back(address);
                    last=address;
                    if (uint32_t(millis()-began)>5000) { error=-ETIMEDOUT; break; }
                    k_msleep(1);
                }
            }
            k_mutex_lock(&lock, K_FOREVER);
            scanAddresses=addresses; scanError=error; scanLast=last;
            ++scanGeneration; scanState=3;
            k_mutex_unlock(&lock);
        }
        for (unsigned a=0; a<AXES; ++a) {
            encoder::Sample value;
            if (ready) value = encoder::read(bus, a);
            else value.error = encoder::BUS;
            value.sampled = millis();
            // I2C transactions may time out. Never hold the motor mutex during IO.
            k_mutex_lock(&lock, K_FOREVER);
            sensors[a] = value;
            k_mutex_unlock(&lock);
        }
        // About 110 Hz per joint at 100 kHz I2C; the closed loop needs fresh angles.
        k_msleep(5);
    }
}

static int output(unsigned axis) {
    if(armOwner)return 0;
    auto &m = motors[axis];
    const auto &p = outputs[axis];
    if (m.direction != appliedDirection[axis]) {
        digitalWrite(DIR_PINS[axis], m.direction ? HIGH : LOW);
        appliedDirection[axis] = m.direction;
    }
    const uint32_t hz = power.ready(millis()) ? m.hz : 0;
    if (hz == applied[axis]) return 0;
    // Force a zero compare immediately on STOP, rather than waiting a long
    // low-frequency period. The timer is exclusively owned by this motor.
    if (!hz) {
        int err = pwm_set_cycles(p.dev, p.channel, 1000, 0, p.flags);
        if (err) return err;
        LL_TIM_GenerateEvent_UPDATE(timers[axis]);
    } else {
        uint32_t period = (tickHz[axis] + hz/2) / hz;
        int err = pwm_set_cycles(p.dev, p.channel, period, period/2, p.flags);
        if (err) return err;
    }
    applied[axis] = hz;
    return 0;
}
static void applyOrFault(unsigned a) {
    int err = output(a);
    if (err) {
        motors[a].fault = err; motors[a].stop(stepper::ERROR);
        // Even if the PWM driver reports an error, explicitly force compare zero.
        if (a == 1) LL_TIM_OC_SetCompareCH4(timers[a], 0);
        else LL_TIM_OC_SetCompareCH3(timers[a], 0);
        LL_TIM_GenerateEvent_UPDATE(timers[a]);
        applied[a] = 0;
    }
}
static void stopCountedJog();
static void releaseTrains();
static void syncPower() {
    bool moving = (armMotion.running() || servoCtl.wantsPower()) && armOwner;
    for (unsigned a=0; a<AXES; ++a) moving |= motors[a].running;
    if (!moving) { stopCountedJog(); releaseTrains(); }
    if (power.update(moving,millis())) {
        // All stopped: clear PWM outputs before removing shared coil current.
        if (!power.awake) for (unsigned a=0; a<AXES; ++a) applyOrFault(a);
        digitalWrite(SLEEP_PIN,power.awake ? HIGH : LOW);
        power.wokeAt=millis(); // Delay is measured from the physical transition.
    }
}
static void rollTick(uint32_t now) {
    headRoll.tick(now);
    const unsigned us = headRoll.pulseUs();   // 0 = no pulses: limp
    if (us == rollApplied || rollFault) return;
    int err = pwm_set_dt(&rollOutput, PWM_USEC(roll::PERIOD_US), PWM_USEC(us));
    if (err) { rollFault = err; headRoll.release(); }
    else rollApplied = us;
}
static void lightTick(uint32_t now) {
    lamp.tick(now);
#if LIGHT_ENABLED
    if (lightFault || !lightPeriod) return;
    const uint32_t c = lamp.compare(lightPeriod);
    if (c == lightApplied) return;
    int err = pwm_set_cycles(lightOutput.dev, lightOutput.channel, lightPeriod, c, lightOutput.flags);
    if (err) lightFault = err;
    else lightApplied = c;
#endif
}
static void lightSetup() {
#if LIGHT_ENABLED
    analogWrite(LIGHT_PIN, 0); // Initialize PWM and only this pin's mux.
    uint64_t hz = 0;
    int err = !pwm_is_ready_dt(&lightOutput) ? -ENODEV
            : pwm_get_cycles_per_sec(lightOutput.dev, lightOutput.channel, &hz);
    if (!err) {
        // Same trick as the STEP timers: recover the timer clock, then run TIM8 undivided so
        // a 20 kHz period has thousands of counts (fine steps at the dim end of the gamma curve).
        const uint64_t clock = hz * (LL_TIM_GetPrescaler(TIM8) + 1);
        LL_TIM_SetPrescaler(TIM8, 0);
        lightPeriod = uint32_t(clock / LIGHT_PWM_HZ);
        if (lightPeriod < 1000 || lightPeriod > 65535) err = -ERANGE;
        else err = pwm_set_cycles(lightOutput.dev, lightOutput.channel, lightPeriod, 0, lightOutput.flags);
        LL_TIM_GenerateEvent_UPDATE(TIM8);
    }
    lightFault = err;
    lightApplied = 0;
#endif
}
#include "arm_control.hpp"

static void guardMotion(unsigned a, uint32_t now) {
    auto &m=motors[a];
    if (m.running && !limits[a].permits(a,sensors[joint_limit::SENSOR_FOR_MOTOR[a]],now,m.wanted))
        m.stop(stepper::LIMIT_STOP);
}
static void control(void *, void *, void *) {
    for (;;) {
        k_mutex_lock(&lock, K_FOREVER);
        uint32_t now = millis();
        for (unsigned a=0; a<AXES; ++a) { motors[a].tick(now); guardMotion(a,now); }
        syncPower();
        armTick(now);
        rollTick(now);
        lightTick(now);
        for (unsigned a=0; a<AXES; ++a) applyOrFault(a);
        syncPower(); // A PWM failure may have stopped the final active motor.
        k_mutex_unlock(&lock);
        k_msleep(1);
    }
}
static std::vector<int> status(unsigned a, int ack, int error) {
    const auto &m = motors[a];
    const auto &sensor = sensors[a];
    uint32_t age = millis()-sensor.sampled;
    if (age > 0x7fffffff) age = 0x7fffffff;
    const auto &guard=limits[a];
    const auto &jointSensor=sensors[joint_limit::SENSOR_FOR_MOTOR[a]];
    const int guardState=guard.state(a,jointSensor,millis());
    const int angle=(guard.configured && guard.usable(jointSensor,millis())) ? int(std::lround(guard.angle(jointSensor)*1000)) : INT32_MIN;
    return {7, int(a), ack, error, m.fault, int(m.hz), int(m.target),
            int(m.direction), int(m.wanted), int(m.running), int(m.ppr),
            int(m.phase), int(MAX_HZ), int(m.session),
            sensor.raw, sensor.magnet, sensor.error, int(age),
            guardState, int(guard.configured), guard.zeroUdeg, int(guard.reversed), guard.forwardSign,
            angle, joint_limit::MIN_MDEG[a], joint_limit::MAX_MDEG[a], 1, int(power.awake), int(power.generation), int(power.ready(millis())), int(power.enabled)};
}
// op 0 claim, 1 curve, 2 hold, 3 stop, 4 status, 5 limit calibration, 6 shared sleep/wake. All calls use an owner token.
// One mutex covers callbacks and the independent 5 ms watchdog thread.
static std::vector<int> command(std::vector<int> args) {
    if (args.size() < 3 || args[1] < 0 || args[1] >= int(AXES) || args[2] <= 0)
        return {3, -1, 0, -EINVAL};
    unsigned a = args[1];
    int op = args[0], ack = 0, err = 0;
    uint32_t owner = args[2], now = millis();
    k_mutex_lock(&lock, K_FOREVER);
    auto &m = motors[a];
    if(armOwner){
        if(op==3){armMotion.stop();syncPower();}
        auto r=status(a,0,-EBUSY);k_mutex_unlock(&lock);return r;
    }
    if (op == 0 && args.size() == 3) {
        err = m.claim(owner);
        if (!err) limits[a]=joint_limit::Guard{}; // Every new owner must synchronize calibration.
    }
    else if (m.owner != owner) err = -EACCES;
    else if (op == 6 && args.size() == 5) {
        err = power.change(args[3], uint32_t(args[4]));
        if (!err) {
            // Stop every timer before changing the shared physical sleep line.
            for (unsigned j=0; j<AXES; ++j) { motors[j].stop(); applyOrFault(j); }
            syncPower();
        }
    }
    else if (op == 5 && args.size() == 6) {
        if (m.running) err=-EBUSY;
        else if (!limits[a].configure(args[3],args[4],args[5])) err=-EINVAL;
    }
    else if (op == 1 && args.size() >= 7) {
        ack = args[3];
        int count = args[6];
        Point points[MAX_POINTS];
        if (count < 2 || count > int(MAX_POINTS) || args.size() != size_t(8+2*count)) err = -EINVAL;
        else {
            for (unsigned i=3; i<args.size(); ++i) if (args[i] < 0) err = -EINVAL;
            for (int i=0; i<count; ++i) points[i] = {uint32_t(args[7+2*i]), uint32_t(args[8+2*i])};
            if (!err && !power.permits(uint32_t(args.back()))) err = -EPERM;
            if (!err && (scanState == 1 || scanState == 2)) err = -EBUSY;
            if (!err && !limits[a].permits(a,sensors[joint_limit::SENSOR_FOR_MOTOR[a]],now,args[4])) err=-EPERM;
            if (!err) err = m.start(owner, args[3], args[4], args[5], points, count, now);
        }
    } else if (op == 2 && args.size() == 4) err = m.hold(owner, args[3], now);
    else if (op == 3 && args.size() == 3) m.stop();
    else if (op != 4 || args.size() != 3) err = -EINVAL;
    m.tick(now);
    guardMotion(a,now);
    syncPower();
    applyOrFault(a);
    syncPower();
    auto result = status(a, ack, err);
    k_mutex_unlock(&lock);
    return result;
}

void setup() {
    digitalWrite(SLEEP_PIN, LOW);
    pinMode(SLEEP_PIN, OUTPUT);
    k_mutex_init(&lock);
    for (unsigned a=0; a<AXES; ++a) {
        pinMode(DIR_PINS[a], OUTPUT); digitalWrite(DIR_PINS[a], LOW);
        analogWrite(STEP_PINS[a], 0); // Initialize PWM and only this pin's mux.
        const auto &p = outputs[a];
        uint64_t hz = 0;
        int err = !pwm_is_ready_dt(&p) ? -ENODEV : pwm_get_cycles_per_sec(p.dev, p.channel, &hz);
        if (!err) {
            // Recover the peripheral clock before choosing a 1 MHz counter.
            uint64_t clock = hz * (LL_TIM_GetPrescaler(timers[a])+1);
            uint32_t divisor = (clock+999999)/1000000;
            if (!divisor || divisor > 65536) err = -ERANGE;
            else {
                tickHz[a] = clock/divisor;
                LL_TIM_SetPrescaler(timers[a], divisor-1);
                err = pwm_set_cycles(p.dev, p.channel, 1000, 0, p.flags);
                LL_TIM_GenerateEvent_UPDATE(timers[a]);
            }
        }
        if (err) { motors[a].fault = err; motors[a].stop(stepper::ERROR); }
    }
    analogWrite(ROLL_PIN, 0); // Initialize PWM and only this pin's mux.
    rollFault = !pwm_is_ready_dt(&rollOutput) ? -ENODEV : pwm_set_dt(&rollOutput, PWM_USEC(roll::PERIOD_US), 0);
    rollApplied = 0;
    lightSetup();
    k_thread_create(&motionThread, motionStack, K_THREAD_STACK_SIZEOF(motionStack),
                    control, nullptr, nullptr, nullptr, 4, 0, K_NO_WAIT);
    k_thread_create(&sensorThread, sensorStack, K_THREAD_STACK_SIZEOF(sensorStack),
                    readSensors, nullptr, nullptr, nullptr, 6, 0, K_NO_WAIT);
    Bridge.begin();
    Bridge.provide("stepper_command", command);
    Bridge.provide("stepper_scan", scanBus);
    Bridge.provide("arm_command",armCommand);
    Bridge.provide("arm_observe",armObserve);
    Bridge.provide("arm_lock",armLock);
    Bridge.provide("arm_servo_status",armServoStatus);
    Bridge.provide("arm_roll_status",armRollStatus);
    Bridge.provide("arm_light",armLight);
    Bridge.provide("arm_selftest",armSelftest);
    Bridge.provide("arm_pulse_selftest",armPulseSelftest);
    Monitor.begin();
    Monitor.println(LIGHT_ENABLED
        ? "Stepper Desk Q ready: STEP D3/D5/D9, DIR D6/D4/D12; SLEEP D8; roll servo D2; light D7; all asleep and stopped"
        : "Stepper Desk Q ready: STEP D3/D5/D9, DIR D6/D4/D7; SLEEP D8; roll servo D2; all asleep and stopped");
}
void loop() { delay(100); }
