// SPDX-License-Identifier: MIT
// Arduino UNO Q: three DRV8825 drivers. No motion until a valid CURVE + HOLD.
#include <Arduino_RouterBridge.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <stm32_ll_tim.h>
#include <vector>
#include "motion.hpp"
#include "sensors.hpp"
#include "limits.hpp"
#include <Wire.h>
#include <zephyr/drivers/i2c.h>

using namespace stepper;
constexpr uint8_t STEP_PINS[AXES] = {3, 5, 9};
constexpr uint8_t DIR_PINS[AXES] = {2, 4, 7};
// These are PWM spec indices, not Arduino digital pin numbers.
static const pwm_dt_spec outputs[AXES] = {
    PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1), // D3 TIM3 CH3
    PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2), // D5 TIM1 CH4
    PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6), // D9 TIM4 CH3
};
static TIM_TypeDef *const timers[AXES] = {TIM3, TIM1, TIM4};
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
        k_msleep(20);
    }
}

static int output(unsigned axis) {
    auto &m = motors[axis];
    const auto &p = outputs[axis];
    if (m.direction != appliedDirection[axis]) {
        digitalWrite(DIR_PINS[axis], m.direction ? HIGH : LOW);
        appliedDirection[axis] = m.direction;
    }
    if (m.hz == applied[axis]) return 0;
    // Force a zero compare immediately on STOP, rather than waiting a long
    // low-frequency period. The timer is exclusively owned by this motor.
    if (!m.hz) {
        int err = pwm_set_cycles(p.dev, p.channel, 1000, 0, p.flags);
        if (err) return err;
        LL_TIM_GenerateEvent_UPDATE(timers[axis]);
    } else {
        uint32_t period = (tickHz[axis] + m.hz/2) / m.hz;
        int err = pwm_set_cycles(p.dev, p.channel, period, period/2, p.flags);
        if (err) return err;
    }
    applied[axis] = m.hz;
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
static void guardMotion(unsigned a, uint32_t now) {
    auto &m=motors[a];
    if (m.running && !limits[a].permits(a,sensors[joint_limit::SENSOR_FOR_MOTOR[a]],now,m.wanted))
        m.stop(stepper::LIMIT_STOP);
}
static void control(void *, void *, void *) {
    for (;;) {
        k_mutex_lock(&lock, K_FOREVER);
        uint32_t now = millis();
        for (unsigned a=0; a<AXES; ++a) { motors[a].tick(now); guardMotion(a,now); applyOrFault(a); }
        k_mutex_unlock(&lock);
        k_msleep(5);
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
    return {5, int(a), ack, error, m.fault, int(m.hz), int(m.target),
            int(m.direction), int(m.wanted), int(m.running), int(m.ppr),
            int(m.phase), int(MAX_HZ), int(m.session),
            sensor.raw, sensor.magnet, sensor.error, int(age),
            guardState, int(guard.configured), guard.zeroUdeg, int(guard.reversed), guard.forwardSign,
            angle, joint_limit::MIN_MDEG[a], joint_limit::MAX_MDEG[a], 1};
}
// op 0 claim, 1 curve, 2 hold, 3 stop, 4 status, 5 limit calibration. All calls use an owner token.
// One mutex covers callbacks and the independent 5 ms watchdog thread.
static std::vector<int> command(std::vector<int> args) {
    if (args.size() < 3 || args[1] < 0 || args[1] >= int(AXES) || args[2] <= 0)
        return {3, -1, 0, -EINVAL};
    unsigned a = args[1];
    int op = args[0], ack = 0, err = 0;
    uint32_t owner = args[2], now = millis();
    k_mutex_lock(&lock, K_FOREVER);
    auto &m = motors[a];
    if (op == 0 && args.size() == 3) {
        err = m.claim(owner);
        if (!err) limits[a]=joint_limit::Guard{}; // Every new owner must synchronize calibration.
    }
    else if (m.owner != owner) err = -EACCES;
    else if (op == 5 && args.size() == 6) {
        if (m.running) err=-EBUSY;
        else if (!limits[a].configure(args[3],args[4],args[5])) err=-EINVAL;
    }
    else if (op == 1 && args.size() >= 7) {
        ack = args[3];
        int count = args[6];
        Point points[MAX_POINTS];
        if (count < 2 || count > int(MAX_POINTS) || args.size() != size_t(7+2*count)) err = -EINVAL;
        else {
            for (unsigned i=3; i<args.size(); ++i) if (args[i] < 0) err = -EINVAL;
            for (int i=0; i<count; ++i) points[i] = {uint32_t(args[7+2*i]), uint32_t(args[8+2*i])};
            if (!err && (scanState == 1 || scanState == 2)) err = -EBUSY;
            if (!err && !limits[a].permits(a,sensors[joint_limit::SENSOR_FOR_MOTOR[a]],now,args[4])) err=-EPERM;
            if (!err) err = m.start(owner, args[3], args[4], args[5], points, count, now);
        }
    } else if (op == 2 && args.size() == 4) err = m.hold(owner, args[3], now);
    else if (op == 3 && args.size() == 3) m.stop();
    else if (op != 4 || args.size() != 3) err = -EINVAL;
    m.tick(now);
    guardMotion(a,now);
    applyOrFault(a);
    auto result = status(a, ack, err);
    k_mutex_unlock(&lock);
    return result;
}

void setup() {
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
    k_thread_create(&motionThread, motionStack, K_THREAD_STACK_SIZEOF(motionStack),
                    control, nullptr, nullptr, nullptr, 4, 0, K_NO_WAIT);
    k_thread_create(&sensorThread, sensorStack, K_THREAD_STACK_SIZEOF(sensorStack),
                    readSensors, nullptr, nullptr, nullptr, 6, 0, K_NO_WAIT);
    Bridge.begin();
    Bridge.provide("stepper_command", command);
    Bridge.provide("stepper_scan", scanBus);
    Monitor.begin();
    Monitor.println("Stepper Desk Q ready: STEP D3/D5/D9, DIR D2/D4/D7; all stopped");
}
void loop() { delay(100); }
