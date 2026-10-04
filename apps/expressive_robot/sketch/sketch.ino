// Expressive robot - MCU side (STM32U585).
//
// Python (Linux side) sends joint targets over the Bridge at ~20 Hz with
//   Bridge.notify("set_joints", [j0, j1, j2, j3, j4, j5])   // integer joint degrees
// This sketch runs its own faster loop that eases each joint toward its target
// with a speed limit, so motion stays smooth even if Python is slow or jittery.
// If Python stops sending, the robot eases back to HOME.
//
// sim/view.py --demo runs a Python copy of this easing (ServoModel); keep them in sync.

#include "Arduino_RouterBridge.h"
#include <vector>

// NUM_JOINTS, JOINT_MIN/MAX, HOME, MAX_SPEED_DEG_S, CONTROL_PERIOD_MS.
// Generated from sim/robot.toml by sim/configurator.py.
#include "robot_config.h"

const unsigned long COMMAND_TIMEOUT_MS = 1000;  // no command for this long -> go HOME
const unsigned long STATUS_PERIOD_MS = 1000;

// target[] is written by the Bridge callback, current[] only by loop().
volatile float target[NUM_JOINTS];
float current[NUM_JOINTS];
volatile unsigned long lastCommandMs = 0;

// TODO: drive the real servo (Servo library on a PWM pin, or a PCA9685 over I2C).
// `degrees` is the joint angle (0 = straight pose); map it to the servo's own
// range here, e.g. servoAngle = 90 + direction * degrees, per joint.
void writeServo(int joint, float degrees) {
}

// Called by Python via Bridge.notify("set_joints", [...]).
void set_joints(std::vector<int> degrees) {
    if (degrees.size() != NUM_JOINTS) {
        Monitor.print("set_joints: expected ");
        Monitor.print(NUM_JOINTS);
        Monitor.print(" values, got ");
        Monitor.println((int)degrees.size());
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        target[i] = constrain((float)degrees[i], JOINT_MIN[i], JOINT_MAX[i]);
    }
    lastCommandMs = millis();
}

void setup() {
    Monitor.begin(115200);

    for (int i = 0; i < NUM_JOINTS; i++) {
        current[i] = target[i] = HOME[i];
        writeServo(i, current[i]);
    }

    Bridge.begin();
    Bridge.provide("set_joints", set_joints);
}

void loop() {
    static unsigned long lastControlMs = 0;
    static unsigned long lastStatusMs = 0;
    unsigned long now = millis();

    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
        float dt = (now - lastControlMs) / 1000.0;
        lastControlMs = now;

        bool timedOut = now - lastCommandMs > COMMAND_TIMEOUT_MS;
        float maxStep = MAX_SPEED_DEG_S * dt;
        for (int i = 0; i < NUM_JOINTS; i++) {
            float goal = timedOut ? HOME[i] : target[i];
            current[i] += constrain(goal - current[i], -maxStep, maxStep);
            writeServo(i, current[i]);
        }
    }

    if (now - lastStatusMs >= STATUS_PERIOD_MS) {
        lastStatusMs = now;
        Monitor.print("joints:");
        for (int i = 0; i < NUM_JOINTS; i++) {
            Monitor.print(" ");
            Monitor.print(current[i], 1);
        }
        Monitor.println(now - lastCommandMs > COMMAND_TIMEOUT_MS ? "  (no commands, homing)" : "");
    }
}
