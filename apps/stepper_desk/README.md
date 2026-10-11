# Stepper Desk Q

Three DRV8825 stepper channels on Arduino UNO Q, controlled by a native macOS app. Status is on the Mac only. The original nRF9151 project is unchanged.

## Wiring

Disconnect motor power before moving wires. Use the digital header labels on the UNO Q:

| Driver | STEP | DIR | PWM peripheral |
| --- | --- | --- | --- |
| Motor 1 | D3 | D2 | TIM3 channel 3 |
| Motor 2 | D5 | D4 | TIM1 channel 4 |
| Motor 3 | D9 | D7 | TIM4 channel 3 |

For each DRV8825:
- Connect logic GND and motor supply GND to UNO Q GND.
- Connect RESET and SLEEP to 3.3 V to keep the driver awake. Tie ENABLE to GND for enabled outputs. These signals are not controlled by this sketch.
- Supply VMOT from the external motor supply (your existing 20 V setup), never from a UNO Q GPIO. Keep a suitably rated bulk capacitor close to each driver's VMOT/GND.
- Connect the two motor coil pairs to A1/A2 and B1/B2.
- Set coil current using each driver's trimmer. The app's VREF calculator assumes R100 sense resistors: current limit = 2 × VREF. It does not change current or measure torque.
- Set MODE pins physically and match the app setting. M0 high with M1/M2 low is **half stepping: choose 1/2**. All MODE pins low is full stepping. The configured hardware is Motor 1 at **1/2**, Motors 2 and 3 at **1/16**. This update applies those values once on first launch, then preserves later choices.

UNO Q control signals are 3.3 V. Do not connect VMOT to the logic header. Confirm the actual sense-resistor marking before using the current calculator.

## AS5600 angle sensors (PCA9548A / TCA9548A channels 0–2)

| PCA9548A / TCA9548A | UNO Q |
| --- | --- |
| SDA | SDA / D20 / PB11 |
| SCL | SCL / D21 / PB10 |
| VCC | 3.3 V |
| GND | GND |
| A0, A1, A2 | GND (address 0x70) |
| RESET | 3.3 V (or the breakout's existing pull-up) |

Sensor 1 SDA/SCL → SD0/SC0; sensor 2 → SD1/SC1; sensor 3 → SD2/SC2. Use a common ground and appropriate sensor-module power wiring. SDA/SCL need pull-ups on the upstream bus and each sensor branch; check what the breakouts already include. Keep the UNO Q side pulled up to 3.3 V.

All AS5600s use address 0x36. Firmware selects exactly one channel, reads STATUS (0x0B) and the 12-bit RAW ANGLE (0x0C–0x0D), then deselects all channels. No sensor calibration or permanent programming registers are written.

The default `Wire` interface is the SDA/SCL header's I2C2 bus. Sampling uses 100 kHz I²C in a lower-priority thread, about 50 updates/sec per sensor when the bus is healthy. The Mac receives readings roughly 10 times/sec. I²C waits happen outside the motion mutex, so absent sensors do not block motor commands or the hold watchdog. After an I²C error, polling continues automatically; a physically stuck bus may require fixing the wiring and resetting the board.

The **Angle sensor** panel shows absolute angle (raw × 360/4096, wrapping at 360°) and magnet health. Low-field readings remain visible while the magnet-detected bit is set. Missing, stale (>500 ms), no-magnet or strong-field readings are unavailable. The sensor needs a suitable diametrically magnetized magnet aligned over its center. Sensor 1 / CH0 is shoulder (Motor 2), sensor 2 / CH1 is elbow (Motor 1), and sensor 3 / CH2 is wrist/head (Motor 3). Sensor channel numbering on the wire is unchanged; the Mac pairs each motor with the correct independent sensor stream.

Motor speed/ramp control remains open-loop, with no automatic position correction, homing or stall detection. The joint-limit guard below automatically stops forbidden motion using calibrated angles. The displayed angle is sensor orientation, not a calibrated mechanical zero or accumulated multi-turn position.

References: [AS5600 product documentation](https://ams-osram.com/products/sensors/position-sensors/ams-as5600-position-sensor), [TCA9548A datasheet](https://www.ti.com/lit/ds/symlink/tca9548a.pdf), [UNO Q header pinout](https://docs.arduino.cc/resources/pinouts/ABX00162-full-pinout.pdf).

## Use the Mac app

Open `mac/Stepper Desk Q.app` after building, or run `scripts/launch.command`.

1. Plug Arduino Q into this Mac by USB. Passwordless SSH must already work for `arduino@localhost` on forwarded port 2222.
2. Click **Connect all**, or connect an individual motor.
3. Select a motor, match its steps/revolution and microstepping, then set target RPM.
4. Press **Arm**, then keep **Hold Forward** or **Hold Reverse** pressed. Release or drag away to stop pulses without a deceleration ramp.
5. Edit **Startup ramp** for that motor. The default ramps to 150 RPM in 0.06 seconds, then to the target at 3 seconds. Interior speeds are capped at the target. A short stationary direction-settling interval precedes the ramp.

Each motor has its own settings, ramp, timer, and connection. Selecting another motor disarms all controls, so the mouse-driven app intentionally runs one selected motor at a time. The firmware supports three simultaneous independent channels for future coordinated controls. Leaving the app/window or pressing Escape disarms. Connecting or arming alone never starts motion.

200 full steps/revolution is a provisional default for the Creality 42-40. Confirm it for your exact motor. RPM is calculated from commanded pulses, not measured shaft speed. The ceiling is 2,400 pulses/sec **per motor**: 720 RPM at 200 full steps, 360 RPM at half step and 45 RPM at 1/16 step. AS5600 angle feedback is displayed separately; automatic stall detection is not included.

Stopping pulses retains holding torque because ENABLE is held low. The software stop is subject to USB/SSH/RPC latency. A separate MCU thread checks the 300 ms hold lease every 5 ms; lost renewals stop pulses even if the Mac or Linux gateway disconnects. This is not a hardwired emergency stop.

## Architecture

`Mac SwiftUI app → SSH over ADB USB forwarding → Python gateway → Arduino router socket → RouterBridge → STM32 hardware PWM`

The Mac establishes `adb -s 2709356903 forward tcp:2222 tcp:22`. Each axis uses its own SSH subprocess and random ownership token. The gateway calls `stepper_command` through the official `arduino-router-bridge` Python package. It never opens `/dev/ttyHS1` and never invents HOLD renewals. The existing router and monitor services remain in charge of serial transport.

The board-side installation is `/home/arduino/stepper-desk-q`. Update the device serial and SSH endpoint in `mac/SSHLink.swift` and `scripts/deploy.sh` if using a different board.

Hardware PWM uses separate TIM1/TIM3/TIM4 counters configured at approximately 1 MHz. Do not use other PWM channels on these timers in this sketch. Supported pulse frequencies are 20–2,400 Hz. Each motor's piecewise-linear ramp has 2–8 points and lasts up to 60 seconds. PWM compare is forced to zero on STOP. A direction change waits at least 60 ms stationary and then 20 ms after the actual DIR update before the ramp starts.

## Build and deploy

Requirements on Mac: Xcode command-line tools and Homebrew `android-platform-tools`. Board requirements: its installed Arduino tools, Python venv support, working router service and passwordless SSH.

```sh
# From this folder, build and install firmware + Python gateway on the board:
bash scripts/deploy.sh
# Build the native Mac app:
zsh mac/build.sh
# Open it:
open 'mac/Stepper Desk Q.app'
```

The deploy script flashes the MCU; it replaces the currently running sketch. It leaves Linux services and other projects intact. Firmware builds use the pinned core/libraries in `sketch/sketch.yaml`. Python dependencies are pinned in `python/requirements.txt`. No app or motor automatically starts on Linux boot; the MCU starts with all pulse outputs stopped.

```sh
# Local model, gateway and native control tests:
bash scripts/test.sh
```

Key files: `sketch/motion.hpp` (portable state machine), `sketch/sketch.ino` (PWM and RPC), `python/gateway.py` (SSH line protocol), `mac/Controller.swift` (hold/ramp logic), `mac/SSHLink.swift` (transport), `mac/StepperDesk.swift` (UI).

References: [UNO Q core](https://github.com/arduino/ArduinoCore-zephyr), [RouterBridge MCU library](https://github.com/arduino-libraries/Arduino_RouterBridge), [Python router bridge](https://github.com/arduino/arduino-router-bridge-py).

## Verified on this board (2026-10-10)

- Built with Arduino Zephyr core 1.0.0: 96,072 bytes flash, 42,116 bytes global RAM.
- Flashed the core and sketch and verified their flash contents against the compiled artifacts.
- All three native Mac connections remained connected during the integration check; every channel reported zero pulses/sec, no firmware fault, and stopped/disarmed controls.
- Passed portable motion checks (three independent channels, direction dwell, stale commands, lease expiry, clock wrap), six Python gateway checks, native controller checks and mouse-release/drag-away checks.
- Built and visually checked the Mac interface. Hardware rotation, waveform timing, load torque and missed steps have not been measured; no motor-start command was issued during deployment checks.

`mac/ConnectionTests.swift` is a read-only integration harness for the deployed board. It connects all three channels and checks stopped/disarmed status without issuing CURVE or HOLD commands.

## Sensor update validation (2026-10-11)

Built and flashed the AS5600 update (101,612 bytes flash, 47,952 bytes global RAM), with flash verification. Motion regression checks, channel-isolation/failure tests, seven gateway tests, and native sensor-display tests pass. All three motor channels still report stopped, disarmed and fault-free.

The first physical sensor check returned **multiplexer not responding at 0x70** on all channels. Sensor communication and actual angles are not yet verified. Check mux power/GND, RESET high, A0/A1/A2 low, SDA/SCL routing and pull-ups. Firmware keeps polling, so correctly connected sensors can appear without restarting the app. To run the live read-only integration check with sensor assertions, compile `mac/ConnectionTests.swift` alongside `SSHLink.swift`, `Controller.swift`, and `ArmModel.swift`, then run it with `--sensors`.

## I²C scanner

Run `bash scripts/scan.sh` on the Mac with Arduino Q connected over USB. It calls `stepper_scan` on the STM32, scanning header SDA/SCL (I2C2) addresses 0x08–0x77 with one-byte reads. This detects PCA9548A/TCA9548A at any of their addresses 0x70–0x77; it does not automatically change the configured 0x70 address. Read probes may miss write-only devices. The default multiplexer at 0x70 is deselected before scanning, so downstream AS5600s normally do not appear.

A scan is rejected while a motor is running, and new motor starts are rejected until it finishes. Probes run in the sensor thread, outside the motor mutex. The scanner stops with a partial-result error after about five seconds if bus timeouts prevent completion. Its reported SDA/SCL levels are instantaneous STM32 input samples, not oscilloscope measurements. No AS5600 configuration or OTP writes are performed.

Scanner run on 2026-10-11: completed 0x08–0x77 with no responding addresses, bus ready, and sampled SDA/SCL both high. This includes all possible PCA9548A addresses. Scanner firmware built at 107,164 bytes flash / 50,164 bytes global RAM and passed flash verification. No motor-start command was sent.

### Standard Arduino Wire scanner comparison

`diagnostics/i2c_scanner/i2c_scanner.ino` uses the standard `Wire.beginTransmission(address)` / `Wire.endTransmission()` probe for addresses 1–126, repeats every five seconds, and prints through UNO Q Monitor. An additional read-only Bridge method retrieves the exact return codes. STEP pins D3/D5/D9 are held low. This is a separate temporary sketch; it does not overwrite the saved motor-controller build.

On 2026-10-11, completed scan 4 found no successful responses; all 126 addresses returned 1. In installed Arduino Zephyr core 1.0.0, `Wire.endTransmission()` maps any underlying I²C failure to 1, so this is not a specific diagnosis of buffer overflow, address NACK, or wiring damage. Exact results are saved in `diagnostics/standard-scanner-result.json`.

## Live CAD arm view (2026-10-11)

The **Live arm** workspace uses the Fusion **ARM v29** mesh geometry and transforms from `origin/full-model-animations`, commit `ef1b0d2057c5e449a0e02638a1ff7d5b58dd3cbd`. It does not use the simplified `sim/robot.toml` dimensions. Both main links are 185 mm joint-to-joint; the wrist-to-head-tilt pivot is 45 mm. Bundled meshes and provenance are in `mac/Resources/ArmCAD`.

The removed base swivel is replaced with a fixed schematic pedestal. Only shoulder, elbow, and wrist/head rotate from sensors. The CAD model’s separate head-tilt servo is held at zero; its pose is not measured. Hidden cycloidal cam/plate assemblies are omitted from this visual model. No physics simulation or automatic position control runs here.

| Arm joint | Sensor / mux channel | Motor | Microstepping |
| --- | --- | --- | --- |
| Base → first segment (shoulder) | 1 / CH0 | 2 | 1/16 |
| First → second segment (elbow) | 2 / CH1 | 1 | 1/2 |
| Second segment → head (wrist) | 3 / CH2 | 3 | 1/16 |

Connect all three channels for the complete pose. Select a joint at the top, then arm and hold a direction beneath the model. Switching joints/workspaces, opening calibration, leaving the window, or releasing/dragging away from the hold button retains the existing stop/disarm behavior. Motor settings and ramp editing remain in the second workspace. RPM always refers to the motor shaft; the display does not silently divide by a gearbox ratio.

**Calibration:** open **Calibrate angles…** with motors stopped. Shoulder zero is upright; elbow/wrist zero continue straight from the preceding link. Positive joint angles bend toward the front (clockwise in the default Side camera), matching the CAD branch. Enter each joint’s current known physical angle, then press **Set angle**. Reverse the sensor if it counts the wrong way and recalibrate that joint. Offsets/directions are saved locally on this Mac and never write AS5600 OTP or start motors. The app cannot determine mechanical zero from a raw reading alone.

Solid parts follow calibrated, fresh readings. Faded parts use a reference pose when calibration or an upstream sensor reading is missing; they are not measured positions. Low-field angles continue while AS5600 still reports magnet detection. No-magnet, stale, bus-error, or too-strong readings remain unavailable. Joint angles wrap to −180…180° and are not accumulated turn counts.

Drag the model to orbit or scroll to zoom; these gestures only move the camera. **3D / Side** resets the camera to the chosen view. Tests cover sensor-to-motor pairing, microstep defaults/migration, offset/direction calibration and wraparound, exact CAD joint transforms and downstream missing-data behavior, plus existing hold/release and firmware/gateway checks. The update requires only rebuilding the Mac app, with no MCU flash.

## Automatic joint limits (2026-10-11)

User limits in calibrated relative joint degrees:

| Joint | Sensor | Motor | Minimum | Maximum | Hold Forward |
| --- | --- | --- | --- | --- | --- |
| Shoulder | 1 / CH0 | 2 | −45° | +38° | Increases angle |
| Elbow | 2 / CH1 | 1 | −72° | +79° | Decreases angle |
| Head / wrist | 3 / CH2 | 3 | −80° | +87° | Decreases angle |

The STM32 checks each moving motor against its mapped sensor every 5 ms. It stops STEP pulses when the measured angle reaches a boundary while moving outward, or if that sensor is unavailable, reports no magnet/too strong a field, or is more than 100 ms old. Weak-field readings remain usable while magnet detection is set, as requested. Sensor sampling remains about 50 Hz. These are software limits: sampling, scheduling, mechanics and sensor error can allow physical overshoot; they are not hardwired end stops. No automatic reverse or position correction is issued.

The opposite (inward) direction remains allowed at or beyond a boundary. A limit stop disarms the Mac controls; release, arm again, and hold the permitted direction to move back into range. Disconnects still use the existing 300 ms hold lease. Invalid/stale sensor data locks both directions until it recovers.

Every new motor owner starts with an unconfigured guard. The Mac sends its saved joint zero offset, sensor reversal and motor-direction sign using `LIMITS`, then requires the board to echo matching calibration, limits and revision before enabling Arm. A missing calibration or old firmware keeps the app locked. Configuration cannot change while the motor runs. Configuration lives in MCU RAM and is resynchronized on connection/reboot; limits and sensor-to-motor mapping are fixed in firmware. Recalibrating from the app synchronizes the new zero before motion can resume.

Confirmed wiring implies Forward increases each raw AS5600 angle; the calibrated direction sign follows the saved reversal flag. Reversing a sensor during calibration consequently flips the guard's direction convention too. Changing motor coil order, driver direction wiring, or sensor mounting requires verifying this relationship again.

`sketch/limits.hpp` implements the guard. Firmware RPC v5 adds guard state/configuration and calibrated angle to telemetry; Mac JSON remains schema v2 with added fields. The app accepts only matching limit revision 1 before arming. `calibration/joint-limits-2026-10-11.json` records ranges, motor/sensor mapping and the user-confirmed direction signs.

Validation: portable tests cover boundaries in both directions, inward recovery, all swapped mappings, calibration/wraparound, weak field, unavailable/stale data and timer wrap. Native tests cover board acknowledgement before arming, forbidden-direction rejection, limit-stop disarm and changed calibration. `mac/LiveLimitTests.swift` verifies the deployed board's guard/calibration and stopped state without any CURVE or HOLD command. Physical stopping distance is not measured by these tests.

Deployment verified on 2026-10-11: guard firmware built at 110,072 bytes flash / 51,464 bytes global RAM, programmed and flash-verified. The live Mac check synchronized all three saved calibrations and read back matching bounds and Forward signs, with all motors stopped, disarmed and fault-free. No motor-start commands were used. SSH startup was measured at about 3.4 s on this board; its transport keepalive was relaxed from 1 s to 5 s to avoid premature login failure. The Mac's 0.6 s telemetry-loss stop and MCU's 0.3 s hold lease are unchanged.
