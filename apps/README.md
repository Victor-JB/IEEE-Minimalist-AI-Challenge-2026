# Apps on the Arduino UNO Q

The UNO Q is two computers: a Qualcomm Linux side (Python, camera, AI) and an STM32 microcontroller
(motor timing, safety). **The STM32 runs exactly one sketch at a time**, and whatever is flashed
stays there across reboots.

| App | What it is | Runs on | Status |
|---|---|---|---|
| `stepper_desk/` | Arm firmware (closed-loop control on the joint encoders, limits, lease watchdog) + Mac app + Python arm API | STM32 sketch + Mac/Linux tools | **Keep.** Its sketch is the firmware the real arm needs. |
| `lumen/` | The product: camera → AI → state machine → expressive motion | Linux side only (Python), talks to the stepper_desk firmware | **This is the one that runs at boot.** |
| `expressive_robot/` | Early App Lab scaffold for the old 6-servo design (test sweep, servo sketch) | App Lab app | **Legacy. Never set it to run at startup**: starting it flashes a servo sketch over the arm firmware. |

## How boot works

- **App Lab apps** (folders with `app.yaml` + `python/` + `sketch/`, stored on the board in
  `/home/arduino/ArduinoApps`) can be marked **Run at startup**. One app at a time is the default,
  shown with a DEFAULT badge in App Lab; the CLI equivalent is
  `arduino-app-cli properties set default user:<app-name>`. Starting an App Lab app compiles and
  **flashes its sketch to the STM32**, and runs its Python in a Docker container.
- **Lumen is not an App Lab app on purpose.** If it were, it would need its own sketch, and
  starting it would re-flash the MCU. Instead Lumen is Python only, uses the firmware that
  `stepper_desk/scripts/deploy.sh` already flashed, and starts at boot as a **systemd user service**
  (`lumen/lumen.service`), the same way Tailscale already runs on this board. It also keeps
  direct access to the camera and the router socket, with no Docker in the way.

**So exactly one thing must hold for a clean boot:** no App Lab app is set to run at startup.
Check once in App Lab (no DEFAULT badge on any app) after flashing the arm firmware.

Which one runs at boot, in practice:

```
stepper_desk/scripts/deploy.sh   → flashes the arm firmware (once; again only if firmware changes)
lumen/scripts/deploy.sh          → installs + enables lumen.service (Python) → runs at every boot
systemctl --user disable lumen   → stop Lumen from starting at boot (e.g. while commissioning)
```
