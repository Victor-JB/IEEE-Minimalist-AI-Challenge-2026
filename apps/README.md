# Apps on the Arduino UNO Q

The UNO Q is two computers: a Qualcomm Linux side (Python, camera, AI) and an STM32 microcontroller
(motor timing, safety). **The STM32 runs exactly one sketch at a time**, and whatever is flashed
stays there across reboots. That sketch is the arm firmware in `arm/sketch`.

| Folder | What it is | Branch |
|---|---|---|
| `lumen/` | The product: camera → AI → state machine → expressive motion. **Runs at boot.** | main + dev |
| `stepper_desk/` | Manual / test tools for the arm: Mac app, `emote.py`, `armctl.py`, analysis scripts, commissioning records | dev |

Both import the shared `arm/` module (firmware, Python API, `arm_config`, clips), so the app and
the hand-testing tools always run the same parameters.

## How boot works

- **App Lab apps** (folders with `app.yaml` + `python/` + `sketch/`, stored on the board in
  `/home/arduino/ArduinoApps`) can be marked **Run at startup**. One app at a time is the default,
  shown with a DEFAULT badge in App Lab; the CLI equivalent is
  `arduino-app-cli properties set default user:<app-name>`. Starting an App Lab app compiles and
  **flashes its sketch to the STM32**, and runs its Python in a Docker container.
- **Lumen is not an App Lab app on purpose.** If it were, starting it would re-flash the MCU over
  the arm firmware. Lumen is Python only, uses the firmware that `arm/scripts/deploy.sh` flashed,
  and starts at boot as a **systemd user service** (`lumen/lumen.service`), the same way Tailscale
  already runs on this board.

**For a clean boot, no App Lab app may be set to run at startup.** Check once in App Lab that no
app shows the DEFAULT badge.

```
arm/scripts/deploy.sh            → flashes the arm firmware + installs arm/.venv (once; again if firmware changes)
apps/lumen/scripts/deploy.sh     → installs + enables lumen.service → runs at every boot
systemctl --user disable lumen   → keep Lumen from starting at boot (e.g. while testing by hand)
```
