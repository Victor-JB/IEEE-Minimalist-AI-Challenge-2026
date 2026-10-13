"""LAMP LIGHT: brightness, ramps and which backend drives the LED. Edit here."""
import os as _os

# --- Lamp light (light/light.py) -------------------------------------------------
# Levels are perceptual (0-1, what the HUD shows); the backend gets duty = level ** LIGHT_GAMMA.
LIGHT_RAMP_S = 0.5                       # default ramp (ILLUMINATE on, GOODBYE off, FAULT dim)
LIGHT_STEP_RAMP_S = 0.3                  # ADJUST brightness steps
LIGHT_GAMMA = 2.2
LIGHT_CHECKIN_SCALE = 0.7                # LightCoupling: dim to 70 % while glancing at the user
LIGHT_COUPLING_RAMP_S = 0.3
# Backend, chosen with env LUMEN_LIGHT: "sim" (default: logs only), "firmware" (LIGHT RPC on the
# STM32 through the motion worker's arm session), "linux-pwm" (sysfs PWM), "gpio" (gpiod on/off).
LIGHT_BACKEND = _os.environ.get("LUMEN_LIGHT", "sim")
LIGHT_FIRMWARE_EVERY_S = 0.08            # firmware backend: at most one LIGHT per this, faded over it
LIGHT_PWM_CHIP = int(_os.environ.get("LUMEN_LIGHT_PWMCHIP", "0"))    # /sys/class/pwm/pwmchipN
LIGHT_PWM_CHANNEL = int(_os.environ.get("LUMEN_LIGHT_PWM_CHANNEL", "0"))
LIGHT_PWM_HZ = int(_os.environ.get("LUMEN_LIGHT_PWM_HZ", "20000"))
LIGHT_GPIO_CHIP = _os.environ.get("LUMEN_LIGHT_GPIOCHIP", "/dev/gpiochip0")
LIGHT_GPIO_LINE = int(_os.environ.get("LUMEN_LIGHT_GPIO_LINE", "0"))
