"""CAMERA, POWER and TEMPERATURE. Edit here."""

WARMUP_FRAMES = 5                        # frames dropped when the camera opens (auto-exposure settles)
SLEEP_CAMERA_FPS = 2.0                   # camera frames per second in SLEEP (the gate runs on each)
SLEEP_CPU_GOVERNOR = None                # e.g. "powersave" while asleep; needs write access to
                                         # cpufreq/scaling_governor (udev rule), skipped otherwise
OVERTEMP_C = 85.0                        # hottest thermal zone / hwmon sensor at or above -> FAULT
OVERTEMP_CLEAR_C = 75.0                  # ...and FAULT -> BOOT only once every sensor is below this
OVERTEMP_CHECK_S = 1.0                   # temperature sampled this often (perfstats logs it with PERF=1)
