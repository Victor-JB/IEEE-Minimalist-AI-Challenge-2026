"""STATE MACHINE timers. Edit here.

The states themselves (which models run at what rate, which animation pool, which event goes
where) are the two tables in state/machine.py.
"""

BOOT_TIMEOUT_S = 20.0                    # not homed by then -> FAULT (connect retries + a 3 s home move)
FAULT_COOLDOWN_S = 10.0                  # FAULT -> BOOT after this long (one auto retry)
FAULT_FORGET_S = 120.0                   # a fault this long after the last one counts as the first again
NOTICE_TIMEOUT_S = 1.5                   # false alarm -> IDLE
LOST_TIMEOUT_S = 6.0                     # LOST USER -> IDLE (4 s + 6 s = 10 s total)
CHECKIN_EVERY_S = (8.0, 15.0)            # HOLD -> CHECK-IN, random in this range
REACQUIRE_TIMEOUT_S = 4.0
