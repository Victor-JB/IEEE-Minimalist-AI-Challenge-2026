"""Make the repo's shared code importable without copying it.

Lumen reuses, rather than duplicates:
  inference/                  camera + face detector (+ future hand/book models)
  perf/                       perfstats timers (no-op unless PERF=1)
  motion_lib/                 clip player and clips
  apps/stepper_desk/python    ArmMotion: closed-loop arm API (talks to the flashed firmware)
  apps/stepper_desk/scripts   emote.trajectory(): clip -> real-arm joint trajectory
"""
import sys
from pathlib import Path

APP = Path(__file__).resolve().parents[1]          # apps/lumen
REPO = APP.parents[1]                              # repo root
STEPPER = REPO / "apps" / "stepper_desk"
LOGS = APP / "logs"

for p in (REPO / "inference", REPO / "perf", REPO / "motion_lib",
          STEPPER / "python", STEPPER / "scripts"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
