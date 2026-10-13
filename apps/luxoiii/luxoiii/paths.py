"""Where luxoiii's files live, and the one link outside the app: the shared arm module.

arm/ (repo root) owns everything about the arm: firmware, the Python API, arm_config (caps,
ranges, home), calibration, the clip player and the clips. luxoiii and the manual tools both import
it, so they always run with the same parameters.
"""
import sys
from pathlib import Path

APP = Path(__file__).resolve().parents[1]      # apps/luxoiii
REPO = APP.parents[1]
MODELS = APP / "models"                        # .tflite models + metadata
LOGS = APP / "logs"                            # transitions CSV, arm session JSONL (gitignored)
ARM = REPO / "arm"

# The arm modules import each other by plain name (from arm_client import ...).
if str(ARM / "python") not in sys.path:
    sys.path.insert(0, str(ARM / "python"))
