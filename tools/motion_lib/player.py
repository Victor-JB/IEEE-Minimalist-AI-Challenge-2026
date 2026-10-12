"""The clip player lives in the shared arm module (arm/python/player.py), so the robot and
these authoring tools always agree. This shim re-exports it under the old `player` name so
author.py, validate.py, review.py, demo.py and the tests keep working unchanged.
Clips are read from and written to arm/clips/authored/.
"""
import importlib.util as _util
from pathlib import Path as _Path

_SRC = _Path(__file__).resolve().parents[2] / "arm" / "python" / "player.py"
_spec = _util.spec_from_file_location("lumen_player", _SRC)
_mod = _util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
globals().update({k: v for k, v in vars(_mod).items() if not k.startswith("__")})
