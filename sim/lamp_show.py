"""Open the full luxoiii X-ray animation in a browser (no hardware connection)."""
import runpy
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "apps/luxoiii"))
runpy.run_module("luxoiii.preview", run_name="__main__")
