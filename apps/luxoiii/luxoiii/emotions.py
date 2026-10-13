"""Full-model animation branch choreography, for the browser only."""
import json
from pathlib import Path

CLIPS = Path(__file__).resolve().parents[3] / 'arm/clips/full-model.json'


def bundle():
    return json.loads(CLIPS.read_text())
