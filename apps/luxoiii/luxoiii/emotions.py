"""Curated original motion curves for the digital arm; never hardware commands."""
import json
from pathlib import Path

CLIPS=Path(__file__).resolve().parents[3]/'arm/clips/authored'
REFS=('idle_breathe__calm','idle_breathe__alert','scan_room__calm',
      'curious_head_tilt__single','lean_in_interest__subtle','perk_up__small',
      'double_take__subtle','tail_wag__small','nod_yes__medium',
      'nod_yes__small_calm','shrug_fail__small','excited_bounce__medium')


def bundle():
    clips={}
    for ref in REFS:
        clip=json.loads((CLIPS/(ref+'.json')).read_text())
        clips[ref]={k:clip[k] for k in ('name','variant','mode','joint_mask','keyframes')}
    return clips
