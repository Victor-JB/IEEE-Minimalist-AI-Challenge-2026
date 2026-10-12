"""Animation pools: which clips each state may play.

Entries are (clip ref, energy) where energy is "low", "mid" or "high"; the expressive layer
weights high-energy clips up when the lamp is excited and low-energy ones when it is calm.
Clip refs are "name/variant": authored clips in arm/clips/authored (browse them in
tools/motion_lib/CATALOG.md on the dev branch) or the arm-specific "lamp_*/arm" clips in arm/clips/lamp.

A pool marked PROCEDURAL is generated in code instead (look-at, hold, sweep, adjust); its
clips, if any, are flourishes played on top. Keep clips the real arm can do: try them with
apps/stepper_desk/scripts/emote.py first (lamp_wake needs more shoulder torque than it has).
"""

PROCEDURAL = {"attentive", "holding", "searching", "adjust", "found"}

POOLS = {
    "boot":           [("home/gentle", "mid")],
    "asleep":         [("go_to_sleep/drowsy", "low")],
    "bored":          [("idle_breathe/calm", "low"), ("scan_room/calm", "low"),
                       ("relaxed_sit_down/calm", "low"), ("stretch_reach_limit/short_try", "mid"),
                       ("tail_wag/small", "mid"), ("sad_droop/slight", "low"),
                       ("dance_90/groove", "high")],
    "notice":         [("double_take/subtle", "mid"), ("perk_up/small", "mid"), ("double_take/big", "high")],
    "attentive":      [("idle_breathe/alert", "mid"), ("curious_head_tilt/single", "mid"),
                       ("curious_head_tilt/double", "high"), ("lean_in_interest/subtle", "low"),
                       ("lamp_curious/arm", "mid")],
    "mirror":         [("tail_wag/small", "mid"), ("excited_bounce/medium", "high")],
    "looking_around": [("scan_room/alert", "mid"), ("look_at_user_return/slow_right", "low"),
                       ("look_at_user_return/quick_left", "mid")],
    "acknowledge":    [("nod_yes/small_calm", "low"), ("nod_yes/medium", "mid"),
                       ("lamp_yes/arm", "mid"), ("confident_snap/firm", "high")],
    "searching":      [("point_with_head/left_small", "low"), ("point_with_head/right_big", "high"),
                       ("glance_then_reach/left", "mid"), ("glance_then_reach/right", "mid"),
                       ("hesitate/subtle", "low")],
    "found":          [("celebrate/medium", "high"), ("excited_bounce/medium", "high"), ("lamp_happy/arm", "mid")],
    "quizzical":      [("shake_no/medium", "mid"), ("shrug_fail/small", "low"), ("shrug_fail/big", "high"),
                       ("hesitate/unsure", "low")],
    "holding":        [("idle_breathe/calm", "low")],
    "adjust":         [],
    "checkin":        [("look_at_user_return/slow_right", "low"), ("look_at_user_return/quick_left", "mid")],
    "peek":           [("curious_head_tilt/single", "mid")],
    "done":           [("nod_yes/small_calm", "low"), ("home/gentle", "low")],
    "fault":          [("shrug_fail/small", "low")],
}
