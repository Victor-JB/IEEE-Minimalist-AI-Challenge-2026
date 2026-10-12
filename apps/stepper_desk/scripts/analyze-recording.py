#!/usr/bin/env python3
"""Read-only commissioning analysis of Stepper Desk Q JSONL recordings.

No board/network access, command execution, or calibration writes. All arrays in
input use motor order; reports use shoulder, elbow, head order. Counts measure
issued STEP edges, not successful steps or torque. CLI accepts explicit files.
"""
import argparse
import json
import math
import statistics
from pathlib import Path

JOINTS = ("shoulder", "elbow", "head")
MOTORS = (1, 0, 2)
FORWARD = (1, -1, -1)
STATES = ("idle", "preparing", "moving", "measuring", "reached_pose",
          "released", "hold_expired", "sensor_unavailable", "joint_limit",
          "tracking_error", "timing_fault", "measurement_travel_reached", "powered_settling")
FAULTS = set(range(6, 11))


def healthy_axis(axis):
    return (axis.get("angle") != -2147483648 and
            isinstance(axis.get("angle"), (int, float)) and
            math.isfinite(axis["angle"]) and
            0 <= axis.get("age", -1) <= 100 and
            axis.get("sensor_error") == 0 and
            axis.get("magnet", 0) & 0x28 == 0x20)


def angles(sample):
    return [sample["axes"][a]["angle"] / 1000 if healthy_axis(sample["axes"][a]) else None
            for a in MOTORS]


def read_events(path):
    events = []
    previous = -math.inf
    with Path(path).open() as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            event = json.loads(line)
            if event.get("version") != 1:
                raise ValueError(f"{path}:{number}: unsupported recording version")
            time = event.get("time")
            if not isinstance(time, (int, float)) or not math.isfinite(time) or time < previous:
                raise ValueError(f"{path}:{number}: invalid or decreasing timestamp")
            previous = time
            if event.get("type") == "sample":
                sample = event.get("sample", {})
                if sample.get("v") != 1 or len(sample.get("axes", [])) != 3:
                    raise ValueError(f"{path}:{number}: unsupported sample")
                required = ("error", "owner", "state", "id", "ms", "run")
                if any(type(sample.get(key)) is not int for key in required):
                    raise ValueError(f"{path}:{number}: malformed sample header")
                for axis in sample["axes"]:
                    if any(type(axis.get(key)) is not int for key in
                           ("angle", "reference", "target", "count", "direction", "age", "magnet", "sensor_error")):
                        raise ValueError(f"{path}:{number}: malformed axis sample")
            events.append(event)
    return events


def regression(samples, motor):
    """Interior linear fit; repeats in transport are deduplicated by MCU time."""
    rows = {}
    for event in samples:
        sample = event["sample"]
        axis = sample["axes"][motor]
        if sample["run"] and sample["state"] != 12 and healthy_axis(axis):
            rows[sample["ms"]] = (axis["count"], axis["angle"] / 1000)
    rows = list(rows.values())
    if not rows:
        return None
    lower, upper = min(x for x, _ in rows), max(x for x, _ in rows)
    rows = [(x, y) for x, y in rows if lower + .1 * (upper - lower) <= x <= upper - .1 * (upper - lower)]
    if len(rows) < 8 or len(set(x for x, _ in rows)) < 8 or max(y for _, y in rows) - min(y for _, y in rows) < 2:
        return None
    xs, ys = zip(*rows)
    xm, ym = statistics.mean(xs), statistics.mean(ys)
    xx = sum((x-xm)**2 for x in xs)
    yy = sum((y-ym)**2 for y in ys)
    slope = sum((x-xm)*(y-ym) for x, y in rows) / xx
    intercept = ym - slope*xm
    residual = sum((y-intercept-slope*x)**2 for x, y in rows)
    return {"samples": len(rows), "signed_degrees_per_issued_pulse": slope,
            "pulses_per_degree": abs(1/slope) if slope else None,
            "r_squared": 1-residual/yy if yy else None,
            "rms_error_degrees": math.sqrt(residual/len(rows))}


def analyze(events, source="<memory>"):
    metadata = next((e for e in events if e.get("type") == "metadata"), {})
    moves = []
    command_indices = [i for i, e in enumerate(events) if e.get("type") == "command" and
                       e.get("text", "").split()[:1] in (["JOG"], ["POSE"])]
    for position, index in enumerate(command_indices):
        event = events[index]
        words = event["text"].split()
        end = command_indices[position+1] if position+1 < len(command_indices) else len(events)
        move = {"command": words[0], "command_time_s": event["time"], "accepted": False}
        moves.append(move)
        try:
            move_id = int(words[1])
            motor = int(words[2]) if words[0] == "JOG" else None
            direction = int(words[3]) if motor is not None else None
            if len(words) not in ((5, 6, 7) if motor is not None else (6,)) or (motor is not None and (motor not in range(3) or direction not in (0, 1))):
                raise ValueError()
        except (ValueError, IndexError):
            move["reason"] = "malformed move command"
            continue
        if motor is not None and len(words)>=6:
            try:
                rate=int(words[4])
                if not 20<=rate<=2400: raise ValueError()
            except ValueError:
                move["reason"]="invalid pulse rate"
                continue
            move["nominal_rate_hz"]=rate
        prepare_ms=80
        if motor is not None and len(words)==7:
            try:
                prepare_ms=int(words[5])
                if not 80<=prepare_ms<=500:raise ValueError()
            except ValueError:
                move["reason"]="invalid preparation interval"
                continue
        move["preparation_ms"]=prepare_ms
        move["id"] = move_id
        baseline = next((e for e in reversed(events[:index]) if e.get("type") == "sample"), None)
        after = [e for e in events[index+1:end] if e.get("type") == "sample"]
        # A prior ID matching this command is not evidence that this request ran.
        if baseline is None or baseline["sample"]["id"] == move_id or baseline["sample"]["run"]:
            move["reason"] = "missing idle baseline or ID already active; acceptance is ambiguous"
            continue
        accepted = next((e for e in after if e["sample"]["id"] == move_id and
                         e["sample"]["owner"] == baseline["sample"]["owner"] and
                         e["sample"]["error"] == 0 and
                         ((e["sample"]["run"] == 1 and e["sample"]["state"] in (1, 2, 3)) or
                          (e["sample"]["state"] >= 4 and any(a["count"] > 0 for a in e["sample"]["axes"])))), None)
        if accepted is None:
            move["reason"] = "no telemetry evidence that command was accepted"
            move["response_errors"] = sorted(set(e["sample"]["error"] for e in after if e["sample"]["error"]))
            continue
        move["accepted"] = True
        owner = accepted["sample"]["owner"]
        samples = [e for e in after if e["sample"]["owner"] == owner and e["sample"]["id"] == move_id]
        stopped = next((e for e in samples if e["sample"]["run"] == 0 and e["sample"]["state"] >= 4), None)
        # Use the first 0.3 seconds of the settled tail; require at least 0.3 s since
        # stopping and distinct MCU observations. Never borrow the next move.
        tail = [e for e in samples if stopped and e["time"] >= stopped["time"]+.3 and e["sample"]["run"] == 0]
        final = samples[-1]
        settled = [e for e in tail if e["time"] <= tail[0]["time"]+.3] if tail else []
        unique = {e["sample"]["ms"]: e for e in settled}
        settled = list(unique.values())
        settled_ok = len(settled) >= 2 and all(all(healthy_axis(a) for a in e["sample"]["axes"]) for e in settled)
        settlement_range = [max(angles(e["sample"])[j] for e in settled)-min(angles(e["sample"])[j] for e in settled) for j in range(3)] if settled_ok else [None]*3
        initial = angles(baseline["sample"])
        last = [statistics.median(angles(e["sample"])[j] for e in settled) for j in range(3)] if settled_ok else angles(final["sample"])
        asleep_last=last[:]
        asleep_settled_ok=settled_ok
        powered=any(e.get("type")=="command" and e.get("text","").split()==["PAUSE",str(move_id)] for e in events[index+1:end])
        powered_reasons=[]
        if powered:
            preparing=[e for e in samples if e["sample"]["state"]==1 and e["sample"].get("awake")==1 and e["sample"]["run"]==1 and all(a["count"]==0 and healthy_axis(a) for a in e["sample"]["axes"])]
            first_motion=next((e for e in samples if e["sample"]["state"]==3),None)
            preparing=[e for e in preparing if first_motion and e["time"]<first_motion["time"]]
            if prepare_ms>=400 and preparing:
                first_prepare=next(e for e in samples if e["sample"]["state"]==1)
                stable_prepare=list({e["sample"]["ms"]:e for e in preparing if ((e["sample"]["ms"]-first_prepare["sample"]["ms"]) & 0xffffffff)>=200}.values())
                if len(stable_prepare)<3: powered_reasons.append("insufficient powered baseline settlement")
                elif any(max(angles(e["sample"])[j] for e in stable_prepare)-min(angles(e["sample"])[j] for e in stable_prepare)>.176001 for j in range(3)): powered_reasons.append("unstable powered baseline")
                if stable_prepare: initial=[statistics.median(angles(e["sample"])[j] for e in stable_prepare) for j in range(3)]
            elif preparing: initial=angles(preparing[-1]["sample"])
            else: powered_reasons.append("no fresh powered zero-count baseline")
            pauses=[e for e in samples if e["sample"]["state"]==12]
            pause_tail=list({e["sample"]["ms"]:e for e in pauses if ((e["sample"]["ms"]-pauses[0]["sample"]["ms"]) & 0xffffffff)>=200}.values()) if pauses else []
            valid_pause=len(pause_tail)>=2 and all(e["sample"].get("awake")==1 and e["sample"]["run"]==1 and all(healthy_axis(a) for a in e["sample"]["axes"]) for e in pauses)
            if not valid_pause: powered_reasons.append("no fresh powered settled endpoint")
            pause_counts=[tuple(a["count"] for a in e["sample"]["axes"]) for e in pauses]
            final_counts=tuple(a["count"] for a in final["sample"]["axes"])
            if not pause_counts or any(c!=pause_counts[0] for c in pause_counts) or pause_counts[-1]!=final_counts: powered_reasons.append("pulse count changed during or after powered settlement")
            if stopped is None or not pauses or stopped["time"]<=pauses[-1]["time"]: powered_reasons.append("powered settlement ended without clean release")
            if valid_pause:
                last=[statistics.median(angles(e["sample"])[j] for e in pause_tail) for j in range(3)]
                settlement_range=[max(angles(e["sample"])[j] for e in pause_tail)-min(angles(e["sample"])[j] for e in pause_tail) for j in range(3)]
            else:
                last=[None]*3;settlement_range=[None]*3
            settled_ok=valid_pause
        move["measurement_mode"]="powered_settle" if powered else "sleep_settle"
        move["asleep_endpoint_settled"]=asleep_settled_ok
        move["asleep_final_angles_deg"]=dict(zip(JOINTS,asleep_last))
        move["sleep_drift_deg"]={name: asleep_last[j]-last[j] if asleep_last[j] is not None and last[j] is not None else None for j,name in enumerate(JOINTS)} if powered else None
        move["sleep_accuracy_warning"] = bool(powered and (not asleep_settled_ok or any(v is not None and abs(v) > .3 for v in move["sleep_drift_deg"].values())))
        delta = [b-a if a is not None and b is not None else None for a, b in zip(initial, last)]
        state = final["sample"]["state"]
        active = [e for e in samples if stopped is None or e["time"] <= stopped["time"]]
        counts = [final["sample"]["axes"][a]["count"] for a in MOTORS]
        move.update(owner=owner, state=STATES[state] if 0 <= state < len(STATES) else f"unknown_{state}",
                    stopped=stopped is not None, settled=settled_ok,
                    duration_s=(stopped or final)["time"]-event["time"],
                    baseline_age_s=event["time"]-baseline["time"],
                    initial_angles_deg=dict(zip(JOINTS, initial)), final_angles_deg=dict(zip(JOINTS, last)),
                    signed_delta_deg=dict(zip(JOINTS, delta)), issued_pulses=dict(zip(JOINTS, counts)),
                    max_sensor_age_ms={j: max(e["sample"]["axes"][a]["age"] for e in active) for j, a in zip(JOINTS, MOTORS)},
                    peak_excursion_deg={j: max((abs(e["sample"]["axes"][a]["angle"]/1000-initial[k]) for e in active if initial[k] is not None and healthy_axis(e["sample"]["axes"][a])), default=None) for k, (j, a) in enumerate(zip(JOINTS, MOTORS))},
                    invalid_sensor_samples=sum(not all(healthy_axis(a) for a in e["sample"]["axes"]) for e in active),
                    fault_reason=next((STATES[e["sample"]["state"]] for e in samples if e["sample"]["state"] in FAULTS), None))
        move["interior_regression"] = {j: regression(active, a) for j, a in zip(JOINTS, MOTORS) if max(e["sample"]["axes"][a]["count"] for e in active) >= 50}
        if motor is not None:
            joint = MOTORS.index(motor)
            d, count = delta[joint], counts[joint]
            move.update(joint=JOINTS[joint], direction=direction,
                        untouched_joint_drift_deg={JOINTS[j]: delta[j] for j in range(3) if j != joint},
                        pulses_per_degree=count/abs(d) if d is not None and d != 0 else None)
            move["settled_range_deg"] = dict(zip(JOINTS,settlement_range))
            move["post_stop_drift_deg"] = {name: asleep_last[j]-angles(stopped["sample"])[j] if stopped and asleep_last[j] is not None and angles(stopped["sample"])[j] is not None else None for j,name in enumerate(JOINTS)}
            fit=move["interior_regression"].get(JOINTS[joint])
            move["endpoint_slope_relative_difference"] = abs(move["pulses_per_degree"]/fit["pulses_per_degree"]-1) if fit and fit["pulses_per_degree"] and move["pulses_per_degree"] else None
            reasons = powered_reasons[:]
            if any(v is not None and v > .176001 for v in settlement_range): reasons.append("unstable settled endpoint")
            if not powered and any(v is not None and abs(v) > .3 for v in move["post_stop_drift_deg"].values()): reasons.append("excessive post-stop drift")
            if any(j != joint and (delta[j] is None or abs(delta[j]) > .75 or move["peak_excursion_deg"][JOINTS[j]] is None or move["peak_excursion_deg"][JOINTS[j]] > .75) for j in range(3)): reasons.append("uncommanded joint moved")
            if not settled_ok: reasons.append("no fresh settled endpoint")
            if move["baseline_age_s"] > .3 or any(v is None for v in initial): reasons.append("stale or invalid baseline")
            if state not in (5, 11): reasons.append("trial did not finish by release or bounded travel")
            if move["fault_reason"] or move["invalid_sensor_samples"]: reasons.append("fault or invalid sensor sample")
            if count < 50: reasons.append("fewer than 50 issued pulses")
            if d is None or not 5 <= abs(d) <= 13: reasons.append("travel outside 5–13 degrees")
            if d is None or abs(d) < .1: reasons.append("no resolved net angle response")
            elif d*(FORWARD[joint] if direction == 0 else -FORWARD[joint]) < 0: reasons.append("angle direction disagrees with calibrated mapping")
            if not move["pulses_per_degree"] or not 1 <= move["pulses_per_degree"] <= 10000: reasons.append("conversion outside allowed range")
            move["ratio_candidate"] = not reasons
            move["ratio_rejection_reasons"] = reasons
    candidates = {}
    for joint in JOINTS:
        trials = [m for m in moves if m.get("joint") == joint and m.get("ratio_candidate")]
        values = sorted(m["pulses_per_degree"] for m in trials)
        median = statistics.median(values) if values else None
        spread = (values[-1]-values[0])/median if values else None
        both = set(m["direction"] for m in trials) == {0, 1}
        candidates[joint] = {"eligible_trials": len(values), "both_directions": both,
                             "median_pulses_per_degree": median, "relative_spread": spread,
                             "consistency_criteria_met": len(values) >= 3 and both and spread <= .05,
                             "verified": False}
    return {"source": str(source), "fingerprint": metadata.get("fingerprint"),
            "notes": ["Read-only analysis; never saves or verifies calibration.",
                      "Issued pulses and angle motion do not measure torque or prove absence of skipped steps.",
                      "Direction checks use shoulder forward positive, elbow/head forward negative.",
                      "Endpoint settlement means fresh post-stop samples; inspect drift before accepting a ratio."],
            "sample_count": sum(e.get("type") == "sample" for e in events),
            "moves": moves, "ratio_consistency": candidates}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", type=Path)
    args = parser.parse_args()
    try:
        reports = [analyze(read_events(path), path) for path in args.recordings]
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.exit(2, f"Analysis failed: {error}\n")
    print(json.dumps(reports[0] if len(reports) == 1 else reports, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
