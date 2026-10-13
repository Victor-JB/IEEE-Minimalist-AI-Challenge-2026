"""Scorecard: per-state time, inference rates and energy from luxoiii's logs, as markdown for
bench/RESULTS.md (dev branch).

Inputs
  transitions CSV(s)   apps/luxoiii/logs/transitions-<date>.csv (luxoiii/translog.py), or tools/simulate.py --csv
  --perf PERF_CSV      optional: perfstats long CSV (PERF=1 PERF_CSV=run.csv python -m luxoiii.legacy --headless)
  --power METER_CSV    optional: power meter log with a time column and watts (or volts + amps)
  --idle-watts W       board idle baseline (nothing running), for the "above idle" numbers

    python3 apps/luxoiii/tools/scorecard.py logs/transitions-20261012.csv
    python3 apps/luxoiii/tools/scorecard.py run.csv --perf perf.csv --power meter.csv --idle-watts 2.1 \\
        --change "face gated by motion" > card.md

How it reads the log: each row is a transition, so the state a row enters lasts until the next
row, and the inference counts between the two rows belong to it. The state before the first row
(BOOT at start-up) and the one after the last row have no known end and are left out unless
--end-t gives the end of the run (on the translog `t` clock). A new session starts when `t` goes
backwards or a row's `from` is not the previous row's `to` (luxoiii restarted, the CSV appends).

Power alignment: meter timestamps can be epoch seconds, ISO date-times or HH:MM:SS[.fff]
(local time; matched to the translog's wall_time column, which has 1 s resolution), or small
numbers already on the translog `t` clock (the simulator's synthetic trace uses epoch).
--power-offset S shifts the meter's clock if it was not synced to the board.
"""
import argparse
import csv
import datetime as dt
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
if str(APP) not in sys.path:
    sys.path.insert(0, str(APP))
try:
    from luxoiii.machine import STATES
except Exception:                     # the scorecard must work on a copy of the logs anywhere
    STATES = {}


# -- transitions -------------------------------------------------------------------
def _kv(text, sep):
    out = {}
    for part in (text or "").split():
        if sep in part:
            k, v = part.split(sep, 1)
            out[k] = v
    return out


def _sod(hms):
    """'HH:MM:SS[.f]' -> seconds of day."""
    h, m, s = hms.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


@dataclass
class Segment:
    state: str
    start: float
    end: float
    rates: dict                    # configured, from the row's models column
    inferences: dict               # model -> count during the segment
    wall_offset: float = None      # seconds of day = t + wall_offset
    counted: bool = True           # False: closed by --end-t, its inferences are unknown

    @property
    def duration(self):
        return self.end - self.start


def read_transitions(path):
    with open(path, newline="") as f:
        return [r for r in csv.DictReader(f) if r.get("t")]


def sessions(rows):
    out, cur = [], []
    for r in rows:
        if cur and (float(r["t"]) < float(cur[-1]["t"]) or r["from"] != cur[-1]["to"]):
            out.append(cur)
            cur = []
        cur.append(r)
    if cur:
        out.append(cur)
    return out


def wall_offset(session):
    """Best estimate of (seconds of day - t). wall_time is truncated to whole seconds, so each
    row gives a value up to 1 s low; the largest is the closest."""
    vals = []
    for r in session:
        try:
            vals.append(_sod(r["wall_time"]) - float(r["t"]))
        except (KeyError, ValueError, AttributeError):
            pass
    return max(vals) + 0.5 if vals else None   # +0.5: middle of the remaining uncertainty


def segments(rows, end_t=None):
    """Segments from one or more sessions. end_t closes the last state of the LAST session."""
    out, dropped = [], []
    ss = sessions(rows)
    for si, s in enumerate(ss):
        off = wall_offset(s)
        for a, b in zip(s, s[1:]):
            ca, cb = _kv(a["inferences_so_far"], "="), _kv(b["inferences_so_far"], "=")
            inf = {k: int(cb[k]) - int(ca.get(k, 0)) for k in cb}
            out.append(Segment(a["to"], float(a["t"]), float(b["t"]), _kv(a["models"], "@"), inf, off))
        last = s[-1]
        if end_t is not None and si == len(ss) - 1 and end_t > float(last["t"]):
            out.append(Segment(last["to"], float(last["t"]), end_t, _kv(last["models"], "@"), {}, off, False))
            dropped.append((last["to"], float(last["t"]), "closed by --end-t: in the time share, not in the rates"))
        else:
            dropped.append((last["to"], float(last["t"]), "open at the end of the log"))
    return out, dropped


def label(state):
    parent = getattr(STATES.get(state), "parent", None)
    return f"{parent}/{state}" if parent else state


# -- power ------------------------------------------------------------------------
def _col(header, *names):
    low = [h.strip().lower() for h in header]
    for n in names:
        for i, h in enumerate(low):
            if h == n or h.startswith(n + " ") or h.startswith(n + "("):
                return i
    return None


def _parse_time(text):
    """-> (kind, value): ('epoch', s), ('sod', seconds of day) or ('t', translog clock)."""
    text = text.strip()
    try:
        v = float(text)
        return ("epoch", v) if v > 1e6 else ("t", v)
    except ValueError:
        pass
    if re.fullmatch(r"\d{1,2}:\d{2}:\d{2}(\.\d+)?", text):
        return "sod", _sod(text)
    d = dt.datetime.fromisoformat(text.replace("Z", "+00:00"))
    if d.tzinfo is None:
        return "sod", d.hour * 3600 + d.minute * 60 + d.second + d.microsecond / 1e6
    return "epoch", d.timestamp()


def read_power(path, offset=0.0):
    """[(kind, time, watts)] from a meter CSV."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    ti = _col(header, "timestamp", "time", "t", "date")
    wi = _col(header, "watts", "power", "w", "p")
    vi, ai = _col(header, "volts", "voltage", "v"), _col(header, "amps", "current", "a", "i")
    if ti is None or (wi is None and (vi is None or ai is None)):
        raise SystemExit(f"{path}: need a time column and watts (or volts and amps); header is {header}")
    out = []
    for r in data:
        if not r or not r[ti].strip():
            continue
        kind, t = _parse_time(r[ti])
        w = float(r[wi]) if wi is not None else float(r[vi]) * float(r[ai])
        out.append((kind, t + offset, w))
    return out


def _to_sod(kind, t):
    if kind == "epoch":
        lt = dt.datetime.fromtimestamp(t)
        return lt.hour * 3600 + lt.minute * 60 + lt.second + lt.microsecond / 1e6
    return t


def power_in(seg, samples):
    """Watts samples that fall inside a segment."""
    out = []
    for kind, t, w in samples:
        if kind == "t":
            inside = seg.start <= t < seg.end
        elif seg.wall_offset is None:
            continue
        else:
            s = _to_sod(kind, t)
            inside = seg.start + seg.wall_offset <= s < seg.end + seg.wall_offset
        if inside:
            out.append(w)
    return out


# -- perfstats --------------------------------------------------------------------
def read_perf(path):
    timings, system = defaultdict(list), defaultdict(list)
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            (timings if r["kind"] == "timing_ms" else system)[r["name"]].append((float(r["t_s"]), float(r["value"])))
    return timings, system


def _pct(sorted_values, p):
    return sorted_values[min(len(sorted_values) - 1, int(p / 100 * len(sorted_values)))]


# -- report -----------------------------------------------------------------------
def _f(v, nd=1):
    return "" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{nd}f}"


def _table(header, rows):
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return lines


@dataclass
class StateStats:
    time: float = 0.0
    count_time: float = 0.0        # time whose inference counts are known (rates divide by this)
    visits: int = 0
    inferences: dict = field(default_factory=lambda: defaultdict(int))
    rates: dict = field(default_factory=dict)
    watts: list = field(default_factory=list)


def scorecard(transition_paths, perf=None, power=None, idle_watts=None, end_t=None, power_offset=0.0,
              change="", date=None):
    rows = [r for p in transition_paths for r in read_transitions(p)]
    segs, dropped = segments(rows, end_t)
    samples = read_power(power, power_offset) if power else []
    stats = defaultdict(StateStats)
    for s in segs:
        st = stats[s.state]
        st.time += s.duration
        st.count_time += s.duration if s.counted else 0.0
        st.visits += 1
        st.rates = s.rates
        for k, n in s.inferences.items():
            st.inferences[k] += n
        if samples:
            st.watts += power_in(s, samples)
    total = sum(st.time for st in stats.values()) or float("nan")
    models = sorted({k for st in stats.values() for k in st.inferences} | {k for st in stats.values() for k in st.rates})
    order = [s for s in STATES if s in stats] + sorted(s for s in stats if s not in STATES)
    date = date or dt.date.today().isoformat()
    src = ", ".join(Path(p).name for p in transition_paths)

    out = [f"### Scorecard {date}" + (f": {change}" if change else ""), "",
           f"Source: `{src}`, {len(segs)} timed segments in {len(sessions(rows))} session(s), "
           f"{_f(total)} s logged." + (f" Power: `{Path(power).name}`." if power else ""), ""]

    # 1. time share + inference rates
    head = ["state", "visits", "time s", "share %"] + [f"{m} /min (cfg)" for m in models] + ["all models /min"]
    body = []
    tot_inf = defaultdict(int)
    for s in order:
        st = stats[s]
        mins = st.count_time / 60 if st.count_time else float("nan")
        cells = [label(s), st.visits, _f(st.time), _f(100 * st.time / total)]
        for m in models:
            n = st.inferences.get(m, 0)
            tot_inf[m] += n
            cfg = st.rates.get(m)
            cfg_txt = "" if cfg is None else (cfg if cfg == "per_pose" else _f(float(cfg) * 60, 0))
            cells.append(f"{_f(n / mins)}" + (f" ({cfg_txt})" if cfg_txt else "") if (n or cfg) else "")
        cells.append(_f(sum(st.inferences.values()) / mins))
        body.append(cells)
    tmin = (sum(st.count_time for st in stats.values()) or float("nan")) / 60
    body.append(["**whole run**", sum(st.visits for st in stats.values()), _f(total), "100.0"]
                + [_f(tot_inf[m] / tmin) for m in models] + [_f(sum(tot_inf.values()) / tmin)])
    out += ["Time and inferences per state (measured per minute in the state; configured rate × 60 in brackets):", ""]
    out += _table(head, body) + [""]

    # 2. power
    if samples:
        measured = {s: sum(stats[s].watts) / len(stats[s].watts) for s in order if stats[s].watts}
        idle_note = ""
        if idle_watts is None and measured:
            low = min(measured, key=measured.get)
            idle_watts, idle_note = measured[low], f" (no --idle-watts: using the lowest state, {label(low)})"
        head = ["state", "avg W", "above idle W", "energy J", "inferences", "mJ / inference", "mJ / inference above idle"]
        body, e_tot, e_above, n_tot, t_cov = [], 0.0, 0.0, 0, 0.0
        for s in order:
            st = stats[s]
            if s not in measured:
                body.append([label(s), "no samples", "", "", sum(st.inferences.values()), "", ""])
                continue
            w = measured[s]
            n = sum(st.inferences.values())
            e, ea = w * st.time, (w - idle_watts) * st.time
            e_tot, e_above, n_tot, t_cov = e_tot + e, e_above + ea, n_tot + n, t_cov + st.time
            body.append([label(s), _f(w, 2), _f(w - idle_watts, 2), _f(e), n,
                         _f(1000 * e / n) if n else "", _f(1000 * ea / n) if n else ""])
        body.append(["**whole run**", _f(e_tot / t_cov, 2) if t_cov else "", _f(e_above / t_cov, 2) if t_cov else "",
                     _f(e_tot), n_tot, _f(1000 * e_tot / n_tot) if n_tot else "",
                     _f(1000 * e_above / n_tot) if n_tot else ""])
        out += [f"Power per state, idle baseline {_f(idle_watts, 2)} W{idle_note}. mJ / inference charges the "
                "state's whole power to its inferences (camera, motion and light included), so it is an upper "
                "bound; bench_detector.py gives the clean per-model number.", ""]
        out += _table(head, body) + [""]

    # 3. perfstats
    loop_fps = detect_ms = cam_ms = cpu = perf_w = None
    if perf:
        timings, system = read_perf(perf)
        head = ["timer", "calls", "mean ms", "p50", "p90", "p99", "max"]
        body = []
        for name in sorted(timings):
            ms = sorted(v for _, v in timings[name])
            body.append([name, len(ms), _f(sum(ms) / len(ms), 2), _f(_pct(ms, 50), 2), _f(_pct(ms, 90), 2),
                         _f(_pct(ms, 99), 2), _f(ms[-1], 2)])
        out += [f"Latency (`{Path(perf).name}`):", ""] + _table(head, body) + [""]
        cam = timings.get("camera.read")
        if cam and len(cam) > 1:
            span = cam[-1][0] - cam[0][0]
            loop_fps = (len(cam) - 1) / span if span > 0 else None
            cam_ms = sum(v for _, v in cam) / len(cam)
        det = timings.get("detector.total")
        detect_ms = sum(v for _, v in det) / len(det) if det else None
        cpu = sum(v for _, v in system["cpu_pct"]) / len(system["cpu_pct"]) if system.get("cpu_pct") else None
        perf_w = sum(v for _, v in system["power_w"]) / len(system["power_w"]) if system.get("power_w") else None

    # 4. the RESULTS.md "Full system" row
    avg_w = (sum(w for _, _, w in samples) / len(samples)) if samples else perf_w
    if perf or samples:
        out += ["Row for RESULTS.md → Full system:", "",
                "| date | change | loop fps | detect ms (mean) | camera.read ms | CPU % | avg W | idle W | notes |",
                "|---|---|---|---|---|---|---|---|---|",
                f"| {date} | {change} | {_f(loop_fps)} | {_f(detect_ms)} | {_f(cam_ms)} | {_f(cpu, 0)} | "
                f"{_f(avg_w, 2)} | {_f(idle_watts, 2)} | scorecard: {_f(sum(tot_inf.values()) / tmin)} inf/min "
                f"over {_f(total / 60)} min |", ""]

    out += ["Not counted / partly counted: " + "; ".join(f"{label(s)} from t={t:.1f} ({why})" for s, t, why in dropped) + ".", ""]
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("transitions", nargs="+", help="transitions CSV(s), in time order")
    ap.add_argument("--perf", help="perfstats CSV (PERF_CSV)")
    ap.add_argument("--power", help="power meter CSV: time + watts (or volts + amps)")
    ap.add_argument("--idle-watts", type=float, help="idle baseline W (board + camera, nothing running)")
    ap.add_argument("--power-offset", type=float, default=0.0, help="seconds to add to meter timestamps")
    ap.add_argument("--end-t", type=float, help="end of the run on the translog t clock (closes the last state)")
    ap.add_argument("--change", default="", help="what changed in this run (the RESULTS 'change' column)")
    ap.add_argument("--date", help="date for the heading (default today)")
    a = ap.parse_args(argv)
    print(scorecard(a.transitions, a.perf, a.power, a.idle_watts, a.end_t, a.power_offset, a.change, a.date))


if __name__ == "__main__":
    main()
