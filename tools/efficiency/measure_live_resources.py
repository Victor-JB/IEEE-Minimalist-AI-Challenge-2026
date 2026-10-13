#!/usr/bin/env python3
"""Read-only Linux sampler; records resource use, not electrical energy.

Run on the UNO Q while its services and browser demo remain running. Only
aggregate counts are retained from /spatial, never camera images or landmarks.
CPU percentages use the whole machine's logical CPU capacity (0–100%).
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import time
from urllib.request import urlopen


def fields_kib(path):
    result = {}
    for line in Path(path).read_text().splitlines():
        key, _, value = line.partition(":")
        parts = value.split()
        if parts and parts[0].isdigit():
            result[key] = int(parts[0])
    return result


def process_sample(pid):
    # comm may contain spaces or parentheses; fields after the final ')' start at 3.
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    status = fields_kib(f"/proc/{pid}/status")
    rollup = fields_kib(f"/proc/{pid}/smaps_rollup")
    return dict(pid=pid, start_ticks=int(fields[19]),
                cpu_ticks=int(fields[11]) + int(fields[12]),
                rss_kib=status["VmRSS"], pss_kib=rollup["Pss"],
                lifetime_peak_rss_kib=status["VmHWM"])


def system_sample():
    ticks = list(map(int, Path("/proc/stat").read_text().splitlines()[0].split()[1:9]))
    mem = fields_kib("/proc/meminfo")
    return dict(cpu_total_ticks=sum(ticks), cpu_busy_ticks=sum(ticks) - ticks[3] - ticks[4],
                ram_total_kib=mem["MemTotal"], ram_available_kib=mem["MemAvailable"],
                ram_used_kib=mem["MemTotal"] - mem["MemAvailable"])


def spatial_sample(url):
    try:
        with urlopen(url, timeout=1.5) as response:
            data = json.load(response)
        pipeline = data.get("pipeline", {})
        if data.get("error"):
            raise ValueError(f"Observer error: {data['error']}")
        age = data.get("age_s")
        if not pipeline or not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= 2:
            raise ValueError("Missing pipeline or stale/malformed observation (age must be 0–2 s)")
        return dict(counters={key: pipeline.get(key) for key in
                    ("face_calls", "hand_calls", "palm_calls", "gated_frames")},
                    mode=pipeline.get("mode"), uptime_s=pipeline.get("uptime_s"),
                    age_s=data.get("age_s"), inference_ms=data.get("inference_ms"),
                    faces=len(data.get("detections", [])), hands=len(data.get("hands", [])))
    except Exception as error:
        return dict(error=f"{type(error).__name__}: {error}")


def stats(values):
    if not values:
        return None
    return dict(mean=statistics.mean(values), min=min(values), max=max(values))


def summarize(samples, cpus, ticks_per_second):
    elapsed = samples[-1]["t_s"] - samples[0]["t_s"]
    intervals = []
    for a, b in zip(samples, samples[1:]):
        dt = b["t_s"] - a["t_s"]
        total = b["system"]["cpu_total_ticks"] - a["system"]["cpu_total_ticks"]
        row = dict(duration_s=dt, system_cpu_percent=100 *
                   (b["system"]["cpu_busy_ticks"] - a["system"]["cpu_busy_ticks"]) / total)
        for name in a["processes"]:
            before, after = a["processes"][name], b["processes"][name]
            if (before["pid"], before["start_ticks"]) != (after["pid"], after["start_ticks"]):
                raise RuntimeError(f"{name} restarted; cannot compare CPU counters")
            row[f"{name}_cpu_percent"] = 100 * (after["cpu_ticks"] - before["cpu_ticks"]) / ticks_per_second / dt / cpus
        row["app_cpu_percent"] = sum(row[f"{name}_cpu_percent"] for name in a["processes"])
        intervals.append(row)
    first, last = samples[0]["system"], samples[-1]["system"]
    summary = dict(duration_s=elapsed, interval_count=len(intervals),
                   system_cpu_percent=dict(mean=100 * (last["cpu_busy_ticks"] - first["cpu_busy_ticks"]) /
                    (last["cpu_total_ticks"] - first["cpu_total_ticks"]),
                    max=max(row["system_cpu_percent"] for row in intervals)),
                   system_ram_used_mib=stats([s["system"]["ram_used_kib"] / 1024 for s in samples]),
                   system_ram_total_mib=first["ram_total_kib"] / 1024,
                   processes={})
    for name in samples[0]["processes"]:
        cpu_s = (samples[-1]["processes"][name]["cpu_ticks"] - samples[0]["processes"][name]["cpu_ticks"]) / ticks_per_second
        summary["processes"][name] = dict(cpu_seconds=cpu_s,
            cpu_percent=dict(mean=100 * cpu_s / elapsed / cpus, max=max(row[f"{name}_cpu_percent"] for row in intervals)),
            rss_mib=stats([s["processes"][name]["rss_kib"] / 1024 for s in samples]),
            pss_mib=stats([s["processes"][name]["pss_kib"] / 1024 for s in samples]))
    summary["app_cpu_percent"] = dict(mean=sum(p["cpu_percent"]["mean"] for p in summary["processes"].values()),
                                     max=max(row["app_cpu_percent"] for row in intervals))
    summary["app_pss_mib"] = stats([sum(p["pss_kib"] for p in s["processes"].values()) / 1024 for s in samples])
    observations = [s["spatial"] for s in samples if "error" not in s["spatial"]]
    summary["workload"] = dict(successful_polls=len(observations), failed_polls=len(samples) - len(observations),
        mode_poll_counts=dict(Counter(s.get("mode") for s in observations)))
    if observations:
        summary["workload"].update(faces_seen=stats([s["faces"] for s in observations]), hands_seen=stats([s["hands"] for s in observations]),
            observation_age_s=stats([s["age_s"] for s in observations if s.get("age_s") is not None]))
        start, end = observations[0], observations[-1]
        window = (end.get("uptime_s") or 0) - (start.get("uptime_s") or 0)
        counters = {}
        for key, value in start["counters"].items():
            final = end["counters"].get(key)
            values = [s["counters"].get(key) for s in observations]
            continuous = all(isinstance(v, (int, float)) for v in values) and all(b >= a for a, b in zip(values, values[1:]))
            if continuous and value is not None and final is not None and window > 0:
                counters[key] = dict(count=final - value, per_second=(final - value) / window)
        summary["workload"].update(counter_window_s=window, counters=counters)
    return summary, intervals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--interval", type=float, default=1)
    parser.add_argument("--observer-url", default="http://127.0.0.1:8083/spatial")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.duration <= 0 or args.interval <= 0:
        parser.error("duration and interval must be positive")
    services = {"observer": "luxoiii-observer.service", "web": "luxoiii.service"}
    pids = {name: int(subprocess.check_output(["systemctl", "--user", "show", service, "-p", "MainPID", "--value"], text=True).strip()) for name, service in services.items()}
    if not all(pids.values()):
        raise RuntimeError("Both services must already be running")
    started_utc = datetime.now(timezone.utc).isoformat()
    start = time.monotonic()
    samples = []
    while True:
        row = dict(t_s=time.monotonic() - start, system=system_sample(),
                   processes={name: process_sample(pid) for name, pid in pids.items()})
        row["spatial"] = spatial_sample(args.observer_url)
        samples.append(row)
        if row["t_s"] >= args.duration:
            break
        target = min(len(samples) * args.interval, args.duration)
        time.sleep(max(0, start + target - time.monotonic()))
    cpus, hz = os.cpu_count(), os.sysconf("SC_CLK_TCK")
    summary, intervals = summarize(samples, cpus, hz)
    report = dict(schema_version=1, started_utc=started_utc, ended_utc=datetime.now(timezone.utc).isoformat(),
        platform=platform.platform(), logical_cpus=cpus, clock_ticks_per_second=hz,
        services=services, requested_interval_s=args.interval,
        scope="Live uncontrolled scene; observer and web are single-process services. No services paused or hardware controls changed. Browser rendering runs on the viewing computer.",
        definitions={
            "cpu_percent": "Percentage of all logical CPU capacity; 100% means all cores busy. Process CPU includes all threads. System idle and iowait are excluded from busy time.",
            "ram": "MiB = 1048576 bytes. System used = MemTotal - MemAvailable; process RSS is resident memory, PSS apportions shared pages. Combined app RAM sums PSS, not RSS.",
            "peak": "Largest sampled memory or interval CPU value in this window; not an instantaneous or lifetime peak.",
            "workload": "face_calls = face passes; hand_calls = hand pipeline updates; palm_calls = palm model invocations. These overlap and must not be summed as total neural inferences. Rates use observer uptime between counter snapshots.",
            "sampling": "Approximately 1 Hz /proc plus read-only /spatial HTTP polling; sampler overhead is included in whole-board metrics. Only counts and timings are retained from the camera API."},
        energy=dict(status="not_measured", inferences_per_joule=None, reason="No exposed power/energy sensor and no external power measurement available.",
                    formula="Defined completed workloads / measured input joules, or matching workloads per second / average measured watts.",
                    limitation="CPU utilization and supply ratings cannot establish electrical energy or inferences per joule."),
        summary=summary, intervals=intervals, samples=samples)
    serialized = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized)
        print(json.dumps(summary, indent=2))
    else:
        print(serialized, end="")


if __name__ == "__main__":
    main()
