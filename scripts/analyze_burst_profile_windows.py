#!/usr/bin/env python3
"""
Estimate burst timing inference windows from existing CSV logs.

Primary outputs (grouped by burst_profile_id):
- sequence completion and end-reason summaries
- feed-latency and shot-spacing timer recommendations
- bb1->bb2 delay windows for shot1/2/3
- bb2 clear-gap timing windows (between ball events)
- edge visibility rates for bb1/bb2 timestamps
- high-rate stream loop-time and edge-count summaries (if available)

This script is robust to mixed old/new schemas:
- Old files without burst_profile_id are grouped under profile -1.
- If linked IDs/cumulative counters are absent, edge pairing falls back safely.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple


def parse_float(value: object) -> Optional[float]:
    try:
        if value is None:
            return None
        text = str(value).strip()
        if text == "":
            return None
        return float(text)
    except Exception:
        return None


def parse_int(value: object, default: int = -1) -> int:
    number = parse_float(value)
    if number is None or not math.isfinite(number):
        return default
    return int(round(number))


def parse_bool(value: object) -> bool:
    if value is None:
        return False
    text = str(value).strip().lower()
    return text in {"1", "1.0", "true", "t", "yes", "y"}


def percentile(sorted_values: List[float], p: float) -> float:
    if not sorted_values:
        return float("nan")
    idx = int((len(sorted_values) - 1) * p)
    return sorted_values[idx]


def format_stat(values: List[float]) -> str:
    if not values:
        return "n=0"
    vals = sorted(values)
    p10 = percentile(vals, 0.10)
    p50 = percentile(vals, 0.50)
    p90 = percentile(vals, 0.90)
    return "n={} p10/p50/p90={:.0f}/{:.0f}/{:.0f}".format(len(vals), p10, p50, p90)


def recommend_window(values: List[float], margin_ms: float = 10.0) -> Optional[Tuple[int, int]]:
    if not values:
        return None
    vals = sorted(values)
    low = max(0.0, percentile(vals, 0.10) - margin_ms)
    high = percentile(vals, 0.90) + margin_ms
    return int(round(low)), int(round(high))


def combine_windows(a: Optional[Tuple[int, int]], b: Optional[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
    if a is None:
        return b
    if b is None:
        return a
    return min(a[0], b[0]), max(a[1], b[1])


def get_profile_id(row: Dict[str, str]) -> int:
    if "burst_profile_id" not in row:
        return -1
    return parse_int(row.get("burst_profile_id"), default=-1)


@dataclass
class Completion:
    expected_total: int = 0
    completed_total: int = 0
    end_reasons: Dict[str, int] = field(default_factory=dict)


@dataclass
class Visibility:
    expected_rows: int = 0
    bb1_seen: int = 0
    bb2_seen: int = 0


def collect_from_shot_breakbeam(
    shot_files: Iterable[str],
) -> Tuple[
    Dict[int, Dict[int, List[float]]],
    Dict[int, Dict[str, List[float]]],
    Dict[int, Completion],
    Dict[int, Dict[int, Visibility]],
]:
    # profile -> shot_index(1..3) -> list of (bb2_fall - bb1_fall) ms
    b1_to_b2: Dict[int, Dict[int, List[float]]] = defaultdict(lambda: defaultdict(list))
    # profile -> interval_name -> list (shot1/2/3 interval ms)
    intervals: Dict[int, Dict[str, List[float]]] = defaultdict(lambda: defaultdict(list))
    completion: Dict[int, Completion] = defaultdict(Completion)
    visibility: Dict[int, Dict[int, Visibility]] = defaultdict(lambda: defaultdict(Visibility))

    for path in shot_files:
        with open(path, newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                if row.get("trigger_reason") != "dumbshoot":
                    continue
                profile = get_profile_id(row)

                expected = parse_int(row.get("expected_shots"), default=-1)
                observed = parse_int(row.get("bb2_fall_count"), default=-1)
                if expected > 0:
                    completion[profile].expected_total += 1
                    if observed >= expected:
                        completion[profile].completed_total += 1
                end_reason = (row.get("end_reason") or "").strip()
                if end_reason:
                    completion[profile].end_reasons[end_reason] = completion[profile].end_reasons.get(end_reason, 0) + 1

                for idx in (1, 2, 3):
                    b1 = parse_float(row.get(f"bb1_fall_{idx}_ms"))
                    b2 = parse_float(row.get(f"bb2_fall_{idx}_ms"))
                    vis = visibility[profile][idx]
                    vis.expected_rows += 1
                    if b1 is not None and b1 > 0:
                        vis.bb1_seen += 1
                    if b2 is not None and b2 > 0:
                        vis.bb2_seen += 1

                    if b1 is None or b2 is None:
                        continue
                    # Legacy logs may encode missing events as 0; ignore those sentinels.
                    if b1 <= 0 or b2 <= 0:
                        continue
                    if b2 >= b1:
                        b1_to_b2[profile][idx].append(b2 - b1)

                for name in ("shot1_interval_ms", "shot2_interval_ms", "shot3_interval_ms"):
                    value = parse_float(row.get(name))
                    if value is not None and value > 0:
                        intervals[profile][name].append(value)

                for name in ("bb2_clear_gap_1to2_ms", "bb2_clear_gap_2to3_ms"):
                    value = parse_float(row.get(name))
                    if value is not None and value > 0:
                        intervals[profile][name].append(value)

    return b1_to_b2, intervals, completion, visibility


def collect_from_dumbshoot_rpm(
    rpm_files: Iterable[str],
) -> Tuple[Dict[int, Dict[int, List[float]]], Dict[int, List[float]], Dict[int, Dict[str, List[int]]]]:
    """
    Derive bb1->bb2 delays from high-rate stream by sequence.
    Returns profile -> shot_index -> list delay_ms.
    """
    derived: Dict[int, Dict[int, List[float]]] = defaultdict(lambda: defaultdict(list))
    loop_times: Dict[int, List[float]] = defaultdict(list)
    edge_counts: Dict[int, Dict[str, List[int]]] = defaultdict(lambda: defaultdict(list))

    for path in rpm_files:
        with open(path, newline="", encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))

        by_seq: Dict[int, List[Dict[str, str]]] = defaultdict(list)
        for row in rows:
            seq = parse_int(row.get("sequence_id"), default=-1)
            if seq < 0:
                continue
            by_seq[seq].append(row)

        for seq_rows in by_seq.values():
            seq_rows.sort(
                key=lambda r: (
                    parse_float(r.get("t_ms")) or 0.0,
                    parse_float(r.get("t_since_start_ms")) or 0.0,
                )
            )
            profile = get_profile_id(seq_rows[0])
            for row in seq_rows:
                loop = parse_float(row.get("loop_time_ms"))
                if loop is not None and loop > 0:
                    loop_times[profile].append(loop)

            b1_fall_times: Dict[int, float] = {}
            b2_fall_times: Dict[int, float] = {}
            b1_counter = 0
            b2_counter = 0

            has_cumulative = "bb1_fall_count_total" in seq_rows[0] and "bb2_fall_count_total" in seq_rows[0]
            has_b0_cumulative = "bb0_fall_count_total" in seq_rows[0]

            for row in seq_rows:
                t_rel = parse_float(row.get("t_since_start_ms"))
                if t_rel is None:
                    continue

                b1_edge = parse_bool(row.get("bb1_fall_edge"))
                b2_edge = parse_bool(row.get("bb2_fall_edge"))
                if not b1_edge and not b2_edge:
                    continue

                if has_cumulative:
                    if b1_edge:
                        idx = parse_int(row.get("bb1_fall_count_total"), default=-1)
                        if idx > 0 and idx not in b1_fall_times:
                            b1_fall_times[idx] = t_rel
                    if b2_edge:
                        idx = parse_int(row.get("bb2_fall_count_total"), default=-1)
                        if idx > 0 and idx not in b2_fall_times:
                            b2_fall_times[idx] = t_rel
                else:
                    if b1_edge:
                        b1_counter += 1
                        if b1_counter not in b1_fall_times:
                            b1_fall_times[b1_counter] = t_rel
                    if b2_edge:
                        b2_counter += 1
                        if b2_counter not in b2_fall_times:
                            b2_fall_times[b2_counter] = t_rel

            for idx in (1, 2, 3):
                if idx in b1_fall_times and idx in b2_fall_times:
                    dt = b2_fall_times[idx] - b1_fall_times[idx]
                    if dt >= 0:
                        derived[profile][idx].append(dt)

            # per-sequence final cumulative counts (best available from stream)
            last = seq_rows[-1]
            if has_b0_cumulative:
                for key in (
                    "bb0_rise_count_total",
                    "bb0_fall_count_total",
                    "bb1_rise_count_total",
                    "bb1_fall_count_total",
                    "bb2_rise_count_total",
                    "bb2_fall_count_total",
                ):
                    value = parse_int(last.get(key), default=-1)
                    if value >= 0:
                        edge_counts[profile][key].append(value)
            else:
                # fallback for older logs: count from edge flags
                counts = {
                    "bb0_rise_count_total": sum(1 for r in seq_rows if parse_bool(r.get("bb0_rise_edge"))),
                    "bb0_fall_count_total": sum(1 for r in seq_rows if parse_bool(r.get("bb0_fall_edge"))),
                    "bb1_rise_count_total": sum(1 for r in seq_rows if parse_bool(r.get("bb1_rise_edge"))),
                    "bb1_fall_count_total": sum(1 for r in seq_rows if parse_bool(r.get("bb1_fall_edge"))),
                    "bb2_rise_count_total": sum(1 for r in seq_rows if parse_bool(r.get("bb2_rise_edge"))),
                    "bb2_fall_count_total": sum(1 for r in seq_rows if parse_bool(r.get("bb2_fall_edge"))),
                }
                for key, value in counts.items():
                    edge_counts[profile][key].append(value)

    return derived, loop_times, edge_counts


def print_window_line(indent: str, name: str, values: List[float], margin_ms: float) -> None:
    rec = recommend_window(values, margin_ms=margin_ms)
    rec_text = "recommended={}..{}ms".format(rec[0], rec[1]) if rec else "recommended=n/a"
    print("{}{}: {} {}".format(indent, name, format_stat(values), rec_text))


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze burst timing windows by profile.")
    parser.add_argument(
        "--shot-glob",
        default="Data/pickles2025_shot_breakbeam_*.csv",
        help="Glob for shot summary logs.",
    )
    parser.add_argument(
        "--rpm-glob",
        default="Data/pickles2025_dumbshoot_rpm_*.csv",
        help="Glob for high-rate dumbshoot RPM logs.",
    )
    parser.add_argument(
        "--window-margin-ms",
        type=float,
        default=10.0,
        help="Margin added around p10/p90 for recommended windows.",
    )
    args = parser.parse_args()

    shot_files = sorted(glob.glob(args.shot_glob))
    rpm_files = sorted(glob.glob(args.rpm_glob))

    print("Shot summary files:", len(shot_files))
    print("Dumbshoot RPM files:", len(rpm_files))
    if not shot_files and not rpm_files:
        print("No input files matched.")
        return 1

    shot_b1_to_b2, shot_intervals, completion, visibility = collect_from_shot_breakbeam(shot_files)
    rpm_b1_to_b2, rpm_loop_times, rpm_edge_counts = collect_from_dumbshoot_rpm(rpm_files)

    profiles = sorted(
        set(shot_b1_to_b2.keys())
        | set(shot_intervals.keys())
        | set(completion.keys())
        | set(visibility.keys())
        | set(rpm_b1_to_b2.keys())
        | set(rpm_loop_times.keys())
        | set(rpm_edge_counts.keys())
    )
    if not profiles:
        print("No relevant dumbshoot rows found.")
        return 1

    print("\n=== Burst Timing Recommendations ===")
    for profile in profiles:
        comp = completion.get(profile, Completion())
        if comp.expected_total > 0:
            rate = comp.completed_total / comp.expected_total
            comp_text = "completion {}/{} ({:.1%})".format(comp.completed_total, comp.expected_total, rate)
        else:
            comp_text = "completion n/a"

        print("\nProfile {} -> {}".format(profile, comp_text))
        if comp.end_reasons:
            top = sorted(comp.end_reasons.items(), key=lambda kv: kv[1], reverse=True)
            print("  end reasons:", ", ".join("{}={}".format(k, v) for k, v in top))

        # Timer backbone recommendations
        print("  timer backbone (shot summary):")
        print_window_line("    ", "feed_latency_ms (shot1_interval_ms)", shot_intervals.get(profile, {}).get("shot1_interval_ms", []), args.window_margin_ms)
        print_window_line("    ", "t12_est_ms (shot2_interval_ms)", shot_intervals.get(profile, {}).get("shot2_interval_ms", []), args.window_margin_ms)
        print_window_line("    ", "t23_est_ms (shot3_interval_ms)", shot_intervals.get(profile, {}).get("shot3_interval_ms", []), args.window_margin_ms)
        print_window_line("    ", "bb2_clear_gap_1to2_ms", shot_intervals.get(profile, {}).get("bb2_clear_gap_1to2_ms", []), args.window_margin_ms)
        print_window_line("    ", "bb2_clear_gap_2to3_ms", shot_intervals.get(profile, {}).get("bb2_clear_gap_2to3_ms", []), args.window_margin_ms)

        print("  bb1->bb2 from shot summary:")
        for idx in (1, 2, 3):
            vals = shot_b1_to_b2.get(profile, {}).get(idx, [])
            print_window_line("    ", f"shot{idx}", vals, args.window_margin_ms)

        print("  bb1->bb2 from high-rate stream:")
        for idx in (1, 2, 3):
            vals = rpm_b1_to_b2.get(profile, {}).get(idx, [])
            print_window_line("    ", f"shot{idx}", vals, args.window_margin_ms)

        print("  bb1->bb2 combined recommendation:")
        for idx in (1, 2, 3):
            shot_win = recommend_window(shot_b1_to_b2.get(profile, {}).get(idx, []), args.window_margin_ms)
            rpm_win = recommend_window(rpm_b1_to_b2.get(profile, {}).get(idx, []), args.window_margin_ms)
            merged = combine_windows(shot_win, rpm_win)
            if merged is None:
                print("    shot{}: n/a".format(idx))
            else:
                print("    shot{}: {}..{}ms".format(idx, merged[0], merged[1]))

        print("  edge visibility from shot summary:")
        for idx in (1, 2, 3):
            vis = visibility.get(profile, {}).get(idx, Visibility())
            if vis.expected_rows <= 0:
                print("    shot{}: n/a".format(idx))
                continue
            bb1_rate = vis.bb1_seen / vis.expected_rows
            bb2_rate = vis.bb2_seen / vis.expected_rows
            print(
                "    shot{}: bb1_seen={}/{} ({:.1%}), bb2_seen={}/{} ({:.1%})".format(
                    idx,
                    vis.bb1_seen,
                    vis.expected_rows,
                    bb1_rate,
                    vis.bb2_seen,
                    vis.expected_rows,
                    bb2_rate,
                )
            )

        loops = rpm_loop_times.get(profile, [])
        if loops:
            print("  high-rate stream loop_time_ms:", format_stat(loops))

        counts = rpm_edge_counts.get(profile, {})
        if counts:
            print("  high-rate stream edge-count totals per sequence:")
            for key in (
                "bb0_rise_count_total",
                "bb0_fall_count_total",
                "bb1_rise_count_total",
                "bb1_fall_count_total",
                "bb2_rise_count_total",
                "bb2_fall_count_total",
            ):
                vals = counts.get(key, [])
                if vals:
                    print("    {}: {}".format(key, format_stat([float(v) for v in vals])))

    print("\nNote: profile -1 means legacy logs without burst_profile_id.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

