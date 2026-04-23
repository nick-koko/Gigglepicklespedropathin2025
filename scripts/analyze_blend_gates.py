"""Analyze Limelight blend accept/reject reasons in a SOTM CSV log.

Reproduces the gating logic from Pickles2025Teleop.java so we can see why
ll_pose_accepted is rarely True and which gate is dominating.

Usage:
    python scripts/analyze_blend_gates.py <csv_path>
"""

import csv
import sys
import statistics
from collections import Counter


def to_float(v, default=float("nan")):
    try:
        if v == "" or v is None:
            return default
        return float(v)
    except (TypeError, ValueError):
        return default


def to_bool(v):
    if v is None:
        return False
    s = str(v).strip().lower()
    return s in ("1", "true", "t", "yes", "y")


def normalize_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def median(xs):
    xs = [x for x in xs if x == x]
    if not xs:
        return 0.0
    return statistics.median(xs)


def main():
    path = sys.argv[1]

    # Constants from Pickles2025Teleop.java (current values).
    MAX_TRANS = 24.0
    MAX_HEAD = 10.0
    MAX_DELTA_HIST = 10.0
    HIST_WINDOW = 5  # number of recent samples kept

    dx_hist = []
    dy_hist = []

    total = 0
    mt_valid_count = 0
    blend_active_count = 0
    accepted_count = 0
    rej_translation = 0
    rej_heading = 0
    rej_history = 0
    pass_all = 0
    applied_count = 0
    reasons = Counter()

    max_raw_dist = 0.0
    raw_dist_series = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            total += 1
            blend_enabled = to_bool(row.get("ll_blend_enabled", ""))
            pose_valid = to_bool(row.get("ll_pose_valid", ""))
            pose_accepted = to_bool(row.get("ll_pose_accepted", ""))
            pose_applied = to_bool(row.get("ll_pose_applied", ""))
            raw_dx = to_float(row.get("ll_raw_dx_in", ""))
            raw_dy = to_float(row.get("ll_raw_dy_in", ""))
            raw_dist = to_float(row.get("ll_raw_dist_in", ""))
            raw_head = to_float(row.get("ll_raw_heading_err_deg", ""))

            if blend_enabled:
                blend_active_count += 1
            if pose_valid:
                mt_valid_count += 1
            if pose_accepted:
                accepted_count += 1
            if pose_applied:
                applied_count += 1

            # Emulate gates (only meaningful when pose_valid)
            if not pose_valid or raw_dist != raw_dist:
                continue
            raw_dist_series.append(raw_dist)
            max_raw_dist = max(max_raw_dist, raw_dist)

            translation_ok = raw_dist <= MAX_TRANS
            heading_ok = abs(raw_head) <= MAX_HEAD if raw_head == raw_head else False

            history_ok = True
            if len(dx_hist) >= 3:
                hdx = median(dx_hist[-HIST_WINDOW:])
                hdy = median(dy_hist[-HIST_WINDOW:])
                history_delta = ((raw_dx - hdx) ** 2 + (raw_dy - hdy) ** 2) ** 0.5
                history_ok = history_delta <= MAX_DELTA_HIST

            # A frame only gets pushed to history if it passes all gates.
            if not translation_ok:
                rej_translation += 1
                reasons["translation"] += 1
            elif not heading_ok:
                rej_heading += 1
                reasons["heading"] += 1
            elif not history_ok:
                rej_history += 1
                reasons["history"] += 1
            else:
                pass_all += 1
                reasons["accept"] += 1
                dx_hist.append(raw_dx)
                dy_hist.append(raw_dy)
                if len(dx_hist) > HIST_WINDOW:
                    dx_hist.pop(0)
                    dy_hist.pop(0)

    print(f"File: {path}")
    print(f"Total rows: {total}")
    print(f"  ll_blend_enabled: {blend_active_count}")
    print(f"  ll_pose_valid (MT sees tag): {mt_valid_count}")
    print(f"  ll_pose_accepted (logged):   {accepted_count}")
    print(f"  ll_pose_applied  (logged):   {applied_count}")
    print()
    print(f"Simulated gate breakdown (only valid-pose rows):")
    print(f"  passed all gates:        {pass_all}")
    print(f"  rejected - translation>  {MAX_TRANS}: {rej_translation}")
    print(f"  rejected - heading>      {MAX_HEAD}: {rej_heading}")
    print(f"  rejected - history delta>{MAX_DELTA_HIST}: {rej_history}")
    print()
    print(f"Raw dist stats (in): n={len(raw_dist_series)}")
    if raw_dist_series:
        print(f"  max  {max(raw_dist_series):.2f}")
        print(f"  p50  {statistics.median(raw_dist_series):.2f}")
        print(f"  p90  {sorted(raw_dist_series)[int(0.9*len(raw_dist_series))-1]:.2f}")
        print(f"  p99  {sorted(raw_dist_series)[int(0.99*len(raw_dist_series))-1]:.2f}")


if __name__ == "__main__":
    main()
