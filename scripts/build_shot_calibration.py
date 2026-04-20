"""
Aggregate shot-tuning CSV logs into an updated ShotCalibrationTable seed.

Reads every `pickles2025_shot_tuning_*.csv` file under the configured log
directory, filters rows where `label == KEEP` and `cal_point_index >= 0`,
groups by `cal_point_index`, computes the median `target_rpm_fire`,
`hood_pos_fire`, `target_x`, and `target_y` per group, and emits two artifacts:

    1. A Java `new ShotSample(...)` block that can be pasted into
       ShotCalibrationTable.DEFAULT, replacing only the indices for which
       KEEP shots are available. Indices without KEEP shots fall back to
       their existing seed values (left as-is in the output).

    2. A markdown drift/coverage report written to
       `scripts/shot_calibration_report.md` listing: count per index, median
       vs seed delta, and warning rows where bot-pose drift from the pinned
       waypoint exceeds 3 inches (suggesting the driver was not actually
       parked at the target when the shot was labeled KEEP, and the value
       should be treated with suspicion or re-tested).

Usage:
    python scripts/build_shot_calibration.py [--log-dir path] [--out out.java]

The script does not modify the Java source directly; it prints the new
ShotSample block to stdout and leaves integration (copy/paste) to the
operator. This is intentional: table edits are safety-critical and should
be reviewed before commit.
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional


# Seed values baked into ShotCalibrationTable.java at the time this script was
# written. Used for "no KEEP shots yet -> fall back to seed" and for delta
# reporting. Keep in sync with ShotCalibrationTable.DEFAULT.
SEED_ROWS: List[Dict[str, object]] = [
    {"idx": 0, "x": 72.0, "y": 72.0, "rpm": 3265.0, "hood": 0.525, "aimX": -6.0, "aimY": 144.0, "zone": "A"},
    {"idx": 1, "x": 36.0, "y": 108.0, "rpm": 2800.0, "hood": 0.22, "aimX": -0.5, "aimY": 133.0, "zone": "A"},
    {"idx": 2, "x": 108.0, "y": 108.0, "rpm": 4000.0, "hood": 0.525, "aimX": -1.5, "aimY": 128.0, "zone": "A"},
    {"idx": 3, "x": 72.0, "y": 108.0, "rpm": 3400.0, "hood": 0.44, "aimX": 0.5, "aimY": 132.0, "zone": "A"},
    {"idx": 4, "x": 40.0, "y": 134.0, "rpm": 2700.0, "hood": 0.00, "aimX": 0.0, "aimY": 134.0, "zone": "A"},
    {"idx": 5, "x": 50.0, "y": 87.0, "rpm": 3325.0, "hood": 0.525, "aimX": 1.5, "aimY": 136.0, "zone": "A"},
    {"idx": 6, "x": 72.0, "y": 130.0, "rpm": 3250.0, "hood": 0.44, "aimX": 0.5, "aimY": 132.0, "zone": "A"},
    {"idx": 7, "x": 86.0, "y": 89.0, "rpm": 3500.0, "hood": 0.525, "aimX": -0.5, "aimY": 129.0, "zone": "A"},
    {"idx": 8, "x": 110.0, "y": 130.0, "rpm": 3875.0, "hood": 0.525, "aimX": -0.5, "aimY": 135.0, "zone": "A"},
    {"idx": 9, "x": 72.0, "y": 24.0, "rpm": 4150.0, "hood": 0.525, "aimX": 1.5, "aimY": 129.0, "zone": "B"},
    {"idx": 10, "x": 54.0, "y": 10.0, "rpm": 4150.0, "hood": 0.525, "aimX": 1.5, "aimY": 129.0, "zone": "B"},
    {"idx": 11, "x": 72.0, "y": 12.0, "rpm": 4200.0, "hood": 0.525, "aimX": 1.5, "aimY": 129.0, "zone": "B"},
    {"idx": 12, "x": 90.0, "y": 14.0, "rpm": 4450.0, "hood": 0.525, "aimX": -1.5, "aimY": 134.0, "zone": "B"},
]

DRIFT_WARN_IN = 3.0


@dataclass
class SampleBucket:
    idx: int
    rpms: List[float] = field(default_factory=list)
    hoods: List[float] = field(default_factory=list)
    aim_xs: List[float] = field(default_factory=list)
    aim_ys: List[float] = field(default_factory=list)
    bot_xs: List[float] = field(default_factory=list)
    bot_ys: List[float] = field(default_factory=list)

    def add(self, row: Dict[str, str]) -> None:
        self.rpms.append(float(row["target_rpm_fire"]))
        self.hoods.append(float(row["hood_pos_fire"]))
        self.aim_xs.append(float(row["target_x"]))
        self.aim_ys.append(float(row["target_y"]))
        self.bot_xs.append(float(row["bot_x_fire"]))
        self.bot_ys.append(float(row["bot_y_fire"]))

    @property
    def count(self) -> int:
        return len(self.rpms)


def find_csv_files(log_dir: Path) -> List[Path]:
    return sorted(log_dir.rglob("pickles2025_shot_tuning*.csv"))


def load_keep_rows(paths: Iterable[Path]) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    for path in paths:
        try:
            with path.open("r", newline="", encoding="utf-8") as fh:
                reader = csv.DictReader(fh)
                for row in reader:
                    label = (row.get("label") or "").strip().upper()
                    cal_str = (row.get("cal_point_index") or "").strip()
                    if label != "KEEP" or not cal_str:
                        continue
                    try:
                        idx = int(float(cal_str))
                    except ValueError:
                        continue
                    if idx < 0:
                        continue
                    row["_cal_idx"] = idx
                    row["_source_file"] = path.name
                    rows.append(row)
        except (OSError, UnicodeDecodeError) as exc:
            print(f"[warn] skipping {path}: {exc}", file=sys.stderr)
    return rows


def bucket_rows(rows: Iterable[Dict[str, str]]) -> Dict[int, SampleBucket]:
    buckets: Dict[int, SampleBucket] = defaultdict(lambda: SampleBucket(idx=-1))
    for row in rows:
        idx = row["_cal_idx"]
        bucket = buckets[idx]
        bucket.idx = idx
        try:
            bucket.add(row)
        except (KeyError, ValueError) as exc:
            print(f"[warn] skipping malformed row in {row.get('_source_file')}: {exc}", file=sys.stderr)
    return buckets


def median_or(values: List[float], fallback: float) -> float:
    if not values:
        return fallback
    return statistics.median(values)


def emit_java_block(buckets: Dict[int, SampleBucket]) -> str:
    lines = ["        // AUTO-GENERATED BY scripts/build_shot_calibration.py",
             "        // Review deltas vs previous seed before pasting."]
    for seed in SEED_ROWS:
        idx = int(seed["idx"])
        bucket = buckets.get(idx)
        if bucket and bucket.count > 0:
            rpm = median_or(bucket.rpms, float(seed["rpm"]))
            hood = median_or(bucket.hoods, float(seed["hood"]))
            aim_x = median_or(bucket.aim_xs, float(seed["aimX"]))
            aim_y = median_or(bucket.aim_ys, float(seed["aimY"]))
            note = f"median of {bucket.count} KEEP"
        else:
            rpm = float(seed["rpm"])
            hood = float(seed["hood"])
            aim_x = float(seed["aimX"])
            aim_y = float(seed["aimY"])
            note = "seed fallback (no KEEP yet)"
        lines.append(
            f"        new ShotSample({idx}, {float(seed['x']):.1f}, {float(seed['y']):.1f}, "
            f"{rpm:.0f}, {hood:.2f}, {aim_x:.1f}, {aim_y:.1f}, \"{seed['zone']}\", \"{note}\"),"
        )
    return "\n".join(lines)


def emit_report(buckets: Dict[int, SampleBucket]) -> str:
    out = ["# Shot calibration aggregation report",
           "",
           f"Drift-warn threshold: {DRIFT_WARN_IN} in",
           "",
           "| idx | x | y | n | rpm_med | hood_med | aim_x_med | aim_y_med | drift_max_in | warn |",
           "|-----|---|---|---|---------|----------|-----------|-----------|--------------|------|"]
    for seed in SEED_ROWS:
        idx = int(seed["idx"])
        bucket = buckets.get(idx)
        if bucket and bucket.count > 0:
            drift_max = max(
                ((bx - float(seed["x"])) ** 2 + (by - float(seed["y"])) ** 2) ** 0.5
                for bx, by in zip(bucket.bot_xs, bucket.bot_ys)
            )
            warn = "DRIFT>{}".format(DRIFT_WARN_IN) if drift_max > DRIFT_WARN_IN else ""
            rpm_med = median_or(bucket.rpms, float(seed["rpm"]))
            hood_med = median_or(bucket.hoods, float(seed["hood"]))
            aim_x_med = median_or(bucket.aim_xs, float(seed["aimX"]))
            aim_y_med = median_or(bucket.aim_ys, float(seed["aimY"]))
            out.append(
                f"| {idx} | {seed['x']} | {seed['y']} | {bucket.count} | "
                f"{rpm_med:.0f} | {hood_med:.2f} | {aim_x_med:.1f} | {aim_y_med:.1f} | "
                f"{drift_max:.2f} | {warn} |"
            )
        else:
            out.append(
                f"| {idx} | {seed['x']} | {seed['y']} | 0 | "
                f"{float(seed['rpm']):.0f} | {float(seed['hood']):.2f} | "
                f"{float(seed['aimX']):.1f} | {float(seed['aimY']):.1f} | - | NO_DATA |"
            )
    return "\n".join(out) + "\n"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    default_log_dir = Path(os.environ.get("PICKLES_LOG_DIR", "scripts/logs"))
    parser.add_argument("--log-dir", type=Path, default=default_log_dir,
                        help=f"directory to scan for shot-tuning CSVs (default: {default_log_dir})")
    parser.add_argument("--report", type=Path, default=Path("scripts/shot_calibration_report.md"),
                        help="markdown report output path")
    args = parser.parse_args(argv)

    if not args.log_dir.exists():
        print(f"[error] log-dir does not exist: {args.log_dir}", file=sys.stderr)
        return 2

    csv_files = find_csv_files(args.log_dir)
    if not csv_files:
        print(f"[warn] no pickles2025_shot_tuning*.csv files found under {args.log_dir}",
              file=sys.stderr)

    rows = load_keep_rows(csv_files)
    buckets = bucket_rows(rows)

    print(emit_java_block(buckets))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(emit_report(buckets), encoding="utf-8")
    print(f"\n[ok] report written: {args.report}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
