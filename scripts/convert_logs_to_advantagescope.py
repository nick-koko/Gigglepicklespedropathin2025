#!/usr/bin/env python3
"""
Convert existing Pickles CSV logs to AdvantageScope-compatible CSV.

Usage examples:
  python scripts/convert_logs_to_advantagescope.py --input Data/pickles2025_turret_20260328_125859.csv
  python scripts/convert_logs_to_advantagescope.py --input Data/*.csv --merge --output Data/pickles2025_all_ascope.csv
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple


TIME_KEYS_PRIORITY = ("t_ms", "timestamp_ms", "time_ms", "match_t_ms")
OUTPUT_TIME_HEADER = "Timestamp"
OUTPUT_LIST_HEADERS = [OUTPUT_TIME_HEADER, "Key", "Value"]
INCHES_TO_METERS = 0.0254
PEDRO_FIELD_CENTER_IN = 72.0
DEFAULT_TURRET_COMPONENT_X_IN = 0.0
DEFAULT_TURRET_COMPONENT_Y_IN = 0.0
DEFAULT_TURRET_COMPONENT_Z_IN = 0.0


@dataclass
class SourceData:
    path: str
    prefix: str
    rows: List[Dict[str, str]]
    output_columns: List[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert FTC CSV logs into AdvantageScope-friendly CSV format."
    )
    parser.add_argument(
        "--input",
        nargs="+",
        required=True,
        help="Input CSV paths (supports glob patterns).",
    )
    parser.add_argument(
        "--output",
        help=(
            "Output CSV path. If omitted, per-file outputs are generated next to each input "
            "with suffix '_ascope.csv'."
        ),
    )
    parser.add_argument(
        "--merge",
        action="store_true",
        help="Merge all inputs into one output CSV (requires --output).",
    )
    parser.add_argument(
        "--no-prefix",
        action="store_true",
        help="Do not prefix field names with file stem (useful for single-file conversion).",
    )
    parser.add_argument(
        "--format",
        choices=("table", "field3d"),
        default="table",
        help=(
            "Output format. 'table' creates wide CSV for plots. "
            "'field3d' creates Timestamp/Key/Value CSV with legacy geometry arrays "
            "for 3D Field usage."
        ),
    )
    parser.add_argument(
        "--turret-component-x-in",
        type=float,
        default=DEFAULT_TURRET_COMPONENT_X_IN,
        help="Robot-relative turret component X offset (inches, forward+).",
    )
    parser.add_argument(
        "--turret-component-y-in",
        type=float,
        default=DEFAULT_TURRET_COMPONENT_Y_IN,
        help="Robot-relative turret component Y offset (inches, left+).",
    )
    parser.add_argument(
        "--turret-component-z-in",
        type=float,
        default=DEFAULT_TURRET_COMPONENT_Z_IN,
        help="Robot-relative turret component Z offset (inches, up+).",
    )
    parser.add_argument(
        "--skip-turret-component",
        action="store_true",
        help="Disable turret component Pose3d export in field3d mode.",
    )
    return parser.parse_args()


def expand_inputs(patterns: Iterable[str]) -> List[str]:
    files: List[str] = []
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            files.extend(matches)
        elif os.path.isfile(pattern):
            files.append(pattern)
    unique = []
    seen = set()
    for path in files:
        norm = os.path.normpath(path)
        if norm.lower().endswith(".csv") and norm not in seen:
            seen.add(norm)
            unique.append(norm)
    return unique


def infer_prefix(path: str) -> str:
    stem = os.path.splitext(os.path.basename(path))[0]
    cleaned = "".join(ch if ch.isalnum() else "_" for ch in stem).strip("_")
    return cleaned or "log"


def pick_time_column(fieldnames: Iterable[str]) -> Optional[str]:
    names = {name.strip() for name in fieldnames if name}
    for key in TIME_KEYS_PRIORITY:
        if key in names:
            return key
    return None


def normalize_value(raw: str) -> str:
    if raw is None:
        return ""
    value = raw.strip()
    if value == "":
        return ""
    upper = value.upper()
    if upper == "TRUE":
        return "1"
    if upper == "FALSE":
        return "0"
    if upper in ("NAN", "+NAN", "-NAN", "INF", "+INF", "-INF", "INFINITY", "+INFINITY", "-INFINITY"):
        return ""
    return value


def parse_time_seconds(row: Dict[str, str], time_key: str) -> Optional[float]:
    raw = normalize_value(row.get(time_key, ""))
    if raw == "":
        return None
    try:
        ms = float(raw)
        return ms / 1000.0
    except ValueError:
        return None


def parse_float(row: Dict[str, str], keys: Iterable[str]) -> Optional[float]:
    for key in keys:
        raw = normalize_value(row.get(key, ""))
        if raw == "":
            continue
        try:
            value = float(raw)
            if not math.isfinite(value):
                continue
            return value
        except ValueError:
            continue
    return None


def format_legacy_array(values: Iterable[float]) -> str:
    return "[" + "; ".join(f"{value:.6f}" for value in values) + "]"


def normalize_angle_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion_wxyz(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return math.cos(half), 0.0, 0.0, math.sin(half)


def pedro_inches_to_ftc_decode_meters(x_in: float, y_in: float) -> Tuple[float, float]:
    """
    Convert Pedro coordinates (origin bottom-left, inches) to FTC DECODE coordinates
    (origin center, inverted FTC frame), then to meters.

    From Pedro's own transform:
      InvertedFTCCoordinates.convertFromPedro = (pose - [72,72]) rotated +90deg.
    """
    dx = x_in - PEDRO_FIELD_CENTER_IN
    dy = y_in - PEDRO_FIELD_CENTER_IN
    ftc_x_in = -dy
    ftc_y_in = dx
    return ftc_x_in * INCHES_TO_METERS, ftc_y_in * INCHES_TO_METERS


def convert_file(path: str, use_prefix: bool) -> SourceData:
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"No header found: {path}")
        time_key = pick_time_column(reader.fieldnames)
        if time_key is None:
            raise ValueError(
                f"Could not find a time column in {path}. Expected one of: {TIME_KEYS_PRIORITY}"
            )

        prefix = infer_prefix(path)
        output_cols: List[str] = []
        out_rows: List[Dict[str, str]] = []

        for field in reader.fieldnames:
            if not field:
                continue
            field = field.strip()
            if field == time_key:
                continue
            col = f"{prefix}/{field}" if use_prefix else field
            output_cols.append(col)

        for row in reader:
            t_sec = parse_time_seconds(row, time_key)
            if t_sec is None:
                continue
            out: Dict[str, str] = {OUTPUT_TIME_HEADER: f"{t_sec:.6f}"}
            for field in reader.fieldnames:
                if not field:
                    continue
                field = field.strip()
                if field == time_key:
                    continue
                col = f"{prefix}/{field}" if use_prefix else field
                out[col] = normalize_value(row.get(field, ""))
            out_rows.append(out)

    return SourceData(path=path, prefix=prefix, rows=out_rows, output_columns=output_cols)


def convert_file_field3d(
    path: str,
    use_prefix: bool,
    turret_component_xyz_m: Tuple[float, float, float],
    include_turret_component: bool,
) -> List[Dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"No header found: {path}")
        time_key = pick_time_column(reader.fieldnames)
        if time_key is None:
            raise ValueError(
                f"Could not find a time column in {path}. Expected one of: {TIME_KEYS_PRIORITY}"
            )

        prefix = infer_prefix(path)
        base = f"/{prefix}/" if use_prefix else "/"
        rows_out: List[Dict[str, str]] = []
        has_geometry = False
        turret_x_m, turret_y_m, turret_z_m = turret_component_xyz_m

        for row in reader:
            t_sec = parse_time_seconds(row, time_key)
            if t_sec is None:
                continue
            t_str = f"{t_sec:.6f}"

            bot_x = parse_float(row, ("bot_x", "x", "robot_x"))
            bot_y = parse_float(row, ("bot_y", "y", "robot_y"))
            heading_deg = parse_float(row, ("bot_heading_deg", "heading_deg", "robot_heading_deg"))
            heading_rad = parse_float(row, ("bot_heading_rad", "heading_rad", "robot_heading_rad"))

            if heading_rad is None and heading_deg is not None:
                heading_rad = math.radians(heading_deg)

            if bot_x is not None and bot_y is not None:
                bot_x, bot_y = pedro_inches_to_ftc_decode_meters(bot_x, bot_y)
            if heading_rad is not None:
                heading_rad = normalize_angle_rad(heading_rad + (math.pi / 2.0))

            if bot_x is not None and bot_y is not None and heading_rad is not None:
                rows_out.append(
                    {
                        OUTPUT_TIME_HEADER: t_str,
                        "Key": f"{base}RobotPose2d",
                        "Value": format_legacy_array((bot_x, bot_y, heading_rad)),
                    }
                )
                has_geometry = True

            target_x = parse_float(row, ("shoot_target_x", "target_x"))
            target_y = parse_float(row, ("shoot_target_y", "target_y"))
            if target_x is not None and target_y is not None:
                target_x, target_y = pedro_inches_to_ftc_decode_meters(target_x, target_y)
                rows_out.append(
                    {
                        OUTPUT_TIME_HEADER: t_str,
                        "Key": f"{base}ShootTargetTranslation2d",
                        "Value": format_legacy_array((target_x, target_y)),
                    }
                )
                has_geometry = True

            turret_measured_deg = parse_float(row, ("turret_measured_deg",))
            if (
                bot_x is not None
                and bot_y is not None
                and heading_deg is not None
                and turret_measured_deg is not None
            ):
                turret_world_heading_rad = normalize_angle_rad(
                    math.radians(heading_deg + turret_measured_deg) + (math.pi / 2.0)
                )
                rows_out.append(
                    {
                        OUTPUT_TIME_HEADER: t_str,
                        "Key": f"{base}TurretMeasuredPose2d",
                        "Value": format_legacy_array((bot_x, bot_y, turret_world_heading_rad)),
                    }
                )
                has_geometry = True

            turret_target_deg = parse_float(row, ("turret_target_deg",))
            if (
                bot_x is not None
                and bot_y is not None
                and heading_deg is not None
                and turret_target_deg is not None
            ):
                turret_target_world_heading_rad = normalize_angle_rad(
                    math.radians(heading_deg + turret_target_deg) + (math.pi / 2.0)
                )
                rows_out.append(
                    {
                        OUTPUT_TIME_HEADER: t_str,
                        "Key": f"{base}TurretTargetPose2d",
                        "Value": format_legacy_array(
                            (bot_x, bot_y, turret_target_world_heading_rad)
                        ),
                    }
                )
                has_geometry = True

            if include_turret_component:
                if turret_measured_deg is not None:
                    turret_yaw_rad = math.radians(turret_measured_deg)
                    q_w, q_x, q_y, q_z = yaw_to_quaternion_wxyz(turret_yaw_rad)
                    rows_out.append(
                        {
                            OUTPUT_TIME_HEADER: t_str,
                            "Key": f"{base}TurretMeasuredComponentPose3d",
                            "Value": format_legacy_array(
                                (turret_x_m, turret_y_m, turret_z_m, q_w, q_x, q_y, q_z)
                            ),
                        }
                    )
                    has_geometry = True

                if turret_target_deg is not None:
                    turret_target_yaw_rad = math.radians(turret_target_deg)
                    q_w, q_x, q_y, q_z = yaw_to_quaternion_wxyz(turret_target_yaw_rad)
                    rows_out.append(
                        {
                            OUTPUT_TIME_HEADER: t_str,
                            "Key": f"{base}TurretTargetComponentPose3d",
                            "Value": format_legacy_array(
                                (turret_x_m, turret_y_m, turret_z_m, q_w, q_x, q_y, q_z)
                            ),
                        }
                    )
                    has_geometry = True

        if not has_geometry:
            raise ValueError(
                f"No geometry fields found in {path}. Need pose-like columns such as "
                "bot_x/bot_y/bot_heading_deg."
            )

    return rows_out


def write_csv(path: str, rows: List[Dict[str, str]], columns: List[str]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        # Force LF newlines so AdvantageScope's CSV(List) header detection works reliably.
        writer = csv.DictWriter(
            f, fieldnames=columns, extrasaction="ignore", lineterminator="\n"
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def default_output_path(input_path: str) -> str:
    directory = os.path.dirname(input_path)
    stem, _ = os.path.splitext(os.path.basename(input_path))
    return os.path.join(directory, f"{stem}_ascope.csv")


def default_field3d_output_path(input_path: str) -> str:
    directory = os.path.dirname(input_path)
    stem, _ = os.path.splitext(os.path.basename(input_path))
    return os.path.join(directory, f"{stem}_ascope_field3d.csv")


def merge_sources(sources: List[SourceData]) -> Tuple[List[Dict[str, str]], List[str]]:
    all_columns = [OUTPUT_TIME_HEADER]
    seen = {OUTPUT_TIME_HEADER}
    for source in sources:
        for col in source.output_columns:
            if col not in seen:
                seen.add(col)
                all_columns.append(col)

    merged_rows: List[Dict[str, str]] = []
    for source in sources:
        merged_rows.extend(source.rows)

    merged_rows.sort(key=lambda r: float(r[OUTPUT_TIME_HEADER]))
    return merged_rows, all_columns


def merge_list_rows(list_rows: List[List[Dict[str, str]]]) -> List[Dict[str, str]]:
    merged: List[Dict[str, str]] = []
    for rows in list_rows:
        merged.extend(rows)
    merged.sort(key=lambda r: float(r[OUTPUT_TIME_HEADER]))
    return merged


def main() -> int:
    args = parse_args()
    input_files = expand_inputs(args.input)
    if not input_files:
        print("No input CSV files found.")
        return 1

    use_prefix = not args.no_prefix
    if args.format == "field3d":
        turret_component_xyz_m = (
            args.turret_component_x_in * INCHES_TO_METERS,
            args.turret_component_y_in * INCHES_TO_METERS,
            args.turret_component_z_in * INCHES_TO_METERS,
        )
        include_turret_component = not args.skip_turret_component
        converted_list: List[Tuple[str, List[Dict[str, str]]]] = []
        for path in input_files:
            try:
                converted_list.append(
                    (
                        path,
                        convert_file_field3d(
                            path,
                            use_prefix=use_prefix,
                            turret_component_xyz_m=turret_component_xyz_m,
                            include_turret_component=include_turret_component,
                        ),
                    )
                )
            except ValueError as exc:
                print(f"Skipping {path}: {exc}")

        if not converted_list:
            print("No files converted.")
            return 1

        if args.merge:
            if not args.output:
                print("--merge requires --output")
                return 1
            merged_rows = merge_list_rows([rows for _, rows in converted_list])
            write_csv(args.output, merged_rows, OUTPUT_LIST_HEADERS)
            print(f"Wrote merged AdvantageScope 3D CSV: {args.output} ({len(merged_rows)} rows)")
            return 0

        if args.output and len(converted_list) > 1:
            print("Single --output with multiple inputs requires --merge.")
            return 1

        for path, rows in converted_list:
            output_path = args.output or default_field3d_output_path(path)
            write_csv(output_path, rows, OUTPUT_LIST_HEADERS)
            print(f"Wrote AdvantageScope 3D CSV: {output_path} ({len(rows)} rows)")
        return 0

    converted: List[SourceData] = []
    for path in input_files:
        try:
            converted.append(convert_file(path, use_prefix=use_prefix))
        except ValueError as exc:
            print(f"Skipping {path}: {exc}")

    if not converted:
        print("No files converted.")
        return 1

    if args.merge:
        if not args.output:
            print("--merge requires --output")
            return 1
        merged_rows, merged_columns = merge_sources(converted)
        write_csv(args.output, merged_rows, merged_columns)
        print(f"Wrote merged AdvantageScope CSV: {args.output} ({len(merged_rows)} rows)")
        return 0

    if args.output and len(converted) > 1:
        print("Single --output with multiple inputs requires --merge.")
        return 1

    for source in converted:
        output_path = args.output or default_output_path(source.path)
        columns = [OUTPUT_TIME_HEADER] + source.output_columns
        write_csv(output_path, source.rows, columns)
        print(f"Wrote AdvantageScope CSV: {output_path} ({len(source.rows)} rows)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
