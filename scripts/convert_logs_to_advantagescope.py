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


TIME_KEYS_PRIORITY = ("match_t_ms", "t_ms", "timestamp_ms", "time_ms")
OUTPUT_TIME_HEADER = "Timestamp"
OUTPUT_LIST_HEADERS = [OUTPUT_TIME_HEADER, "Key", "Value"]
INCHES_TO_METERS = 0.0254
PEDRO_FIELD_CENTER_IN = 72.0
DEFAULT_TURRET_COMPONENT_X_IN = -6.5
DEFAULT_TURRET_COMPONENT_Y_IN = 1.4
DEFAULT_TURRET_COMPONENT_Z_IN = 0.0
DEFAULT_ARTIFACT_SHOT_DELAY_MS = 200.0
DEFAULT_ARTIFACT_TARGET_Z_IN = 42.0
DEFAULT_ARTIFACT_SPEED_IN_PER_SEC = 150.0
DEFAULT_ARTIFACT_LAUNCH_X_IN = DEFAULT_TURRET_COMPONENT_X_IN
DEFAULT_ARTIFACT_LAUNCH_Y_IN = DEFAULT_TURRET_COMPONENT_Y_IN
DEFAULT_ARTIFACT_LAUNCH_Z_IN = 16.0
DEFAULT_ARTIFACT_HELD_Z_IN = 6.0
DEFAULT_ARTIFACT_GROUND_Z_IN = 3.0
DEFAULT_ARTIFACT_MODEL_OFFSET_X_IN = -2.5
DEFAULT_ARTIFACT_MODEL_OFFSET_Y_IN = 2.45
DEFAULT_ARTIFACT_MAX_COUNT = 3
DEFAULT_HELD_BALL_SLOT_SPACING_IN = 2.5
GRAVITY_M_PER_SEC2 = 9.80665
HIDDEN_ARTIFACT_Z_M = -10.0


@dataclass
class SourceData:
    path: str
    prefix: str
    rows: List[Dict[str, str]]
    output_columns: List[str]


@dataclass
class ActiveArtifactShot:
    shot_index: int
    launch_time_sec: float
    flight_time_sec: float
    launch_x_m: float
    launch_y_m: float
    launch_z_m: float
    impact_x_m: float
    impact_y_m: float
    target_z_m: float
    yaw_rad: float


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
    parser.add_argument(
        "--artifact-shot-delay-ms",
        type=float,
        default=DEFAULT_ARTIFACT_SHOT_DELAY_MS,
        help="Delay between burst artifact shots (ms).",
    )
    parser.add_argument(
        "--artifact-target-z-in",
        type=float,
        default=DEFAULT_ARTIFACT_TARGET_Z_IN,
        help="Artifact target impact height (inches).",
    )
    parser.add_argument(
        "--artifact-speed-in-per-sec",
        type=float,
        default=DEFAULT_ARTIFACT_SPEED_IN_PER_SEC,
        help="Artifact horizontal speed used for ballistic trajectory (in/s).",
    )
    parser.add_argument(
        "--artifact-max-count",
        type=int,
        default=DEFAULT_ARTIFACT_MAX_COUNT,
        help="Maximum number of artifact shots visualized per burst.",
    )
    parser.add_argument(
        "--artifact-launch-x-in",
        type=float,
        default=DEFAULT_ARTIFACT_LAUNCH_X_IN,
        help="Robot-relative artifact launch X offset (inches, forward+).",
    )
    parser.add_argument(
        "--artifact-launch-y-in",
        type=float,
        default=DEFAULT_ARTIFACT_LAUNCH_Y_IN,
        help="Robot-relative artifact launch Y offset (inches, left+).",
    )
    parser.add_argument(
        "--artifact-launch-z-in",
        type=float,
        default=DEFAULT_ARTIFACT_LAUNCH_Z_IN,
        help="Artifact launch height above field plane (inches).",
    )
    parser.add_argument(
        "--artifact-held-z-in",
        type=float,
        default=DEFAULT_ARTIFACT_HELD_Z_IN,
        help="Artifact held-in-robot height above field plane (inches).",
    )
    parser.add_argument(
        "--artifact-ground-z-in",
        type=float,
        default=DEFAULT_ARTIFACT_GROUND_Z_IN,
        help="Artifact preview-on-ground center height above field plane (inches).",
    )
    parser.add_argument(
        "--artifact-model-offset-x-in",
        type=float,
        default=DEFAULT_ARTIFACT_MODEL_OFFSET_X_IN,
        help="In-flight artifact pose X shift to center model origin (inches).",
    )
    parser.add_argument(
        "--artifact-model-offset-y-in",
        type=float,
        default=DEFAULT_ARTIFACT_MODEL_OFFSET_Y_IN,
        help="In-flight artifact pose Y shift to center model origin (inches).",
    )
    parser.add_argument(
        "--skip-artifact-shots",
        action="store_true",
        help="Disable artifact shot Pose3d export in field3d mode.",
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


def parse_bool(row: Dict[str, str], keys: Iterable[str]) -> Optional[bool]:
    for key in keys:
        raw = normalize_value(row.get(key, ""))
        if raw == "":
            continue
        lowered = raw.lower()
        if lowered in ("true", "t", "yes", "y"):
            return True
        if lowered in ("false", "f", "no", "n"):
            return False
        try:
            return float(raw) != 0.0
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


def robot_relative_offset_to_world(
    bot_x_m: float, bot_y_m: float, heading_rad: float, rel_x_m: float, rel_y_m: float
) -> Tuple[float, float]:
    world_x = bot_x_m + (math.cos(heading_rad) * rel_x_m) - (math.sin(heading_rad) * rel_y_m)
    world_y = bot_y_m + (math.sin(heading_rad) * rel_x_m) + (math.cos(heading_rad) * rel_y_m)
    return world_x, world_y


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
    artifact_launch_xy_m: Tuple[float, float],
    artifact_model_offset_xy_m: Tuple[float, float],
    include_turret_component: bool,
    include_artifact_shots: bool,
    artifact_shot_delay_sec: float,
    artifact_launch_z_m: float,
    artifact_held_z_m: float,
    artifact_ground_z_m: float,
    artifact_target_z_m: float,
    artifact_speed_m_per_sec: float,
    artifact_max_count: int,
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
        rows_in = list(reader)
        rows_out: List[Dict[str, str]] = []
        has_geometry = False
        turret_x_m, turret_y_m, turret_z_m = turret_component_xyz_m
        artifact_launch_x_m, artifact_launch_y_m = artifact_launch_xy_m
        artifact_model_offset_x_m, artifact_model_offset_y_m = artifact_model_offset_xy_m
        previous_dumbshoot_active = False
        previous_ball_count: Optional[int] = None
        active_artifact_shots: List[ActiveArtifactShot] = []
        held_artifact_slots: List[int] = []
        ground_artifact_slots: Dict[int, Tuple[float, float]] = {}
        ground_artifact_pickup_rows: Dict[int, int] = {}
        held_slot_spacing_m = DEFAULT_HELD_BALL_SLOT_SPACING_IN * INCHES_TO_METERS
        last_bot_x_m: Optional[float] = None
        last_bot_y_m: Optional[float] = None
        last_heading_rad: Optional[float] = None
        pickup_events: List[Tuple[int, float, float]] = []
        burst_start_indices: List[int] = []
        prev_ball_count_scan: Optional[int] = None
        prev_dumbshoot_scan = False
        for scan_index, scan_row in enumerate(rows_in):
            scan_dumbshoot_active = parse_bool(scan_row, ("dumbshoot_timer_active",))
            scan_dumbshoot_active = bool(scan_dumbshoot_active) if scan_dumbshoot_active is not None else False
            if scan_dumbshoot_active and not prev_dumbshoot_scan:
                burst_start_indices.append(scan_index)
            prev_dumbshoot_scan = scan_dumbshoot_active

            scan_ball_count_value = parse_float(scan_row, ("ball_count",))
            scan_ball_count = int(round(scan_ball_count_value)) if scan_ball_count_value is not None else None
            if (
                prev_ball_count_scan is not None
                and scan_ball_count is not None
                and scan_ball_count > prev_ball_count_scan
            ):
                pickup_x_in = parse_float(scan_row, ("bot_x", "x", "robot_x"))
                pickup_y_in = parse_float(scan_row, ("bot_y", "y", "robot_y"))
                if pickup_x_in is not None and pickup_y_in is not None:
                    pickup_x_m, pickup_y_m = pedro_inches_to_ftc_decode_meters(pickup_x_in, pickup_y_in)
                    pickup_events.append((scan_index, pickup_x_m, pickup_y_m))
            if scan_ball_count is not None:
                prev_ball_count_scan = scan_ball_count

        next_pickup_event_index = 0
        next_burst_start_pointer = 0

        for row_index, row in enumerate(rows_in):
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
                last_bot_x_m = bot_x
                last_bot_y_m = bot_y
                last_heading_rad = heading_rad
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

            if include_artifact_shots:
                dumbshoot_active = parse_bool(row, ("dumbshoot_timer_active",))
                dumbshoot_active = bool(dumbshoot_active) if dumbshoot_active is not None else False
                ball_count_value = parse_float(row, ("ball_count",))
                current_ball_count = int(round(ball_count_value)) if ball_count_value is not None else None
                ball_count_before_update = previous_ball_count
                promoted_from_ground_count = 0

                for slot, pickup_row in list(ground_artifact_pickup_rows.items()):
                    if row_index < pickup_row:
                        continue
                    if slot not in held_artifact_slots:
                        held_artifact_slots.append(slot)
                        promoted_from_ground_count += 1
                    ground_artifact_pickup_rows.pop(slot, None)
                    ground_artifact_slots.pop(slot, None)

                if current_ball_count is not None:
                    if ball_count_before_update is None:
                        delta_count = 0
                    else:
                        delta_count = current_ball_count - ball_count_before_update
                    if delta_count > 0:
                        remaining_additions = max(0, delta_count - promoted_from_ground_count)
                        for _ in range(remaining_additions):
                            used_slots = set(held_artifact_slots)
                            used_slots.update(ground_artifact_slots.keys())
                            used_slots.update(shot.shot_index for shot in active_artifact_shots)
                            for slot in range(1, max(0, artifact_max_count) + 1):
                                if slot not in used_slots:
                                    held_artifact_slots.append(slot)
                                    break
                    elif delta_count < 0 and not dumbshoot_active:
                        drop_count = min(len(held_artifact_slots), -delta_count)
                        if drop_count > 0:
                            held_artifact_slots = held_artifact_slots[:-drop_count]

                if (
                    dumbshoot_active
                    and not previous_dumbshoot_active
                    and last_bot_x_m is not None
                    and last_bot_y_m is not None
                    and last_heading_rad is not None
                    and target_x is not None
                    and target_y is not None
                ):
                    ground_artifact_slots.clear()
                    ground_artifact_pickup_rows.clear()
                    launch_slots = held_artifact_slots[:max(0, artifact_max_count)]
                    # Conservative behavior: never launch more artifacts than we can account for as "held".
                    # Bootstrap for logs that begin mid-match where ball_count is known but held slots
                    # have not yet been reconstructed from pickup deltas.
                    bootstrap_ball_count_estimate: Optional[int] = None
                    if current_ball_count is not None and ball_count_before_update is not None:
                        bootstrap_ball_count_estimate = max(
                            int(current_ball_count),
                            int(ball_count_before_update),
                        )
                    elif current_ball_count is not None:
                        bootstrap_ball_count_estimate = int(current_ball_count)
                    elif ball_count_before_update is not None:
                        bootstrap_ball_count_estimate = int(ball_count_before_update)

                    if (
                        not launch_slots
                        and bootstrap_ball_count_estimate is not None
                        and bootstrap_ball_count_estimate > 0
                    ):
                        bootstrap_count = min(
                            max(0, artifact_max_count),
                            max(0, bootstrap_ball_count_estimate),
                        )
                        used_slots = set(shot.shot_index for shot in active_artifact_shots)
                        for slot in range(1, max(0, artifact_max_count) + 1):
                            if len(launch_slots) >= bootstrap_count:
                                break
                            if slot not in used_slots:
                                launch_slots.append(slot)
                                used_slots.add(slot)

                    burst_ball_count_estimate: Optional[int] = None
                    if current_ball_count is not None and ball_count_before_update is not None:
                        burst_ball_count_estimate = max(
                            int(current_ball_count),
                            int(ball_count_before_update),
                        )
                    elif current_ball_count is not None:
                        burst_ball_count_estimate = int(current_ball_count)
                    elif ball_count_before_update is not None:
                        burst_ball_count_estimate = int(ball_count_before_update)

                    if burst_ball_count_estimate is not None:
                        launch_slots = launch_slots[:max(0, burst_ball_count_estimate)]

                    if launch_slots:

                        launch_x_m, launch_y_m = robot_relative_offset_to_world(
                            last_bot_x_m,
                            last_bot_y_m,
                            last_heading_rad,
                            artifact_launch_x_m,
                            artifact_launch_y_m,
                        )
                        launch_z_m = artifact_launch_z_m
                        dx_m = target_x - launch_x_m
                        dy_m = target_y - launch_y_m
                        horizontal_distance_m = math.hypot(dx_m, dy_m)
                        flight_time_sec = max(
                            0.20,
                            horizontal_distance_m / max(1e-6, artifact_speed_m_per_sec),
                        )
                        yaw_rad = math.atan2(dy_m, dx_m)
                        if turret_measured_deg is not None:
                            yaw_rad = normalize_angle_rad(
                                last_heading_rad + math.radians(turret_measured_deg) + math.pi
                            )
                        impact_x_m = launch_x_m + (math.cos(yaw_rad) * horizontal_distance_m)
                        impact_y_m = launch_y_m + (math.sin(yaw_rad) * horizontal_distance_m)

                        for shot_idx, launch_slot in enumerate(launch_slots):
                            active_artifact_shots.append(
                                ActiveArtifactShot(
                                    shot_index=launch_slot,
                                    launch_time_sec=t_sec + (shot_idx * artifact_shot_delay_sec),
                                    flight_time_sec=flight_time_sec,
                                    launch_x_m=launch_x_m,
                                    launch_y_m=launch_y_m,
                                    launch_z_m=launch_z_m,
                                    impact_x_m=impact_x_m,
                                    impact_y_m=impact_y_m,
                                    target_z_m=artifact_target_z_m,
                                    yaw_rad=yaw_rad,
                                )
                            )
                elif previous_dumbshoot_active and not dumbshoot_active:
                    ground_artifact_slots.clear()
                    ground_artifact_pickup_rows.clear()

                if not dumbshoot_active and not active_artifact_shots:
                    while (
                        next_burst_start_pointer < len(burst_start_indices)
                        and burst_start_indices[next_burst_start_pointer] <= row_index
                    ):
                        next_burst_start_pointer += 1

                    next_burst_start_index = (
                        burst_start_indices[next_burst_start_pointer]
                        if next_burst_start_pointer < len(burst_start_indices)
                        else (len(rows_in) + 1)
                    )

                    while (
                        next_pickup_event_index < len(pickup_events)
                        and pickup_events[next_pickup_event_index][0] <= row_index
                    ):
                        next_pickup_event_index += 1

                    while len(ground_artifact_slots) < max(0, artifact_max_count):
                        if next_pickup_event_index >= len(pickup_events):
                            break

                        pickup_row, pickup_x_m, pickup_y_m = pickup_events[next_pickup_event_index]
                        if pickup_row >= next_burst_start_index:
                            break

                        used_slots = set(held_artifact_slots)
                        used_slots.update(ground_artifact_slots.keys())
                        used_slots.update(shot.shot_index for shot in active_artifact_shots)
                        slot_to_fill: Optional[int] = None
                        for slot in range(1, max(0, artifact_max_count) + 1):
                            if slot not in used_slots:
                                slot_to_fill = slot
                                break
                        if slot_to_fill is None:
                            break

                        ground_artifact_slots[slot_to_fill] = (pickup_x_m, pickup_y_m)
                        ground_artifact_pickup_rows[slot_to_fill] = pickup_row
                        next_pickup_event_index += 1

                launched_slots = {
                    shot.shot_index
                    for shot in active_artifact_shots
                    if t_sec >= shot.launch_time_sec
                }
                if launched_slots:
                    held_artifact_slots = [
                        slot for slot in held_artifact_slots if slot not in launched_slots
                    ]

                for shot_index in range(1, max(0, artifact_max_count) + 1):
                    active_shot: Optional[ActiveArtifactShot] = None
                    for shot in active_artifact_shots:
                        if shot.shot_index != shot_index:
                            continue
                        shot_elapsed_sec = t_sec - shot.launch_time_sec
                        if 0.0 <= shot_elapsed_sec <= shot.flight_time_sec:
                            active_shot = shot
                            break

                    if active_shot is not None:
                        shot_elapsed_sec = t_sec - active_shot.launch_time_sec
                        progress = shot_elapsed_sec / max(1e-6, active_shot.flight_time_sec)
                        pos_x_m = active_shot.launch_x_m + (
                            (active_shot.impact_x_m - active_shot.launch_x_m) * progress
                        )
                        pos_y_m = active_shot.launch_y_m + (
                            (active_shot.impact_y_m - active_shot.launch_y_m) * progress
                        )
                        launch_vz_mps = (
                            active_shot.target_z_m
                            - active_shot.launch_z_m
                            + (0.5 * GRAVITY_M_PER_SEC2 * active_shot.flight_time_sec ** 2)
                        ) / max(1e-6, active_shot.flight_time_sec)
                        pos_z_m = (
                            active_shot.launch_z_m
                            + (launch_vz_mps * shot_elapsed_sec)
                            - (0.5 * GRAVITY_M_PER_SEC2 * shot_elapsed_sec ** 2)
                        )
                        q_w, q_x, q_y, q_z = yaw_to_quaternion_wxyz(active_shot.yaw_rad)
                        pose_values = (
                            pos_x_m + artifact_model_offset_x_m,
                            pos_y_m + artifact_model_offset_y_m,
                            pos_z_m,
                            q_w,
                            q_x,
                            q_y,
                            q_z,
                        )
                    elif (
                        shot_index in held_artifact_slots
                        and last_bot_x_m is not None
                        and last_bot_y_m is not None
                        and last_heading_rad is not None
                    ):
                        slot_center = (artifact_max_count + 1) * 0.5
                        slot_offset_m = (shot_index - slot_center) * held_slot_spacing_m
                        held_x_m, held_y_m = robot_relative_offset_to_world(
                            last_bot_x_m,
                            last_bot_y_m,
                            last_heading_rad,
                            turret_x_m,
                            turret_y_m + slot_offset_m,
                        )
                        q_w, q_x, q_y, q_z = yaw_to_quaternion_wxyz(last_heading_rad)
                        pose_values = (
                            held_x_m,
                            held_y_m,
                            artifact_held_z_m,
                            q_w,
                            q_x,
                            q_y,
                            q_z,
                        )
                    elif shot_index in ground_artifact_slots:
                        ground_x_m, ground_y_m = ground_artifact_slots[shot_index]
                        pose_values = (
                            ground_x_m,
                            ground_y_m,
                            artifact_ground_z_m,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                        )
                    else:
                        pose_values = (0.0, 0.0, HIDDEN_ARTIFACT_Z_M, 1.0, 0.0, 0.0, 0.0)

                    rows_out.append(
                        {
                            OUTPUT_TIME_HEADER: t_str,
                            "Key": f"{base}ArtifactShot{shot_index}Pose3d",
                            "Value": format_legacy_array(pose_values),
                        }
                    )
                    has_geometry = True

                active_artifact_shots = [
                    shot
                    for shot in active_artifact_shots
                    if t_sec <= (shot.launch_time_sec + shot.flight_time_sec)
                ]
                previous_dumbshoot_active = dumbshoot_active
                if current_ball_count is not None:
                    previous_ball_count = current_ball_count

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
        artifact_launch_xy_m = (
            args.artifact_launch_x_in * INCHES_TO_METERS,
            args.artifact_launch_y_in * INCHES_TO_METERS,
        )
        artifact_model_offset_xy_m = (
            args.artifact_model_offset_x_in * INCHES_TO_METERS,
            args.artifact_model_offset_y_in * INCHES_TO_METERS,
        )
        include_turret_component = not args.skip_turret_component
        include_artifact_shots = not args.skip_artifact_shots
        artifact_shot_delay_sec = max(0.0, args.artifact_shot_delay_ms) / 1000.0
        artifact_launch_z_m = args.artifact_launch_z_in * INCHES_TO_METERS
        artifact_held_z_m = args.artifact_held_z_in * INCHES_TO_METERS
        artifact_ground_z_m = args.artifact_ground_z_in * INCHES_TO_METERS
        artifact_target_z_m = args.artifact_target_z_in * INCHES_TO_METERS
        artifact_speed_m_per_sec = max(1e-6, args.artifact_speed_in_per_sec) * INCHES_TO_METERS
        artifact_max_count = max(0, args.artifact_max_count)
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
                            artifact_launch_xy_m=artifact_launch_xy_m,
                            artifact_model_offset_xy_m=artifact_model_offset_xy_m,
                            include_turret_component=include_turret_component,
                            include_artifact_shots=include_artifact_shots,
                            artifact_shot_delay_sec=artifact_shot_delay_sec,
                            artifact_launch_z_m=artifact_launch_z_m,
                            artifact_held_z_m=artifact_held_z_m,
                            artifact_ground_z_m=artifact_ground_z_m,
                            artifact_target_z_m=artifact_target_z_m,
                            artifact_speed_m_per_sec=artifact_speed_m_per_sec,
                            artifact_max_count=artifact_max_count,
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
