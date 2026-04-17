#!/usr/bin/env python3
"""
Interactive dashboard for teleop shooting performance review.

Run with:
  streamlit run scripts/teleop_analysis_dashboard.py
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import streamlit as st


WINDOW_SECONDS_DEFAULT = 120.0
FAR_ZONE_Y_THRESHOLD_DEFAULT = 48.0
LEVER_X_DEFAULT = 9.0
LEVER_Y_DEFAULT = 72.0
LEVER_RADIUS_DEFAULT = 7.0
ROBOT_LENGTH_DEFAULT = 18.0
ROBOT_WIDTH_DEFAULT = 18.0

TIME_COLUMNS = ("match_t_ms", "t_ms", "timestamp_ms", "time_ms")
BALL_COUNT_COLUMNS = ("ball_count",)
X_COLUMNS = ("bot_x", "x", "robot_x")
Y_COLUMNS = ("bot_y", "y", "robot_y")
HEADING_DEG_COLUMNS = ("bot_heading_deg", "heading_deg", "robot_heading_deg")
SHOT_EDGE_COLUMNS = (
    "bb2_fall_edge",
    "shot_edge",
    "shot_event",
)


@dataclass
class ShotEvent:
    t_s: float
    x_in: Optional[float]
    y_in: Optional[float]
    zone: str
    row_index: int


@dataclass
class FileAnalysis:
    file_path: Path
    file_name: str
    time_col: str
    ball_count_col: Optional[str]
    x_col: Optional[str]
    y_col: Optional[str]
    heading_col: Optional[str]
    shots_total: int
    shots_far: int
    shots_close: int
    pickups_total: int
    pickups_per_sec: float
    avg_time_between_shots_s: float
    avg_distance_traveled_between_shots_in: float
    estimated_lever_visits: int
    start_x_in: float
    start_y_in: float
    end_x_in: float
    end_y_in: float
    estimated_x_drift_in: float
    estimated_y_drift_in: float
    estimated_xy_drift_in: float
    end_heading_deg: float
    end_heading_target_deg: float
    estimated_angle_drift_from_wall_heading_deg: float
    events: List[ShotEvent]
    cumulative_all: pd.DataFrame
    cumulative_far: pd.DataFrame
    cumulative_close: pd.DataFrame
    row_count_window: int
    notes: List[str]


def choose_column(columns: Sequence[str], candidates: Sequence[str]) -> Optional[str]:
    existing = set(columns)
    for candidate in candidates:
        if candidate in existing:
            return candidate
    return None


def parse_bool_series(series: pd.Series) -> pd.Series:
    lowered = series.astype(str).str.strip().str.lower()
    return lowered.isin({"1", "1.0", "true", "t", "yes", "y"})


def build_cumulative_curve(events: List[ShotEvent], t_end_s: float, zone_filter: Optional[str]) -> pd.DataFrame:
    filtered = [e for e in events if zone_filter is None or e.zone == zone_filter]
    filtered.sort(key=lambda e: e.t_s)
    xs = [0.0]
    ys = [0]
    cumulative = 0
    for event in filtered:
        t = max(0.0, min(t_end_s, event.t_s))
        xs.extend([t, t])
        ys.extend([cumulative, cumulative + 1])
        cumulative += 1
    xs.append(t_end_s)
    ys.append(cumulative)
    return pd.DataFrame({"t_s": xs, "cumulative": ys})


def average_shot_spacing(events: List[ShotEvent]) -> float:
    if len(events) < 2:
        return float("nan")
    times = np.array(sorted(e.t_s for e in events), dtype=float)
    deltas = np.diff(times)
    if len(deltas) == 0:
        return float("nan")
    return float(np.mean(deltas))


def normalize_degrees(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def nearest_wall_heading_target(angle_deg: float) -> float:
    error_to_0 = abs(normalize_degrees(angle_deg - 0.0))
    error_to_180 = abs(normalize_degrees(angle_deg - 180.0))
    return 0.0 if error_to_0 <= error_to_180 else 180.0


def average_path_distance_between_shots(
    events: List[ShotEvent],
    cumulative_distance_in: Optional[pd.Series],
) -> float:
    if cumulative_distance_in is None or len(events) < 2:
        return float("nan")
    dists: List[float] = []
    for i in range(1, len(events)):
        curr_idx = events[i].row_index
        prev_idx = events[i - 1].row_index
        if curr_idx not in cumulative_distance_in.index or prev_idx not in cumulative_distance_in.index:
            continue
        distance = float(cumulative_distance_in.loc[curr_idx] - cumulative_distance_in.loc[prev_idx])
        if math.isfinite(distance) and distance >= 0.0:
            dists.append(distance)
    if not dists:
        return float("nan")
    return float(np.mean(dists))


def estimate_lever_visits(
    df_window: pd.DataFrame,
    x_col: Optional[str],
    y_col: Optional[str],
    heading_col: Optional[str],
    lever_x_in: float,
    lever_y_in: float,
    lever_radius_in: float,
    robot_length_in: float,
    robot_width_in: float,
) -> int:
    if x_col is None or y_col is None or lever_radius_in <= 0.0:
        return 0
    x = pd.to_numeric(df_window[x_col], errors="coerce")
    y = pd.to_numeric(df_window[y_col], errors="coerce")
    heading_deg = (
        pd.to_numeric(df_window[heading_col], errors="coerce")
        if heading_col is not None
        else pd.Series(index=df_window.index, dtype=float)
    )
    valid = x.notna() & y.notna() & heading_deg.notna()
    if not valid.any():
        valid = x.notna() & y.notna()
        if not valid.any():
            return 0
        distance = np.sqrt((x - lever_x_in) ** 2 + (y - lever_y_in) ** 2)
        in_zone = (distance <= lever_radius_in) & valid
    else:
        half_l = max(0.0, robot_length_in) * 0.5
        half_w = max(0.0, robot_width_in) * 0.5
        corner_offsets = (
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, half_w),
            (-half_l, -half_w),
        )
        in_zone = pd.Series(False, index=df_window.index)
        for idx in df_window.index[valid]:
            cx = float(x.loc[idx])
            cy = float(y.loc[idx])
            theta = math.radians(float(heading_deg.loc[idx]))
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            corner_hit = False
            for ox, oy in corner_offsets:
                corner_x = cx + (cos_t * ox) - (sin_t * oy)
                corner_y = cy + (sin_t * ox) + (cos_t * oy)
                if math.hypot(corner_x - lever_x_in, corner_y - lever_y_in) <= lever_radius_in:
                    corner_hit = True
                    break
            in_zone.loc[idx] = corner_hit
    entries = (~in_zone.shift(1, fill_value=False)) & in_zone
    return int(entries.sum())


def analyze_file(
    path: Path,
    window_seconds: float,
    far_zone_y_threshold_in: float,
    lever_x_in: float,
    lever_y_in: float,
    lever_radius_in: float,
    robot_length_in: float,
    robot_width_in: float,
) -> FileAnalysis:
    notes: List[str] = []
    df = pd.read_csv(path)
    if df.empty:
        raise ValueError(f"{path.name}: file is empty")

    time_col = choose_column(df.columns, TIME_COLUMNS)
    if time_col is None:
        raise ValueError(f"{path.name}: missing time column ({', '.join(TIME_COLUMNS)})")

    df[time_col] = pd.to_numeric(df[time_col], errors="coerce")
    df = df.dropna(subset=[time_col]).sort_values(time_col).reset_index(drop=True)
    if df.empty:
        raise ValueError(f"{path.name}: no valid rows with parseable time column {time_col}")

    t0_ms = float(df[time_col].iloc[0])
    df["t_s"] = (df[time_col] - t0_ms) / 1000.0
    df_window = df[(df["t_s"] >= 0.0) & (df["t_s"] <= window_seconds)].copy()

    ball_count_col = choose_column(df_window.columns, BALL_COUNT_COLUMNS)
    x_col = choose_column(df_window.columns, X_COLUMNS)
    y_col = choose_column(df_window.columns, Y_COLUMNS)
    heading_col = choose_column(df_window.columns, HEADING_DEG_COLUMNS)

    cumulative_path_distance_in: Optional[pd.Series] = None
    if x_col is not None and y_col is not None:
        x_vals = pd.to_numeric(df_window[x_col], errors="coerce")
        y_vals = pd.to_numeric(df_window[y_col], errors="coerce")
        dx = x_vals.diff()
        dy = y_vals.diff()
        step_distance = np.sqrt(dx ** 2 + dy ** 2)
        invalid_steps = ~(x_vals.notna() & y_vals.notna() & x_vals.shift(1).notna() & y_vals.shift(1).notna())
        step_distance[invalid_steps] = 0.0
        cumulative_path_distance_in = step_distance.fillna(0.0).cumsum()

    events: List[ShotEvent] = []
    pickups_total = 0

    if ball_count_col is not None:
        bc = pd.to_numeric(df_window[ball_count_col], errors="coerce").round()
        delta = bc.diff()
        shot_deltas = (-delta[delta < 0]).dropna()
        pickup_deltas = delta[delta > 0].dropna()
        pickups_total = int(pickup_deltas.sum()) if not pickup_deltas.empty else 0

        for idx, drop in shot_deltas.items():
            count = int(max(0, round(drop)))
            if count <= 0:
                continue
            row = df_window.loc[idx]
            x_val = pd.to_numeric(pd.Series([row[x_col]]), errors="coerce").iloc[0] if x_col else np.nan
            y_val = pd.to_numeric(pd.Series([row[y_col]]), errors="coerce").iloc[0] if y_col else np.nan
            x_in = float(x_val) if pd.notna(x_val) else None
            y_in = float(y_val) if pd.notna(y_val) else None
            zone = "unknown"
            if y_in is not None:
                zone = "far" if y_in < far_zone_y_threshold_in else "close"
            for _ in range(count):
                events.append(ShotEvent(float(row["t_s"]), x_in, y_in, zone, int(idx)))
    else:
        edge_col = choose_column(df_window.columns, SHOT_EDGE_COLUMNS)
        if edge_col is None:
            notes.append(
                "No ball_count or shot-edge column found. This file contributes no shot events."
            )
        else:
            edges = parse_bool_series(df_window[edge_col])
            edge_rows = df_window[edges]
            for _, row in edge_rows.iterrows():
                x_val = pd.to_numeric(pd.Series([row[x_col]]), errors="coerce").iloc[0] if x_col else np.nan
                y_val = pd.to_numeric(pd.Series([row[y_col]]), errors="coerce").iloc[0] if y_col else np.nan
                x_in = float(x_val) if pd.notna(x_val) else None
                y_in = float(y_val) if pd.notna(y_val) else None
                zone = "unknown"
                if y_in is not None:
                    zone = "far" if y_in < far_zone_y_threshold_in else "close"
                events.append(ShotEvent(float(row["t_s"]), x_in, y_in, zone, int(row.name)))
            notes.append(f"Shot events inferred from {edge_col} (fallback mode).")

    events.sort(key=lambda e: e.t_s)
    shots_total = len(events)
    shots_far = sum(1 for e in events if e.zone == "far")
    shots_close = sum(1 for e in events if e.zone == "close")

    cumulative_all = build_cumulative_curve(events, window_seconds, None)
    cumulative_far = build_cumulative_curve(events, window_seconds, "far")
    cumulative_close = build_cumulative_curve(events, window_seconds, "close")

    avg_time = average_shot_spacing(events)
    avg_dist = average_path_distance_between_shots(events, cumulative_path_distance_in)
    estimated_lever_visits = estimate_lever_visits(
        df_window,
        x_col,
        y_col,
        heading_col,
        lever_x_in,
        lever_y_in,
        lever_radius_in,
        robot_length_in,
        robot_width_in,
    )

    start_x_in = float("nan")
    start_y_in = float("nan")
    end_x_in = float("nan")
    end_y_in = float("nan")
    estimated_x_drift_in = float("nan")
    estimated_y_drift_in = float("nan")
    estimated_xy_drift_in = float("nan")
    if x_col is not None and y_col is not None:
        full_x = pd.to_numeric(df[x_col], errors="coerce")
        full_y = pd.to_numeric(df[y_col], errors="coerce")
        valid_xy = full_x.notna() & full_y.notna()
        valid_indices = list(df.index[valid_xy])
        if valid_indices:
            first_idx = valid_indices[0]
            last_idx = valid_indices[-1]
            start_x_in = float(full_x.loc[first_idx])
            start_y_in = float(full_y.loc[first_idx])
            end_x_in = float(full_x.loc[last_idx])
            end_y_in = float(full_y.loc[last_idx])
            estimated_x_drift_in = end_x_in - start_x_in
            estimated_y_drift_in = end_y_in - start_y_in
            estimated_xy_drift_in = math.hypot(estimated_x_drift_in, estimated_y_drift_in)

    end_heading_deg = float("nan")
    end_heading_target_deg = float("nan")
    angle_drift_deg = float("nan")
    if heading_col is not None:
        full_heading = pd.to_numeric(df[heading_col], errors="coerce").dropna()
        if not full_heading.empty:
            end_heading_deg = float(full_heading.iloc[-1])
            end_heading_target_deg = nearest_wall_heading_target(end_heading_deg)
            angle_drift_deg = normalize_degrees(end_heading_deg - end_heading_target_deg)

    return FileAnalysis(
        file_path=path,
        file_name=path.name,
        time_col=time_col,
        ball_count_col=ball_count_col,
        x_col=x_col,
        y_col=y_col,
        heading_col=heading_col,
        shots_total=shots_total,
        shots_far=shots_far,
        shots_close=shots_close,
        pickups_total=pickups_total,
        pickups_per_sec=(pickups_total / max(1e-9, window_seconds)),
        avg_time_between_shots_s=avg_time,
        avg_distance_traveled_between_shots_in=avg_dist,
        estimated_lever_visits=estimated_lever_visits,
        start_x_in=start_x_in,
        start_y_in=start_y_in,
        end_x_in=end_x_in,
        end_y_in=end_y_in,
        estimated_x_drift_in=estimated_x_drift_in,
        estimated_y_drift_in=estimated_y_drift_in,
        estimated_xy_drift_in=estimated_xy_drift_in,
        end_heading_deg=end_heading_deg,
        end_heading_target_deg=end_heading_target_deg,
        estimated_angle_drift_from_wall_heading_deg=angle_drift_deg,
        events=events,
        cumulative_all=cumulative_all,
        cumulative_far=cumulative_far,
        cumulative_close=cumulative_close,
        row_count_window=len(df_window),
        notes=notes,
    )


@st.cache_data(show_spinner=False)
def load_analyses(
    data_dir: str,
    window_seconds: float,
    far_zone_y_threshold_in: float,
    lever_x_in: float,
    lever_y_in: float,
    lever_radius_in: float,
    robot_length_in: float,
    robot_width_in: float,
) -> Tuple[List[FileAnalysis], List[str]]:
    errors: List[str] = []
    root = Path(data_dir)
    files = sorted(root.rglob("*.csv"))
    analyses: List[FileAnalysis] = []
    for file_path in files:
        try:
            analyses.append(
                analyze_file(
                    file_path,
                    window_seconds=window_seconds,
                    far_zone_y_threshold_in=far_zone_y_threshold_in,
                    lever_x_in=lever_x_in,
                    lever_y_in=lever_y_in,
                    lever_radius_in=lever_radius_in,
                    robot_length_in=robot_length_in,
                    robot_width_in=robot_width_in,
                )
            )
        except Exception as exc:
            errors.append(f"{file_path.name}: {exc}")
    return analyses, errors


def format_float(value: float, digits: int = 2) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def build_summary_table(analyses: Sequence[FileAnalysis]) -> pd.DataFrame:
    rows = []
    for a in analyses:
        rows.append(
            {
                "file": a.file_name,
                "shots": a.shots_total,
                "far zone shots": a.shots_far,
                "close zone shots": a.shots_close,
                "intake balls": a.pickups_total,
                "intakes per sec": a.pickups_per_sec,
                "avg_time_between_shots_s": a.avg_time_between_shots_s,
                "avg_distance_traveled_between_shots_in": a.avg_distance_traveled_between_shots_in,
                "estimated_lever_visits": a.estimated_lever_visits,
                "x drift (in)": a.estimated_x_drift_in,
                "y drift (in)": a.estimated_y_drift_in,
                "xy drift distance (in)": a.estimated_xy_drift_in,
                "end_angle_deg": a.end_heading_deg,
                "estimated_angle_drift_from_wall_heading_deg": a.estimated_angle_drift_from_wall_heading_deg,
            }
        )
    return pd.DataFrame(rows).sort_values(["shots", "file"], ascending=[False, True])


def plot_comparison_curves(
    analyses: Sequence[FileAnalysis],
    mode: str,
) -> go.Figure:
    fig = go.Figure()
    for a in analyses:
        if mode == "Cumulative Shots":
            curve = a.cumulative_all
            label = a.file_name
        elif mode == "Cumulative Far-Zone Shots":
            curve = a.cumulative_far
            label = a.file_name
        else:
            curve = a.cumulative_close
            label = a.file_name

        fig.add_trace(
            go.Scatter(
                x=curve["t_s"],
                y=curve["cumulative"],
                mode="lines",
                name=label,
            )
        )
    fig.update_layout(
        xaxis_title="Time Since File Start (s)",
        yaxis_title="Cumulative Shots",
        legend_title="File",
        template="plotly_white",
        margin=dict(l=20, r=20, t=30, b=20),
    )
    return fig


def render_file_drilldown(analysis: FileAnalysis, far_zone_y_threshold_in: float) -> None:
    st.subheader(analysis.file_name)
    m1, m2, m3, m4, m5, m6 = st.columns(6)
    m1.metric("Shots (0-120s)", analysis.shots_total)
    m2.metric("Far Zone Shots", analysis.shots_far)
    m3.metric("Close Zone Shots", analysis.shots_close)
    m4.metric("Pickups / s", format_float(analysis.pickups_per_sec, 3))
    m5.metric("Avg Time Between Shots (s)", format_float(analysis.avg_time_between_shots_s, 2))
    m6.metric(
        "Avg Dist Traveled Between Shots (in)",
        format_float(analysis.avg_distance_traveled_between_shots_in, 1),
    )

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=analysis.cumulative_all["t_s"],
            y=analysis.cumulative_all["cumulative"],
            mode="lines",
            name="all shots",
            line=dict(width=3),
        )
    )
    fig.add_trace(
        go.Scatter(
            x=analysis.cumulative_far["t_s"],
            y=analysis.cumulative_far["cumulative"],
            mode="lines",
            name=f"far shots (y < {far_zone_y_threshold_in:.1f}\")",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=analysis.cumulative_close["t_s"],
            y=analysis.cumulative_close["cumulative"],
            mode="lines",
            name=f"close shots (y >= {far_zone_y_threshold_in:.1f}\")",
        )
    )
    fig.update_layout(
        template="plotly_white",
        xaxis_title="Time Since File Start (s)",
        yaxis_title="Cumulative Shots",
        margin=dict(l=20, r=20, t=30, b=20),
    )
    st.plotly_chart(fig, use_container_width=True)

    events_with_pos = [e for e in analysis.events if e.x_in is not None and e.y_in is not None]
    if events_with_pos:
        points = pd.DataFrame(
            {
                "x_in": [e.x_in for e in events_with_pos],
                "y_in": [e.y_in for e in events_with_pos],
                "t_s": [e.t_s for e in events_with_pos],
                "zone": [e.zone for e in events_with_pos],
            }
        )
        shot_map = px.scatter(
            points,
            x="x_in",
            y="y_in",
            color="zone",
            hover_data=["t_s"],
            title="Shot Locations (Robot Pose At Shot Event)",
        )
        shot_map.update_layout(template="plotly_white", margin=dict(l=20, r=20, t=40, b=20))
        st.plotly_chart(shot_map, use_container_width=True)
    else:
        st.info("No shot-position data available in this file (missing bot_x/bot_y columns).")

    st.caption(
        "Estimated lever visits are heuristic (robot-footprint contact estimate near lever coordinates)."
    )
    st.write(
        {
            "estimated_lever_visits": analysis.estimated_lever_visits,
            "start_x_in": analysis.start_x_in,
            "start_y_in": analysis.start_y_in,
            "end_x_in": analysis.end_x_in,
            "end_y_in": analysis.end_y_in,
            "estimated_x_drift_in": analysis.estimated_x_drift_in,
            "estimated_y_drift_in": analysis.estimated_y_drift_in,
            "estimated_xy_drift_in": analysis.estimated_xy_drift_in,
            "end_angle_deg": analysis.end_heading_deg,
            "estimated_angle_drift_from_wall_heading_deg": analysis.estimated_angle_drift_from_wall_heading_deg,
            "time_column": analysis.time_col,
            "ball_count_column": analysis.ball_count_col or "n/a",
            "x_column": analysis.x_col or "n/a",
            "y_column": analysis.y_col or "n/a",
            "heading_column": analysis.heading_col or "n/a",
            "rows_in_120s_window": analysis.row_count_window,
            "notes": analysis.notes or [],
        }
    )


def main() -> None:
    st.set_page_config(page_title="Teleop Shooting Analysis", layout="wide")
    st.title("Teleop Shooting Analysis Dashboard")
    st.write(
        "Analyzes every CSV in `Data/TeleopAnalysis` (or your chosen folder), "
        "using only the first 120s by default."
    )

    with st.sidebar:
        st.header("Inputs")
        default_data_dir = os.environ.get("TELEOP_ANALYSIS_DIR", "Data/TeleopAnalysis")
        data_dir = st.text_input("Data folder", value=default_data_dir)
        window_seconds = st.number_input(
            "Analysis window (seconds)",
            min_value=10.0,
            max_value=600.0,
            value=WINDOW_SECONDS_DEFAULT,
            step=10.0,
        )
        far_zone_y_threshold_in = st.number_input(
            "Far-zone threshold: y < (in)",
            value=FAR_ZONE_Y_THRESHOLD_DEFAULT,
            step=1.0,
        )
        st.subheader("Lever Heuristic")
        lever_x_in = st.number_input("Lever X (in)", value=LEVER_X_DEFAULT, step=0.5)
        lever_y_in = st.number_input("Lever Y (in)", value=LEVER_Y_DEFAULT, step=0.5)
        lever_radius_in = st.number_input(
            "Lever trigger radius (in)",
            min_value=0.0,
            value=LEVER_RADIUS_DEFAULT,
            step=0.5,
        )
        robot_length_in = st.number_input(
            "Robot length (in, for lever estimate)",
            min_value=1.0,
            value=ROBOT_LENGTH_DEFAULT,
            step=0.5,
        )
        robot_width_in = st.number_input(
            "Robot width (in, for lever estimate)",
            min_value=1.0,
            value=ROBOT_WIDTH_DEFAULT,
            step=0.5,
        )

    analyses, errors = load_analyses(
        data_dir=data_dir,
        window_seconds=float(window_seconds),
        far_zone_y_threshold_in=float(far_zone_y_threshold_in),
        lever_x_in=float(lever_x_in),
        lever_y_in=float(lever_y_in),
        lever_radius_in=float(lever_radius_in),
        robot_length_in=float(robot_length_in),
        robot_width_in=float(robot_width_in),
    )

    if errors:
        with st.expander("File parsing warnings", expanded=False):
            for err in errors:
                st.warning(err)

    if not analyses:
        st.error("No valid CSV files parsed. Check the folder path and CSV schema.")
        return

    summary_df = build_summary_table(analyses)
    st.subheader("Session Summary (First Window Only)")
    display_df = summary_df.rename(
        columns={
            "avg_time_between_shots_s": "avg time between shots (s)",
            "avg_distance_traveled_between_shots_in": "avg distance traveled between shots (in)",
            "estimated_lever_visits": "estimated lever visits",
            "end_angle_deg": "end angle (deg)",
            "estimated_angle_drift_from_wall_heading_deg": "estimated angle drift from nearest wall heading (deg)",
        }
    )
    styled_df = display_df.style.set_table_styles(
        [{"selector": "th", "props": [("font-weight", "bold")]}]
    ).format(
        {
            "intakes per sec": "{:.3f}",
            "avg time between shots (s)": "{:.2f}",
            "avg distance traveled between shots (in)": "{:.1f}",
            "x drift (in)": "{:.2f}",
            "y drift (in)": "{:.2f}",
            "xy drift distance (in)": "{:.2f}",
            "end angle (deg)": "{:.2f}",
            "estimated angle drift from nearest wall heading (deg)": "{:.2f}",
        },
        na_rep="n/a",
    )
    st.dataframe(styled_df, use_container_width=True, hide_index=True)

    tab1, tab2, tab3 = st.tabs(
        ["Per-File Drilldown", "Cross-File Comparisons", "Metric Comparison Bars"]
    )

    with tab1:
        file_names = [a.file_name for a in analyses]
        selected_name = st.selectbox("Select file", options=file_names, index=0)
        selected = next(a for a in analyses if a.file_name == selected_name)
        render_file_drilldown(selected, far_zone_y_threshold_in=float(far_zone_y_threshold_in))

    with tab2:
        mode = st.selectbox(
            "Comparison graph",
            options=[
                "Cumulative Shots",
                "Cumulative Far-Zone Shots",
                "Cumulative Close-Zone Shots",
            ],
            index=0,
        )
        include_files = st.multiselect(
            "Files to compare",
            options=[a.file_name for a in analyses],
            default=[a.file_name for a in analyses],
        )
        filtered = [a for a in analyses if a.file_name in include_files]
        if not filtered:
            st.info("Pick at least one file to compare.")
        else:
            fig = plot_comparison_curves(filtered, mode)
            st.plotly_chart(fig, use_container_width=True)

    with tab3:
        metric_options: Dict[str, str] = {
            "Total shots (0-120s)": "shots",
            "Far-zone shots": "far zone shots",
            "Close-zone shots": "close zone shots",
            "Intake balls": "intake balls",
            "Intakes per second": "intakes per sec",
            "Avg time between shots (s)": "avg_time_between_shots_s",
            "Avg distance traveled between shots (in)": "avg_distance_traveled_between_shots_in",
            "Estimated lever visits": "estimated_lever_visits",
            "X drift (in)": "x drift (in)",
            "Y drift (in)": "y drift (in)",
            "XY drift distance (in)": "xy drift distance (in)",
            "End angle (deg)": "end_angle_deg",
            "Estimated angle drift from nearest wall heading (deg)": "estimated_angle_drift_from_wall_heading_deg",
        }
        metric_label = st.selectbox("Bar metric", options=list(metric_options.keys()))
        metric_col = metric_options[metric_label]
        bar_df = summary_df[["file", metric_col]].copy()
        bar_df = bar_df.sort_values(metric_col, ascending=False)
        fig = px.bar(bar_df, x="file", y=metric_col, title=metric_label)
        fig.update_layout(template="plotly_white", margin=dict(l=20, r=20, t=40, b=20))
        st.plotly_chart(fig, use_container_width=True)


if __name__ == "__main__":
    main()
