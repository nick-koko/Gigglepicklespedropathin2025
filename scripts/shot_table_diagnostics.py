"""
Build a quick visual diagnostic for the live shot calibration table.

Outputs:
  - scripts/shot_table_diagnostic.md
  - scripts/shot_table_diagnostic.svg

The report uses leave-one-out IDW prediction against each point's nearest
neighbors to estimate how much that point disagrees with the surrounding
surface. Large disagreement is not automatically "wrong", but it is a good
place to review when shots feel inconsistent.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TABLE_PATH = ROOT / "TeamCode" / "src" / "main" / "java" / "org" / "firstinspires" / "ftc" / "teamcode" / "subsystems" / "shot" / "ShotCalibrationTable.java"
MD_OUT = ROOT / "scripts" / "shot_table_diagnostic.md"
SVG_OUT = ROOT / "scripts" / "shot_table_diagnostic.svg"

K = 3
POWER = 2.0

RPM_SCALE = 200.0
HOOD_SCALE = 0.10
AIM_X_SCALE = 2.0
AIM_Y_SCALE = 5.0

FULL_XMIN = -12.0
FULL_XMAX = 144.0
FULL_YMIN = 0.0
FULL_YMAX = 144.0
SURFACE_CELL_IN = 6.0


@dataclass
class ShotPoint:
    idx: int
    x: float
    y: float
    rpm: float
    hood: float
    aim_x: float
    aim_y: float
    zone: str
    note: str


@dataclass
class Diagnostic:
    point: ShotPoint
    pred_rpm: float
    pred_hood: float
    pred_aim_x: float
    pred_aim_y: float
    rpm_delta: float
    hood_delta: float
    aim_x_delta: float
    aim_y_delta: float
    score: float
    neighbors: list[tuple[int, float]]


def parse_points(text: str) -> list[ShotPoint]:
    pattern = re.compile(
        r'new ShotSample\('
        r'(?P<idx>\d+),\s*'
        r'(?P<x>-?\d+(?:\.\d+)?),\s*'
        r'(?P<y>-?\d+(?:\.\d+)?),\s*'
        r'(?P<rpm>-?\d+(?:\.\d+)?),\s*'
        r'(?P<hood>-?\d+(?:\.\d+)?),\s*'
        r'(?P<aim_x>-?\d+(?:\.\d+)?),\s*'
        r'(?P<aim_y>-?\d+(?:\.\d+)?),\s*'
        r'"(?P<zone>[A-Z])",\s*"(?P<note>[^"]*)"'
        r'\)'
    )
    points: list[ShotPoint] = []
    for m in pattern.finditer(text):
        points.append(
            ShotPoint(
                idx=int(m.group("idx")),
                x=float(m.group("x")),
                y=float(m.group("y")),
                rpm=float(m.group("rpm")),
                hood=float(m.group("hood")),
                aim_x=float(m.group("aim_x")),
                aim_y=float(m.group("aim_y")),
                zone=m.group("zone"),
                note=m.group("note"),
            )
        )
    if not points:
        raise RuntimeError(f"No ShotSample rows found in {TABLE_PATH}")
    return points


def distance(a: ShotPoint, b: ShotPoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def leave_one_out(points: list[ShotPoint], point: ShotPoint) -> Diagnostic:
    others = [p for p in points if p.idx != point.idx]
    nearest = sorted(((distance(point, p), p) for p in others), key=lambda item: item[0])[:K]
    weights = [1.0 / max(d, 1e-6) ** POWER for d, _ in nearest]
    weight_sum = sum(weights)

    pred_rpm = pred_hood = pred_aim_x = pred_aim_y = 0.0
    neighbors: list[tuple[int, float]] = []
    for weight, (dist, neighbor) in zip(weights, nearest):
        norm = weight / weight_sum
        pred_rpm += norm * neighbor.rpm
        pred_hood += norm * neighbor.hood
        pred_aim_x += norm * neighbor.aim_x
        pred_aim_y += norm * neighbor.aim_y
        neighbors.append((neighbor.idx, dist))

    rpm_delta = point.rpm - pred_rpm
    hood_delta = point.hood - pred_hood
    aim_x_delta = point.aim_x - pred_aim_x
    aim_y_delta = point.aim_y - pred_aim_y

    score = math.sqrt(
        (rpm_delta / RPM_SCALE) ** 2 +
        (hood_delta / HOOD_SCALE) ** 2 +
        (aim_x_delta / AIM_X_SCALE) ** 2 +
        (aim_y_delta / AIM_Y_SCALE) ** 2
    )

    return Diagnostic(
        point=point,
        pred_rpm=pred_rpm,
        pred_hood=pred_hood,
        pred_aim_x=pred_aim_x,
        pred_aim_y=pred_aim_y,
        rpm_delta=rpm_delta,
        hood_delta=hood_delta,
        aim_x_delta=aim_x_delta,
        aim_y_delta=aim_y_delta,
        score=score,
        neighbors=neighbors,
    )


def severity_color(score: float) -> str:
    if score >= 2.8:
        return "#d73027"
    if score >= 1.9:
        return "#fc8d59"
    if score >= 1.1:
        return "#fee08b"
    return "#91cf60"


def zone_points(diags: list[Diagnostic], zone: str) -> list[Diagnostic]:
    return [d for d in diags if d.point.zone == zone]


def zone_bounds(zone: str) -> tuple[float, float, float, float]:
    if zone == "A":
        return 0.0, 144.0, 72.0, 144.0
    return 48.0, 96.0, 0.0, 24.0


def zone_polygon(zone: str) -> list[tuple[float, float]]:
    if zone == "A":
        return [(72.0, 72.0), (0.0, 144.0), (144.0, 144.0)]
    return [(48.0, 0.0), (96.0, 0.0), (72.0, 24.0)]


def map_point(x: float, y: float, x0: float, y0: float, w: float, h: float,
              xmin: float, xmax: float, ymin: float, ymax: float) -> tuple[float, float]:
    sx = x0 + (x - xmin) * w / (xmax - xmin)
    sy = y0 + h - (y - ymin) * h / (ymax - ymin)
    return sx, sy


def polygon_svg(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in points)


def point_in_triangle(px: float, py: float, tri: list[tuple[float, float]]) -> bool:
    (ax, ay), (bx, by), (cx, cy) = tri

    def sign(x1: float, y1: float, x2: float, y2: float, x3: float, y3: float) -> float:
        return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3)

    d1 = sign(px, py, ax, ay, bx, by)
    d2 = sign(px, py, bx, by, cx, cy)
    d3 = sign(px, py, cx, cy, ax, ay)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)


def in_any_shooting_zone(x: float, y: float) -> bool:
    return point_in_triangle(x, y, zone_polygon("A")) or point_in_triangle(x, y, zone_polygon("B"))


def idw_value(points: list[ShotPoint], x: float, y: float, attr: str) -> float:
    nearest = sorted(
        ((math.hypot(x - p.x, y - p.y), p) for p in points),
        key=lambda item: item[0]
    )[:K]
    if nearest[0][0] < 1e-6:
        return float(getattr(nearest[0][1], attr))

    weights = [1.0 / max(dist, 1e-6) ** POWER for dist, _ in nearest]
    weight_sum = sum(weights)
    value = 0.0
    for weight, (_, point) in zip(weights, nearest):
        value += (weight / weight_sum) * float(getattr(point, attr))
    return value


def lerp_color(c0: tuple[int, int, int], c1: tuple[int, int, int], t: float) -> str:
    r = round(c0[0] + (c1[0] - c0[0]) * t)
    g = round(c0[1] + (c1[1] - c0[1]) * t)
    b = round(c0[2] + (c1[2] - c0[2]) * t)
    return f"#{r:02x}{g:02x}{b:02x}"


def gradient_color(value: float, vmin: float, vmax: float) -> str:
    if vmax <= vmin:
        return "#cccccc"
    t = max(0.0, min(1.0, (value - vmin) / (vmax - vmin)))
    stops = [
        (0.00, (49, 54, 149)),
        (0.25, (69, 117, 180)),
        (0.50, (116, 173, 209)),
        (0.70, (171, 217, 233)),
        (0.82, (255, 255, 191)),
        (0.92, (253, 174, 97)),
        (1.00, (215, 48, 39)),
    ]
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            local_t = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return lerp_color(c0, c1, local_t)
    return lerp_color(stops[-2][1], stops[-1][1], 1.0)


def render_panel(svg_parts: list[str], diags: list[Diagnostic], zone: str,
                 x0: float, y0: float, w: float, h: float) -> None:
    xmin, xmax, ymin, ymax = zone_bounds(zone)
    poly = [map_point(x, y, x0, y0, w, h, xmin, xmax, ymin, ymax) for x, y in zone_polygon(zone)]
    svg_parts.append(f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="#f8f8f8" stroke="#cccccc" />')
    svg_parts.append(f'<polygon points="{polygon_svg(poly)}" fill="#dfefff" stroke="#4a7dbb" stroke-width="2" />')
    svg_parts.append(f'<text x="{x0}" y="{y0 - 10}" font-size="18" font-weight="bold">Zone {zone}</text>')

    for gx in range(int(xmin), int(xmax) + 1, 24):
        sx, _ = map_point(gx, ymin, x0, y0, w, h, xmin, xmax, ymin, ymax)
        svg_parts.append(f'<line x1="{sx:.1f}" y1="{y0:.1f}" x2="{sx:.1f}" y2="{y0+h:.1f}" stroke="#e6e6e6" stroke-width="1" />')
        svg_parts.append(f'<text x="{sx+2:.1f}" y="{y0+h-4:.1f}" font-size="10" fill="#777">{gx}</text>')
    for gy in range(int(ymin), int(ymax) + 1, 12):
        _, sy = map_point(xmin, gy, x0, y0, w, h, xmin, xmax, ymin, ymax)
        svg_parts.append(f'<line x1="{x0:.1f}" y1="{sy:.1f}" x2="{x0+w:.1f}" y2="{sy:.1f}" stroke="#e6e6e6" stroke-width="1" />')
        svg_parts.append(f'<text x="{x0+4:.1f}" y="{sy-2:.1f}" font-size="10" fill="#777">{gy}</text>')

    for d in diags:
        sx, sy = map_point(d.point.x, d.point.y, x0, y0, w, h, xmin, xmax, ymin, ymax)
        color = severity_color(d.score)
        svg_parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="10" fill="{color}" stroke="#333" stroke-width="1.5" />')
        svg_parts.append(f'<text x="{sx:.1f}" y="{sy+4:.1f}" text-anchor="middle" font-size="10" font-weight="bold">{d.point.idx}</text>')
        svg_parts.append(
            f'<text x="{sx+14:.1f}" y="{sy-6:.1f}" font-size="10">'
            f'({d.point.x:.0f},{d.point.y:.0f}) rpm {d.point.rpm:.0f} hood {d.point.hood:.3f}</text>'
        )


def render_surface_panel(svg_parts: list[str], points: list[ShotPoint], title: str, attr: str,
                         x0: float, y0: float, w: float, h: float) -> None:
    vmin = min(float(getattr(p, attr)) for p in points)
    vmax = max(float(getattr(p, attr)) for p in points)
    svg_parts.append(f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="#f8f8f8" stroke="#cccccc" />')
    svg_parts.append(f'<text x="{x0}" y="{y0 - 10}" font-size="18" font-weight="bold">{title}</text>')
    svg_parts.append(
        f'<text x="{x0 + 260}" y="{y0 - 10}" font-size="12" fill="#555">'
        f'range {vmin:.3f} to {vmax:.3f}' + (' rpm' if attr == "rpm" else '') + '</text>'
    )

    # Heatmap cells only inside the legal shooting zones.
    y = FULL_YMIN
    while y < FULL_YMAX:
        x = FULL_XMIN
        while x < FULL_XMAX:
            cx = x + (SURFACE_CELL_IN * 0.5)
            cy = y + (SURFACE_CELL_IN * 0.5)
            if in_any_shooting_zone(cx, cy):
                value = idw_value(points, cx, cy, attr)
                color = gradient_color(value, vmin, vmax)
                sx1, sy1 = map_point(x, y + SURFACE_CELL_IN, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
                sx2, sy2 = map_point(x + SURFACE_CELL_IN, y, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
                rx = min(sx1, sx2)
                ry = min(sy1, sy2)
                rw = abs(sx2 - sx1)
                rh = abs(sy2 - sy1)
                svg_parts.append(
                    f'<rect x="{rx:.1f}" y="{ry:.1f}" width="{rw:.1f}" height="{rh:.1f}" '
                    f'fill="{color}" stroke="none" />'
                )
            x += SURFACE_CELL_IN
        y += SURFACE_CELL_IN

    # Zone outlines on top of the heatmap.
    for zone in ("A", "B"):
        poly = [map_point(px, py, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
                for px, py in zone_polygon(zone)]
        svg_parts.append(
            f'<polygon points="{polygon_svg(poly)}" fill="none" stroke="#2f4f6f" stroke-width="2" />'
        )

    # Light field grid.
    for gx in range(0, 145, 24):
        sx, _ = map_point(gx, FULL_YMIN, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
        svg_parts.append(f'<line x1="{sx:.1f}" y1="{y0:.1f}" x2="{sx:.1f}" y2="{y0+h:.1f}" stroke="#ffffff" stroke-opacity="0.5" stroke-width="1" />')
        svg_parts.append(f'<text x="{sx+2:.1f}" y="{y0+h-4:.1f}" font-size="10" fill="#444">{gx}</text>')
    for gy in range(0, 145, 24):
        _, sy = map_point(FULL_XMIN, gy, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
        svg_parts.append(f'<line x1="{x0:.1f}" y1="{sy:.1f}" x2="{x0+w:.1f}" y2="{sy:.1f}" stroke="#ffffff" stroke-opacity="0.5" stroke-width="1" />')
        svg_parts.append(f'<text x="{x0+4:.1f}" y="{sy-2:.1f}" font-size="10" fill="#444">{gy}</text>')

    # Dashed aim lines from each shot point to its target position.
    for point in points:
        sx, sy = map_point(point.x, point.y, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
        tx, ty = map_point(point.aim_x, point.aim_y, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
        svg_parts.append(
            f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{tx:.1f}" y2="{ty:.1f}" '
            f'stroke="#222" stroke-opacity="0.45" stroke-width="1.2" stroke-dasharray="6,4" />'
        )
        svg_parts.append(
            f'<circle cx="{tx:.1f}" cy="{ty:.1f}" r="3.0" fill="none" stroke="#222" stroke-opacity="0.65" stroke-width="1.2" />'
        )

    # Shot points and labels.
    for point in points:
        sx, sy = map_point(point.x, point.y, x0, y0, w, h, FULL_XMIN, FULL_XMAX, FULL_YMIN, FULL_YMAX)
        svg_parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="9" fill="#ffffff" stroke="#111" stroke-width="1.5" />')
        svg_parts.append(f'<text x="{sx:.1f}" y="{sy+4:.1f}" text-anchor="middle" font-size="10" font-weight="bold">{point.idx}</text>')
        svg_parts.append(
            f'<text x="{sx+12:.1f}" y="{sy-6:.1f}" font-size="10" fill="#111">({point.x:.0f},{point.y:.0f})</text>'
        )

    # Simple low-to-high legend.
    legend_x = x0 + w - 230
    legend_y = y0 + 16
    legend_w = 180
    legend_h = 14
    for i in range(60):
        t0 = i / 59.0
        color = gradient_color(vmin + t0 * (vmax - vmin), vmin, vmax)
        svg_parts.append(
            f'<rect x="{legend_x + (legend_w * i / 60):.1f}" y="{legend_y:.1f}" width="{legend_w / 60 + 0.5:.1f}" height="{legend_h}" fill="{color}" stroke="none" />'
        )
    svg_parts.append(f'<rect x="{legend_x}" y="{legend_y}" width="{legend_w}" height="{legend_h}" fill="none" stroke="#444" stroke-width="1" />')
    svg_parts.append(f'<text x="{legend_x}" y="{legend_y - 2}" font-size="10" fill="#333">low</text>')
    svg_parts.append(f'<text x="{legend_x + legend_w - 18}" y="{legend_y - 2}" font-size="10" fill="#333">high</text>')


def build_svg(diags: list[Diagnostic]) -> str:
    width = 1260
    height = 1660
    points = [d.point for d in diags]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>text { font-family: Arial, sans-serif; }</style>',
        '<text x="24" y="34" font-size="24" font-weight="bold">Shot Table Diagnostic</text>',
        '<text x="24" y="58" font-size="13">Top panels: color = neighbor-disagreement score. Bottom panels: interpolated IDW surfaces with dashed shot-to-target lines.</text>',
    ]
    render_panel(parts, zone_points(diags, "A"), "A", 40, 90, 700, 360)
    render_panel(parts, zone_points(diags, "B"), "B", 40, 500, 700, 180)

    parts.append('<rect x="780" y="90" width="380" height="590" fill="#fbfbfb" stroke="#cccccc" />')
    parts.append('<text x="800" y="120" font-size="18" font-weight="bold">Top Review Candidates</text>')
    y = 150
    for rank, diag in enumerate(sorted(diags, key=lambda d: d.score, reverse=True)[:7], start=1):
        parts.append(
            f'<text x="800" y="{y}" font-size="13" font-weight="bold">#{rank} pt {diag.point.idx} ({diag.point.x:.0f},{diag.point.y:.0f}) zone {diag.point.zone}</text>'
        )
        parts.append(
            f'<text x="800" y="{y+18}" font-size="12">score {diag.score:.2f} | rpm d {diag.rpm_delta:+.0f} | hood d {diag.hood_delta:+.3f}</text>'
        )
        parts.append(
            f'<text x="800" y="{y+36}" font-size="12">aimX d {diag.aim_x_delta:+.2f} | aimY d {diag.aim_y_delta:+.2f} | nbrs {", ".join(str(n) for n, _ in diag.neighbors)}</text>'
        )
        y += 72

    parts.append('<text x="800" y="660" font-size="16" font-weight="bold">Known context</text>')
    parts.append('<text x="800" y="686" font-size="12">- pt 0 (72,72): recently raised from 3265 to 3400 because it was short.</text>')
    parts.append('<text x="800" y="706" font-size="12">- pt 3 (72,108): drive team reported as bad; raw neighbor score here is low.</text>')
    parts.append('<text x="800" y="726" font-size="12">- pt 4 (40,134): close-to-goal shot that intentionally uses hood 0.000.</text>')
    render_surface_panel(parts, points, "RPM Surface", "rpm", 40, 760, 1180, 390)
    render_surface_panel(parts, points, "Hood Surface", "hood", 40, 1210, 1180, 390)
    parts.append('</svg>')
    return "\n".join(parts)


def build_md(diags: list[Diagnostic]) -> str:
    lines = [
        "# Shot Table Diagnostic",
        "",
        f"Inputs: live `ShotCalibrationTable.java`, leave-one-out IDW with `k={K}`, `power={POWER}`.",
        "",
        "This report does **not** prove a point is wrong. It highlights rows whose values differ the most from what nearby points would have predicted.",
        "",
        "## Diagram",
        "",
        f"- SVG: `{SVG_OUT.relative_to(ROOT)}`",
        "- Includes two interpolated x/y surfaces: RPM and hood.",
        "- Surface panels also show dashed lines from each shot point to its target position.",
        "",
        "## Top review candidates",
        "",
    ]
    for rank, d in enumerate(sorted(diags, key=lambda diag: diag.score, reverse=True), start=1):
        nbr_text = ", ".join(f"{idx}@{dist:.1f}in" for idx, dist in d.neighbors)
        lines.append(
            f"{rank}. `pt {d.point.idx}` at `({d.point.x:.0f}, {d.point.y:.0f})` zone `{d.point.zone}`: "
            f"score `{d.score:.2f}`, rpm delta `{d.rpm_delta:+.0f}`, hood delta `{d.hood_delta:+.3f}`, "
            f"aimX delta `{d.aim_x_delta:+.2f}`, aimY delta `{d.aim_y_delta:+.2f}`; neighbors `{nbr_text}`."
        )

    lines.extend([
        "",
        "## Specific points you mentioned",
        "",
        "- `pt 0` / `(72,72)`: neighbor disagreement is dominated by aim (`aimX -6`, `aimY 144`) more than RPM. The recent RPM increase to `3400` barely moved its local outlier score; if it still shoots short after batteries recover, the issue may be more trajectory/target shape than pure RPM alone.",
        "- `pt 3` / `(72,108)`: this point does **not** look like a strong numeric outlier relative to nearby rows. If the drive team says it is bad, I would suspect repeatability at that location, battery state, or a target/hood interaction rather than a wildly wrong table value.",
        "- `pt 5` / `(50,87)`: shoots well, but its values are noticeably more aggressive than nearby left-side neighbors, especially hood and aimX. That can be valid, but it also means interpolation between `pt 5`, `pt 1`, and `pt 0` may bend sharply in that region.",
        "- `pt 4` / `(40,134)`: very strong structural outlier because `hood=0.000` is intentionally extreme. Since this is a known close-lip shot, treat it as a valid exception, not an automatic fix target.",
        "",
        "## Suggested retune order",
        "",
        "1. Recheck `pt 0 (72,72)` after batteries charge, since you already observed it short.",
        "2. Recheck an in-between shot near `(72,90-96)` or `(60,96)` to see whether the left/interior surface between `pt 0`, `pt 3`, and `pt 5` is bending the way you expect.",
        "3. Recheck `pt 3 (72,108)` with the same battery state as the good `pt 5` run; its local math looks reasonable, so the failure may be operational rather than table-shape.",
        "4. If upper-left interpolated shots look too flat, add one more nearby point instead of changing `pt 4` away from `hood=0.000`.",
        "",
        "## Raw table",
        "",
        "| idx | zone | x | y | rpm | hood | aimX | aimY | note |",
        "|---:|:---:|---:|---:|---:|---:|---:|---:|---|",
    ])
    for d in sorted(diags, key=lambda item: item.point.idx):
        p = d.point
        lines.append(
            f"| {p.idx} | {p.zone} | {p.x:.0f} | {p.y:.0f} | {p.rpm:.0f} | {p.hood:.3f} | {p.aim_x:.1f} | {p.aim_y:.1f} | {p.note} |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    text = TABLE_PATH.read_text(encoding="utf-8")
    points = parse_points(text)
    diags = [leave_one_out(points, point) for point in points]
    SVG_OUT.write_text(build_svg(diags), encoding="utf-8")
    MD_OUT.write_text(build_md(diags), encoding="utf-8")
    print(f"Wrote {SVG_OUT}")
    print(f"Wrote {MD_OUT}")


if __name__ == "__main__":
    main()
