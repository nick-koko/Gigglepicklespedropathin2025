#!/usr/bin/env python3
"""
Export Pedro path definitions from a close* auton Java file into .pp files
that can be loaded in the Pedro Pathing visualizer.

Behavior:
- Reads the provided Java file (e.g. closeBlueSide.java or closeBlueSide_hydra.java).
- Finds all commands scheduled in onStartButtonPressed().
- For each such command, collects the FollowPath pathchains it uses.
- Reconstructs each pathchain's BezierLine/BezierCurve segments and emits a
  visualizer .pp file whose lines are named "<pathchain>_<finalPoseName>".
- Output filename format:
    <javaFileBase>__<CommandName>__YYYY_MM_DD_HHMMSS.pp

Usage:
    python scripts/export_pedro_paths.py --java TeamCode/src/.../closeBlueSide.java \
        --color blue --outdir exports

Notes:
- By default the "blue" alias mapping in buildPaths() is used. Use --color red
  to map aliases to the red poses.
- This parser is intentionally lightweight and tailored to the existing coding
  style (BezierLine/BezierCurve inside PedroComponent.follower().pathBuilder()).
"""

import argparse
import datetime
import json
import pathlib
import re
import sys
from typing import Dict, List, Tuple, Optional

# Regexes for parsing
POSE_RE = re.compile(
    r"(?P<name>\w+)\s*=\s*new\s+Pose\(\s*(?P<x>[-\d.]+)\s*,\s*(?P<y>[-\d.]+)\s*,\s*Math\.toRadians\(\s*(?P<deg>[-\d.]+)\s*\)\s*\)"
)
CHAIN_RE = re.compile(
    r"(?P<name>\w+)\s*=\s*[^;]*?pathBuilder\(\)(?P<body>.*?\.build\(\)\s*;)",
    re.S,
)
LINE_RE = re.compile(r"new\s+BezierLine\(\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*\)")
CURVE4_RE = re.compile(
    r"new\s+BezierCurve\(\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*\)"
)
CURVE3_RE = re.compile(
    r"new\s+BezierCurve\(\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*\)"
)
ASSIGN_RE = re.compile(r"(\w+)\s*=\s*(\w+);")
FOLLOW_PATH_RE = re.compile(r"FollowPath\(\s*([A-Za-z0-9_]+)\s*\)")
FOLLOWPATH_CALL_RE = re.compile(r"\bfollowPath\(\s*([A-Za-z0-9_]+)\s*\)")
SCHEDULE_RE = re.compile(r"([A-Za-z0-9_]+)\s*\(\)\s*\.schedule\(\)")
COMMAND_DEF_RE = re.compile(r"public\s+Command\s+([A-Za-z0-9_]+)\s*\(\)\s*{", re.M)
# capture FollowPath(...) OR any method call token; used for ordered expansion
FOLLOW_OR_CALL_RE = re.compile(
    r"FollowPath\(\s*([A-Za-z0-9_]+)\s*\)|"
    r"\bfollowPath\(\s*([A-Za-z0-9_]+)\s*\)|"
    r"\b([A-Za-z0-9_]+)\s*\(\s*\)"
)


def extract_block(text: str, start_idx: int) -> Optional[str]:
    """Extracts the brace-balanced block starting at the brace after start_idx."""
    brace_idx = text.find("{", start_idx)
    if brace_idx == -1:
        return None
    depth = 0
    for i in range(brace_idx, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[brace_idx + 1 : i]
    return None


def strip_comments(text: str) -> str:
    """Remove // and /* */ comments to avoid parsing commented-out commands."""
    # Remove /* */ first
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    # Remove // to end of line
    text = re.sub(r"//.*?$", "", text, flags=re.M)
    return text


def extract_method_body(text: str, method_name: str) -> Optional[str]:
    """Finds the body of a method by name (first match)."""
    m = re.search(rf"\b(?:public|protected|private)?\s*(?:static\s+)?\S*\s+{re.escape(method_name)}\s*\([^)]*\)\s*", text)
    if not m:
        return None
    return extract_block(text, m.end())


def parse_poses(text: str) -> Dict[str, Tuple[float, float, float]]:
    poses = {}
    for m in POSE_RE.finditer(text):
        poses[m.group("name")] = (
            float(m.group("x")),
            float(m.group("y")),
            float(m.group("deg")),
        )
    return poses


def parse_alias_maps(build_paths_body: str, poses):
    """Parses alias assignments inside the blue and red branches of buildPaths()."""
    blue_alias, red_alias = {}, {}

    def next_block(text: str, start: int) -> Optional[Tuple[str, int]]:
        brace = text.find("{", start)
        if brace == -1:
            return None
        depth = 0
        for i in range(brace, len(text)):
            ch = text[i]
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    return text[brace + 1 : i], i + 1
        return None

    if_idx = build_paths_body.find("GlobalRobotData.allianceSide")
    if if_idx == -1:
        return blue_alias, red_alias

    blue_res = next_block(build_paths_body, if_idx)
    if not blue_res:
        return blue_alias, red_alias
    blue_block, after_blue = blue_res

    else_idx = build_paths_body.find("else", after_blue)
    if else_idx == -1:
        return blue_alias, red_alias
    red_res = next_block(build_paths_body, else_idx)
    if not red_res:
        return blue_alias, red_alias
    red_block, _ = red_res

    for mm in ASSIGN_RE.finditer(blue_block):
        blue_alias[mm.group(1)] = mm.group(2)
    for mm in ASSIGN_RE.finditer(red_block):
        red_alias[mm.group(1)] = mm.group(2)

    # Fallback: if nothing parsed, try simple Blue/Red suffix mapping
    if not blue_alias:
        for mm in re.finditer(r"(\w+)\s*=\s*(\w+Blue)\s*;", build_paths_body):
            blue_alias[mm.group(1)] = mm.group(2)
    if not red_alias:
        for mm in re.finditer(r"(\w+)\s*=\s*(\w+Red)\s*;", build_paths_body):
            red_alias[mm.group(1)] = mm.group(2)
    # Global fallback: scan all assignments and bucket by Blue/Red suffix
    for mm in ASSIGN_RE.finditer(build_paths_body):
        lhs, rhs = mm.group(1), mm.group(2)
        if rhs.endswith("Blue"):
            blue_alias.setdefault(lhs, rhs)
        elif rhs.endswith("Red"):
            red_alias.setdefault(lhs, rhs)
    # Final fallback: if still missing, map aliases by suffix of defined poses
    for pose_name in poses.keys():
        if pose_name.endswith("Blue"):
            base = pose_name[:-4]
            blue_alias.setdefault(base, pose_name)
        if pose_name.endswith("Red"):
            base = pose_name[:-3]
            red_alias.setdefault(base, pose_name)
    # Normalize GateLeverPush vs GatePush naming
    for k, v in list(blue_alias.items()):
        if "GateLeverPush" in k:
            variant = k.replace("GateLeverPush", "GatePush")
            blue_alias.setdefault(variant, v)
    for k, v in list(red_alias.items()):
        if "GateLeverPush" in k:
            variant = k.replace("GateLeverPush", "GatePush")
            red_alias.setdefault(variant, v)
    return blue_alias, red_alias


def resolve_pose(name: str, poses: Dict[str, Tuple[float, float, float]], alias: Dict[str, str]):
    seen = set()
    current = name
    while current in alias:
        if current in seen:
            raise ValueError(f"Alias loop detected for {name}")
        seen.add(current)
        current = alias[current]
    if current not in poses:
        raise KeyError(f"Pose {current} not found (from {name})")
    return current, poses[current]


def parse_pathchains(text: str):
    chains = {}
    for m in CHAIN_RE.finditer(text):
        name = m.group("name")
        body = m.group("body")
        segments = []
        # preserve order of appearance
        idx = 0
        while True:
            line_m = LINE_RE.search(body, idx)
            curve4_m = CURVE4_RE.search(body, idx)
            curve3_m = CURVE3_RE.search(body, idx)
            candidates = [m for m in [line_m, curve4_m, curve3_m] if m]
            if not candidates:
                break
            next_m = min(candidates, key=lambda x: x.start())
            idx = next_m.end()
            if next_m.re == LINE_RE:
                a, b = next_m.groups()
                segments.append(("line", (a, b)))
            elif next_m.re == CURVE4_RE:
                a, c1, c2, b = next_m.groups()
                segments.append(("curve4", (a, c1, c2, b)))
            else:
                a, c1, b = next_m.groups()
                segments.append(("curve3", (a, c1, b)))
        chains[name] = segments
    return chains


def find_command_names(on_start_body: str) -> List[str]:
    return list(dict.fromkeys(SCHEDULE_RE.findall(on_start_body)))  # preserve order, unique


def parse_commands(text: str) -> Dict[str, str]:
    commands = {}
    for m in COMMAND_DEF_RE.finditer(text):
        name = m.group(1)
        body = extract_block(text, m.start())
        if body is not None:
            commands[name] = body
    return commands


def expand_follow_paths(cmd_name: str, commands: Dict[str, str], cache: Dict[str, List[str]], stack: Optional[List[str]] = None) -> List[str]:
    """Return ordered list of FollowPath chain names executed by this command (including nested commands), preserving order."""
    if cmd_name in cache:
        return cache[cmd_name]
    if stack is None:
        stack = []
    if cmd_name in stack:
        return []
    body = commands.get(cmd_name)
    if body is None:
        cache[cmd_name] = []
        return []
    stack.append(cmd_name)
    result: List[str] = []
    for m in FOLLOW_OR_CALL_RE.finditer(body):
        path1, path2, call = m.group(1), m.group(2), m.group(3)
        path = path1 or path2
        if path:
            result.append(path)
        elif call and call in commands:
            result.extend(expand_follow_paths(call, commands, cache, stack))
    stack.pop()
    cache[cmd_name] = result
    return result


def build_pp(start_pose_name: str, chain_names: List[str], chains, poses, alias, color_cycle):
    lines = []
    for chain_idx, chain_name in enumerate(chain_names):
        if chain_name not in chains:
            print(f"Warning: chain {chain_name} not found; skipping", file=sys.stderr)
            continue
        segments = chains[chain_name]
        for seg_type, args in segments:
            if seg_type == "line":
                a, b = args
                _, (ax, ay, adeg) = resolve_pose(a, poses, alias)
                final_name, (bx, by, bdeg) = resolve_pose(b, poses, alias)
                lines.append(
                    {
                        "name": f"{chain_name}_{b}",
                        "endPoint": {
                            "x": bx,
                            "y": by,
                            "heading": "linear",
                            "startDeg": adeg,
                            "endDeg": bdeg,
                        },
                        "controlPoints": [],
                        "color": color_cycle[(len(lines)) % len(color_cycle)],
                    }
                )
            elif seg_type == "curve4":
                a, c1, c2, b = args
                _, (ax, ay, adeg) = resolve_pose(a, poses, alias)
                _, (c1x, c1y, _) = resolve_pose(c1, poses, alias)
                _, (c2x, c2y, _) = resolve_pose(c2, poses, alias)
                _, (bx, by, bdeg) = resolve_pose(b, poses, alias)
                lines.append(
                    {
                        "name": f"{chain_name}_{b}",
                        "endPoint": {
                            "x": bx,
                            "y": by,
                            "heading": "linear",
                            "startDeg": adeg,
                            "endDeg": bdeg,
                        },
                        "controlPoints": [
                            {"x": c1x, "y": c1y},
                            {"x": c2x, "y": c2y},
                        ],
                        "color": color_cycle[(len(lines)) % len(color_cycle)],
                    }
                )
            else:  # curve3 quadratic; duplicate control point for viewer
                a, c1, b = args
                _, (ax, ay, adeg) = resolve_pose(a, poses, alias)
                _, (c1x, c1y, _) = resolve_pose(c1, poses, alias)
                _, (bx, by, bdeg) = resolve_pose(b, poses, alias)
                lines.append(
                    {
                        "name": f"{chain_name}_{b}",
                        "endPoint": {
                            "x": bx,
                            "y": by,
                            "heading": "linear",
                            "startDeg": adeg,
                            "endDeg": bdeg,
                        },
                        "controlPoints": [
                            {"x": c1x, "y": c1y},
                            {"x": c1x, "y": c1y},
                        ],
                        "color": color_cycle[(len(lines)) % len(color_cycle)],
                    }
                )
    # start point derived from first line start pose
    if not lines:
        raise ValueError("No lines generated for command.")
    start_resolved, (sx, sy, sdeg) = resolve_pose(start_pose_name, poses, alias)
    pp = {
        "startPoint": {
            "x": sx,
            "y": sy,
            "heading": "linear",
            "startDeg": sdeg,
            "endDeg": sdeg,
        },
        "lines": lines,
        "shapes": [],
    }
    return pp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--java", required=True, help="Path to close*.java")
    parser.add_argument("--color", choices=["blue", "red"], default="blue", help="Which alias mapping to use")
    parser.add_argument("--outdir", default="exports", help="Directory for output .pp files")
    args = parser.parse_args()

    main_path = pathlib.Path(args.java)
    text = main_path.read_text()

    # Auto-include superclass file (same folder) if present to resolve helper commands / poses
    combined_texts = [text]
    extends_m = re.search(r"class\s+\w+\s+extends\s+([A-Za-z0-9_]+)", text)
    if extends_m:
        super_name = extends_m.group(1)
        super_path = main_path.parent / f"{super_name}.java"
        if super_path.exists():
            combined_texts.append(super_path.read_text())

    combined_text = "\n".join(combined_texts)
    combined_text_clean = strip_comments(combined_text)
    poses = parse_poses(combined_text)

    build_paths_body = extract_method_body(combined_text_clean, "buildPaths")
    if not build_paths_body:
        print("Could not find buildPaths() in file.", file=sys.stderr)
        sys.exit(1)
    blue_alias, red_alias = parse_alias_maps(build_paths_body, poses)
    alias = blue_alias if args.color == "blue" else red_alias

    print(f"Debug: blue alias keys: {sorted(blue_alias.keys())}", file=sys.stderr)
    print(f"Debug: red alias keys: {sorted(red_alias.keys())}", file=sys.stderr)
    chains = parse_pathchains(build_paths_body)
    if not chains:
        # fallback: try whole file in case extraction failed
        chains = parse_pathchains(combined_text_clean)
    if not chains:
        print("Debug: no pathchains parsed. Check CHAIN_RE or buildPaths content.", file=sys.stderr)
    else:
        print(f"Debug: parsed pathchains: {', '.join(sorted(chains.keys()))}", file=sys.stderr)

    on_start_body = extract_method_body(combined_text_clean, "onStartButtonPressed")
    if not on_start_body:
        print("Could not find onStartButtonPressed() in file.", file=sys.stderr)
        sys.exit(1)
    command_names = find_command_names(on_start_body)
    commands = parse_commands(combined_text_clean)
    cache = {}

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    colors = ["#6ACDA5", "#BF8605", "#BB86D5", "#CB6A76", "#DCDD95", "#B9A966", "#55A978", "#8B8A95"]

    java_base = pathlib.Path(args.java).stem
    # Match requested style (e.g., 2025_12_09_0933)
    timestamp = datetime.datetime.now().strftime("%Y_%m_%d_%H%M")

    for cmd in command_names:
        chain_usage = expand_follow_paths(cmd, commands, cache)
        if not chain_usage:
            print(f"Warning: command {cmd} has no FollowPath calls, skipping.", file=sys.stderr)
            continue
        # debug per-command
        print(f"Debug: command {cmd} chains -> {chain_usage}", file=sys.stderr)
        # Use the first chain's first segment's start pose as startPoint
        first_chain = chain_usage[0]
        if first_chain not in chains or not chains[first_chain]:
            print(f"Warning: first chain {first_chain} missing for {cmd}, skipping.", file=sys.stderr)
            continue
        first_segment = chains[first_chain][0]
        start_pose_name = first_segment[1][0]  # first argument of segment

        pp = build_pp(start_pose_name, chain_usage, chains, poses, alias, colors)
        fname = f"{java_base}__{cmd}__{timestamp}.pp"
        out_path = outdir / fname
        out_path.write_text(json.dumps(pp, indent=2))
        print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()

