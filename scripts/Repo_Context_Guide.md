# Repo Context Guide

## Purpose

This file is a high-context "start here" guide for future chat instances working in this repo.
It summarizes:

- what this robot/software project is,
- what game behavior and strategy the code is trying to support,
- which files are the most important entry points,
- what major features and recent changes matter for context,
- and what the other markdown files in this repo are for.

This guide is not meant to replace the deeper docs. It is the fast orientation layer that points
to the right detailed document next.

---

## Project At A Glance

- Repo: FTC robot code for the 2025-2026 **DECODE** season.
- Framework/base:
  - FTC SDK
  - Android Studio / `TeamCode`
  - NextFTC opmodes/subsystems
  - Pedro Pathing for pose/path following
- Team-specific focus:
  - fast intake + multi-ball burst shooting,
  - turret-based aiming,
  - SOTM (shooting on the move),
  - heavy logging and post-match analysis,
  - custom visualization/export tools in `scripts/`.

The codebase is not just "drive and shoot". It is built around repeated cycle optimization:

1. pick up balls/artifacts quickly,
2. shoot them in fast bursts,
3. manage lever/ramp interactions efficiently,
4. preserve enough telemetry/logging to diagnose why a cycle was good or bad.

---

## Game / Robot Understanding

The current working mental model from the repo and recent discussions is:

- The game uses ball-like "artifacts" that are collected from the field.
- The robot shoots them into a goal.
- The scoring/ramp system fills up and must periodically be emptied by hitting a lever.
- The lever interaction matters strategically because it can slow cycling if mistimed or if balls do
  not fully roll out.
- The robot operates from close-side and far-side autonomous starts, with blue/red mirroring.
- In TeleOp, the team wants repeatable 120-second practice analysis to measure cycle quality over time.

Important practical gameplay ideas that show up throughout the code and docs:

- "Close" and "far" shooting positions matter.
- Shot location may vary intentionally by robot field pose; there is interest in dynamic target
  selection rather than one fixed goal coordinate.
- Lever visits are an expected part of TeleOp cycles and should be visible in analysis.
- Burst reliability, especially ball 2 and ball 3, matters a lot.

---

## Most Important Code Entry Points

If a future chat needs to understand robot behavior quickly, start with these files:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Pickles2025Teleop.java`
  - Main TeleOp.
  - Central place for driver controls, shooter logic, SOTM logging, targeting behavior, and many
    tuning flags.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/closeAutonPaths.java`
  - Shared close-side autonomous base flow.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/closeBlueSide.java`
  - Blue close autonomous wrapper.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/closeRedSide.java`
  - Red close autonomous wrapper.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeWithSensorsSubsystem.java`
  - Intake/transfer motors and servos, breakbeams, ball count, dumbshoot sequence support.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java`
  - Flywheel control, boost logic, target RPM behavior, gating-related internals.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretSubsystem.java`
  - Turret startup/calibration, servo-angle control, readiness state.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/LEDControlSubsystem.java`
  - Driver feedback and state indication.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/GlobalRobotData.java`
  - Cross-opmode handoff state, especially auton-to-teleop pose and turret state.

Important supporting scripts:

- `scripts/convert_logs_to_advantagescope.py`
  - Converts logs to AdvantageScope-friendly field/3D artifact visualization data.
- `scripts/teleop_analysis_dashboard.py`
  - Streamlit dashboard for TeleOp practice analysis.
- `scripts/export_pedro_paths.py`
  - Exports Pedro paths from Java auton definitions.

---

## Current Software Architecture Themes

### 1. TeleOp is the main integration point

`Pickles2025Teleop.java` is where many subsystems, logs, diagnostics, and experiments come together.
Future chats should assume a large share of behavior decisions live there, even when the behavior
depends on subsystem internals.

### 2. The robot is heavily data-driven

The team uses logs not just for debugging crashes, but for:

- burst timing analysis,
- SOTM tuning,
- turret lag analysis,
- odometry drift analysis,
- MT1 vs odometry comparison,
- TeleOp cycle-quality tracking over multiple sessions,
- AdvantageScope animation/replay.

### 3. Turret, shooter, and feed are tightly coupled

A "shooting problem" may actually be caused by:

- turret not ready,
- bad SOTM timing,
- flywheel recovery behavior,
- transfer timing,
- breakbeam visibility,
- or intake ball position drift.

### 4. Recent work prioritizes reliability over theoretical elegance

There are deeper long-term plans for advanced control, but recent decisions generally favor:

- non-blocking startup logic,
- explicit state-machine behavior,
- robust logging,
- simple but testable heuristics,
- and behavior that works on the real robot under practice conditions.

---

## Key Robot Behaviors And Terms

### Dumbshoot

"Dumbshoot" in this repo refers to a fast feed/shoot sequence where the robot runs intake/transfer
to launch one or more balls quickly. It is used in both TeleOp and autonomous flows, but TeleOp
behavior is an especially active tuning area.

Things commonly discussed with dumbshoot:

- timeout length,
- whether any early-abort path exists,
- ball 2 / ball 3 consistency,
- high-rate RPM logging,
- hybrid feed boost behavior,
- whether breakbeam edges are reliable enough to time all shots directly.

### Hybrid boost

This is the shot feed boost strategy meant to shape flywheel command around expected ball contacts.
It uses staged timing and can include pre-boost, contact windows, and recovery windows.

### SOTM

SOTM means shooting while moving. In this repo it generally includes:

- aiming compensation based on robot motion,
- turret angular compensation,
- predictive distance / RPM ideas,
- and logging so the team can compare theoretical aim behavior to what happened in practice.

### Ball count

`ball_count` is an important state variable and logging field. It is used for:

- in-robot inventory state,
- shot inference,
- pickup inference,
- AdvantageScope artifact animation,
- and TeleOp performance analysis.

### Breakbeams

Breakbeams `BB0`, `BB1`, and `BB2` are used for ball movement/state inference. They matter for:

- counting,
- feed diagnostics,
- burst timing,
- and fault detection.

### MT1

MT1 means Limelight MegaTag 1 pose data. The team wants MT1 pose logged and visible in telemetry
so it can be compared against odometry in post-processing.

---

## Important Recent Context From Ongoing Work

### Turret startup / initialization

One of the most important recent changes was refactoring turret startup so it does **not** behave
the same in every opmode state.

Current intent:

- Autonomous init may allow controlled startup behavior needed for calibration.
- TeleOp init during a real match should avoid unwanted actuator movement.
- TeleOp after autonomous should be able to inherit/calibrate from known carried-over state.

The turret startup flow moved toward:

- explicit startup phases,
- non-blocking waits,
- expected-angle-based calibration,
- and a separation between "initialize hardware" and "command turret movement".

### Auton-to-TeleOp handoff

`GlobalRobotData` is used to preserve important state across opmodes, including:

- end autonomous pose,
- whether auton has run,
- ball count context,
- and turret angle context.

This matters because the team wants TeleOp startup to be different when coming directly from auton
versus when starting fresh in practice.

### Burst-shooting reliability

There has been a major emphasis on understanding why the third ball sometimes does not launch
reliably. This led to:

- more burst logging,
- timing analysis docs/scripts,
- discussion of breakbeam limitations,
- timeout review,
- hybrid boost review,
- and per-ball intake profile tuning.

### Intake per-ball behavior

The intake path now conceptually supports different behavior based on how many balls are already
inside the robot. This is intended to reduce the chance that the first collected ball gets pushed
too far upward into the flywheel path.

### Logging expansion

Recent logging interest has included:

- `ball_count`,
- MT1 pose fields,
- intake motor encoder counts,
- shot timing,
- burst RPM traces,
- odometry drift metrics.

Not every planned logging addition may already be complete at any given commit, so future chats
should verify the actual code state before assuming a field is already present.

---

## AdvantageScope / Visualization Context

The team uses custom conversion scripts to replay logs in AdvantageScope with game-specific visuals.
This is not generic FTC logging; it includes a custom artifact model and lifecycle.

Important visualization ideas already discussed/implemented in this repo area:

- held artifacts shown inside/on the robot,
- preview artifacts shown on the ground before pickup,
- in-flight artifacts during shooting,
- separate turret-component position vs artifact launch point,
- launch point offset from robot center to turret location,
- model-origin offsets for visuals,
- ballistic flight visualization,
- and transitions based on `ball_count` and dumbshoot activity.

One important design decision: the desired visualization is for the shot **distance** to come from
robot-to-target geometry, while the **direction** can reflect turret pointing, so aiming error is
visible in the animation.

This area is primarily handled by `scripts/convert_logs_to_advantagescope.py`.

---

## TeleOp Analysis Workflow

The team wants TeleOp data to show real improvement over time, not just one-off debug traces.

The current analysis direction is:

- analyze CSVs under `Data/TeleopAnalysis`,
- focus on the first 120 seconds to mimic TeleOp,
- compare files across sessions,
- graph cumulative shots over time,
- show per-file and cross-file comparison tabs,
- estimate lever visits,
- estimate odometry heading drift against the nearest wall heading (0 or 180),
- estimate x/y drift from start to end pose,
- split far-zone vs close-zone shots.

This workflow is centered on `scripts/teleop_analysis_dashboard.py`.

Related assumptions used in analysis:

- shot events can be inferred from `ball_count` drops or fallback event markers,
- pickup events can be inferred from `ball_count` increases,
- lever visits are heuristic rather than ground truth,
- odometry drift is estimated from log endpoints, not absolute truth.

---

## Shot Tuning / Driver Testing Context

There is active use of a shot tuning mode in TeleOp for field testing. The main interests have been:

- flywheel RPM adjustment,
- hood angle adjustment,
- shot target adjustment,
- and verifying that the chosen target appears in logs.

Future chats working on shooter tuning should check:

- whether `SHOT_TUNING_MODE` is enabled,
- how target position is currently selected,
- what gamepad bindings exist for test adjustments,
- and which logger captures those adjustments.

---

## Important Strategy Notes

### The team values fast cycle time more than overly cautious gating

Many recent decisions reflect a desire to keep shooting and collection fast, while diagnosing the
real reason for failures instead of simply slowing everything down.

### Stationary performance is still the baseline

Even though there is strong interest in SOTM, there is a recurring theme in the docs that the team
should stabilize:

1. burst timing,
2. burst consistency,
3. stationary RPM/hood mapping,
4. time-of-flight model,
5. only then final SOTM gains.

### Practice analytics matter

The team is trying to use practice data to learn:

- when drivers were doing well,
- when they were not,
- whether lever interactions were the cause,
- whether odometry drift was hurting shot quality,
- and whether performance is improving over time.

---

## Known Open / Ongoing Areas

These are good examples of topics that may come up again in future chats:

- turret startup robustness and handoff behavior,
- SOTM tuning and turret lag compensation,
- hybrid boost behavior in TeleOp and auto,
- breakbeam reliability vs inferred shot timing,
- voltage compensation improvements for the flywheel,
- intake motor encoder logging for backdrive diagnosis,
- target-selection logic based on robot field position,
- TeleOp analysis dashboard refinements,
- AdvantageScope replay accuracy improvements.

Future chats should not assume these are "finished"; many are active tuning subjects rather than
fully closed tasks.

---

## How To Onboard A New Chat Quickly

If a future chat needs to help effectively, a good first-read order is:

1. this file,
2. `Pickles2025Teleop.java`,
3. the relevant subsystem file (`ShooterSubsystem`, `IntakeWithSensorsSubsystem`, or `TurretSubsystem`),
4. the matching deep-dive doc from the list below,
5. the relevant analysis/conversion script if the issue is post-processing or visualization.

For problems involving opmode transitions, also read:

- `GlobalRobotData.java`
- the relevant autonomous opmode file

For data-analysis or replay issues, also inspect:

- `scripts/convert_logs_to_advantagescope.py`
- `scripts/teleop_analysis_dashboard.py`

---

## Markdown File Map

This section describes the main team-authored markdown files and when to use them.

### Primary orientation docs

- `README.md`
  - Upstream FTC SDK/project README.
  - Useful for general FTC SDK context, but not for team-specific robot behavior.

- `scripts/Repo_Context_Guide.md`
  - This file.
  - Use first when a future chat needs overall team/project/game context quickly.

### SOTM / moving-shot docs

- `scripts/SOTM_Plan.md`
  - Big-picture staged SOTM implementation plan.
  - Best doc for understanding intended moving-shot geometry, staged rollout, and the long-term
    architecture for turret-based SOTM.

- `scripts/SOTM_Tuning_Roadmap_Checklist.md`
  - Practical phase-gate checklist for test sessions.
  - Use when deciding what to tune next and in what order.

- `scripts/SOTM_Turret_Lag_Tuning_Guide.md`
  - Field tuning guide for SOTM angular behavior, turret lag feedforward, and shot gating.
  - Best when the issue is "the moving-shot math exists, but now we need to tune it".

### Turret docs

- `scripts/turret_analysis.md`
  - Main in-depth turret architecture analysis.
  - Covers reference-team comparisons, servo vs CR strategy, calibration, readiness, and teleop
    integration. This is the most important turret design doc.

- `scripts/turret_implementation_plan.md`
  - Older / more draft-like turret design and teaching document.
  - Still useful for explaining the original control intent and student-facing rationale, but
    `turret_analysis.md` is the stronger current reference.

- `scripts/TurretServoTest_Constants_Guide.md`
  - Explains the constants and variables in `TurretServoTest.java`.
  - Use when calibrating or interpreting that test opmode specifically.

### Burst / shooting diagnostics docs

- `scripts/shooting_logging_and_burst_improvements.md`
  - Main doc for shot/burst logging strategy and burst-improvement plan.
  - Best reference for dumbshoot logging, hybrid timing, event confidence, and burst-analysis intent.

- `scripts/ThreeShotBurst_Tuning_Procedure.md`
  - More tactical step-by-step procedure for 3-ball burst testing.
  - Use for structured test sessions and profile sweeps.

- `scripts/Breakbeam_Fault_Diagnostics.md`
  - Focused draft for BB2 fault detection logic.
  - Use when debugging a suspected breakbeam/sensor-wiring issue.

### Broader improvement notes

- `scripts/other_benchmarking_improvements.md`
  - Captures useful ideas from other teams that are not primarily turret-specific.
  - Includes topics like voltage compensation, heading lock, bulk reads, and shot configuration ideas.

### Temporary / imported docs

- `scripts/_tmp_pedro_src/...`
- `scripts/_tmp_advantagescope_src/...`

These are temporary or imported reference materials, not the team's primary docs.
They may still be useful for research, but future chats should not treat them as the canonical
team strategy documents.

---

## Suggested Maintenance Rule For This File

When major strategy, logging, or architecture decisions change, update this file with:

- what changed,
- where the real code lives,
- and which deeper doc is now authoritative.

The goal is not to capture every commit. The goal is to keep future chats from having to reconstruct
the same team context from scratch every time.
