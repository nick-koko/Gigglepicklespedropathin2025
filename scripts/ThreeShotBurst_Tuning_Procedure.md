# 3-Shot Burst Tuning Procedure

## Goal

Improve 3-ball burst consistency first (especially shot 3), then return to location-specific RPM/hood table tuning.

## Naming Convention (student-facing)

Use duration-first naming in code and discussion:

- `t_ShotFeedStartMs`: time origin for one feed sequence (`= 0` at feed command start)
- `dt_ShotFeedStartToBall1ContactMs_est`: estimated delay from feed start to ball1 flywheel contact start
- `dt_Ball1ToBall2ContactStartMs_est`: estimated delay from ball1 contact start to ball2 contact start
- `dt_Ball2ToBall3ContactStartMs_est`: estimated delay from ball2 contact start to ball3 contact start

Derived contact-start estimates:

- `t_Ball1ContactStartMs_est = dt_ShotFeedStartToBall1ContactMs_est`
- `t_Ball2ContactStartMs_est = t_Ball1ContactStartMs_est + dt_Ball1ToBall2ContactStartMs_est`
- `t_Ball3ContactStartMs_est = t_Ball2ContactStartMs_est + dt_Ball2ToBall3ContactStartMs_est`

Rule: treat breakbeam edges as state/window cues, not global "edge #1/#2/#3" truth.

## Current Logging Readiness

Use both logs together during dedicated burst testing:

- `pickles2025_dumbshoot_rpm_*.csv` (high-rate stream)
  - includes RPM, command, saturation, boost states, battery compensation, per-loop breakbeam edges, cumulative bb0/bb1/bb2 edge totals, and `loop_time_ms`.
- `pickles2025_shot_breakbeam_*.csv` (one row per sequence)
  - includes expected shots, per-shot edge timestamps, interval fields, clear-gap fields, edge counts, end reason, and profile metadata.

This pairing gives both control behavior and physical outcome.

## Session Setup (burst tuning only)

Set TeleOp tunables:

- `ENABLE_DUMBSHOOT_RPM_LOGGING = true`
- `ENABLE_SHOT_INFO_LOGGING = true`
- `ENABLE_SOTM_LOGGING = false`
- `ENABLE_TURRET_LOGGING = false`
- `DUMBSHOOT_RPM_LOG_EVERY_LOOP = true`
- `DUMBSHOOT_RPM_LOG_DURATION_MS = 1100` (or slightly longer if needed)

Before each block:

- set `BURST_PROFILE_ID` explicitly
- confirm exactly 1/2/3 balls loaded based on the block plan
- keep starting pose and driver actions consistent

## Data Collection Plan (latest)

### Phase A: Sensor characterization first

Run dedicated profile blocks to characterize edge visibility:

1. 1-ball sequences, 10 runs
2. 2-ball sequences, 10 runs
3. 3-ball sequences, 10 runs

Do this for near, then far (if time allows). Keep one profile at a time.

Purpose:

- quantify how often bb1/bb2 events are captured per shot index
- estimate timer fallbacks (`dt_ShotFeedStartToBall1ContactMs_est`, `dt_Ball1ToBall2ContactStartMs_est`, `dt_Ball2ToBall3ContactStartMs_est`) from summary intervals
- estimate `bb1 -> bb2` windows for hybrid timing logic

### Phase B: 3-ball profile sweep

Only do this after first-pass hybrid state logic is implemented.

For each location (near and far), run:

1. Baseline profile, 10 sequences
2. Candidate A, 10 sequences
3. Candidate B, 10 sequences

Change one parameter group at a time:

- `BOOST_DELAY_MS`
- per-ball preboost lead windows:
  - `HYBRID_PREBOOST1_LEAD_MS`
  - `HYBRID_PREBOOST2_LEAD_MS`
  - `HYBRID_PREBOOST3_LEAD_MS`
- per-ball preboost amounts:
  - `HYBRID_PREBOOST1_AMOUNT`
  - `HYBRID_PREBOOST2_AMOUNT`
  - `HYBRID_PREBOOST3_AMOUNT`
- stage multipliers (`BOOST_STAGE1_MULTIPLIER_*`, `BOOST_STAGE2_MULTIPLIER_*`)
- feed-path shaping (optional):
  - `DUMBSHOOT_DELAY_M1_MS`
  - `DUMBSHOOT_DELAY_M2_MS`
  - `DUMBSHOOT_M1_POWER`, `DUMBSHOOT_M2_POWER`, `DUMBSHOOT_M3_POWER`

Notes:

- If preboost is not helping, set `HYBRID_PREBOOST*_AMOUNT = 0`.
- For burst tuning, keep `DISABLE_SLEW_DURING_BOOST = true` so short boost windows are not rate-limited.

## During-Run Labeling

Keep a simple side note keyed by `sequence_id`:

- `KEEP` or `IGNORE`
- optional short reason only when obvious (misload, bad aim, driver interruption)

## Analysis Workflow

Run:

`python scripts/analyze_burst_profile_windows.py`

Read these sections in output for each `burst_profile_id`:

- timer backbone recommendations (`feed_latency`, `t12`, `t23`)
- `bb1->bb2` windows (summary, high-rate, and combined recommendation)
- edge visibility rates by shot index
- `bb2_clear_gap_*` windows
- high-rate `loop_time_ms` and cumulative edge-count totals

Map script outputs to code variable names:

- `feed_latency_ms` -> `dt_ShotFeedStartToBall1ContactMs_est`
- `t12_est_ms` -> `dt_Ball1ToBall2ContactStartMs_est`
- `t23_est_ms` -> `dt_Ball2ToBall3ContactStartMs_est`

## Pass/Fail Metrics

Per profile:

- completion: `% with expected shots achieved`
- edge observability: bb1/bb2 seen-rate for shot 1/2/3
- shot spacing spread: `shot2_interval_ms`, `shot3_interval_ms`
- RPM quality at shot events and recovery trend
- saturation fraction and command behavior during burst

Quick acceptance target:

- 3-shot completion >= 95%
- shot2/shot3 spacing stable
- no persistent shot3 under-speed trend relative to shot1

## Current first-pass windows from 20260410 data

Use these as starting points (not final):

- Close:
  - `dt_ShotFeedStartToBall1ContactMs_est` ~ 135 ms
  - `dt_Ball1ToBall2ContactStartMs_est` ~ 115 ms
  - `dt_Ball2ToBall3ContactStartMs_est` ~ 155 ms (wide confidence)
- Far:
  - `dt_ShotFeedStartToBall1ContactMs_est` ~ 195 ms
  - `dt_Ball1ToBall2ContactStartMs_est` ~ 125 ms
  - `dt_Ball2ToBall3ContactStartMs_est` ~ 145 ms (very low sample confidence)

Breakbeam usage direction from this dataset:

- Shot1: bb2 fall is strong.
- Shot2: bb2 fall is useful but incomplete.
- Shot3: rely on timer backbone, with bb1 fall in-window as assist.

So first-pass control should be timer-first with edge-assisted advancement.

## Scrimmage-Minimum Workflow

If you must keep changes minimal:

1. Keep current profile.
2. Capture 10 near + 10 far sequences.
3. Analyze for one dominant issue (timing miss vs RPM recovery).
4. Apply one focused change and retest.

## After Burst Stability

Once 3-ball consistency is reliable, return to:

1. RPM/hood map refinement per location
2. TOF table updates
3. final SOTM gain cleanup
