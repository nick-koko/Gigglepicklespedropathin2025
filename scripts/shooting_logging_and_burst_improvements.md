# Shooting Logging and Burst Improvements Plan

## Purpose

This document captures the **shooting-data logging strategy** and the **burst-shooting improvement path** for fast 3-ball sequences.  
It is intentionally separate from turret-control and SOTM planning docs.

---

## Scope and Constraints

### In scope
- Logging shot/burst data for analysis.
- Improving flywheel control during fast 3-ball bursts.
- Using breakbeam and/or RPM-derived events to measure what actually happens.

### Out of scope (for now)
- Slowing feed motors or pausing transfer between balls.
- Large architecture changes to turret control.
- Full hood-while-shooting optimization (can be revisited later).

### Hard constraint
- We want to shoot as fast as possible; transfer/feed should remain continuous.

---

## Current Logging Direction

`Pickles2025Teleop` now has a two-tier burst logging path:

- `ENABLE_SHOT_INFO_LOGGING` (one row per burst sequence)
- `ENABLE_DUMBSHOOT_RPM_LOGGING` (high-rate burst stream)
- `DUMBSHOOT_RPM_LOG_EVERY_LOOP` (for transient-focused tuning sessions)
- `BURST_PROFILE_ID` (manual profile tag for A/B/C testing)

### Shot summary logger (per sequence)

Includes:
- trigger reason, start ball count, expected shots, start sensor states
- breakbeam edge timestamps and counts
- interval fields and end reason
- burst profile metadata at sequence start:
  - `burst_profile_id`
  - `start_target_rpm`
  - `start_hood_pos`
  - `start_boost_delay_ms`
  - `start_pre_boost_amount`
  - `start_boost_mult1`
  - `start_boost_mult2`
  - `linked_dumbshoot_rpm_sequence_id`

### High-rate burst stream logger

Includes:
- sequence ids and linkage fields:
  - `sequence_id`
  - `shot_info_sequence_id`
  - `burst_profile_id`
- rpm/power/battery/voltage-comp/boost state
- hybrid burst-state fields:
  - `hybrid_feed_boost_active`
  - `hybrid_phase`
  - `hybrid_t_since_shot_feed_start_ms`
  - `hybrid_expected_contact_ms`
  - `hybrid_preboost_amount_active`
  - `hybrid_last_advance_reason`
  - `hybrid_last_advance_rel_ms`
- breakbeam edge flags and cumulative bb0/bb1/bb2 edge counts:
  - `bb0_rise_count_total`, `bb0_fall_count_total`
  - `bb1_rise_count_total`, `bb1_fall_count_total`
  - `bb2_rise_count_total`, `bb2_fall_count_total`
- `loop_time_ms`

This gives both:
- outcome-level sequence metrics, and
- transient-level controller behavior during each burst.

---

## Shot Tuning Logging Strategy (Detailed)

### Logging mode design

Use two explicit modes so tuning data is useful without changing match behavior:

- `COMP_MODE`: normal competition behavior, logging off.
- `TUNE_MODE`: same control behavior, logging on.

Important: control math should remain identical between modes.  
Only data capture and CSV writing behavior should change.

### Low-overhead logging rules

For dedicated burst tuning sessions:

1. Keep logging window short and burst-scoped (not full-match heavy logging).
2. Keep data in memory during run.
3. Save CSV at opmode stop. (or occasional chunk flush if needed).
4. Avoid extra hardware polling only for logging.
5. Avoid enabling unrelated loggers during burst testing.

Recommended burst-tuning config:

- `ENABLE_DUMBSHOOT_RPM_LOGGING = true`
- `ENABLE_SHOT_INFO_LOGGING = true`
- `ENABLE_SOTM_LOGGING = false`
- `ENABLE_TURRET_LOGGING = false`
- `DUMBSHOOT_RPM_LOG_EVERY_LOOP = true` (best transient visibility)

### Row granularity

Use two granularities together:

- One row per burst sequence (shot summary logger)
- One row per loop during active burst logging window (dumbshoot RPM logger)

This is preferred over a single monolithic logger because sequence outcomes and transient control data have different sampling needs.

### Core fields to keep in every sequence row

- `t_ms`, `match_t_ms`
- `sequence_id`
- `trigger_reason` (`dumbshoot`, `single_ball_feed`, `multi_single_shot`)
- `start_ball_count`
- `expected_shots`
- `start_bb0`, `start_bb1`, `start_bb2`
- breakbeam edge times (up to 3 rising/falling events per sensor)
- interval fields:
  - `shot1_interval_ms` (trigger -> first shot event)
  - `shot2_interval_ms` (shot1 -> shot2)
  - `shot3_interval_ms` (shot2 -> shot3)
- edge counts for each sensor
- `end_reason` and `duration_ms`
- `burst_profile_id`
- `start_target_rpm`, `start_hood_pos`
- `start_boost_delay_ms`, `start_pre_boost_amount`, `start_boost_mult1`, `start_boost_mult2`
- `linked_dumbshoot_rpm_sequence_id`

### Recommended post-processing metrics (derived from existing logs)

Compute these in analysis scripts (no extra runtime logging required):
- `rpm_at_shot1/2/3` (prefer breakbeam event time, fallback to inferred event time)
- `rpm_min_post_shotN`
- `recovery_ms_shotN`
- `% time saturated during burst`
- shot event confidence labels (`EDGE` vs `INFERRED`)
- per-sequence edge-observability score using cumulative totals:
  - `expected_shots` vs final cumulative `bb1_fall_count_total` and `bb2_fall_count_total`
  - mismatch flag for sensor-under-capture risk

### Driver/operator labeling workflow

Since CSV does not know quality on its own, use a simple primary label per burst:
- `use_for_fit = KEEP` or `IGNORE`

Optional secondary labels (only when easy/useful):
- `outcome` (`3/3`, `2/3`, `1/3`, `0/3`, or make/miss pattern)
- `reason_ignore` (optional; leave blank by default)

Recommended team default for speed:
- Always record `KEEP/IGNORE`.
- Record reason codes only when there is a clear, recurring issue and it takes little effort.

This can be recorded in a small side note keyed by `sequence_id`, or appended in telemetry
and copied after the run.

### Why KEEP/IGNORE first

During large tuning sweeps, speed and consistency of labeling matter more than rich metadata.
`KEEP/IGNORE` gives fast filtering for table fitting without slowing down collection.

---

## Battery Voltage Logging Strategy

Voltage should be logged in a **consistent loaded state**, not only resting values.

### Required fields
- `vbat_loaded_pre`: average battery voltage in a short pre-shot window (for example 80-120 ms before burst trigger)
- `vbat_loaded_min`: minimum voltage during burst window

### Optional context fields
- `vbat_loaded_post`: short post-burst average
- `vbat_rest_start`: resting voltage at start of session/run block

### Practical guidance
- For fitting/tables, use `vbat_loaded_pre` as the primary voltage feature.
- Keep window definitions fixed across all runs.
- If data was collected at unusually low loaded voltage, mark `IGNORE` instead of trying to force-fit it.
- If enough data exists, analyze by voltage bins to verify consistency.

### Session procedure

1. Run `TUNE_MODE` only during planned tuning sessions.
2. Keep burst style consistent for each run block (normal cadence vs stress cadence).
3. Export CSV at stop.
4. Save run notes with:
   - battery range,
   - target RPM profile,
   - boost settings,
   - observed hit quality summary.

This creates comparable datasets for iteration-to-iteration decisions.

---

## Why Event Logging (Not Heavy Continuous Logging)

We want useful data without changing loop behavior during tuning.

Preferred pattern:
1. Keep control loop unchanged.
2. Record key events + compact telemetry snapshots.
3. Flush to CSV at opmode stop (or infrequent chunks), not every loop write.

This avoids the "tuned with logging overhead, competed without it" mismatch.

---

## Burst Analysis Metrics We Care About

Per burst:
- start ball count
- trigger mode (`dumbshoot`, single-feed, multi-single-shot)
- timing between ball events (event-to-event spacing)

Per ball (target state for next phase):
- RPM at shot event
- pre-shot peak RPM
- post-shot minimum RPM
- sag magnitude (`peak - min`)
- recovery time back to target window

These metrics will determine whether boost logic for balls 2 and 3 is working.

---

## Breakbeam Reliability Test Strategy

### Goal
Determine whether breakbeam edges can reliably identify each ball in fast bursts.

### Test cases
- single-ball pass (baseline)
- normal burst cadence (typical 0.1s-0.2s spacing)
- intentionally fast cadence (stress test)
- varied lighting/orientation

### Validation method
- compare expected balls vs detected edge patterns
- verify no frequent missed edges or double-counts
- use short slow-motion video for ground truth when needed

### Decision point (updated from data)

- Breakbeam at current loop timing is not reliable as the sole event source in fast 3-ball bursts.
- Use breakbeam as preferred signal when present.
- Add an inferred timing fallback (commanded feed window / expected interval) when an edge is missed.
- Keep confidence tags so analysis can separate edge-confirmed vs inferred events.
- Use cumulative edge totals to validate whether missed timestamps are likely logging/quantization misses vs true path failures.

### Fast-loop practical rule

For shot-state transitions in high-speed bursts:
- Use single-sample edge acceptance inside expected timing windows.
- Do not rely on multi-sample debounce for transition gating (windows are often too short).
- Use timer fallback so state machine always advances even when edges are missed.
- Keep fallback windows profile-specific (`BURST_PROFILE_ID`) instead of global constants.

---

## Data Insights So Far (Existing Logs)

Based on existing burst logs:

- Many expected 3-shot sequences physically shot all 3 balls, but not all `bb2` transitions were captured.
- Existing summary logs show frequent `third_missing_after_second` / timeout-style endings in high-speed bursts.
- Existing RPM streams already show useful guidance:
  - RPM deficit typically grows after burst start and is often largest in the middle of the burst window.
  - Higher target-RPM bursts show larger and longer deficits.

Conclusion:

- We can start boost-profile architecture work now using existing RPM/power traces.
- We still need new profile-tagged dedicated burst runs to converge safely.

---

## Current Code Behavior Notes (Important)

These are implementation realities observed in the current TeleOp + shooter code:

1. **Loop order matters**
   - `ShooterSubsystem.periodic()` runs in component `preUpdate()`.
   - `Pickles2025Teleop.onUpdate()` runs after that.
   - So `setBoostOn()` called in TeleOp affects boost timing for the *next* loop's shooter update.

2. **Held fire re-arming behavior (updated)**
   - Updated: dumbshoot burst start is now gated so one held press does not repeatedly re-arm boost timing inside the same active sequence.
   - A new sequence can start only after the previous one ends.

3. **Slew limiting strongly shapes burst behavior**
   - Shooter command is slew-limited in normal tracking.
   - During boost windows, slew limiting can be bypassed with `DISABLE_SLEW_DURING_BOOST = true` (now default).
   - This keeps distance-noise smoothing while allowing short burst boosts to apply quickly.

4. **Breakbeam clear windows can be shorter than loop period**
   - At fast cadence, some clear windows are shorter than effective logging/loop cadence.
   - Missed edges are expected in this regime.

---

## Boost-Control Improvement Path (Continuous Feed)

Because transfer speed must stay high and continuous, primary improvements should be on flywheel command shaping.  
Feed-path delay/power sweeps can be run as separate experiments when needed.

### Near-term
1. Ensure boost multipliers are panel-tunable and actually applied by control logic.
2. Use per-burst profile tagging (`BURST_PROFILE_ID`) for A/B profile sweeps.
3. Use hybrid shot-event timing (edge-first, inferred fallback) for per-ball scheduling.
4. Tune separately for ball 1, ball 2, ball 3 recovery behavior.
5. Make boost-trigger semantics one-shot per burst (avoid repeated re-arming while button is held).

### Timing model convention

Use two clocks:

- `t_ShotFeedStartMs`: feed-start moment for this sequence (time origin)
- `dt_ShotFeedStartToBall1ContactMs_est`: estimated delay from feed start to ball1 contact start

Then:

- `dt_Ball1ToBall2ContactStartMs_est`: ball1-contact-start -> ball2-contact-start
- `dt_Ball2ToBall3ContactStartMs_est`: ball2-contact-start -> ball3-contact-start

Where:
- `dt_ShotFeedStartToBall1ContactMs_est` comes from `shot1_interval_ms` distributions per profile.
- `dt_Ball1ToBall2ContactStartMs_est` and `dt_Ball2ToBall3ContactStartMs_est` come from profile interval stats and can be corrected by available edges.
- `bb1->bb2` windows are maintained per shot index and profile for edge-assisted correction.

Derived timestamps when needed:
- `t_Ball1ContactStartMs_est = dt_ShotFeedStartToBall1ContactMs_est`
- `t_Ball2ContactStartMs_est = t_Ball1ContactStartMs_est + dt_Ball1ToBall2ContactStartMs_est`
- `t_Ball3ContactStartMs_est = t_Ball2ContactStartMs_est + dt_Ball2ToBall3ContactStartMs_est`

Terminology rule for students:
- `t_*` means timestamp since `t_ShotFeedStartMs`
- `dt_*` means duration between events

### Shot-phase guidance

- Shot 1 preboost should usually be small/zero by default.
- Start recovery at estimated/observed contact start (not after contact ends).
- Keep pre and recovery as separate tunables, even if initial values are equal.
- Gate boost by RPM error to reduce overshoot risk when gaps are longer than nominal.
- Per-ball preboost amounts are independently tunable:
  - `HYBRID_PREBOOST1_AMOUNT`, `HYBRID_PREBOOST2_AMOUNT`, `HYBRID_PREBOOST3_AMOUNT`
- If preboost is not helpful, set amounts to zero and tune multiplier windows only.

### Timing and State Diagram (Example)

Use this as a practical starting sketch (times are relative to `t_ShotFeedStartMs` and should be tuned):

```text
t_ShotFeedStartMs = 0 ms (feed rollers commanded on)

Estimated contacts:
  t_Ball1ContactStartMs_est ~ dt_ShotFeedStartToBall1ContactMs_est
  t_Ball2ContactStartMs_est ~ t_Ball1ContactStartMs_est + dt_Ball1ToBall2ContactStartMs_est
  t_Ball3ContactStartMs_est ~ t_Ball2ContactStartMs_est + dt_Ball2ToBall3ContactStartMs_est

States / phases:

IDLE
  |
  v
ARM / BURST_START
  |
  |---- PRE1 (optional, often 0 or very small)
  |       start: t_Ball1ContactStartMs_est - preLead1
  |       end:   shot1_event or timer fallback
  |
  |---- RECOVER1
  |       start trigger priority:
  |         1) bb1/bb2 edge in expected window
  |         2) timer fallback at t_Ball1ContactStartMs_est
  |       end: recover1_duration or transition into PRE2
  |
  |---- PRE2
  |       start: t_Ball2ContactStartMs_est - preLead2
  |       end:   shot2_event or timer fallback
  |
  |---- RECOVER2
  |       start: shot2_event (edge-first, inferred fallback)
  |       end:   recover2_duration or transition into PRE3
  |
  |---- PRE3
  |       start: t_Ball3ContactStartMs_est - preLead3
  |       end:   shot3_event or timer fallback
  |
  |---- RECOVER3
  |       start: shot3_event (edge-first, inferred fallback)
  |       end:   recover3_duration
  |
  v
DONE / RETURN_TO_BASELINE
```

Event confidence guidance:
- `EDGE`: direct breakbeam event in valid window.
- `INFERRED`: upstream cue (`bb1`) + timing window.
- `TIMING_ONLY`: no usable edge; timer fallback.

Window selection guidance:
- Build windows from p10..p90 plus fixed margin (for example +/- 10 ms).
- If summary and high-rate windows disagree, use the union for safety first, then tighten after more data.
- Recompute per profile after each data-collection block.
- Accept edges by **state + window**, not absolute edge index (`*_1`, `*_2`, `*_3` columns can shift when earlier edges are missed).

Fast-loop note:
- At ~15-20 ms loop times, treat transitions as single-sample decisions in valid windows.
- Do not require multi-sample debounce for state transitions.

### Control shape (target behavior)
- baseline RPM control (PID/FF)
- optional pre-contact pulse
- contact-window handling
- recovery pulse scaled by observed sag
- quick return to normal closed-loop

This keeps feed speed unchanged while improving burst consistency.

---

## If Breakbeam Is Not Reliable

Use RPM-derived event detection:
- detect shot start from sharp negative `dRPM/dt` + minimum sag threshold
- use refractory window to avoid double-counting
- optionally gate detection to known feed-active windows

Then run the same per-ball analysis and boost scheduling.

For this robot, this should be treated as default behavior for high-speed 3-ball bursts:
- breakbeam-confirmed when available,
- inferred fallback when edge is missed.

Additional practical note:
- `bb1` appears useful as a circumstantial upstream cue for `bb2` timing in many sequences.
- Use `bb1` as a fallback predictor, not as a direct contact timestamp.
- `bb1` clear after ball 2 is especially useful for reducing shot-3 timing variance.

---

## Analysis Script Workflow (Current)

Run:

`python scripts/analyze_burst_profile_windows.py`

For each `burst_profile_id`, use:

1. **Timer backbone section**
   - `feed_latency_ms (shot1_interval_ms)` -> `dt_ShotFeedStartToBall1ContactMs_est`
   - `t12_est_ms (shot2_interval_ms)` -> `dt_Ball1ToBall2ContactStartMs_est`
   - `t23_est_ms (shot3_interval_ms)` -> `dt_Ball2ToBall3ContactStartMs_est`
2. **bb1->bb2 windows**
   - review summary and high-rate values
   - apply combined recommendation for scheduler fallback windows
3. **Edge visibility section**
   - verify bb1/bb2 seen-rates by shot index
4. **Loop + edge-count section**
   - confirm loop-time health
   - compare cumulative edge totals vs expected shots

Interpretation rules:
- Low bb2 seen-rate with normal completion suggests quantization/logging miss, not necessarily mechanism failure.
- Broad `t23_est` spread indicates shot-3 phase needs wider adaptive timing or stronger bb1-assisted correction.
- Persistent high saturation plus low shot3 RPM implies recovery shaping issue (boost schedule), not timing-only issue.
- In 3-ball sequences, a missing `bb1_fall_3_ms` does not mean "no final clear"; earlier missed edges can shift ordinal slots. Use state-window acceptance and cumulative counts.

---

## 20260410 Data Direction (applied)

Based on the latest close/far characterization runs:

- Shot1: bb2 fall is highly reliable and should be primary shot1 cue.
- Shot2: bb2 fall is partially reliable; keep timer fallback always enabled.
- Shot3: bb2 is unreliable; use timer backbone with bb1-fall in-window assist.

Initial timing centers for first-pass implementation:

- Close: `dt_ShotFeedStartToBall1ContactMs_est ~ 135`, `dt_Ball1ToBall2ContactStartMs_est ~ 115`, `dt_Ball2ToBall3ContactStartMs_est ~ 155`
- Far: `dt_ShotFeedStartToBall1ContactMs_est ~ 195`, `dt_Ball1ToBall2ContactStartMs_est ~ 125`, `dt_Ball2ToBall3ContactStartMs_est ~ 145`

Use wide shot3 windows initially, then tighten after the first implementation run.

---

## Feed-Path Shaping Knobs (optional)

For transfer/feed timing experiments (without changing flywheel logic), dumbshoot now supports:

- `DUMBSHOOT_M3_POWER` (immediate path, no delay)
- `DUMBSHOOT_M1_POWER`
- `DUMBSHOOT_M2_POWER` (applied to both transfer servos)
- `DUMBSHOOT_DELAY_M1_MS`
- `DUMBSHOOT_DELAY_M2_MS`

This allows testing whether delayed `m1`/transfer engagement improves multi-ball consistency.

---

## Hood-While-Shooting Position

Some teams successfully adjust hood during shooting using an `(distance, rpm) -> hood` manifold.  
That is valid but more complex.

Planned order:
1. First maximize flywheel-only compensation during fast bursts.
2. Add hood-in-burst trim only if needed after data confirms residual error.
3. If added, keep trims small and rate-limited.

---

## Next Session Checklist

1. Set dedicated burst tuning config:
   - `ENABLE_DUMBSHOOT_RPM_LOGGING = true`
   - `ENABLE_SHOT_INFO_LOGGING = true`
   - `ENABLE_SOTM_LOGGING = false`
   - `ENABLE_TURRET_LOGGING = false`
   - `DUMBSHOOT_RPM_LOG_EVERY_LOOP = true`
2. Run characterization blocks first (1-ball, 2-ball, 3-ball; ~10 each) to update timing/visibility model.
3. Pick profile, set `BURST_PROFILE_ID`, and record boost parameters in Panels.
4. Run 10-20 bursts for each profile (near and far target conditions).
5. Export CSV and keep short run notes.
6. Analyze by `burst_profile_id` and linked sequence ids:
   - completion behavior,
   - shot timing,
   - rpm sag/recovery,
   - saturation ratio.
7. Iterate one parameter group at a time (delay, pre-boost, stage multipliers).
8. Verify command-shape behavior:
   - whether command ramps through the burst due to slew,
   - whether repeated boost re-arming is eliminated in experimental logic.
9. If needed, run a separate feed-path sweep:
   - hold flywheel settings fixed,
   - tune `DUMBSHOOT_DELAY_M1_MS` / `DUMBSHOOT_DELAY_M2_MS` and `DUMBSHOOT_M*_POWER`.

