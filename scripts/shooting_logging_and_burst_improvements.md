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

`Pickles2025Teleop` now has a new logging path for shot-sequence breakbeam data:

- `ENABLE_SHOT_INFO_LOGGING`
- `SHOT_INFO_LOG_TIMEOUT_MS`
- CSV logger style matches existing turret logger flow.
- One row per shot sequence, with:
  - sequence start info (trigger reason, ball count, starting sensor states),
  - breakbeam edge timestamps (up to 3 events per sensor),
  - interval columns (up to 3 balls),
  - edge counts and end reason.

This gives first-pass evidence about whether breakbeam edges are reliable enough for per-ball timing.

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

### Decision point
- If breakbeam edge timing is reliable: use it as shot-event timing signal.
- If not reliable enough: use RPM-sag event detection fallback.

---

## Boost-Control Improvement Path (Continuous Feed)

Because transfer speed must stay high and continuous, improvements should be on flywheel command only.

### Near-term
1. Fix shooter boost implementation issues so boost behavior is actually as configured.
2. Use shot event timing (breakbeam or RPM-sag) to move from one-shot fixed-time boost to per-ball boost scheduling.
3. Tune separately for ball 1, ball 2, ball 3 recovery behavior.

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

1. Enable shot-info logging.
2. Run several 3-ball bursts at real cadence.
3. Export CSV and annotate run context (normal vs stressed cadence).
4. Analyze:
   - edge reliability,
   - timing intervals,
   - burst consistency.
5. Decide event source (breakbeam vs RPM-sag).
6. Apply next boost iteration based on measured sag/recovery.

