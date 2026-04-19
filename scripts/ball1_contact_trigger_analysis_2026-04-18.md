# Ball 1 Flywheel Contact Trigger Analysis — 2026-04-18 Test

## Source data

- `Data/April18/pickles2025_dumbshoot_rpm_20260418_101347.csv` — 10 single-shot sequences (expected_shots = 1 or 2).
- `Data/April18/pickles2025_dumbshoot_rpm_20260418_101558.csv` — 12 three-ball burst sequences (expected_shots = 3).
- Both runs used `burst_profile_id = 0` and the new RPM-drop + motor-current observer columns added to the shooter subsystem.

## Methodology

For every shot sequence I looked at the shot-feed window (`hybrid_feed_boost_active = True`) during `WAIT_BALL1_CONTACT`, plus the first sample of `WAIT_BALL2_CONTACT` (to see what the existing advance logic was reacting to).

For each candidate "contact trigger" I recorded the first sample (with `t_since_feed_start ≥ 50ms` to skip the feed-servo transient) that satisfied the trigger, then compared:

- **Fire reliability:** how many of the 12 / 10 sequences did the trigger fire at all.
- **Fire time:** ms since shot-feed-start.
- **Lead vs. ground-truth contact:** where "ground-truth contact" is the first sample satisfying `rpm_delta_from_baseline > 100` **and** (`cur_delta > 0.25A` **or** `cur_derivative > 8 A/s`). In other words, where **both** flywheel RPM and motor current clearly reacted.

## Key finding: three distinct timings

Every shot cleanly shows three time-separated events after shot-feed-start:

| Event                      | Physical meaning                                    | Typical time  |
| -------------------------- | --------------------------------------------------- | ------------- |
| **`bb2_fall_edge`**        | Ball interrupts the last breakbeam before the shooter | **~130 ms**  |
| **Motor current rises**    | Flywheel is actually being loaded by the ball       | **~190 ms**   |
| **RPM drops (visible)**    | Motor controller / encoder sees the deceleration    | **~200 ms**   |

The breakbeam leads the real flywheel load by **~50–90 ms** — this is the ideal "preempt the drop" window, since boost commands need a few tens of ms to take effect through the PID + motor inertia.

## Reliability of each single-signal trigger (burst3, 12 sequences)

| Trigger                               | Fired | Median fire time | Lead vs. ground truth |
| ------------------------------------- | ----- | ---------------- | --------------------- |
| `bb2_fall_edge`                       | 9/12  | 134 ms           | **+85 ms** (early)    |
| `cur_delta > 0.3 A`                   | 8/12  | 200 ms           | ±0 ms                 |
| `cur_derivative > 10 A/s`             | 10/12 | 198 ms           | ±0 ms                 |
| `rpm_delta_from_baseline > 150`       | 11/12 | 199 ms           | ±0 ms                 |
| `rpm_derivative < −3000 rpm/s`        | 11/12 | 203 ms           | ±0 ms                 |
| Existing `BALL1_TIMER_FALLBACK`       | 12/12 | 213 ms           | −12 ms (slightly late) |

Observations:

1. **No single signal is both always-on and early.** `bb2_fall_edge` is 60–90 ms earlier than any of the motor-based signals, but misses ~25% of sequences (ball already past bb2 when feed started).
2. The **current and RPM signals are equivalent in timing** — current is not substantially earlier in this dataset. They differ only in noise characteristics.
3. The **existing timer fallback at 210 ms is actually well-tuned** — on average it fires ~15 ms *after* true contact, so it's neither catastrophically late nor an under-reaction.

## Composite triggers evaluated

| Composite                                                         | Fired | Median fire | Lead vs GT |
| ----------------------------------------------------------------- | ----- | ----------- | ---------- |
| `cur_db>0.3 OR rpm_db>150`                                        | 11/12 | 197 ms      | ±0 ms      |
| `cur_db>0.25 AND rpm_db>100`                                      | 8/12  | 200 ms      | ±0 ms      |
| `bb2_fall OR (rpm_d<−3k AND cur_d>5)`                             | **12/12** | **146 ms**  | **+40 ms** |
| `bb2_fall OR cur_d>10`                                            | **12/12** | **146 ms**  | **+40 ms** |

The "bb2 OR current-derivative" composite is the clear winner on this dataset:

- **100% coverage** (12/12 and 10/10).
- Fires **~60 ms earlier** than the current timer fallback on most sequences.
- When bb2 misses (ball already past), the current-derivative branch fires at roughly the timer-fallback moment, which is already acceptable.

## Recommendation

Keep all existing advancement reasons intact and add a new first-of-any trigger for transitioning out of `WAIT_BALL1_CONTACT`:

```text
advance_from_wait_ball1 =
    bb2_fall_edge                                            // existing
 OR (t_since_feed_start >= 50  AND  cur_derivative > 10 A/s) // NEW: flywheel loaded
 OR (t_since_feed_start >= 50  AND  cur_delta_from_baseline > 0.3 A) // NEW: confirmed current spike
 OR (t_since_feed_start >= 50  AND  rpm_delta_from_baseline > 150)   // NEW: RPM drop confirmation
 OR timer_fallback (~220 ms)                                 // existing safety net
```

### Suggested thresholds (initial tuning, based on this dataset)

- `RPM_DROP_CANDIDATE_DERIVATIVE_RPM_PER_SEC = −3000` (currently present as observer threshold)
- `RPM_DROP_CANDIDATE_BASELINE_DELTA_RPM   = 150`
- `CURRENT_SPIKE_CANDIDATE_DERIVATIVE_A_PER_SEC = 10`
- `CURRENT_SPIKE_CANDIDATE_BASELINE_DELTA_A = 0.3`
- **Minimum time from shot-feed-start** before the motor-based triggers arm: **50 ms** (avoids early noise and baseline still settling).

### Boost strategy implications

Because `bb2_fall_edge` fires ~60–90 ms **before** the flywheel is actually loaded, it's the ideal moment to **begin** transitioning boost command, giving the controller time to ramp before the RPM actually dips.

Two viable boost profiles (both simpler than the current hybrid logic):

1. **Pre-emptive boost** (what you asked about): at the ball-1 advance event, jump directly to 100% power. Because the advance now fires at ~146 ms instead of ~213 ms, the boost reaches the motor roughly at the moment of true ball-flywheel contact (190–210 ms). This is a clean win.

2. **Two-step ramp**: use `bb2_fall_edge` (when present) as an early "pre-boost" step (e.g., +5% power), and the RPM/current confirmation as the "full-boost" step. The single-shot data shows enough time between the two (≈60 ms) to make a two-level ramp meaningful.

Given the user plan of "just go to 100% after first contact," **Option 1 is recommended** — swap the advance reason in the subsystem to the new composite trigger, then in the hybrid state machine after ball 1 simply set stage multiplier to 1.0 (full power) for the rest of the burst.

## Caveats

- Only 22 sequences total (12 three-ball + 10 single/double) — thresholds should be re-validated after a larger sample, ideally with varying battery voltage and different field positions.
- The ground-truth definition (`rpm_db>100 AND (cur_db>0.25 OR cur_d>8)`) is itself a heuristic. A more rigorous ground truth would require a high-speed camera or a flywheel-contact optical sensor.
- Sequence 4 of burst3 and sequence 5 of single are outliers where the ball seems to have been pushed through bb2 without immediately contacting the flywheel (RPM went *up* briefly after bb2 fall). The composite trigger still fires correctly via bb2 in those cases, so no action needed.
- The current signal behavior depends on flywheel/motor state. If boost has already been pre-applied by a previous stage, the derivative can be noisier during the ramp. Revisit thresholds if pre-boost is enabled aggressively.
