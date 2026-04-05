# SOTM and Turret Lag Tuning Guide

This guide is a practical, repeatable process to tune:

- SOTM lead-point behavior (moving-shot targeting)
- Turret lag feedforward (`omega * kVF` style)
- Fire gating (when shots are allowed)
- Steady-state turret error diagnostics (stiction vs control limits)

Use this as a field checklist, not a one-time read.

---

## 1) Goal and Strategy

The control stack is layered:

1. **SOTM geometry layer**: predicts where to aim based on robot translation and time-of-flight.
2. **SOTM angular layer**: adds robot angular-velocity compensation.
3. **Turret lag layer**: adds direct turret lag feedforward (`omega * seconds`) to reduce control-path delay.
4. **Gate layer**: decides if a shot is allowed.

Tune each layer in order so changes are isolated.

---

## 1.5) Prerequisite Roadmap (Do This Before Full SOTM Tuning)

Before spending cycles on final SOTM parameter tuning, finish these prerequisite phases:

1. **Breakbeam timing characterization**
   - Collect repeated 1-ball, 2-ball, 3-ball datasets (for example, ~10 runs each).
   - Use shot/breakbeam logs to quantify launch sequencing consistency.
2. **Boost logic stabilization**
   - Improve boost/feed behavior so ball 1/2/3 launch conditions are consistent.
   - Reduce shot-to-shot spread before touching final ballistics.
3. **Stationary ballistics remap**
   - Re-tune RPM and hood for stationary shooting across key field locations.
4. **TOF table update**
   - Update `SOTM_TOF_FLIGHT_TIME_SEC` (and timing offsets) using improved stationary/moving data.
5. **Final SOTM tuning**
   - Tune lead gains, turret lag compensation, and fire gates.

This order prevents SOTM gains from compensating for upstream inconsistency.

For a run-by-run checklist template, see:
`scripts/SOTM_Tuning_Roadmap_Checklist.md`

---

## 2) Key Parameters (What They Do)

### Core SOTM

- `SOTM_ENABLED`: master switch for SOTM logic.
- `SOTM_MIN_SPEED_IN_PER_SEC`: minimum speed before translational lead is applied.
- `SOTM_LEAD_GAIN`: translational lead scale factor.
- `SOTM_MAX_LEAD_IN`: max lead vector magnitude clamp.
- `SOTM_USE_TOF_LOOKUP`: use distance->flight-time table.
- `SOTM_MECHANICAL_DELAY_SEC`: release delay offset added to ToF.
- `SOTM_BALL_TRANSFER_TIME_SEC`: additional feed delay offset.

### Angular Compensation

- `SOTM_ANGULAR_LEAD_GAIN`: scales `omega * tof` aiming term.
- `SOTM_OMEGA_FILTER_ALPHA`: low-pass strength for omega (`0` heavy filter, `1` no filter).

### Turret Lag Compensation (separate from SOTM math)

- `SOTM_TURRET_LAG_COMP_ENABLED`
- `SOTM_TURRET_LAG_COMP_SEC`: "look-ahead seconds" for lag FF (`turret += omega_deg_s * sec`).
- `SOTM_TURRET_LAG_COMP_MAX_DEG`: safety clamp on lag compensation contribution.

### Always-Tracking / Priming Behavior

- `SOTM_ALWAYS_TRACK_TARGETS`: keep SOTM aim active even when not firing.
- `SOTM_ALWAYS_PRIME_SHOOTER`: keep flywheel RPM primed from SOTM ballistic distance.

### Turret Gate

- `SOTM_FIRE_AIM_TOLERANCE_DEG`: max allowed goal-facing turret error for fire.
- `SOTM_FIRE_MAX_TURRET_SPEED_DEG_PER_SEC`: reject shots while turret moving too fast.
- `SOTM_REACHABILITY_TOLERANCE_DEG`: blocks shots if desired aim is clipped by turret constraints.
- `SOTM_REQUIRE_TURRET_READY_FOR_FIRE`: optionally require subsystem `isTurretReady()`.

### Turret Stiction Diagnostics (CSV only)

- `TURRET_DIAG_TARGET_STABLE_DELTA_DEG`
- `TURRET_DIAG_STICTION_ERROR_MIN_DEG`
- `TURRET_DIAG_STICTION_VELOCITY_MAX_DEG_PER_SEC`
- `TURRET_DIAG_SERVO_DEADBAND_POS_GUESS`
- `TURRET_DIAG_STICTION_MIN_TIME_MS`

---

## 3) Logging Setup (Required)

Enable before tuning:

- `ENABLE_SOTM_LOGGING = true`
- `ENABLE_TURRET_LOGGING = true`

Use:

- `SOTM_LOG_PERIOD_MS = 25`
- `TURRET_LOG_PERIOD_MS = 25`

You need both logs to separate:

- SOTM aiming behavior
- turret actuator behavior/stiction

---

## 4) Recommended Baseline Start Values

Use these as initial values:

- `SOTM_OMEGA_FILTER_ALPHA = 0.20`
- `SOTM_ANGULAR_LEAD_GAIN = 0.25`
- `SOTM_TURRET_LAG_COMP_ENABLED = true`
- `SOTM_TURRET_LAG_COMP_SEC = 0.10`
- `SOTM_TURRET_LAG_COMP_MAX_DEG = 12.0`
- `SOTM_FIRE_AIM_TOLERANCE_DEG = 4.0`
- `SOTM_REACHABILITY_TOLERANCE_DEG = 1.0`
- `SOTM_FIRE_MAX_TURRET_SPEED_DEG_PER_SEC = 220.0`
- `SOTM_REQUIRE_TURRET_READY_FOR_FIRE = false`

---

## 5) Tuning Procedure (Field Steps)

### Step A: Stationary Ballistics Baseline (no moving-shot lead)

First make sure stationary shooting is good, so SOTM tuning is not hiding ballistics error.

- Use stationary shots at near/mid/far distances.
- Verify RPM and hood map are accurate when robot speed is near zero.
- Fix stationary mapping first if needed.

### Step B: TOF Calibration (early, before gains)

Tune time model first, because both translational and angular lead scale with total time.

- Set `SOTM_LEAD_GAIN = 1.0`.
- Keep lag compensation and angular gain modest while doing TOF calibration.
- Tune these first:
  - `SOTM_TOF_FLIGHT_TIME_SEC` (table shape across distance)
  - `SOTM_MECHANICAL_DELAY_SEC` (global timing offset)
  - `SOTM_BALL_TRANSFER_TIME_SEC` (if feed path adds measurable delay)

If TOF is wrong, gains can look right in one scenario and fail at other speeds/distances.

### Step C: Turret Lag Feedforward (`SOTM_TURRET_LAG_COMP_SEC`)

Run Drill A/B below and adjust only `SOTM_TURRET_LAG_COMP_SEC`:

- If turret **lags during robot turns**, increase by `+0.02`.
- If turret **leads/overshoots** and starts wobbling, decrease by `-0.02`.
- Keep within `0.06` to `0.16` initially.

Do not change `SOTM_ANGULAR_LEAD_GAIN` yet.

### Step D: SOTM Angular Lead (`SOTM_ANGULAR_LEAD_GAIN`)

After lag FF is close:

- If moving-shot aim is still behind during rotational motion, increase `+0.05`.
- If shake/over-correction appears, decrease `-0.05`.

Typical working range: `0.15` to `0.35`.

### Step E: Gate Tuning (only after aiming is stable)

- If too many valid shots are blocked: increase `SOTM_FIRE_AIM_TOLERANCE_DEG` by `+0.5`.
- If bad-angle shots sneak through: lower tolerance by `-0.5`.
- Keep reachability protection enabled (`SOTM_REACHABILITY_TOLERANCE_DEG` small, around `1.0`).

---

## 6) Drill Plan

### Drill A: Turn-in-Place Tracking

- Hold fire request mode without feeding balls.
- Command alternating quick CW/CCW turns.
- Goal: minimize persistent turret lag without overshoot.

### Drill B: Constant-Speed Strafe

- Strafe left/right at moderate speed while tracking goal.
- Goal: stable aim, no head-shake, smooth target updates.

### Drill C: Real Shots While Moving

- Repeat B while firing.
- Goal: high gate pass rate with stable hit quality.

Collect a new SOTM and turret log for each parameter change set.

---

## 7) Log Columns to Watch

### SOTM Log

- `sotm_turret_goal_error_deg`
- `sotm_turret_gate`
- `sotm_turret_reachable`
- `sotm_turret_speed_gate`
- `sotm_omega_raw_deg_s`
- `sotm_omega_deg_s`
- `sotm_turret_lag_comp_deg`
- `sotm_can_shoot_gate`

### Turret Log

- `turret_target_deg`, `turret_measured_deg`, `turret_error_deg`
- `turret_outer_trim_deg`
- `turret_servo_command_deg`
- `turret_servo_pos`
- `turret_encoder_vel_deg_s`
- `turret_cmd_above_deadband_guess`
- `turret_target_stable`
- `turret_limit_clipped`
- `turret_stiction_suspect`

---

## 8) Diagnosing 1-2 Degree Steady-State Gap

If target is stable and error stays non-zero:

- If `turret_limit_clipped = true`: you are constraint-limited, not stiction.
- If `turret_stiction_suspect = true`: likely stiction/deadband or insufficient command magnitude.
- If `turret_cmd_above_deadband_guess = false` with steady error: trim command too small.

Then adjust in this order:

1. Verify no clipping.
2. Increase turret authority modestly (for example turret outer-loop gain in turret subsystem).
3. Re-check `turret_stiction_suspect` frequency and shot accuracy.

---

## 9) Safe One-Session Tuning Script

1. Start baseline values.
2. Run stationary shots at multiple distances and confirm baseline RPM/hood.
3. Calibrate TOF first (`SOTM_TOF_FLIGHT_TIME_SEC`, then delay terms).
4. Run Drill A/B for 20-30 sec, log.
5. Change only `SOTM_TURRET_LAG_COMP_SEC` by `0.02`.
6. Repeat A/B and pick best lag FF value.
7. Tune `SOTM_ANGULAR_LEAD_GAIN` in `0.05` steps.
8. Run Drill C and tune gate tolerance last.

Avoid changing more than one primary parameter between logs.

---

## 10) Quick Rollback If Behavior Gets Worse

If oscillation or unstable shot gating appears:

- `SOTM_TURRET_LAG_COMP_SEC -= 0.02` (or disable lag comp temporarily)
- `SOTM_ANGULAR_LEAD_GAIN -= 0.05`
- keep `SOTM_OMEGA_FILTER_ALPHA` at `0.20`

Re-test before changing anything else.

