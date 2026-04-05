# SOTM Roadmap Checklist

Use this as a phase-gate checklist so each tuning step is built on stable data.

---

## Phase 0: Test Standards (Do Once)

- [ ] Use same robot config each run (battery target, compression state, ball type/condition).
- [ ] Use same warmup procedure (flywheel spin-up time and pre-shot routine).
- [ ] Use consistent driver profile per test (stationary vs controlled strafe vs turn).
- [ ] Enable required logs:
  - [ ] `ENABLE_SHOT_INFO_LOGGING`
  - [ ] `ENABLE_TURRET_LOGGING`
  - [ ] `ENABLE_SOTM_LOGGING`
- [ ] Keep one variable change per test batch.

Exit criteria:
- [ ] Repeatability baseline is acceptable (no major run-to-run setup drift).

---

## Phase 1: Breakbeam Timing Characterization

Goal: quantify shot sequencing timing for 1-ball, 2-ball, 3-ball cases.

### Required data
- [ ] At least 10x one-ball runs
- [ ] At least 10x two-ball runs
- [ ] At least 10x three-ball runs
- [ ] Shot breakbeam CSV saved for each batch

### Metrics to summarize
- [ ] Shot interval means and spread
- [ ] BB2 clear gaps between shots
- [ ] Missing third-shot frequency
- [ ] Outlier timing events

### Suggested pass criteria
- [ ] Low outlier rate
- [ ] Stable interval spread across repeat runs
- [ ] No unexplained sequencing failures

Notes:
- If this phase is noisy, do not continue to SOTM tuning yet.

---

## Phase 2: Boost Logic Stabilization

Goal: make ball 1/2/3 launch behavior consistent.

### Actions
- [ ] Adjust boost timing/logic based on Phase 1 timing distributions
- [ ] Re-run 1/2/3-ball tests after each logic update
- [ ] Compare spread before/after

### Metrics to watch
- [ ] Ball-to-ball consistency (especially ball 2 and ball 3)
- [ ] Reduction in timing outliers
- [ ] Reduction in distance/impact variance across burst shots

### Suggested pass criteria
- [ ] 3-ball burst consistency is acceptable for your scoring goals
- [ ] Ball 2/3 no longer show systematic under/over behavior vs ball 1

---

## Phase 3: Stationary RPM/Hood Mapping

Goal: retune stationary ballistics once feed/boost is stable.

### Required test set
- [ ] Near, mid, far fixed locations
- [ ] Multiple repeats per location
- [ ] RPM and hood values logged with outcome quality

### Actions
- [ ] Tune RPM map for stationary accuracy
- [ ] Tune hood map for stationary accuracy
- [ ] Re-verify after map updates

### Suggested pass criteria
- [ ] Stationary shot accuracy is stable across all key locations
- [ ] No large location-dependent bias remains

---

## Phase 4: TOF Model Calibration (Before Final SOTM Gains)

Goal: fix timing model so SOTM gains are not compensating wrong flight time.

### Actions
- [ ] Update `SOTM_TOF_FLIGHT_TIME_SEC` table by distance
- [ ] Tune `SOTM_MECHANICAL_DELAY_SEC`
- [ ] Tune `SOTM_BALL_TRANSFER_TIME_SEC` if needed
- [ ] Keep `SOTM_LEAD_GAIN = 1.0` during initial TOF calibration

### Validation
- [ ] Check moving shots at multiple distances/speeds
- [ ] Confirm improvements transfer across scenarios (not just one case)

### Suggested pass criteria
- [ ] Lead timing bias reduced across distances
- [ ] No strong underlead/overlead trend remains

---

## Phase 5: Final SOTM and Turret-Lag Tuning

Goal: refine moving-shot behavior once upstream systems are stable.

### Tune in this order
1. [ ] `SOTM_TURRET_LAG_COMP_SEC`
2. [ ] `SOTM_ANGULAR_LEAD_GAIN`
3. [ ] `SOTM_LEAD_GAIN` (small correction only)
4. [ ] Fire gates (`SOTM_FIRE_AIM_TOLERANCE_DEG`, etc.)

### Logs to use
- [ ] `sotm_turret_goal_error_deg`
- [ ] `sotm_turret_lag_comp_deg`
- [ ] `sotm_omega_raw_deg_s`, `sotm_omega_deg_s`
- [ ] `sotm_turret_gate`, `sotm_can_shoot_gate`
- [ ] Turret stiction/clip diagnostics from turret CSV

### Suggested pass criteria
- [ ] No oscillation under shooting-on-the-move conditions
- [ ] Reliable fire gate pass rate when aim quality is good
- [ ] Good hit consistency while moving

---

## Per-Session Log Sheet Template

Session metadata:
- Date:
- Driver:
- Battery state:
- Field setup:
- Ball type/condition:

Run table (copy/paste rows as needed):

| Run | Phase | Scenario | Parameters changed | Files saved | Observed result | Next action |
|-----|-------|----------|--------------------|------------|-----------------|------------|
| 1 |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |

---

## Stop Conditions (Avoid Wasting Test Time)

Stop current phase and fix fundamentals if any are true:

- [ ] Significant run-to-run setup drift
- [ ] Inconsistent feed sequencing not explained by current model
- [ ] Stationary ballistics not stable enough for TOF extraction
- [ ] Multiple parameters changed simultaneously without clear attribution

