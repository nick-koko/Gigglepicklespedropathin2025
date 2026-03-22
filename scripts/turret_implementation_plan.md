# Turret Implementation Plan (Draft for Team Review)

This document summarizes the turret control approach we discussed so the team can align on design decisions before coding.

**Audience note:** This document is written for both programmers and newer students.  
If you are new, read sections in this order:
1) Goals
2) Hardware & Sensors
3) State Machine
4) Control Architecture
5) Comparison / Borrow-Avoid checklist
Then come back to tuning and formulas.

---

## Goals

- **Shoot faster** by decoupling aiming from chassis rotation.
- **Reduce driver workload** by automatically aiming the turret at the goal.
- **Enable future “shooting while moving”** by using smooth tracking + motion planning.
- **Stay reliable at high speed** (no lost encoder counts, no wire damage, predictable behavior).

---

## Quick Concept Glossary (Student-Friendly)

- **Turret:** rotating shooter assembly that can aim without turning the whole robot.
- **Closed-loop control:** we measure where the turret is, compare to where we want it, and correct continuously.
- **Setpoint / target:** desired turret angle right now.
- **Feedback:** sensor data used to correct error.
- **Feedforward:** “preemptive” output based on expected motion (reduces lag).
- **Quadrature encoder:** high-resolution incremental angle sensor (excellent for fast motion tracking).
- **Absolute encoder:** gives real absolute angle after power-up (great for zeroing and sanity checks).
- **kP / kD / kV / kS:** controller gains:
  - `kP`: position correction strength.
  - `kD`: damping against overshoot/oscillation.
  - `kV`: command needed to maintain velocity.
  - `kS`: minimum push to break static friction.
- **SOFT limit:** warning/behavior region before wire-risk area.
- **HARD limit:** absolute software clamp to protect wiring.
- **LOS rate (line-of-sight rate):** how fast the goal-bearing angle changes due to robot motion.

---

## Hardware & Sensors (Current Plan)

### Actuation
- **2× Axon MINI MK2 servos** mechanically linked to the turret at **1:1**.
- Run servos in **Continuous Rotation (CR) mode** and implement **our own closed-loop control**.
  - Avoids two “position-mode servo loops” fighting each other.
  - Lets us use external sensors (quadrature) as the source of truth.
  - Gives us one consistent turret controller regardless of servo firmware behavior.

### Position Sensing (Two-Sensor Strategy)
1. **Absolute analog angle (0–3.2 V, 0–360°)**  
   Use for:
   - **Zeroing** turret at init (establish absolute heading)
   - **Sanity checks / fault detection** (detect slip, missed counts, unplugged sensor)

2. **Quadrature encoder** for incremental tracking during motion  
   Use for:
   - **Primary turret angle** after initialization (high precision, robust to power rail noise)
   - **Velocity estimation** (for damping + feedforward)

**Why this split is important:**  
- Absolute encoder is best at answering: “Where am I right now after boot?”  
- Quadrature is best at answering: “How much did I move, and how fast?”  
Combining these roles gives both startup certainty and runtime precision.

### Encoder Port Assignment (Important)
- Put the turret quadrature encoder on a **hardware-decoded encoder port** (**Control Hub ports 0 or 3**).
- Avoid ports 1/2 for high CPR/high-speed encoders (software decode can miss counts).

---

## Key Design Constraints

### Wire management (no slip ring)
- Turret has no hard mechanical stops, but **wiring harness requires software limits**.
- Working range target: **±200°** (HARD limits), with **SOFT limits** to trigger recenter behavior.

**Important student intuition:**  
Even if the turret *mechanically* can spin far, the cable bundle cannot safely twist forever.  
So software limits are not optional; they are part of robot safety and reliability.

Recommended limits:
- **HARD:** ±200°
- **Clamp margin:** ±195° (safety buffer)
- **SOFT:** ±160° (start recenter/handoff behavior)

---

## Control Architecture (4 Layers)

### 1) Geometry: “Where should we aim?”
We already compute goal bearing from robot pose:
- `goalField = atan2(goalY - robotY, goalX - robotX)`
- Desired turret angle relative to robot:
  - `turretDesired = wrapTo180(goalField - robotHeading - turretForwardOffset)`

Plain-language version:
- First, find where the goal is on the field.
- Then subtract where the robot is facing.
- Result = how much the turret should rotate relative to the robot.

### 2) Limit Manager: “How do we protect the wires?”
- Enforce **HARD clamp** always (never exceed safe range).
- When the turret approaches the limits, **shift some aiming responsibility to the chassis** (in allowed states).
- Core idea:
  - **Total aim ≈ robotHeading + turretAngle + turretForwardOffset**
  - If turret can’t rotate further, the robot must rotate to maintain aim.

Plain-language version:
- Turret and chassis can “share” aiming responsibility.
- Near wire-risk zones, chassis helps more so turret does not over-rotate.

### 3) Motion Planner: “How do we move smoothly?”
Avoid setpoint “jumps” that cause:
- huge derivative spikes
- violent turret snaps
- massive velocity feedforward spikes

Use a planner that outputs a **planned position and planned velocity** with limits:
- max turret speed (deg/s)
- max turret accel (deg/s²)

This planner becomes the “smooth setpoint generator.”

Plain-language version:
- Planner prevents “teleporting” targets.
- The turret gets a ramped command it can physically follow.
- This keeps shots stable and hardware stress low.

### 4) Controller: “How do we command servo power?”
Servo CR power command combines:
- **Position feedback (P):** correct residual angle error
- **Velocity feedback (D on velocity error):** damping / stability
- **Velocity feedforward (kV):** reduce lag when setpoint is moving
- **Static friction feedforward (kS):** overcome stiction at low speed

Recommended control form:
- `power = kP*(posPlan - posMeas) + kD*(velPlan - velMeas) + kV*velPlan + kS*sign(velPlan)`

Notes:
- Use **one encoder loop** (“master”) and command both servos with same power.
- Second servo encoder (if available) can be used as a **health check** (optional).

Practical note:
- The formula is not “advanced math for its own sake”; each term solves a specific behavior:
  - `kP` pulls to target.
  - `kD` reduces wobble.
  - `kV` helps track moving plans without lag.
  - `kS` gets the system unstuck when command is small.

---

## One Loop Walkthrough (What Happens Every Cycle)

1. Read sensors (quadrature + absolute + robot pose/heading/velocity).
2. Compute desired turret angle from geometry.
3. Apply limit logic (SOFT/HARD behavior and possible chassis handoff).
4. Planner generates smooth `posPlan` and `velPlan`.
5. Controller computes servo power from feedback + feedforward.
6. Apply voltage compensation and clamps.
7. Command both CR servos.
8. Evaluate readiness logic (stable for N loops before firing).

This repeats continuously during teleop/auto, usually dozens of times per second.

---

## State Machine (Driver/Gameplay Behavior)

We’ll implement turret + chassis behavior using three states:

### State 1 — INTAKE (balls < 3 and aim button NOT pressed)
**Priority:** driver-controlled chassis heading for intake.
- Turret: **tracks goal** within limits.
- Robot auto-yaw: **disabled** (no chassis rotation assist).
- Near HARD limit: turret clamps; aim error may exist, but wires are protected.

Why this helps cycle time:
- Turret already stays “in the aiming neighborhood” while gathering balls.
- Less reacquire time when switching back to shoot.

### State 2 — READY / FULL (balls == 3 and aim button NOT pressed)
**Priority:** get aimed quickly and efficiently.
- Turret: tracks goal.
- Robot auto-yaw: **enabled only when needed** near turret limits (SOFT zone and beyond).
- Driver override: if driver turn stick is active, auto-yaw reduces to 0.

Optional optimization:
- If driver is translating, allow chassis to rotate toward travel direction for faster mecanum driving **only if it doesn’t interfere with defense avoidance** and can be overridden instantly.

### State 3 — AIM (aim button pressed)
**Priority:** strongest aiming automation.
- Behaves like READY/FULL regardless of ball count.
- Optionally enable “auto-shoot when ready” if:
  - turret error < threshold
  - robot heading error < threshold (if yaw assist is active)
  - shooter RPM ready and feeder conditions met

Student note:
- “Ready” should be based on **stable** conditions, not one-frame snapshots.
- That is why loop counters (N consecutive good loops) are important.

---

## Turret Setpoint Selection (Wrap + Multi-turn)

Since our turret angle will be tracked continuously (multi-turn), we must avoid unnecessary long spins.

Steps:
1. Compute canonical desired angle: `turretDesired` in (-180..+180]
2. Consider equivalent targets: `turretDesired + 360*k`
3. Choose the candidate that:
   - stays within `[−CLAMP, +CLAMP]`
   - minimizes `abs(candidate - turretAngleContinuous)`

This prevents “flip across the back” behavior.

Plain-language version:
- Angles repeat every 360 degrees.
- We choose the equivalent target that needs the shortest legal movement.
- This avoids needless full rotations.

---

## Near-Limit Behavior: Blended Handoff (READY/AIM only)

When `|turretAngle|` enters SOFT zone:
- Blend factor:
  - `alpha = smoothstep(SOFT, CLAMP, |turretAngle|)`  (0→1)

Then:
- Pull turret target back toward center:
  - `turretCmd = turretTarget + alpha*(clamp(turretTarget, -SOFT, +SOFT) - turretTarget)`
- Compute robot heading target that preserves aim:
  - `robotHeadingTarget = goalField - turretCmd - turretForwardOffset`
- Apply robot turn PID *scaled by alpha* (and disabled if driver override is active).

This produces:
- turret recenters automatically,
- chassis quietly takes over aiming near limits,
- no sudden snaps.

Student intuition:
- `alpha` is a “how close to danger” slider.
- Far from limit: turret does most aiming.
- Near limit: chassis takes more of the turn.

---

## Encoder Resolution & Count-Rate Notes

### Turret max speed estimate
Based on Axon spec example: ~0.09 s / 60° ⇒ ~110 rpm (~1.833 rev/s).

### Count-rate scaling
If the encoder sees `N_ticks_per_turret_rev`, then:
- `ticks/sec ≈ N * 1.833` at 110 rpm

Higher gear ratio (smaller encoder gear) increases ticks/sec, so:
- Prefer **hardware decoder ports (0/3)** for the turret encoder.
- If needed, cap planned turret speed in the motion planner.

### Your specific ratio update (must-do)
Since your encoder-to-turret ratio is different from 24064:
- `ticksPerTurretRev = encoderTicksPerEncoderRev * encoderRevsPerTurretRev`
- `degreesPerTick = 360 / ticksPerTurretRev`

Do this before tuning. If this conversion is wrong, every gain tune will be misleading.

### Practical guidance
- We already have **more than enough resolution** with 4096 counts/rev (and any reasonable gearing).
- Prioritize **stiff mounting + low backlash + no missed counts** over chasing more ticks.

---

## Calibration & Initialization Plan

1. **Mechanical alignment**
   - Define turret “forward” direction (0° relative to robot).
2. **Analog absolute zero**
   - Read analog angle at init and compute offset so forward = 0.
3. **Quadrature zero**
   - Set quadrature count = 0 at init using the analog-based reference.
4. **Sanity check**
   - During operation, compare quadrature-derived angle vs analog-derived angle.
   - If drift exceeds threshold for >X ms, trigger warning/fault mode.

Recommended fault response ladder:
1. Warning only (log + telemetry flag).
2. Limit max turret speed.
3. Force safe mode / hold.
4. Optional re-zero action when safe.

---

## Telemetry & Driver Feedback (Recommended)

Display/log:
- mode (INTAKE / READY / AIM)
- turret angle (continuous), turret target, planner target
- `alpha` (limit blend factor)
- turret clamp flag (near HARD limit)
- quadrature velocity (filtered)
- analog vs quadrature mismatch
- servo power command

Optional:
- driver rumble or LED indicator when:
  - turret is in SOFT zone
  - turret is clamped at HARD
  - encoder mismatch fault occurs

Student training tip:
- During early testing, always display:
  - `target angle`, `measured angle`, `error`, and `servo power`.
- These four signals usually explain 80% of turret behavior issues.

---

## Development & Test Plan (Parallel Work)

### Bench / Test Stand (Turret-only)
1. Bring up quadrature counting on CH port 0/3.
2. Validate direction, invert follower servo if needed.
3. Validate analog zeroing and multi-turn tracking.
4. Implement planner + controller; tune kS, kV, kD, then kP.
5. Stress test at high speed: verify no missed counts.

Add explicit pass/fail checks:
- Commanded direction matches measured direction.
- No dropped counts during repeated fast sweeps.
- Angle mismatch stays within threshold during acceleration/deceleration.
- System returns to same physical heading after repeated cycles.

### On-Robot Integration
1. Integrate geometry with real robot pose + heading.
2. Implement state machine + driver override.
3. Verify limit handling and chassis handoff (READY/AIM only).
4. Add shooter gating and optional auto-fire.

---

## Tuning Order (Simple & Reliable)

1. **kS (stiction):** smallest value that reliably starts motion.
2. **kV (velocity FF):** map planned velocity to approximate required power.
3. **kD (velocity damping):** reduce overshoot and oscillation.
4. **kP (position):** tighten final pointing accuracy without jitter.

Keep velocity filtered; analog sensors are noisy under load.

Common tuning mistakes to avoid:
- Tuning `kP` first (usually creates oscillation before damping exists).
- Adding too many terms at once.
- Ignoring voltage effects when battery is low.
- Tuning without logging target vs measured angle.

---

## Comparison: This Plan vs 24064-Decode Turret

### What 24064 does well (and we should keep)
- **State-machine orchestration** between shooter readiness and turret tracking (not just a raw PID loop).
- **Geometry-first aiming** from field pose to goal bearing, then conversion into turret-relative domain.
- **Motion compensation** using drivetrain velocity/angular velocity (line-of-sight rate feedforward), not only positional error correction.
- **Stability-gated shooting** with multi-loop tolerance checks (prevents firing on transient in-tolerance spikes).
- **Voltage compensation** so loop behavior is more consistent as battery sags.

### Why their “non-tracking” behavior looked inefficient
- In their non-tracking state (`IDLE`), turret target is effectively parked (toward a neutral angle), rather than holding a useful pre-aim orientation.
- Result: turret can turn away from goal during intake and then must turn back to reacquire before shooting.

### Our plan is better for this point
- Keep your proposed behavior where turret remains useful while not in explicit AIM mode:
  - **INTAKE:** track goal within limits, no chassis auto-yaw.
  - **READY/AIM:** track goal + blended chassis handoff near limits.
- If you ever need a “park” mode, make it explicit (manual/maintenance), not default gameplay behavior.

---

## Recommended Updates Based on 24064 Analysis

### 1) Add explicit LOS-rate feedforward to controller layer
Your current controller section already includes `kV*velPlan`. Add a second feedforward term tied to goal line-of-sight rotation from robot motion:
- `ff_los = kV_los * alphaDot` (and optional `kA_los * alphaDotDot`)
- This is one of the strongest parts of 24064’s moving-shot consistency.

Student explanation:
- If the robot drives sideways, the goal angle changes even if turret hasn’t moved yet.
- LOS feedforward starts turret motion early so PID has less “catch up” work.

### 2) Add battery-voltage scaling to output
- Scale command by `MAX_VOLTAGE / measuredBatteryVoltage` (with sensible clamps).
- This should apply to both PID and feedforward contributions.

Why:
- Same command at 12.8V and 11.8V produces different real torque/speed.
- Voltage compensation makes behavior more repeatable across a match.

### 3) Add anti-oscillation shoot gating counters
- Keep your ready thresholds, but also add loop counters:
  - `readyLoops >= N` before allowing auto-fire.
  - `outOfToleranceLoops >= M` to revoke readiness.
- This mirrors a proven competition pattern in 24064.

### 4) Define non-tracking behavior explicitly
Choose and lock one policy now:
- **Preferred:** hold-last-useful-angle or continue constrained tracking.
- Avoid implicit park-to-zero in match flow.

This directly addresses the observed 24064 behavior where turret turns away from goal in idle.

### 5) Keep two-sensor strategy, but formalize fusion/authority
- Primary runtime angle source: **quadrature** (as already planned).
- Absolute encoder: **init zero + health checks + re-zero conditions**.
- Add an explicit rule for when analog mismatch triggers warning vs forced re-align.

### 6) Add exact conversion formula for your new gear ratio
Because your turret-to-encoder ratio differs, define this in config:
- `ticksPerTurretRev = encoderTicksPerEncoderRev * encoderRevsPerTurretRev`
- `degreesPerTick = 360.0 / ticksPerTurretRev`
- Validate sign/direction and measured angle on bench before tuning gains.

---

## Borrow / Avoid Checklist from 24064

### Borrow directly
- State-machine coupling between shooter and turret.
- Goal-tracking geometry and wrapped angle domain.
- LOS-rate feedforward and predictive compensation.
- Voltage compensation.
- Multi-loop readiness gating.

### Avoid or modify
- Default gameplay park-to-neutral behavior in non-tracking states.
- Overloading too many tunables at once during early bring-up.

---

## Additional Comparison: This Plan vs FTC-23511 Decode-2026

This section adds lessons from a second team that also uses:
- **two CR servos** to drive the turret,
- **an absolute encoder** for initialization/sanity,
- **a quadrature encoder** for runtime control.

Their overall architecture is strong and competition-ready, but your observed behavior (brief overshoot + oscillation while final aiming) is consistent with several implementation details.

### What 23511 does well (and we should keep)

- **Correct two-sensor authority split**:
  - absolute sensor for sync at startup,
  - quadrature for continuous control/velocity.
- **Continuous goal-lock architecture** with explicit turret states (goal lock, angle control, off).
- **Moving-shot prediction flow** that updates turret, hood, and flywheel from robot motion, not just static distance.
- **Voltage-aware control terms** and explicit static friction compensation (`kS` style behavior).
- **Turret/chassis responsibility split near limits** (important for wire safety and practical aim range).

Student explanation:
- This is a good example of "system thinking": they did not build "just a PID loop."  
  They built geometry, constraints, and shot orchestration around the loop.

### Likely causes of the observed overshoot/oscillation

Important note: this is usually **architecture + tuning**, not one or the other.

1) **Setpoint can change quickly during moving aim**
- In moving-shot mode, target angle is recomputed continuously from pose + velocity.
- If setpoint motion is not sufficiently shaped, the turret may chase a moving target aggressively.

2) **Readiness based on instant in-tolerance checks**
- If firing logic accepts "at target now" without requiring stable dwell time,
  it can trigger near transient crossings where residual oscillation still exists.

3) **Control-output discontinuity near setpoint**
- If one term drops out abruptly at setpoint while feedforward remains active,
  it can produce small push/pull behavior around zero error.

4) **Very high far-from-target authority**
- High max output and aggressive gains are excellent for fast acquisition,
  but can cause brief overshoot if damping and setpoint shaping are not balanced.

Student explanation:
- Think of aiming like stopping a shopping cart at a tape mark:
  - high push gets you there fast,
  - but without enough "braking behavior," you roll past and correct back.

### Updates to our plan based on 23511 review

These are incremental updates to your current plan, not a redesign.

#### 1) Promote motion shaping from "recommended" to "required"
- Keep planner limits on turret speed and acceleration.
- Add explicit jerk-awareness if needed (or conservative accel cap) to reduce snap at target transitions.

Why:
- Moving-shot prediction changes target every loop.  
  A shaped setpoint turns that into physically trackable motion.

#### 2) Formalize stability-gated shooting (dwell-time gating)
- Require all of the following before launch authorization:
  - `abs(turretError) < turretErrGate`
  - `abs(turretVel) < turretVelGate`
  - shooter speed and feeder conditions are valid
  - maintained for `N` consecutive loops (or `T` ms)
- Revoke readiness only after `M` bad loops to avoid chatter.

Why:
- Prevents firing during momentary crossings that look "ready" for one frame but are not actually stable.

#### 3) Separate "acquire" vs "settle" gain behavior explicitly
- Use gain/output scheduling:
  - **Acquire region:** stronger authority for fast gross movement.
  - **Settle region:** lower max output + more damping for smooth stop.
- Keep this behavior visible in telemetry so students can learn the transition.

Why:
- Fast to target and smooth at target are different control jobs.

#### 4) Keep feedforward, but gate/shape it near zero crossing
- Maintain velocity/LOS feedforward for moving shots.
- Add a taper or blending rule near tiny error and low planned velocity so feedforward does not cause micro-hunting.

Why:
- Feedforward is powerful for tracking, but unshaped near zero can keep nudging the turret when it should fully settle.

#### 5) Add "shot quality telemetry" for student debugging
- Display:
  - target angle, measured angle, error,
  - measured turret velocity,
  - planner velocity,
  - control output terms (`P`, `D`, `FF`, `kS`),
  - stable-loop counter (`readyLoops`).

Why:
- New students can connect "what the robot does" to "which signal changed."
- This shortens tuning time and reduces guess-based changes.

#### 6) Keep non-tracking behavior intentional (no pointless motion)
- Preserve your existing strategy:
  - INTAKE uses constrained tracking (or hold-last-angle if selected by team), not arbitrary park-to-neutral.
- Only park for explicit service/transport mode.

Why:
- Avoids unnecessary turret motion that costs time when switching back to shoot.

### Borrow / Avoid Checklist from 23511

#### Borrow directly
- Dual-sensor authority split (absolute init + quadrature runtime).
- Continuous goal-lock plus moving-shot prediction architecture.
- Voltage-aware control and static-friction compensation.
- Limit-aware chassis/turret shared responsibility.

#### Modify before adopting
- Instant "ready now" logic -> replace with stable dwell gating.
- Any abrupt control/output transitions near setpoint.
- Any high-authority tuning that is not paired with a clear settle mode.

---

## Teaching Example (Simple Numeric Walkthrough)

Assume:
- Robot heading = 30 deg
- Goal bearing in field frame = 70 deg
- Turret forward offset = 0 deg

Then:
- `turretDesired = wrapTo180(70 - 30 - 0) = +40 deg`

If current turret angle is +20 deg:
- Position error = +20 deg (need to rotate more toward goal)
- Controller outputs positive servo power (plus feedforward if planner says moving target)

Near limits:
- If turret is at +170 deg and SOFT=160, CLAMP=195, `alpha` rises.
- Turret command is pulled toward center while chassis yaw contributes more.
- Net result: still aims, but wiring stays safe.

---

## Open Decisions / Questions for Team

- Final SOFT limit value (e.g., 150–170°).
- Planned max turret speed (cap) for stability and wire safety.
- Thresholds for “ready to shoot” (turret error, heading error, etc.).
- Whether to implement “travel-direction auto yaw” in READY mode (optional).
- Whether INTAKE should be **track-goal** or **hold-last-angle** (recommended: track-goal with limits).
- Initial values for LOS feedforward (`kV_los`, optional `kA_los`) and voltage compensation clamp bounds.
- Readiness dwell settings (`N` good loops, `M` bad loops) and initial error/velocity gate values.
- Final acquire-vs-settle gain schedule boundaries (error thresholds).

---

## Summary

- Use **quadrature encoder as the primary sensor** after init to avoid analog noise issues.
- Use **analog absolute** for zeroing + health checks.
- Use **CR mode** with **planner + PID + feedforward**.
- Implement a **state machine** with strict intake priority and blended limit handoff for READY/AIM.
- Put turret encoder on **hardware decoder ports 0 or 3** and validate max-speed count integrity early.
- Require **stability-gated launch** (error + velocity + dwell), especially for shooting while moving.

