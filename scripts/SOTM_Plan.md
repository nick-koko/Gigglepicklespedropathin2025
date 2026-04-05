A# Shooting On The Move (SOTM) - Staged Implementation Plan

## 1. The Problem

Currently, when the driver holds `gamepad1.left_bumper`, the robot:
1. Rotates to face the goal using a P-controller (`angleErrorDeg * shooterTargetkP`)
2. Calculates RPM based on odometry distance to target (`calculateShooterRPMOdoDistance`)
3. Sets hood angle based on distance (`calculateShooterHoodOdoDistance`)

**This requires the robot to stop moving to shoot.** We want to shoot while driving.

## 2. Why Stationary Aiming Doesn't Work While Moving

When a ball is launched from a moving robot, it inherits the robot's velocity. Think of it
like throwing a ball out of a moving car - the ball keeps moving in the direction the car
was going, even after it leaves.

If we aim directly at the goal while moving, the ball will **drift** in the direction of
our motion and miss.

```
    Goal (X)
      |
      |  <-- ball drifts this way
      | /
      |/
    Robot ---> moving this direction
```

## 3. The SOTM Solution: Lead-Point Aiming

We need to aim at a **virtual target** that compensates for the ball's inherited velocity.
The idea:

1. Get the robot's current velocity (vx, vy) in field coordinates
2. Estimate the ball's **time of flight** from robot to goal
3. Calculate the **drift** the ball will experience: `drift = velocity * time_of_flight`
4. Aim at `virtual_target = actual_target - drift`

This means if we're moving right, we aim slightly LEFT of the goal so the ball's rightward
drift carries it into the goal.

```
    Goal (X)     Virtual Target (O)
      |         /
      |        /  <-- we aim here instead
      |       /
      |      /
    Robot ---> moving right
```

## 4. Available Pedro Pathing APIs

### Translational Velocity (CONFIRMED: Field Coordinates)
```java
// Returns a Vector with x,y velocity components (inches/sec, Pedro field coordinates)
Vector velocity = PedroComponent.follower().getVelocity();
double vx = velocity.getXComponent();  // field-frame x velocity (in/sec)
double vy = velocity.getYComponent();  // field-frame y velocity (in/sec)
double speed = velocity.getMagnitude(); // total speed (in/sec)
```

**This is confirmed to be in FIELD coordinates (Pedro coordinate system), NOT robot-relative.**
We traced through the full source code chain to verify:

1. The GoBilda Pinpoint localizer is initialized with the robot's starting pose in Pedro
   field coordinates via `odo.setPosition(...)`.
2. The Pinpoint chip tracks position in that field frame. `odo.getVelX()` and `odo.getVelY()`
   return the derivative of that field-frame position — so velocity is field-relative.
3. In `PinpointLocalizer.update()`, the velocity is read directly:
   ```java
   currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), ...);
   ```
4. `PoseTracker.getVelocity()` passes this through as a `Vector` with x,y components.
5. `Follower.getVelocity()` returns that Vector directly.
6. Pedro's own `ErrorCalculator` uses this velocity alongside field-coordinate paths and
   poses without any rotation — confirming it's already in field coordinates.

**This means no coordinate rotation is needed for SOTM.** The vx/vy values can be used
directly with the field-coordinate shooting target positions.

**Recommended quick verification:** During testing, add telemetry showing vx and vy while
driving the robot in different directions. If you drive "up" on the field, the same velocity
axis should light up regardless of which direction the robot is facing. This takes 30 seconds
and removes any doubt.

### Angular Velocity
```java
// Returns angular velocity in radians/sec
double omega = PedroComponent.follower().poseTracker.getAngularVelocity();
```

This comes from `odo.getHeadingVelocity()` on the Pinpoint, stored as the heading component
of the velocity Pose in `PinpointLocalizer.update()`.

### Current Pose
```java
// Already used in the current code
Pose currentPose = PedroComponent.follower().getPose();
double botX = currentPose.getX();
double botY = currentPose.getY();
double botHeading = currentPose.getHeading();
```

### How It All Connects (source chain diagram)
```
GoBilda Pinpoint Chip
  │
  ├─ odo.getVelX(INCH) ──► field-frame X velocity
  ├─ odo.getVelY(INCH) ──► field-frame Y velocity
  └─ odo.getHeadingVelocity(RAD) ──► angular velocity
         │
         ▼
PinpointLocalizer.update()
  currentVelocity = new Pose(velX, velY, headingVel)
         │
         ▼
PoseTracker.getVelocity()  ──► returns currentVelocity.getAsVector()  (x,y as Vector)
PoseTracker.getAngularVelocity() ──► returns currentVelocity.getHeading()
         │
         ▼
Follower.getVelocity()  ──► returns poseTracker.getVelocity()
         │
         ▼
YOUR CODE: PedroComponent.follower().getVelocity()
  .getXComponent()  = field vx (in/sec)
  .getYComponent()  = field vy (in/sec)
  .getMagnitude()   = total speed (in/sec)
```

---

## 5. Implementation Stages Overview

The SOTM implementation is broken into stages that match our hardware readiness:

| Stage | When | Robot State | What Changes | Goal |
|-------|------|-------------|-------------|------|
| **1** | Now | No turret (or turret installed but locked forward) | Robot heading aims at virtual target, fixed ToF, hood compensation + predictive RPM for rapid fire | Validate lead-point math; rapid-fire while strafing |
| **2** | Turret SW working | Turret tracks virtual target | Turret aims instead of robot heading; angular compensation; rapid-fire carries forward | SOTM with turret — drive freely while shooting |
| **3** | After ToF measurements | Have measured ball flight times at multiple distances | Distance-dependent ToF replaces fixed constant; add ball transfer time | More accurate lead at all distances |
| **4** | Refinement | Everything working | Velocity blending, turret offset, iterative convergence | Match or exceed 23511's virtual-goal accuracy |

Each stage builds on the previous one. The aiming logic is structured so later stages
are small modifications to the code from earlier stages, not rewrites.

**Rapid-fire support** (hood compensation + predictive RPM targeting) is introduced in
Stage 1 (Sections 6.6-6.7) and carries forward unchanged to all later stages. It works
independently of the SOTM lead-point math and benefits stationary shooting too.

---

## 6. Stage 1 — Robot-Heading SOTM (No Turret Movement)

**Prerequisites:** Robot drives, shoots, has working odometry. No turret needed.

**Purpose:** Validate that the lead-point math produces correct virtual targets. The
robot rotates its heading to aim at the virtual target (same as current aiming, just
at a different point). Uses a fixed `ballTimeOfFlight` constant as a placeholder —
good enough for lateral-strafing validation.

### 6.1 Configurable Parameters

Add these to `Pickles2025Teleop` (tunable via Panels):

| Parameter | Description | Starting Value |
|-----------|-------------|----------------|
| `sotmEnabled` | Master toggle for SOTM mode | `false` |
| `ballTimeOfFlight` | Placeholder total time: mechanical delay + flight (seconds) | `0.35` |
| `sotmLeadGain` | Multiplier to tune lead amount (start at 1.0, adjust) | `1.0` |
| `sotmMinSpeed` | Minimum robot speed to apply SOTM (in/sec) | `5.0` |
| `sotmMaxLead` | Maximum lead offset in inches (safety cap) | `15.0` |

### 6.2 Aiming Logic Change

**Current flow** (inside `if (gamepad1.left_bumper)`):
```
botPose -> shootingTarget -> calculate angle error -> rotate robot
botPose -> shootingTarget -> calculate distance -> set RPM & hood
```

**Stage 1 SOTM flow** (replaces the above when SOTM is enabled):
```
botPose + velocity + timeOfFlight -> virtualTarget
botPose -> virtualTarget -> calculate angle error -> rotate robot
botPose -> virtualTarget -> calculate effective distance -> set RPM & hood
```

The robot still rotates its heading to face the target — the only difference is
*which* target it's facing.

### 6.3 Core Calculation

**Note:** This shows the lead-point aiming math only. Sections 6.6 and 6.7 add hood
compensation and predictive RPM targeting on top of this — see Section 6.7's "Combined
flow" for the recommended final version that includes all three features together.

Variables: `dx`/`dy` = vector from robot to real target, `ODODistance` = real distance
to target (already computed in current code).

```java
// --- Inside the left_bumper block ---

// 1. Get robot velocity in field coordinates
Vector robotVelocity = PedroComponent.follower().getVelocity();
double vx = robotVelocity.getXComponent();
double vy = robotVelocity.getYComponent();
double speed = robotVelocity.getMagnitude();

// 2. Only apply SOTM compensation if moving fast enough
if (sotmEnabled && speed > sotmMinSpeed) {

    // 3. Calculate lead offset (where the ball will drift to)
    double leadX = vx * ballTimeOfFlight * sotmLeadGain;
    double leadY = vy * ballTimeOfFlight * sotmLeadGain;

    // 4. Cap the lead to prevent wild aim points
    double leadMag = Math.sqrt(leadX * leadX + leadY * leadY);
    if (leadMag > sotmMaxLead) {
        double scale = sotmMaxLead / leadMag;
        leadX *= scale;
        leadY *= scale;
    }

    // 5. Virtual target = actual target MINUS the drift
    //    (we aim behind our motion so the ball drifts onto target)
    double virtualTargetX = shootTargetX - leadX;
    double virtualTargetY = shootTargetY - leadY;

    // 6. Recalculate angle and distance to VIRTUAL target
    double dxVirtual = virtualTargetX - botxvalue;
    double dyVirtual = virtualTargetY - botyvalue;
    double virtualFieldAngleRad = Math.atan2(dyVirtual, dxVirtual);
    double virtualAngleErrorRad = normalizeRadians(virtualFieldAngleRad - botHeadingRad);
    double virtualAngleErrorDeg = Math.toDegrees(virtualAngleErrorRad);

    // 7. Effective distance adjusts for radial velocity component
    double radialVelocity = (vx * dx + vy * dy) / ODODistance;
    double effectiveDistance = ODODistance - (radialVelocity * ballTimeOfFlight);

    // 8. Use virtual angle for aiming, effective distance for RPM & hood
    //    (Sections 6.6-6.7 replace these two lines with hood comp + predictive RPM)
    rotate = virtualAngleErrorDeg * shooterTargetkP;
    targetRPM = calculateShooterRPMOdoDistance(Math.max(effectiveDistance, minDistanceForShooting));
    shooterHoodPos = calculateShooterHoodOdoDistance(Math.max(effectiveDistance, minDistanceForShooting));

} else {
    // Standard stationary aiming (existing code)
    rotate = angleErrorDeg * shooterTargetkP;
    targetRPM = calculateShooterRPMOdoDistance(ODODistance);
    shooterHoodPos = calculateShooterHoodOdoDistance(ODODistance);
}
```

### 6.4 Stage 1 Tuning

**Step 1: Validate velocity readings (before touching SOTM)**
- Add telemetry to display `vx`, `vy`, `speed` while driving
- Verify values make sense (~40-60 in/sec max based on Constants: xVelocity=78, yVelocity=62.5)
- Drive the robot in different directions at various headings — confirm vx/vy track with
  field axes, not robot axes

**Step 2: Lateral strafing test**
- Place robot at a known position, roughly broadside to the goal
- Enable SOTM, start with `sotmLeadGain = 1.0`, `ballTimeOfFlight = 0.35`
- Drive sideways (pure strafe) past the goal while holding left_bumper
- Observe where balls land relative to the goal
- If balls consistently lead the goal: reduce `sotmLeadGain` or `ballTimeOfFlight`
- If balls consistently lag the goal: increase them

**Step 3: Diagonal/forward motion test**
- Drive at angles toward and away from the goal
- Verify the RPM adjustment (effective distance) works — balls that are short need more RPM,
  balls that are long need less
- Check that the hood angle adjusts correctly via `effectiveDistance`

**Step 4: Gate check — does it help?**
- Compare shot accuracy with SOTM on vs. off while strafing at moderate speed (~20 in/sec)
- If shots are closer to the goal with SOTM, the math is working — proceed to Stage 2
- If shots are worse, check telemetry for unexpected values before adjusting gains

### 6.5 Stage 1 Telemetry

```java
telemetry.addData("SOTM_enabled", sotmEnabled);
telemetry.addData("SOTM_vx", vx);
telemetry.addData("SOTM_vy", vy);
telemetry.addData("SOTM_speed", speed);
telemetry.addData("SOTM_leadX", leadX);
telemetry.addData("SOTM_leadY", leadY);
telemetry.addData("SOTM_virtualAngleErr", virtualAngleErrorDeg);
telemetry.addData("SOTM_effectiveDist", effectiveDistance);
telemetry.addData("SOTM_radialVel", radialVelocity);
```

### 6.6 Rapid-Fire Hood Compensation (All Stages)

**This applies from Stage 1 onward and carries forward to all later stages.**

When rapid-firing while moving, the flywheel RPM is constantly in flux: recovering from
the previous shot's energy loss while simultaneously chasing a changing target RPM as
distance changes. Waiting for the flywheel to settle before each shot kills fire rate
and is inconsistent. Instead, we let the hood servo absorb the RPM error.

**Core insight:** A slower-than-expected ball behaves like a ball shot at a proportionally
farther distance (more air time, more gravity drop, needs steeper angle). Since the hood
formula is a linear regression on distance, we can correct by computing an "apparent
distance" that accounts for the RPM deficit.

**New parameter:**

| Parameter | Description | Starting Value |
|-----------|-------------|----------------|
| `hoodCompEnabled` | Enable hood compensation for rapid fire | `true` |
| `minRpmRatio` | Minimum RPM ratio before shot is considered invalid | `0.70` |
| `maxRpmRatio` | Maximum RPM ratio clamp (prevents overcorrection) | `1.30` |

**Implementation:**

```java
double targetRPM = calculateShooterRPMOdoDistance(
    Math.max(effectiveDistance, minDistanceForShooting));
setFlywheelTarget(targetRPM);

double actualRPM = getFlywheelVelocity();
boolean hoodValid;
double apparentDistance = effectiveDistance;

if (hoodCompEnabled && actualRPM > 0 && targetRPM > 0) {
    double rpmRatio = actualRPM / targetRPM;
    rpmRatio = Math.max(minRpmRatio, Math.min(maxRpmRatio, rpmRatio));

    // Slower ball ≈ farther effective distance, faster ball ≈ closer
    apparentDistance = effectiveDistance / rpmRatio;

    shooterHoodPos = calculateShooterHoodOdoDistance(
        Math.max(apparentDistance, minDistanceForShooting));

    hoodValid = apparentDistance >= minDistanceForShooting
             && apparentDistance <= maxDistanceForShooting;
} else {
    shooterHoodPos = calculateShooterHoodOdoDistance(
        Math.max(effectiveDistance, minDistanceForShooting));
    hoodValid = true;
}
```

**Why this works:** The hood servo responds in ~50-100ms — fast enough to adjust between
consecutive shots in a rapid-fire sequence. After each shot causes an RPM dip, the hood
steepens slightly on the very next loop cycle. When the flywheel recovers, the hood
flattens back to normal. The driver never has to wait.

**Fire gating with hood compensation:** Replace the traditional "wait for flywheel RPM"
gate with a hood-validity gate. The only reason to block a shot is if the RPM is so far
off that no hood angle can compensate (RPM ratio below `minRpmRatio`):

```java
// OLD: gate on flywheel speed match
// boolean canShoot = Math.abs(actualRPM - targetRPM) < RPM_TOLERANCE;

// NEW: gate on hood compensation validity — allows rapid fire
boolean canShoot = hoodValid;
```

**How the `1/rpmRatio` approximation works:**

Your hood formula is a linear regression: `hoodAngle = m * distance + b`. It was tuned
assuming the flywheel is at the "correct" RPM for that distance. When the flywheel is
at 92% of target (e.g., recovering from a shot dip), the ball exits 8% slower. A ball
that's 8% slower spends ~8% more time in flight, drops ~8% more due to gravity, and
behaves approximately like a ball aimed at a target ~8% farther away. So:

```
apparentDistance = realDistance / 0.92 ≈ realDistance * 1.087
```

The hood formula then computes a steeper angle for this "farther" distance, which
compensates for the slower ball. The approximation is first-order, but your hood
formula is also first-order (linear regression), so they're well-matched for the
±5-15% RPM deviations you'll see in practice.

**Telemetry to add:**

```java
telemetry.addData("HOOD_rpmRatio", rpmRatio);
telemetry.addData("HOOD_apparentDist", apparentDistance);
telemetry.addData("HOOD_compensatedAngle", shooterHoodPos);
telemetry.addData("HOOD_valid", hoodValid);
```

### 6.7 Predictive RPM Targeting (All Stages)

**This also applies from Stage 1 onward.** It complements hood compensation by reducing
how much compensation is needed in the first place.

**Problem:** When driving toward the goal at 40 in/s, the distance shrinks ~10 inches
every 0.25 seconds. The "correct" RPM is dropping continuously, but the flywheel takes
~0.25s to spin down to each new target. The flywheel is always overshooting — and
spinning down is slow because there's only friction to remove energy (no braking).

**Solution:** Instead of targeting RPM for the current distance, target RPM for the
predicted distance when the flywheel will actually reach the new speed:

```java
public static double flywheelResponseTime = 0.25;  // tune: PID settle time (seconds)

// Radial velocity: negative = approaching goal, positive = retreating
double radialVelocity = (vx * dx + vy * dy) / realDistance;

// Predict where we'll be when the flywheel reaches this command
double predictedDistance = effectiveDistance + radialVelocity * flywheelResponseTime;
predictedDistance = Math.max(predictedDistance, minDistanceForShooting);

// Target RPM for the PREDICTED distance, not the current distance
double targetRPM = calculateShooterRPMOdoDistance(predictedDistance);
setFlywheelTarget(targetRPM);
```

When approaching at 40 in/s with `flywheelResponseTime = 0.25`, this targets RPM for
10 inches closer than current. The flywheel starts transitioning earlier and is closer
to the right speed when you arrive. When retreating, it starts spinning up earlier.

**This does not affect fire rate.** It just makes the flywheel command smarter. Hood
compensation (Section 6.6) still handles whatever residual RPM error exists at the
moment of each shot.

**Combined flow (Stage 1 with both features):**

```java
// --- Inside the shooting block ---

// 1. SOTM virtual target and effective distance (Section 6.3)
// ... (lead-point math as before) ...

// 2. Predictive RPM targeting
double radialVelocity = (vx * dx + vy * dy) / ODODistance;
double predictedDist = effectiveDistance + radialVelocity * flywheelResponseTime;
predictedDist = Math.max(predictedDist, minDistanceForShooting);
double targetRPM = calculateShooterRPMOdoDistance(predictedDist);
setFlywheelTarget(targetRPM);

// 3. Hood compensation for actual RPM
double actualRPM = getFlywheelVelocity();
double rpmRatio = Math.max(minRpmRatio, Math.min(maxRpmRatio, actualRPM / targetRPM));
double apparentDistance = effectiveDistance / rpmRatio;
shooterHoodPos = calculateShooterHoodOdoDistance(
    Math.max(apparentDistance, minDistanceForShooting));

// 4. Fire gating — hood validity only, no RPM wait
boolean hoodValid = apparentDistance >= minDistanceForShooting
                 && apparentDistance <= maxDistanceForShooting;
boolean canShoot = hoodValid;
```

### 6.8 Where to Put the Code

**Option A: Inline in Teleop (recommended for Stage 1)**
- Add the SOTM logic directly in the `gamepad1.left_bumper` block in `Pickles2025Teleop.onUpdate()`
- Quickest to test and iterate

**Option B: Helper method (recommended from Stage 2 onward)**
- Create a `calculateSOTMAimPoint(Pose botPose, Pose target, Vector velocity, double timeOfFlight)` method
- Returns a virtual `Pose` target
- Cleaner, reusable for both teleop and autonomous

Start with Option A. Refactor to Option B when moving to Stage 2.

---

## 7. Stage 2 — Turret-Based SOTM

**Prerequisites:** Stage 1 validated (lead-point math works). Turret installed and
turret subsystem software functioning (Track A from `turret_analysis.md` — servo
position mode, basic goal tracking working).

**Purpose:** Move aiming responsibility from robot heading to turret. The driver can
now drive freely in any direction while the turret tracks the virtual target. Add
angular velocity compensation since the turret must handle robot rotation.

### 7.1 What Changes from Stage 1

| Aspect | Stage 1 | Stage 2 |
|--------|---------|---------|
| **What aims** | Robot heading (P-controller on angle error) | Turret servo (position command) |
| **Driver freedom** | Robot must rotate toward virtual target | Robot drives freely, turret handles aim |
| **Angular compensation** | Not included | Added (turret must cancel robot rotation) |
| **Code location** | Inline in left_bumper block | `calculateSOTMAimPoint()` helper method |
| **Autonomous use** | Not practical (robot heading tied up) | Turret can SOTM during auto paths |

### 7.2 Refactor: Extract Helper Method

Move the virtual target calculation into a reusable method:

```java
/**
 * Computes the virtual aim point and effective distance for SOTM.
 * Works for both robot-heading aiming (Stage 1) and turret aiming (Stage 2+).
 */
public static class SOTMResult {
    public final double virtualTargetX;
    public final double virtualTargetY;
    public final double effectiveDistance;
    public final boolean isActive;  // true if SOTM compensation was applied

    public SOTMResult(double vtX, double vtY, double effDist, boolean active) {
        this.virtualTargetX = vtX;
        this.virtualTargetY = vtY;
        this.effectiveDistance = effDist;
        this.isActive = active;
    }
}

public static SOTMResult calculateSOTM(
        double botX, double botY,
        double targetX, double targetY,
        double vx, double vy, double speed,
        double timeOfFlight, double leadGain,
        double minSpeed, double maxLead) {

    double dx = targetX - botX;
    double dy = targetY - botY;
    double realDistance = Math.sqrt(dx * dx + dy * dy);

    if (speed <= minSpeed) {
        return new SOTMResult(targetX, targetY, realDistance, false);
    }

    double leadX = vx * timeOfFlight * leadGain;
    double leadY = vy * timeOfFlight * leadGain;

    double leadMag = Math.sqrt(leadX * leadX + leadY * leadY);
    if (leadMag > maxLead) {
        double scale = maxLead / leadMag;
        leadX *= scale;
        leadY *= scale;
    }

    double vtX = targetX - leadX;
    double vtY = targetY - leadY;

    double radialVelocity = (vx * dx + vy * dy) / realDistance;
    double effDist = realDistance - (radialVelocity * timeOfFlight);

    return new SOTMResult(vtX, vtY, Math.max(effDist, 0), true);
}
```

### 7.3 Turret Aiming Integration

Instead of rotating the robot to face the virtual target, compute the desired turret
angle and command the turret subsystem. Integrates hood compensation (Section 6.6) and
predictive RPM targeting (Section 6.7) for rapid fire.

```java
// In onUpdate(), when shooting mode is active:

// dx, dy, ODODistance = vector and real distance to the actual goal (already computed)
SOTMResult sotm = calculateSOTM(
    botX, botY, shootTargetX, shootTargetY,
    vx, vy, speed,
    ballTimeOfFlight, sotmLeadGain, sotmMinSpeed, sotmMaxLead);

// --- Turret aiming (includes angular compensation) ---
double dxVirtual = sotm.virtualTargetX - botX;
double dyVirtual = sotm.virtualTargetY - botY;
double virtualFieldAngleRad = Math.atan2(dyVirtual, dxVirtual);
double desiredTurretAngleDeg = Math.toDegrees(
    normalizeRadians(virtualFieldAngleRad - botHeadingRad));

// Angular velocity compensation (Section 7.4): bias aim to account for
// robot rotation during ball flight. Without this, every robot turn causes misses.
double omega = PedroComponent.follower().poseTracker.getAngularVelocity();
desiredTurretAngleDeg += Math.toDegrees(omega * ballTimeOfFlight);

TurretSubsystem.INSTANCE.setTargetAngle(desiredTurretAngleDeg);
TurretSubsystem.INSTANCE.setState(TurretState.TRACKING);

// --- Predictive RPM targeting (Section 6.7) ---
double radialVelocity = (vx * dx + vy * dy) / ODODistance;
double predictedDist = sotm.effectiveDistance + radialVelocity * flywheelResponseTime;
predictedDist = Math.max(predictedDist, minDistanceForShooting);
double targetRPM = calculateShooterRPMOdoDistance(predictedDist);
setFlywheelTarget(targetRPM);

// --- Hood compensation for rapid fire (Section 6.6) ---
double actualRPM = getFlywheelVelocity();
double rpmRatio = Math.max(minRpmRatio, Math.min(maxRpmRatio, actualRPM / targetRPM));
double apparentDistance = sotm.effectiveDistance / rpmRatio;
shooterHoodPos = calculateShooterHoodOdoDistance(
    Math.max(apparentDistance, minDistanceForShooting));

// --- Fire gating: turret ready + hood valid — NO flywheel RPM wait ---
boolean turretReady = TurretSubsystem.INSTANCE.isReadyToShoot();
boolean hoodValid = apparentDistance >= minDistanceForShooting
                 && apparentDistance <= maxDistanceForShooting;
boolean canShoot = turretReady && hoodValid;
```

### 7.4 Angular Velocity Compensation

**Already integrated into the Section 7.3 code block above** (the two lines after
"Angular velocity compensation"). Shown here separately for explanation.

With the turret handling aim, the robot can rotate freely. But if the robot turns while
a ball is in flight, the launch direction was set based on the heading at fire time.
Compensate by biasing the turret slightly ahead of the turn:

```java
double omega = PedroComponent.follower().poseTracker.getAngularVelocity();
desiredTurretAngleDeg += Math.toDegrees(omega * ballTimeOfFlight);
```

This should be included from the start of Stage 2 (not deferred) because without
it, every time the robot turns while shooting the balls will miss. With robot-heading
aiming (Stage 1) this didn't matter because the heading was actively controlled.

**Cross-reference:** The turret analysis document's Section 3.0G Phase 2 mentions "LOS
rate pre-compensation" — that is the same concept as this angular compensation, applied
via the turret's own `periodic()` loop. Use *either* this teleop-side calculation *or*
the turret-side LOS feedforward, not both (they'd double-count).

### 7.5 Stage 2 Tuning

**Step 1: Verify basic turret tracking (no SOTM)**
- Disable SOTM, verify turret tracks the real goal correctly from multiple field positions
- This should already be done as part of turret Track A tuning (see `turret_analysis.md`)

**Step 2: Enable SOTM, repeat lateral strafing test**
- Same test as Stage 1 Step 2, but now the robot drives straight (no heading rotation)
  while the turret aims
- `sotmLeadGain` and `ballTimeOfFlight` from Stage 1 should carry over as starting values

**Step 3: Test while turning**
- Drive in a curve or spin while shooting
- Angular compensation should keep shots on target
- If shots consistently miss in the direction of rotation, increase the angular lead
  (try `omega * ballTimeOfFlight * 1.2` as a tuning multiplier)

**Step 4: Test in autonomous**
- Use `calculateSOTM()` during auto paths where the robot is moving past a shooting position
- Verify that the turret pre-aims ahead of the path and fires accurately

### 7.6 Stage 2 Additional Telemetry

Add to the Stage 1 telemetry:

```java
telemetry.addData("SOTM_omega", omega);
telemetry.addData("SOTM_angularLead", Math.toDegrees(angularLeadRad));
telemetry.addData("SOTM_turretTarget", desiredTurretAngleDeg);
telemetry.addData("SOTM_turretActual", TurretSubsystem.INSTANCE.getTurretAngleDeg());
telemetry.addData("SOTM_turretReady", turretReady);
```

---

## 8. Stage 3 — Distance-Dependent Time of Flight

**Prerequisites:** Stage 2 working (turret SOTM functional). Have collected ball flight
time measurements at multiple distances using slow-motion video or timing.

**Purpose:** Replace the fixed `ballTimeOfFlight` constant with a distance-dependent
model. Separate mechanical delay from actual flight time. This significantly improves
accuracy at both short and long range.

### 8.1 Why the Fixed Constant Is Insufficient

A ball shot from 60 inches away arrives much faster than one shot from 120 inches.
Using a single `ballTimeOfFlight = 0.35s` means:
- At short range: the ToF is too long, lead is too large, balls miss behind the goal
- At long range: the ToF is too short, lead is too small, balls miss ahead of the goal

The `sotmLeadGain` knob can compensate for one distance, but not all distances at once.

### 8.2 Collecting Flight Time Data

**Method: slow-motion video**
1. Place the robot at known distances: 60", 80", 100", 120" from the goal
2. Shoot stationary at each distance (standard RPM/hood for that distance)
3. Record with phone in slow-motion (240fps is ideal, 120fps works)
4. Time from ball-leaves-launcher to ball-enters-goal for each shot
5. Take 3-5 shots per distance, average the times
6. Also time from fire-button-press to ball-leaves-launcher — this is the mechanical delay

**Expected results (rough estimates):**

| Distance (in) | Mechanical Delay (s) | Flight Time (s) | Total (s) |
|---------------|---------------------|-----------------|-----------|
| 60 | ~0.15 | ~0.15 | ~0.30 |
| 80 | ~0.15 | ~0.20 | ~0.35 |
| 100 | ~0.15 | ~0.27 | ~0.42 |
| 120 | ~0.15 | ~0.35 | ~0.50 |

The mechanical delay is approximately constant. The flight time scales with distance.

### 8.3 New Parameters

Replace `ballTimeOfFlight` with:

| Parameter | Description | Starting Value |
|-----------|-------------|----------------|
| `mechanicalDelay` | Time from fire command to ball leaving launcher (seconds) | Measured (~0.15) |
| `estimatedBallSpeed` | Average horizontal ball speed (inches/sec) | Calculated from measurements |

```java
// Replace the single constant:
// OLD: double totalTime = ballTimeOfFlight;

// NEW: separate mechanical delay from distance-dependent flight time
double flightTime = ODODistance / estimatedBallSpeed;
double totalTime = mechanicalDelay + flightTime;
```

### 8.4 Option: Lookup Table Instead of Linear Model

If the ball speed varies significantly with distance (it will, because different RPMs
and hood angles are used), a lookup table is more accurate:

```java
// Measured total time (mechanicalDelay + flightTime) at each distance
// Interpolate between entries for distances in between
private static final double[] TOF_DISTANCES = {60, 80, 100, 120};
private static final double[] TOF_TIMES     = {0.30, 0.35, 0.42, 0.50};

private double getTimeOfFlight(double distanceInches) {
    if (distanceInches <= TOF_DISTANCES[0]) return TOF_TIMES[0];
    if (distanceInches >= TOF_DISTANCES[TOF_DISTANCES.length - 1])
        return TOF_TIMES[TOF_TIMES.length - 1];

    for (int i = 0; i < TOF_DISTANCES.length - 1; i++) {
        if (distanceInches < TOF_DISTANCES[i + 1]) {
            double t = (distanceInches - TOF_DISTANCES[i])
                     / (TOF_DISTANCES[i + 1] - TOF_DISTANCES[i]);
            return TOF_TIMES[i] + t * (TOF_TIMES[i + 1] - TOF_TIMES[i]);
        }
    }
    return TOF_TIMES[TOF_TIMES.length - 1];
}
```

### 8.5 Updated Calculation

The only change from Stage 2 is how `totalTime` is computed. Everything else stays
the same.

**Important:** The ToF lookup uses `ODODistance` (the *real* geometric distance to the
target), NOT `effectiveDistance`. The effective distance already accounts for radial
velocity in the RPM/hood calculations — using it for ToF would double-count the
approach/retreat correction.

```java
// In calculateSOTM() or inline:
double totalTime = getTimeOfFlight(ODODistance);  // uses REAL distance, not effective

// Rest of the lead-point math is identical
double leadX = vx * totalTime * sotmLeadGain;
double leadY = vy * totalTime * sotmLeadGain;
// ...
```

### 8.6 Stage 3 Tuning

**Step 1: Validate ToF measurements**
- Add telemetry showing `totalTime` at various distances while stationary
- Verify the interpolated values look reasonable

**Step 2: Re-run lateral strafing test at multiple distances**
- Test at 60", 80", 100", 120" — shots should now be consistently accurate across
  all ranges, not just at the one distance where the old constant happened to be right

**Step 3: Adjust `sotmLeadGain`**
- With distance-dependent ToF, `sotmLeadGain` should be closer to 1.0 than before
- If it's still far from 1.0, the ToF measurements may need refinement

**Step 4: Retire `sotmLeadGain` if possible**
- If `sotmLeadGain` converges near 1.0, consider removing it — it was a Band-Aid
  for the fixed-ToF inaccuracy

---

## 9. Stage 4 — Advanced Refinements

**Prerequisites:** Stage 3 working well. Looking for incremental accuracy improvements.

These refinements are independent and can be added in any order based on what testing
reveals as the biggest remaining error source. Each is a small, isolated change.

### 9.1 Velocity Blending (from 23511's virtual-goal branch)

**Problem:** Sensor velocity is slightly behind actual velocity during acceleration
and deceleration, causing the lead to lag.

**Solution:** Blend measured velocity with commanded/intended velocity to predict
where velocity will be during the shot:

```java
// Alpha = 0.0 means pure sensor, 1.0 means pure commanded
// 23511 uses 0.2 (80% sensor, 20% commanded)
public static double VELOCITY_BLEND_ALPHA = 0.2;

// Assuming you can get the robot's target velocity from Pedro or joystick commands:
double blendedVx = vx + (targetVx - vx) * VELOCITY_BLEND_ALPHA;
double blendedVy = vy + (targetVy - vy) * VELOCITY_BLEND_ALPHA;

// Use blendedVx/blendedVy instead of raw vx/vy in the SOTM calculation
```

**When to add:** If shots consistently lag during acceleration but are accurate at
constant speed.

### 9.2 Turret Physical Offset (from 23511's virtual-goal branch)

**Problem:** The turret is not at the robot's geometric center. When the robot rotates,
the turret traces an arc, adding tangential velocity that isn't captured by the robot's
translational velocity alone.

**Solution:** Account for the turret's offset position:

```java
// Measure on your robot: turret position relative to robot center (inches)
public static double TURRET_OFFSET_X = 0.0;  // forward/backward from center
public static double TURRET_OFFSET_Y = 0.0;  // left/right from center

// In the SOTM calculation, compute turret-base velocity:
double cosH = Math.cos(botHeadingRad);
double sinH = Math.sin(botHeadingRad);

// Turret position in field coordinates
double turretFieldX = botX + TURRET_OFFSET_X * cosH - TURRET_OFFSET_Y * sinH;
double turretFieldY = botY + TURRET_OFFSET_X * sinH + TURRET_OFFSET_Y * cosH;

// Tangential velocity from robot rotation
double tanVx = -omega * (turretFieldY - botY);
double tanVy =  omega * (turretFieldX - botX);

// Total turret velocity = robot velocity + tangential
double turretVx = vx + tanVx;
double turretVy = vy + tanVy;

// Use turretVx/turretVy and turretFieldX/turretFieldY in the lead calculation
```

**When to add:** If your turret is significantly off-center (>1 inch) and shots miss
during fast spins. If the turret is near-center, this correction is negligible.

### 9.3 Iterative Convergence (from 23511's VirtualGoalSolver)

**Problem:** The single-step lead calculation assumes ToF doesn't change when the
virtual target shifts. At high speeds, the virtual target can be 10-15 inches from
the real goal, changing the distance enough to change the ToF.

**Solution:** Iterate: compute lead, get new distance, recompute ToF, recompute lead.
Converges in 2-3 iterations.

```java
double virtualX = targetX;
double virtualY = targetY;
double currentDist = realDistance;

for (int i = 0; i < 3; i++) {
    double totalTime = getTimeOfFlight(currentDist);
    virtualX = targetX - turretVx * totalTime * sotmLeadGain;
    virtualY = targetY - turretVy * totalTime * sotmLeadGain;
    double dvx = virtualX - turretFieldX;
    double dvy = virtualY - turretFieldY;
    currentDist = Math.sqrt(dvx * dvx + dvy * dvy);
}

// Use virtualX, virtualY, currentDist as the final values
```

**When to add:** If shots are accurate at low speed but miss at high speed (>30 in/sec),
and adjusting `sotmLeadGain` fixes one speed but breaks another. The iteration removes
the need for the gain fudge factor.

### 9.4 Hood Compensation Refinement: Empirical Correction Factor

**Note:** The core hood compensation strategy (apparent-distance trick) is implemented
in Section 6.6 as a Stage 1 feature. This refinement is only needed if the first-order
`1/rpmRatio` approximation isn't accurate enough at large RPM deviations.

**Problem:** The `apparentDistance = effectiveDistance / rpmRatio` approximation assumes
a linear relationship between ball speed and effective range. In reality, the relationship
involves gravity, launch angle, and air resistance. For small RPM deviations (±5-10%), the
linear approximation is excellent. For larger deviations (±15-20%), it may over- or
under-correct.

**Solution:** Collect empirical data and apply a correction factor:

1. Set up at a known distance (e.g., 90 inches)
2. Shoot with flywheel deliberately set to 90%, 95%, 100%, 105%, 110% of normal RPM
3. For each, find the hood angle that actually scores
4. Compare the measured correction to what `1/rpmRatio` predicts
5. If they differ, fit a correction curve:

```java
// If empirical data shows the linear model overshoots by ~20% at large deviations:
double correctionExponent = 0.8;  // tune from data (1.0 = pure linear, <1 = less aggressive)
double correctedRatio = Math.pow(rpmRatio, correctionExponent);
double apparentDistance = effectiveDistance / correctedRatio;
```

**When to add:** Only if rapid-fire shots consistently miss at the same distance when
the RPM dip is large (>10%), AND adjusting `minRpmRatio` to reject those shots is
unacceptable because it slows fire rate.

---

## 10. Comparison: Our Staged Plan vs. 23511's Virtual Goal

For reference, here's how our staged approach maps to 23511's full implementation:

| Feature | Our Stage | 23511 Status | Notes |
|---------|-----------|-------------|-------|
| Lead-point virtual target | Stage 1 | Yes | Same core concept |
| Fixed time of flight | Stage 1 | No (calculated) | Our placeholder for initial testing |
| Robot-heading aiming | Stage 1 | No (turret) | Temporary — removed in Stage 2 |
| Hood compensation (rapid fire) | Stage 1 | Yes (in Launcher) | Ours: apparent-distance trick. Theirs: quadratic ballistic solver. Same goal. |
| Predictive RPM targeting | Stage 1 | Partial (velocity blending) | We predict distance; they blend velocity. Complementary approaches. |
| Turret aiming | Stage 2 | Yes | Both use positional servo commands |
| Angular velocity compensation | Stage 2 | Yes (LOS feedforward) | Ours is simpler, theirs uses cross-product |
| Distance-dependent ToF | Stage 3 | Yes (ballistic solver) | Ours uses measured LUT, theirs calculates from physics |
| Ball transfer time | Stage 3 | Yes (`BALL_TRANSFER_TIME = 0.15s`) | Separated from flight time |
| Velocity blending | Stage 4 | Yes (`DRIVE_VEL_PREDICT_ALPHA = 0.2`) | 80% measured + 20% commanded |
| Turret physical offset | Stage 4 | Yes (`TURRET_PHYSICAL_OFFSET`) | Only matters if turret is off-center |
| Iterative convergence | Stage 4 | Yes (5 iterations) | Removes need for `sotmLeadGain` fudge factor |
| Hood comp refinement (empirical) | Stage 4 | N/A (they use full solver) | Only if `1/rpmRatio` approximation is insufficient |
| `sotmLeadGain` tuning knob | Stages 1-3 | No equivalent | Our empirical correction — may retire in Stage 4 |
| `sotmMaxLead` safety cap | All stages | Implicit (NaN fallback) | Ours is more explicit |

**Key difference in philosophy:** Our plan prioritizes getting something working and
testable at each stage, using tuning knobs (`sotmLeadGain`, fixed ToF) to compensate
for model inaccuracy. 23511's plan tries to get the physics exactly right from the start
with a full ballistic solver and iterative convergence. Both approaches converge to
similar accuracy — ours just gets there through measurement and tuning rather than
first-principles calculation.

---

## 11. Risks and Mitigations

| Risk | Stage | Mitigation |
|------|-------|-----------|
| Velocity readings are noisy | 1+ | Use `sotmMinSpeed` threshold; consider smoothing velocity with a moving average |
| ~~Velocity is in wrong coordinate frame~~ | — | **RESOLVED:** Confirmed field-relative from source code analysis. Quick telemetry check recommended but low risk. |
| Fixed ToF is wrong at some distances | 1-2 | `sotmLeadGain` compensates; Stage 3 replaces with measured data |
| Compensation overshoots | 1+ | `sotmMaxLead` safety cap; start with `sotmLeadGain < 1.0` |
| Robot P-controller can't track while moving | 1 | May need to increase `shooterTargetkP` for SOTM; becomes irrelevant in Stage 2 (turret) |
| RPM sag causes inconsistent shots | 1+ | Hood compensation (Section 6.6) adjusts hood per-shot for actual RPM; hardware improvements (fewer bearings, faster feed) reduce sag magnitude |
| RPM target chasing during approach/retreat | 1+ | Predictive RPM targeting (Section 6.7) anticipates distance change; hood compensation handles residual error |
| Hood compensation over-corrects at large RPM dips | 1+ | `minRpmRatio`/`maxRpmRatio` clamps prevent extreme corrections; hardware improvements shrink dips into the accurate range |
| Turret servo response too slow for fast tracking | 2+ | Add LOS lookahead from `turret_analysis.md` Section 3.0G |
| Balls miss during fast turns | 2+ | Angular compensation handles this; tune the angular lead multiplier |
| ToF measurements are tedious | 3 | 4 distances x 3 shots each = ~12 shots. Use slow-mo video. Under 15 minutes. |
| Iterative convergence adds latency | 4 | 3 iterations of simple math is <0.1ms. Not a concern. |

## 12. Summary of Changes Per Stage

| Stage | Files Changed | Effort | Driver Impact |
|-------|---------------|--------|---------------|
| 1 | `Pickles2025Teleop.java` only | 1 session | Rapid-fire while strafing (robot still rotates to aim); hood compensates for RPM sag |
| 2 | Teleop + `TurretSubsystem` integration | 1 session (after turret Track A is done) | Full SOTM — drive freely, turret aims, rapid-fire preserved |
| 3 | Teleop (ToF model) + measurement session | 1 session coding + 1 session measuring | Accurate at all distances |
| 4 | Teleop (small additions) | Incremental, as needed | Polish — diminishing returns |

## 13. Cross-Reference

- Turret subsystem architecture, tuning, and Track A/B details: `scripts/turret_analysis.md`
- Non-turret improvements (flywheel voltage comp, BulkReads, etc.): `scripts/other_benchmarking_improvements.md`
