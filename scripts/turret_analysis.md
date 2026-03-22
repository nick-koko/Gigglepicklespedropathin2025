# Turret Implementation Analysis

**Purpose:** Compare our plan against three reference teams (23511, 24064, 23521), then
provide concrete code recommendations for our team's hardware and framework.

**Review this with your team before coding.**

**Updated Feb 2026:** This document now covers a **dual-track strategy**:
- **Track A (Servo Position Mode):** Get turret working on the competition robot ASAP
  for driver practice. Uses positional servos — the servo firmware handles PID internally.
  This is what 23511 switched to in their `v2-robot` and `virtual-goal` branches after
  struggling with CR mode tuning.
- **Track B (Continuous Rotation Mode):** Continue developing and tuning the full custom
  PID controller on the test stand. If we get it working well, we can swap it onto the
  competition robot later.

Shared between both tracks: calibration, geometry, limit logic, readiness gating,
LOS feedforward math, chassis handoff, and teleop integration. Only the final
"command to servo" layer differs.

---

## 1. Our Setup vs. Reference Teams

| Aspect | Our Team (Track A) | Our Team (Track B) | 23511 master (CR) | 23511 v2/virtual-goal (Servo) | 23521 (Servo) | 24064 (Motor) |
|--------|-------------------|-------------------|-------------------|-------------------------------|---------------|---------------|
| **Framework** | NextFTC | NextFTC | SolversLib | SolversLib | **NextFTC (Kotlin)** | Custom + Pedro |
| **Turret actuators** | 2x Axon Mini **positional** | 2x Axon Mini **CR** | 2x Axon Mini CR | 2x Axon Mini **positional** (`ServoExGroup`) | 2x positional (`ServoEx`) | 1x GoBilda 435 motor |
| **Servo command type** | Position [0, 1] | Power [-1, +1] | Power [-1, +1] | Position [0, 1] | Position [0, 1] | Power [-1, +1] |
| **Software PID needed?** | **No** (servo firmware) | **Yes** (full custom) | Yes (full custom) | **No** (servo firmware) | **No** (servo firmware) | Yes (full custom) |
| **Encoder role** | Verify position only | Primary control loop | Primary control loop | Verify position only | Telemetry only (no gating) | Primary control loop |
| **Encoder type** | Quad + Absolute (Melonbotics) | Quad + Absolute (Melonbotics) | Quad + Absolute (REV + analog) | Quad + Absolute (REV + analog) | Analog only | Quad + Absolute (Melonbotics) |
| **Encoder-to-turret ratio** | 4.8:1 | 4.8:1 | 3.75:1 (quad), 1:1 (abs) | 3.75:1 (quad), 1:1 (abs) | Unknown (analog) | 1:1 |
| **Readiness gating** | Dwell-time counters | Dwell-time counters | Single-frame `atSetPoint()` | Encoder vs. commanded | **None** | 3-loop counters |
| **Auto-aim prediction** | Planned (Phase 2+) | Planned (Phase 2+) | Commanded velocity FF | Virtual goal solver | None (pure geometry) | LOS rate + predicted pose |
| **Tuning complexity** | Low (servo offset + limits) | High (kP/kD/kV/kS + motion planner) | High | Low | **Very low** | High |

### Key Differences to Note

1. **Both our team and 24064 use the same Melonbotics encoder** — a single physical package
   containing both an absolute analog encoder and a quadrature encoder, with two separate wire
   outputs. In 24064's code, `AnalogInput` and `Motor.Encoder` are both from this one device
   even though the code makes them look like separate sensors. If the device fails, we lose both
   signals — but this is a single point of failure shared by 24064's design too.

2. **Our 4.8:1 encoder-to-turret ratio gives excellent quadrature resolution.**
   With 4096 CPR × 4.8 = ~19,661 counts per turret revolution = ~54.6 counts/degree.
   This is significantly better than 24064's 1:1 setup (~11.4 counts/degree) and even
   better than 23511's 3.75:1 setup (~42.7 counts/degree from their REV through-bore's
   ~8192 CPR × 120/32). This makes our velocity estimation cleaner, though filtering
   is still recommended.

3. **The 4.8:1 ratio creates an absolute encoder ambiguity problem** — see Section 3.3
   for the two-stage calibration solution using the Axon Mini's built-in analog encoder
   plus the Melonbotics absolute.

4. **Both we and 23511 use the same Axon Mini servos.** On their `master` branch they
   used CR mode (power control). On `v2-robot` and `virtual-goal` they switched to
   positional mode. They reported that CR mode was difficult to tune and positional mode
   was more reliable in competition. Our Track A follows their positional approach; our
   Track B pursues the CR approach with improvements (motion planner, better feedforward).

5. **NextFTC's `Subsystem` interface** uses `initialize()` and `periodic()` — structurally
   similar to SolversLib's `SubsystemBase` with `periodic()`.

6. **FTC-23521 (Desoto Technix) uses NextFTC with Kotlin** — the only other reviewed team
   using our framework. Their `ho-ho-ho` branch has a working servo-mode turret as a NextFTC
   `Subsystem` singleton. While we use Java, their subsystem structure, `ServoEx` usage, and
   `SubsystemComponent` registration pattern are directly applicable. Key structural detail:
   they use `ServoEx` from NextFTC's hardware library which wraps the SDK `Servo` with position
   caching (only sends I2C commands when the position actually changes) — we should use the
   Java equivalent `dev.nextftc.hardware.impl.ServoEx` in our subsystem rather than raw SDK
   `Servo` for reduced bus traffic.

7. **23521 applies a per-servo position offset** (`RIGHT_OFFSET = 0.028`) to compensate for
   mechanical misalignment between their left and right turret servos. Rather than just
   reversing direction, each servo gets a slightly different position command. This is worth
   having as a tunable constant in our implementation — even small servo-to-servo differences
   can cause binding or uneven loading in a dual-servo turret.

8. **Why 23511 switched from CR to positional mode:** Their `master` branch CR implementation
   had several compounding issues: a hard `power = 0` cutoff at setpoint causing micro-hunting,
   identical large/small PIDF coefficients (suggesting they couldn't find good separate gains),
   no motion profiling on setpoint changes, deprecated/unused velocity estimation, and
   velocity feedforward based on commanded (not measured) chassis velocity. None of these
   are individually fatal, but together they create a system that's extremely sensitive to
   tuning. By switching to positional servos, the servo firmware handles the low-level PID
   at a higher frequency and with knowledge of its own motor characteristics.

---

## 2. Deep Comparison of Reference Turret Implementations

### 2.1 FTC-23511 Decode-2026 — Turret.java

**Architecture:**
- Three states: `GOAL_LOCK_CONTROL`, `ANGLE_CONTROL`, `OFF`
- Single `PIDFController` from SolversLib library
- Gain switching: `TURRET_LARGE_PIDF_COEFFICIENTS` vs `TURRET_SMALL_PIDF_COEFFICIENTS`
  triggered by `TURRET_THRESHOLD = 0.15 rad (~8.6°)`
- Output max: `TURRET_LARGE_MAX_OUTPUT = 1.0` (far) vs `TURRET_SMALL_MAX_OUTPUT = 0.067` (close)

**Hardware setup (from Robot.java):**
- Two Axon Mini CR servos (same model as ours) in a `CRServoGroup` (inverted so they push together), 1:1 with turret
- Absolute encoder: `AbsoluteAnalogEncoder` at 1:1 with turret (on the servo/turret shaft)
  — used for sync at init. Simple because no sector ambiguity.
- Quadrature encoder: REV through-bore on the BL motor encoder port, at 3.75:1 (32T:120T).
  `TURRET_RADIANS_PER_TICK` is empirically calibrated from their measurements.
  This gives them ~42.7 counts/degree for runtime tracking and velocity estimation.

**What they do well:**
- **Simple calibration** — their absolute encoder is 1:1 with turret, so sync is
  straightforward: read absolute, offset quadrature, done. No sector ambiguity.
- **Chassis/turret responsibility split** (`angleToDriveTurretErrors`): cleanly separates
  what the turret can handle vs. what the drivetrain must compensate for
- **Voltage compensation** on both kS and velocity FF terms
- **Distance-dependent tolerance** scaling (`TURRET_TOLERANCE_SCALING`)
- **Goal adjustment LUT** for angle-to-wall compensation (backspin, airflow effects)
- **Sync offset** calibration from absolute to quadrature at init

**What to watch for:**
- `readyToLaunch()` is a single-frame `atSetPoint()` check — no dwell-time gating.
  This can allow firing during transient crossings.
- `TURRET_SMALL_MAX_OUTPUT = 0.067` is very low. With different servos/friction this
  may need significant retuning.
- Velocity is derived from `swerve.getTargetVelocity()` (the *commanded* drivetrain
  velocity, not *measured*). This can be ahead of actual motion.
- The deprecated `updateVelocity()` suggests they moved away from encoder-based
  turret velocity measurement — worth noting for our D-term approach.

**Control equation (reconstructed from `update()`):**
```
power  = PIDF.calculate(position)                           // positional PID
       + TURRET_OPEN_F * (DEFAULT_VOLTAGE / voltage) * sign(power)  // kS with voltage comp
       + targetVel * TURRET_VEL_FF * (DEFAULT_VOLTAGE / voltage)    // velocity FF with voltage comp

if atSetPoint: power = 0   // hard cutoff at tolerance
if position > MAX or < MIN and pushing further: power = 0   // limit protection
```

### 2.2 FTC-24064 (24064-Decode) — Turret.java

**Architecture:**
- Two states: `IDLE`, `ODOM_TRACKING`
- Custom `PIDController` with derivative filtering (`FIRLowPassFilter` on error derivative)
- Close/far gain switching at `PID_SWITCH_ANGLE = 15°`
- IIR low-pass filter on target angle to smooth setpoint changes

**Hardware setup:**
- Single GoBilda 435 RPM motor (NOT CR servos) via `CachedMotor`
- **Same Melonbotics encoder we're using** — single package with both absolute (analog)
  and quadrature outputs, connected as `AnalogInput` + `Motor.Encoder` respectively.
  The code makes these look like separate sensors, but it's one physical device.
- 1:1 encoder-to-turret ratio: `TICKS_TO_DEGREES = 0.2327` (~4.3 ticks/degree or ~1547 CPR),
  which gives them lower resolution than our 4.8:1 setup.
- Their absolute encoder at 1:1 has no sector ambiguity — simple offset calibration.

**What they do well:**
- **LOS (Line-of-Sight) rate feedforward** — the strongest feature:
  ```java
  // Vector from turret to goal
  double dx = goal.getX() - turretPos.getX();
  double dy = goal.getY() - turretPos.getY();
  double denom = dx * dx + dy * dy;
  
  // Robot translational velocity
  double vx = robot.drivetrain.getVelocity().getXComponent();
  double vy = robot.drivetrain.getVelocity().getYComponent();
  double omega = robot.drivetrain.getAngularVelocity();
  
  // LOS angular rate (how fast the goal bearing changes)
  double phiDot = (dy * vx - dx * vy) / denom;
  
  // Turret must compensate for both LOS rate AND robot rotation
  double alphaDot = omega - phiDot;
  ```
- **Predicted pose** for moving shots: `getPredictedPose(LAUNCH_DELAY)` uses velocity
  + acceleration to estimate where the robot will be when the ball exits
- **Tolerance counters**: `READY_TO_SHOOT_LOOPS = 3` and `OUT_OF_TOLERANCE_LOOPS = 3`
  for stability-gated shooting — prevents firing on transient crossings
- **Moving tolerance scale** (`MOVING_TOLERANCE_SCALE = 1.8x`) relaxes aim requirements
  while the robot is in motion
- **kA feedforward** (acceleration FF on LOS rate derivative) — aggressive but forward-looking
- **Voltage compensation** throughout: `MAX_VOLTAGE / batteryVoltage`

**What to watch for:**
- `IDLE` state parks turret to `targetAngle = 0` — wastes time reacquiring when
  switching to shoot. Our plan correctly avoids this.
- Using a GoBilda motor (not CR servos) for turret — their power/torque behavior
  is fundamentally different from CR servos. Don't copy motor-specific tuning values.
- `kV_TURRET = 0.1` and `kA_TURRET = 0` — they haven't fully tuned the acceleration
  feedforward yet, suggesting it's nice-to-have but not critical.
- `CachedMotor` has a built-in slew rate (`SLEW_RATE = 0.2`) which adds latency.
  We should be deliberate about whether we want output slew limiting.
- Target angle filter (`IIR, alpha = 0.10`) smooths but adds lag. Trade-off:
  less jitter vs. slower response.

**Control equation (reconstructed from `run()`):**
```
// Static friction + voltage comp
output = kS * (V_max / V_battery) * sign(error)   // only when |error| > threshold

// LOS feedforward (ODOM_TRACKING only)
output += kV_TURRET * alphaDot * (V_max / V_battery)
output += kA_TURRET * d(alphaDot)/dt * (V_max / V_battery)

// PID
output += controller.calculate(currentAngle)   // kP*error + kI*integral + kD*d(error)
```

### 2.3 FTC-23511 Decode-2026 — v2-robot & virtual-goal Branches (Servo Position Mode)

**Why this exists:** After struggling with CR mode tuning on `master`, 23511 switched
to positional servo mode on their `v2-robot` and `virtual-goal` branches. This is the
approach our Track A is based on.

**Architecture change (from master):**
- Turret servos changed from `CRServoGroup` (continuous rotation, power output)
  to `ServoExGroup` (positional, position output [0, 1]).
- The entire software PIDF loop, velocity estimation, kS/kV feedforward, and
  gain scheduling were **removed** from the turret update.
- Servo receives a **position** command instead of a power command.
- Encoder is now used only for **readiness verification** (`readyToLaunch()`), not
  for the control loop itself.

**Key hardware change in Robot.java:**
```java
// master branch (CR mode):
turretServos = new CRServoGroup(
    new CRServoEx(hwMap, "leftTurretServo").setRunMode(CRServoEx.RunMode.RawPower),
    new CRServoEx(hwMap, "rightTurretServo").setRunMode(CRServoEx.RunMode.RawPower)
).setInverted(true);

// v2-robot / virtual-goal branch (positional mode):
turretServos = new ServoExGroup(
    new ServoEx(hwMap, "leftTurretServo").setCachingTolerance(0.001),
    new ServoEx(hwMap, "rightTurretServo").setCachingTolerance(0.001)
).setInverted(true);
turretServos.set(0.5);  // Initialize to center position
```

**Key conversion math (MathFunctions.java):**
```java
// TURRET_SERVO_ROTATION = 320 degrees (Axon Mini configured range)
// This maps the turret's radian target to a [0, 1] servo position:
//   -160° -> 0.0,  0° -> 0.5,  +160° -> 1.0
public static double convertRadianToServoPos(double radians) {
    return (radians + (Math.toRadians(TURRET_SERVO_ROTATION) / 2.0))
           / Math.toRadians(TURRET_SERVO_ROTATION);
}

public static double convertServoPoseToRadian(double pos) {
    return (pos * Math.toRadians(TURRET_SERVO_ROTATION))
           - (Math.toRadians(TURRET_SERVO_ROTATION) / 2.0);
}
```

**How setTurretPos() works:**
```java
public void setTurretPos(double angle) {
    double value = Range.clip(angle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    value = MathFunctions.convertRadianToServoPos(value);  // radian -> [0,1]
    value = Range.clip(value, ...) + TURRET_SERVO_OFFSET;  // fine-tune center
    robot.turretServos.set(value);  // positional command!
}
```

**How readyToLaunch() works (using encoder for verification):**
```java
public boolean readyToLaunch() {
    // Read back the commanded servo position, convert to radians
    double servoPos = MathFunctions.convertServoPoseToRadian(
        robot.turretServos.get() - TURRET_SERVO_OFFSET);
    if (Double.isNaN(servoPos)) return false;

    // Compare encoder reading against where the servo was told to go
    return (Math.abs(getPosition() - servoPos) <= TURRET_POS_TOLERANCE)
           && !turretState.equals(OFF);
}
```

**Why the range works (not exceeding servo limits):**
- Turret operating range: `MAX_TURRET_ANGLE = ±2.6 rad ≈ ±149°` (wire-limited)
- Servo physical range: `TURRET_SERVO_ROTATION = 320°` = ±160°
- The turret's ±149° maps to servo positions ~0.07 to ~0.93
- There's ~11° of margin on each side before hitting the servo endpoints
- `TURRET_SERVO_OFFSET = 0.03` fine-tunes the mechanical center

**virtual-goal branch additions (most advanced):**
- `VirtualGoalSolver`: instead of aiming at the real goal, computes a "virtual goal"
  that accounts for robot velocity during ball flight time. Iteratively converges
  (5 iterations) on a shifted target using time-of-flight estimates.
- `getShotSolution()` in Robot.java blends measured and intended velocity using
  `DRIVE_VEL_PREDICT_ALPHA = 0.2` for smooth prediction.
- `TURRET_VEL_LAG = 0.067` seconds of velocity feedforward applied to the
  positional command (shifts the target angle slightly ahead of the current
  tracking point to compensate for servo response delay).
- `TURRET_PHYSICAL_OFFSET = (1.96, 0)` inches: accounts for turret not being
  at robot center when calculating aim angles.
- `BALL_TRANSFER_TIME = 0.15` seconds: mechanical delay from "fire" to ball exit.
- Launcher also uses the virtual goal solver for distance/velocity calculations.

**What this means for us (Track A):**
The servo-mode approach eliminates the entire PID tuning problem. The only turret-specific
values to tune are:
1. `TURRET_SERVO_ROTATION` — match your Axon Mini's configured rotation range
2. `TURRET_SERVO_OFFSET` — fine-tune so servo 0.5 = turret forward
3. `MAX_TURRET_ANGLE` / `MIN_TURRET_ANGLE` — match your wire management limits
4. `TURRET_POS_TOLERANCE` — how close the encoder must confirm before firing

Everything else (geometry, goal tracking, limit handling, readiness) is shared logic
that works identically in both servo and CR mode.

### 2.4 FTC-23521 Desoto Technix — ho-ho-ho Branch (Servo Position Mode, NextFTC)

**Why this is relevant:** 23521 is the only reviewed team using **NextFTC** — the same
framework we use. Their turret subsystem is a working example of the servo-mode approach
implemented as a NextFTC `Subsystem` singleton in Kotlin. While we use Java, the structural
patterns translate directly.

**Architecture:**
- Kotlin `object Turret : Subsystem` — singleton pattern, equivalent to our Java
  `public class TurretSubsystem implements Subsystem` with `INSTANCE` field.
- Two positional servos via NextFTC's `ServoEx` (not raw SDK `Servo`).
- One analog encoder (`AnalogInput`) for position readback — no quadrature.
- No software PID, no gain scheduling, no feedforward of any kind.
- States are implicit (no enum): always tracking or set to a fixed angle.

**Hardware initialization:**
```kotlin
// Kotlin (23521) — translates directly to Java
val left = ServoEx("turretLeft")      // NextFTC ServoEx: auto-caches position
val right = ServoEx("turretRight")
lateinit var encoder: AnalogInput     // initialized in initialize()

override fun initialize() {
    encoder = ActiveOpMode.hardwareMap.analogInput["turretEncoder"]
}
```

Java equivalent for our codebase:
```java
// Our Java equivalent using NextFTC ServoEx
private ServoEx turretServo1 = new ServoEx("turret_servo_1");
private ServoEx turretServo2 = new ServoEx("turret_servo_2");
private AnalogInput encoder;

@Override
public void initialize() {
    encoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret_encoder");
}
```

Key detail: `ServoEx` from `dev.nextftc.hardware.impl` wraps the SDK `Servo` with
**position caching** — it only sends I2C commands to the servo when the requested position
actually changes. This reduces bus traffic during `periodic()` when the target position
is stable (common during steady tracking). We should use `ServoEx` instead of raw `Servo`.

**Angle-to-position conversion:**
```kotlin
val DEADZONE = 60.deg
const val RIGHT_OFFSET = 0.028

fun setTargetAngle(angle: Angle) {
    val normalizedAngle = angle.normalized.inDeg
        .coerceIn((-180.deg + DEADZONE).inDeg, (180.deg - DEADZONE).inDeg)
    val targetPosition = -normalizedAngle * (0.558 / 180.0) + 0.5
    left.position = targetPosition
    right.position = targetPosition + RIGHT_OFFSET
}
```

Breakdown:
- `DEADZONE = 60°` limits the turret to **±120°** operational range (180° - 60° each side).
- The constant `0.558` maps 180° of turret rotation to a 0.558 servo position span.
  This implies a total servo rotation of 180 / 0.558 ≈ **323°** (consistent with 23511's 320°).
- Center (0°) = servo position **0.5**, matching 23511's convention.
- `RIGHT_OFFSET = 0.028` applies a small correction to the right servo to compensate for
  mechanical misalignment. Without this, two servos commanded to the same position may
  fight each other slightly due to mounting tolerances.

Compared to 23511's conversion math:
```
23511:  pos = (radians + SERVO_ROTATION/2) / SERVO_ROTATION + OFFSET
23521:  pos = -angle * (0.558 / 180) + 0.5
```
Both are linear maps from angle to [0, 1]. 23521's form is more compact but less
self-documenting — the `0.558` constant buries the servo rotation range. Our implementation
(Section 3.0D) should use the explicit `SERVO_ROTATION_DEG` constant for clarity.

**Encoder reading (telemetry only, no gating):**
```kotlin
override fun periodic() {
    val scale = 360.0 / 3.3
    val positionDeg = encoder.voltage * scale - 180.0
    currentAngle = positionDeg.deg
    // ... telemetry output only, no readiness check
}
```

The encoder maps 0-3.3V linearly to -180° to +180°. They also compute a basic velocity
from voltage differences, but neither value feeds into any control or gating logic.
**This is the biggest gap in 23521's implementation** — they have no `readyToShoot()`
check and could fire while the turret is still slewing.

**Teleop auto-aim (from teleop.kt):**
```kotlin
val deltaX = targetPose.x - currentX
val deltaY = targetPose.y - currentY
val absoluteAngleToTarget = atan2(deltaY, deltaX).rad
val relativeAngleToTarget =
    (heading - absoluteAngleToTarget + 180.deg).normalized

Turret.setTargetAngle(-relativeAngleToTarget)
```

Pure geometry: compute bearing to target, subtract robot heading, command turret.
No LOS feedforward, no velocity prediction, no virtual goal. This is the simplest
auto-aim implementation among all reviewed teams.

They also have a **heading lock mode**: when the driver holds a button, a separate PID
locks the robot heading toward the alliance goal. This complements turret aim by keeping
the turret near center where it's most effective.

**Autonomous usage (from close12.kt):**
```kotlin
val turretAngle = when (BotState.alliance) {
    Alliance.RED -> AutoConstants.Angles["closeTurretRed"]   // -90 deg
    Alliance.BLUE -> AutoConstants.Angles["closeTurretBlue"] //  90 deg
    else -> 0.0.deg
}
InstantCommand { Turret.setTargetAngle(turretAngle) }
```

Fixed turret angle per shooting position — no dynamic tracking during auto.
The turret is set once and held while the robot drives to a known shooting pose.

**NextFTC integration pattern (subsystem registration):**
```kotlin
// In teleop.kt — how they register subsystems with the framework
addComponents(
    BulkReadComponent,          // batch reads, reduces I2C overhead
    BindingsComponent,          // gamepad bindings
    PedroComponent(Constants::createFollower),  // Pedro Pathing
    SubsystemComponent(Tube, Shooter, Flywheel, Hood, Turret),  // all subsystems
)
```

Java equivalent for our codebase:
```java
addComponents(
    BulkReadComponent.INSTANCE,
    BindingsComponent.INSTANCE,
    new PedroComponent(() -> Constants.createFollower()),
    new SubsystemComponent(TubeSubsystem.INSTANCE, ShooterSubsystem.INSTANCE,
                           FlywheelSubsystem.INSTANCE, HoodSubsystem.INSTANCE,
                           TurretSubsystem.INSTANCE)
);
```

This is directly applicable — the `SubsystemComponent` handles calling `initialize()` and
`periodic()` on all registered subsystems automatically.

**What they do well:**
- Simplest working implementation — proof that servo mode works with minimal code
- NextFTC integration is a direct template for our Java implementation
- `ServoEx` position caching reduces I2C bus traffic
- Per-servo offset (`RIGHT_OFFSET`) handles real-world mechanical tolerances
- Heading lock mode as a driver aid complements turret tracking
- `BulkReadComponent` for batched I2C reads (we should also use this)

**What to watch for:**
- **No readiness gating** — their turret fires without encoder confirmation. We must keep
  our planned dwell-time counters (Section 3.0E).
- **No quadrature encoder** — analog-only gives ~0.3° resolution (12-bit ADC over 360°).
  Our Melonbotics quadrature at 4.8:1 gives ~0.018° — 15x better for readiness verification.
- **No velocity compensation** — turret will lag behind during fast driving or turning.
  Our Phase 2 enhancements (Section 3.0G) address this.
- **`MAGIC_NUMBER = 1.04` is defined but never used** — appears to be a leftover constant.
- **Encoder velocity** is computed but unused — suggests they planned to use it but didn't.
- **Narrower operational range** (±120°) vs 23511's ±149° — their deadzone is more
  conservative. Our range should be determined by our own wire management limits.

---

## 3. Recommended Implementation — Dual Track

**Track A (Servo Position Mode)** is described in Sections 3.0A through 3.0G below.
This is what goes on the competition robot first.

**Track B (Continuous Rotation Mode)** is described in Sections 3.1 through 3.9
(the original plan, unchanged). This continues on the test stand.

Both tracks share: calibration (3.3), geometry (3.6), limit handling (4.3),
readiness gating, LOS feedforward (3.5), and teleop integration (Section 4).

---

### 3.0A Track A — Servo Position Mode Quick-Start

**Goal:** Get the turret tracking the goal on the competition robot within 1-2 sessions
so drivers can start practicing. No PID tuning required.

**How it works:**
1. Compute the desired turret angle (geometry — same as Track B).
2. Convert the angle from radians to a servo position [0, 1].
3. Command the positional servo. The servo firmware handles getting there.
4. Read the encoder to verify the turret actually arrived (for shoot gating).

That's it. No kP, no kD, no motion planner, no velocity estimation. The servo's
internal controller runs at a much higher frequency than our 50Hz loop and is
specifically tuned for the Axon Mini's motor characteristics.

**What you give up vs. Track B:**
- No control over acceleration profile (servo decides how fast to move)
- No smooth motion planning (servo does its own thing)
- Slightly less precise dynamic tracking at high speeds
- Cannot tune settle behavior independently

**What you gain:**
- Working turret in one session instead of five
- Zero PID tuning
- No oscillation/hunting issues
- Drivers can practice while Track B is refined on the test stand

### 3.0B Track A — Subsystem Structure

Same singleton `TurretSubsystem` as Track B, but the hardware and control loop are
simpler. Uses NextFTC's `ServoEx` (position-caching wrapper) following the pattern
from 23521's working implementation, translated from Kotlin to Java.

```java
@Configurable
public class TurretSubsystem implements Subsystem {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    private TurretSubsystem() {}

    // --- Hardware ---
    // Use NextFTC ServoEx instead of raw SDK Servo — caches position so
    // repeated setPosition() calls with the same value skip the I2C write.
    // (Pattern from 23521's turret.kt, translated to Java)
    private ServoEx turretServo1 = new ServoEx("turret_servo_1");  // Axon Mini #1 (positional)
    private ServoEx turretServo2 = new ServoEx("turret_servo_2");  // Axon Mini #2 (positional)
    private AnalogInput absEncoder;         // Melonbotics absolute (4.8:1)
    private DcMotorEx quadEncoder;          // Melonbotics quadrature (4.8:1)
    private AnalogInput axonBuiltInEncoder; // Axon Mini built-in analog (1:1, coarse cal)

    // --- Constants ---
    public static double SERVO_ROTATION_DEG = 320.0;
    public static double SERVO_CENTER_OFFSET = 0.0;
    public static double SERVO2_OFFSET = 0.0;  // per-servo correction (like 23521's RIGHT_OFFSET)
    public static double MAX_TURRET_DEG = 149.0;
    public static double MIN_TURRET_DEG = -149.0;

    // --- State ---
    private TurretState state = TurretState.TRACKING;
    private double turretAngleDeg = 0.0;       // continuous angle from quadrature
    private double targetAngleDeg = 0.0;       // current desired angle
    private double lastCommandedServoDeg = 0.0; // what we last told the servo
    private double quadOffsetDeg = 0.0;

    // --- Readiness ---
    private int readyLoops = 0;
    private int notReadyLoops = 0;

    public enum TurretState {
        TRACKING,    // Continuous goal tracking
        ANGLE,       // Hold a specific angle (for auto)
        MANUAL,      // Manual control for testing
        OFF          // Servos unpowered (do not use — servos go limp)
    }
}
```

### 3.0C Track A — Hardware Initialization

```java
@Override
public void initialize() {
    // ServoEx instances are already constructed above (NextFTC initializes them
    // from the hardware name passed to the constructor). Set direction here.
    // turretServo2 is reversed so both servos push the turret the same way.
    turretServo2.setDirection(Servo.Direction.REVERSE);

    // Melonbotics absolute encoder (analog input, 4.8:1 with turret)
    absEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret_melon_abs");

    // Melonbotics quadrature encoder (4.8:1 with turret)
    quadEncoder = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret_quad_encoder");
    quadEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    // Axon Mini built-in analog encoder (1:1 with turret, for coarse calibration)
    axonBuiltInEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret_axon_encoder");

    // Calibrate (same two-stage approach as Track B — see Section 3.3)
    calibrateTwoStage();

    // Center the turret on init
    setTurretPosition(0.0);
}
```

### 3.0D Track A — Servo Position Conversion

These are the core math functions adapted from 23511's `v2-robot` branch and
23521's `turret.kt`. All our turret angle domain is in **degrees** (not radians
like 23511). Constants are declared in the subsystem class (Section 3.0B) and
exposed via `@Configurable` for dashboard tuning.

```java
/**
 * Convert turret angle (degrees) to servo position [0, 1].
 * -SERVO_ROTATION_DEG/2 -> 0.0
 *  0                    -> 0.5
 * +SERVO_ROTATION_DEG/2 -> 1.0
 */
private double degreesToServoPos(double angleDeg) {
    return (angleDeg + SERVO_ROTATION_DEG / 2.0) / SERVO_ROTATION_DEG + SERVO_CENTER_OFFSET;
}

/**
 * Convert servo position [0, 1] back to turret angle (degrees).
 */
private double servoPosToDegrees(double pos) {
    return (pos - SERVO_CENTER_OFFSET) * SERVO_ROTATION_DEG - SERVO_ROTATION_DEG / 2.0;
}

/**
 * Command the turret to a target angle in degrees.
 * Clamps to safe range, converts to servo position, commands both servos.
 * SERVO2_OFFSET compensates for mechanical misalignment between servos
 * (same concept as 23521's RIGHT_OFFSET = 0.028). Tune on-robot: if the
 * turret binds or is noisy at certain angles, adjust until smooth.
 */
public void setTurretPosition(double angleDeg) {
    angleDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, angleDeg));
    lastCommandedServoDeg = angleDeg;

    double servoPos = degreesToServoPos(angleDeg);
    servoPos = Math.max(0.01, Math.min(0.99, servoPos)); // stay off hard endpoints

    turretServo1.setPosition(servoPos);
    turretServo2.setPosition(servoPos + SERVO2_OFFSET);
}
```

### 3.0E Track A — Main Control Loop (periodic)

This is dramatically simpler than Track B. No PID, no motion planner, no velocity
estimation. Just geometry -> clamp -> convert -> command.

```java
@Override
public void periodic() {
    // 1. READ ENCODER (for readiness verification and telemetry)
    turretAngleDeg = getTurretAngleDeg();  // from quadrature, same as Track B

    if (state == TurretState.OFF) {
        return;  // servos hold last position when not commanded — but go limp if
                 // the servo is disabled. Avoid OFF state during match.
    }

    if (state == TurretState.MANUAL) {
        return;  // setTurretPosition() called externally for testing
    }

    if (state == TurretState.ANGLE) {
        // Hold a fixed angle — setTurretPosition() was already called
        // Just update readiness tracking below
    }

    if (state == TurretState.TRACKING) {
        // 2. COMPUTE DESIRED ANGLE (same geometry as Track B)
        double rawDesired = computeDesiredTurretAngle();  // Section 3.6

        // 3. WRAP + MULTI-TURN selection (same as Track B)
        targetAngleDeg = selectBestTarget(rawDesired, turretAngleDeg);  // Section 3.6

        // 4. CLAMP to safe range
        targetAngleDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, targetAngleDeg));

        // 5. COMMAND SERVO (the magic — one line replaces the entire PID loop)
        setTurretPosition(targetAngleDeg);
    }

    // 6. READINESS TRACKING (encoder verification — same pattern as Track B)
    double error = Math.abs(turretAngleDeg - lastCommandedServoDeg);
    boolean inTolerance = error < AIM_TOLERANCE_DEG;

    if (inTolerance) {
        readyLoops++;
        notReadyLoops = 0;
    } else {
        if (notReadyLoops >= NOT_READY_LOOPS_REQUIRED) {
            readyLoops = 0;
        }
        notReadyLoops++;
    }
}

public boolean isReadyToShoot() {
    return readyLoops >= READY_LOOPS_REQUIRED && state != TurretState.OFF;
}
```

### 3.0F Track A — Tuning Checklist

This replaces the 10-step tuning order for Track B. Servo mode needs far less tuning.

| Step | What | How | Pass Criteria |
|------|------|-----|--------------|
| 1 | **Encoder validation** | Display quad and abs angles, manually rotate turret | Both track correctly, same direction |
| 2 | **Calibration** | Run `calibrateTwoStage()`, check turret "forward" reads ~0° | After init, turret-forward = ~0° |
| 3 | **Servo direction** | Command `setTurretPosition(+30)`, verify turret turns correct way | If backward, swap which servo is reversed |
| 4 | **SERVO_ROTATION_DEG** | Command turret to +90° and -90°. Measure actual angle with protractor or encoder. Adjust constant until commanded matches actual | Commanded 90° = measured 90° ±2° |
| 5 | **SERVO_CENTER_OFFSET** | Command turret to 0°. If it's physically off-center, adjust offset | Turret visually points forward at 0° command |
| 5b | **SERVO2_OFFSET** | Command turret to 0° and ~90°. Listen/feel for servo grinding or binding. Adjust `SERVO2_OFFSET` in small increments (±0.005) until smooth | No audible grinding, no excess current draw, smooth motion |
| 6 | **Limit values** | Slowly command increasing angles. Find where wires get tight. Set MAX/MIN 10° inside that | Turret never reaches dangerous wire twist |
| 7 | **Geometry check** | Place robot at known positions, verify turret points at goal | Turret aims correctly from multiple field locations |
| 8 | **Readiness tolerance** | Shoot from various positions. Adjust AIM_TOLERANCE_DEG so ready=true only when shots are accurate | Consistent shot accuracy when isReadyToShoot() is true |

Total estimated time: **1-2 sessions** (vs. 4-6 for Track B).

### 3.0G Track A — Phase 2 Enhancements (After Basic Tracking Works)

Once basic tracking and driver practice are underway, these can be added incrementally:

**1. LOS rate pre-compensation (from Section 3.5):**
Instead of aiming at the current goal angle, shift the target slightly ahead based
on the LOS rate. This doesn't require PID — it just adjusts the angle fed into
`setTurretPosition()`.

```java
// In the TRACKING section of periodic():
double losRateDeg = computeLOSRate();  // degrees/sec, same as Section 3.5
double lookaheadSec = 0.05;  // tune: how far ahead to aim (servo response time)
targetAngleDeg += losRateDeg * lookaheadSec;
```

**2. Velocity lag compensation (from 23511's virtual-goal branch):**
Similar concept but derived from robot angular velocity specifically:

```java
double robotOmegaDeg = Math.toDegrees(
    PedroComponent.follower().poseTracker.getAngularVelocity());
double velLagSec = 0.067;  // from 23511's TURRET_VEL_LAG constant
targetAngleDeg += robotOmegaDeg * velLagSec;
```

> **SOTM overlap note:** Both of the above compensate for robot rotation during ball
> flight — the same effect handled by the angular velocity compensation in
> `SOTM_Plan.md` Section 7.3-7.4. When SOTM is active, the SOTM code already adds
> `omega * ballTimeOfFlight` to the turret angle. If you also apply LOS rate or
> velocity lag here in `periodic()`, you'll double-count. **Choose one location:**
> either the turret-side feedforward (here, using `lookaheadSec` or `velLagSec`) or
> the SOTM-side compensation (using `ballTimeOfFlight`). The SOTM-side version is
> recommended for Stage 2+ since it uses the same ToF value as the lead-point math.

**3. Virtual goal solver (from 23511's virtual-goal branch):**
The full predictive aiming system that accounts for robot velocity during ball
flight — this is effectively what `SOTM_Plan.md` Stage 4 (Section 9.3) implements.
Only add this if Stage 3's lead-point math with distance-dependent ToF isn't
accurate enough at high speeds.

**4. Chassis handoff near limits (from Section 4.3):**
Add the smooth chassis assist rotation when the turret approaches soft limits.
This works identically in servo mode — it adjusts the chassis, not the turret
output method.

---

### 3.1 Subsystem Structure (Track B — CR Mode, Test Stand)

Create a `TurretSubsystem` following NextFTC's `Subsystem` pattern (like your existing
`ShooterSubsystem`). Use the singleton pattern for consistency.

```java
@Configurable
public class TurretSubsystem implements Subsystem {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    private TurretSubsystem() {}

    // --- Hardware ---
    private CRServo turretServo1;          // Axon Mini #1
    private CRServo turretServo2;          // Axon Mini #2
    private AnalogInput absEncoder;        // Melonbotics absolute (0-3.2V, 4.8:1 with turret)
    private DcMotorEx quadEncoder;         // Melonbotics quadrature (on CH port 0 or 3, 4.8:1)
    private AnalogInput axonBuiltInEncoder; // Axon Mini built-in analog (1:1 with turret, for coarse cal)

    // --- State ---
    private TurretState state = TurretState.TRACKING;
    private double turretAngleDeg = 0.0;          // continuous angle from quadrature
    private double targetAngleDeg = 0.0;          // current setpoint
    private double plannedVelDegPerSec = 0.0;     // from motion planner
    private double measuredVelDegPerSec = 0.0;    // from quadrature differentiation
    private double quadOffsetDeg = 0.0;           // set at init from absolute encoder
    private long lastUpdateNanos = 0;

    // --- Tolerance counters (from 24064 pattern) ---
    private int readyLoops = 0;
    private int notReadyLoops = 0;

    public enum TurretState {
        TRACKING,    // Continuous goal tracking (INTAKE + READY + AIM from our plan)
        MANUAL,      // Manual driver control for testing
        OFF          // Servos unpowered
    }

    @Override
    public void initialize() { /* ... */ }

    @Override
    public void periodic() { /* ... main control loop ... */ }
}
```

### 3.2 Hardware Initialization (Track B)

```java
@Override
public void initialize() {
    // CR servos — one will be reversed so they push together
    turretServo1 = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_servo_1");
    turretServo2 = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_servo_2");
    turretServo2.setDirection(DcMotorSimple.Direction.REVERSE);

    // Melonbotics absolute encoder (analog input, 4.8:1 with turret)
    absEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret_melon_abs");

    // Melonbotics quadrature encoder (4.8:1 with turret)
    // USE HARDWARE-DECODED PORT (CH 0 or 3) — at 4.8:1 ratio, the encoder spins
    // 4.8x faster than the turret, making hardware decode essential.
    // Map it to the motor name assigned to port 0 or 3 in the robot config.
    quadEncoder = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret_quad_encoder");
    quadEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);  // we read position manually

    // Axon Mini built-in analog encoder (1:1 with turret, for coarse calibration)
    axonBuiltInEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret_axon_encoder");

    // Two-stage calibration: Axon (coarse) + Melonbotics (precise)
    calibrateTwoStage();

    lastUpdateNanos = System.nanoTime();
}
```

### 3.3 Calibration — Two-Stage Zeroing for 4.8:1 Encoder Ratio (Both Tracks)

**The problem:** Our Melonbotics encoder is geared at 4.8:1 to the turret. The absolute
encoder reads 0-360° of the *encoder shaft*, which maps to only 75° of turret rotation
(360° / 4.8). This means the absolute encoder wraps around 4.8 times per turret revolution.
At startup, reading the absolute encoder tells us our position within a 75° "sector" but
NOT which of the ~5 possible sectors we're in.

**23511 doesn't have this problem** because their absolute encoder is 1:1 with the turret
(on the same shaft as the servos). They can read absolute position directly.

**24064 doesn't have this problem** because their Melonbotics encoder is 1:1 with the turret.

**Our solution: Two-stage calibration**

1. **Stage 1 — Coarse position from Axon Mini's built-in analog encoder** (1:1 with servo,
   and servo is 1:1 with turret, so this is 1:1 with turret).
   - Known to have ±10° variance from battery voltage fluctuation.
   - But we only need this to identify which 75° sector we're in.
   - ±10° accuracy is *more than sufficient* for sector identification (sector width = 75°).

2. **Stage 2 — Precise position from Melonbotics absolute encoder** within that sector.
   - Gives us sub-degree precision within the identified sector.
   - Combined with the sector from Stage 1, we get a precise absolute turret angle.

**Assumption:** The turret starts near 0° at the beginning of autonomous. During teleop,
the turret position can be carried over from autonomous (like we do with robot pose in
`GlobalRobotData`). If the robot is power-cycled mid-match, the two-stage approach
re-establishes position.

```java
// --- Constants ---
public static double ABS_ENCODER_MAX_VOLTAGE = 3.2;       // Melonbotics max voltage
public static double ABS_ENCODER_OFFSET_DEG = 0.0;        // Melonbotics reading when encoder shaft is at turret "0"
public static double AXON_ENCODER_MAX_VOLTAGE = 3.3;      // Axon Mini built-in analog max voltage
public static double AXON_ENCODER_OFFSET_DEG = 0.0;       // Axon reading when turret is at "0"
public static double ENCODER_TO_TURRET_RATIO = 4.8;       // 25T:120T = encoder rotates 4.8x per turret revolution
public static double ENCODER_CPR = 4096.0;                // Melonbotics quadrature counts per encoder rev
public static double QUAD_TICKS_PER_TURRET_REV = ENCODER_CPR * ENCODER_TO_TURRET_RATIO;  // = 19,660.8
public static double DEGREES_PER_TICK = 360.0 / QUAD_TICKS_PER_TURRET_REV;               // = ~0.0183°/tick
public static boolean REVERSE_QUAD_DIRECTION = false;

// Additional hardware for Stage 1
private AnalogInput axonBuiltInEncoder;  // Axon Mini's built-in analog output (1:1 with turret)
private double quadOffsetDeg = 0.0;

@Override
public void initialize() {
    // ... (servo and Melonbotics encoder init as before) ...

    // Axon Mini built-in analog encoder for coarse position
    axonBuiltInEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret_axon_encoder");

    // Two-stage calibration
    calibrateTwoStage();

    lastUpdateNanos = System.nanoTime();
}

/**
 * Two-stage calibration:
 * 1) Axon analog (1:1 with turret, ±10° accuracy) → identifies which Melonbotics sector
 * 2) Melonbotics absolute (within sector) → precise angle
 */
private void calibrateTwoStage() {
    // Stage 1: Coarse turret angle from Axon built-in encoder (1:1 with turret)
    double axonVoltage = axonBuiltInEncoder.getVoltage();
    double coarseTurretDeg = (axonVoltage / AXON_ENCODER_MAX_VOLTAGE) * 360.0;
    coarseTurretDeg = normalizeDeg(coarseTurretDeg - AXON_ENCODER_OFFSET_DEG);
    // coarseTurretDeg is the turret angle ±10° (sufficient to identify sector)

    // Stage 2: Precise position within sector from Melonbotics absolute
    double melonVoltage = absEncoder.getVoltage();
    double melonEncoderDeg = (melonVoltage / ABS_ENCODER_MAX_VOLTAGE) * 360.0;
    melonEncoderDeg = (melonEncoderDeg - ABS_ENCODER_OFFSET_DEG + 720.0) % 360.0;
    // melonEncoderDeg is the encoder shaft angle (0-360°)

    // Convert encoder shaft angle to turret angle (one of ~5 possible values)
    double sectorWidthDeg = 360.0 / ENCODER_TO_TURRET_RATIO;  // = 75°
    double turretAngleInSector = melonEncoderDeg / ENCODER_TO_TURRET_RATIO;
    // turretAngleInSector is in [0, 75°) — precise but ambiguous

    // Find which sector (k) makes the result closest to the coarse Axon reading
    double bestTurretAngle = 0;
    double bestDist = Double.MAX_VALUE;
    // Check sectors: turretAngle = turretAngleInSector + k * sectorWidth
    // We need to cover the full turret range (roughly ±200°)
    for (int k = -3; k <= 3; k++) {
        double candidate = turretAngleInSector + k * sectorWidthDeg;
        // Normalize for comparison
        double dist = Math.abs(normalizeDeg(candidate) - normalizeDeg(coarseTurretDeg));
        if (dist > 180) dist = 360 - dist;
        if (dist < bestDist) {
            bestDist = dist;
            bestTurretAngle = candidate;
        }
    }

    // Normalize to (-180, +180]
    bestTurretAngle = normalizeDeg(bestTurretAngle);

    // Now set the quadrature offset
    double quadRawDeg = getQuadRawDeg();
    quadOffsetDeg = quadRawDeg - bestTurretAngle;
}

/** Normalize angle to (-180, +180] */
private double normalizeDeg(double deg) {
    while (deg > 180) deg -= 360;
    while (deg <= -180) deg += 360;
    return deg;
}

private double getQuadRawDeg() {
    int ticks = quadEncoder.getCurrentPosition();
    double deg = ticks * DEGREES_PER_TICK;
    return REVERSE_QUAD_DIRECTION ? -deg : deg;
}

/** Primary angle source during operation — uses quadrature (high precision, no ambiguity after init) */
public double getTurretAngleDeg() {
    return getQuadRawDeg() - quadOffsetDeg;
}
```

**Why this works reliably:**
- The Axon's ±10° variance is irrelevant — we only need it to pick the right 75° sector.
  Even at worst case (10° off), we're still safely within the 75° sector window (37.5° margin).
- The Melonbotics absolute gives us precise position within the sector (sub-degree).
- After init, the quadrature encoder takes over as the sole angle source — no more
  absolute encoder noise or voltage sensitivity during operation.
- If the turret starts within ±150° of center (which is within normal operating range),
  the sector search from k=-3 to k=3 covers ±225° — more than enough.

**Fallback option:** If the Axon built-in encoder turns out to be too unreliable even
for coarse positioning, we can instead enforce a startup procedure: "turret must start
within ±37.5° of center" and skip Stage 1, using only the Melonbotics absolute with
the assumption that sector k=0 is correct. This is the same approach 24064 uses
(they just assume they're in the right sector because their ratio is 1:1).
```

### 3.4 Main Control Loop (Track B — CR Mode)

This is the core of the CR-mode turret subsystem for the test stand.
It combines the best elements from both reference teams.

```java
// --- Tunable gains ---
public static double kP = 0.012;            // position error gain (power per degree)
public static double kD = 0.0003;           // velocity error gain (damping)
public static double kV = 0.003;            // velocity feedforward (power per deg/sec of planned velocity)
public static double kS = 0.05;             // static friction override
public static double kV_LOS = 0.003;        // LOS rate feedforward (power per deg/sec of LOS rate)
public static double MAX_TURRET_VEL = 300;  // deg/sec planner cap
public static double MAX_TURRET_ACCEL = 600; // deg/sec^2 planner cap
public static double SETTLE_ERROR_THRESHOLD = 10.0; // degrees — below this, use settle gains
public static double SETTLE_MAX_OUTPUT = 0.15;      // max power in settle region
public static double HARD_LIMIT_DEG = 200.0;
public static double SOFT_LIMIT_DEG = 160.0;
public static double CLAMP_LIMIT_DEG = 195.0;

// Tolerance and readiness
public static double AIM_TOLERANCE_DEG = 2.0;
public static double AIM_VEL_TOLERANCE = 15.0;  // deg/sec
public static int READY_LOOPS_REQUIRED = 3;
public static int NOT_READY_LOOPS_REQUIRED = 3;

@Override
public void periodic() {
    long nowNanos = System.nanoTime();
    double dt = (nowNanos - lastUpdateNanos) / 1e9;
    if (dt <= 0 || dt > 0.5) {
        lastUpdateNanos = nowNanos;
        return;
    }
    lastUpdateNanos = nowNanos;

    // 1. READ SENSORS
    turretAngleDeg = getTurretAngleDeg();
    double prevVel = measuredVelDegPerSec;
    measuredVelDegPerSec = computeVelocity(dt);  // filtered differentiation of quadrature

    if (state == TurretState.OFF) {
        setServoPower(0);
        return;
    }

    if (state == TurretState.MANUAL) {
        // Manual power is set externally via setManualPower()
        return;
    }

    // 2. COMPUTE DESIRED ANGLE (geometry)
    double rawDesired = computeDesiredTurretAngle();

    // 3. WRAP + MULTI-TURN selection (pick shortest legal path)
    targetAngleDeg = selectBestTarget(rawDesired, turretAngleDeg);

    // 4. HARD LIMIT CLAMP
    targetAngleDeg = Math.max(-CLAMP_LIMIT_DEG, Math.min(CLAMP_LIMIT_DEG, targetAngleDeg));

    // 5. MOTION PLANNER (trapezoidal profile)
    double[] planned = motionPlanner(targetAngleDeg, dt);
    double posPlan = planned[0];
    double velPlan = planned[1];
    plannedVelDegPerSec = velPlan;

    // 6. CONTROLLER
    double posError = posPlan - turretAngleDeg;
    double velError = velPlan - measuredVelDegPerSec;

    // LOS rate feedforward (from 24064's approach)
    double losRateDegPerSec = computeLOSRate();

    // Acquire vs Settle gain scheduling
    boolean settling = Math.abs(posError) < SETTLE_ERROR_THRESHOLD;

    double power = 0;
    power += kP * posError;                                    // P term
    power += kD * velError;                                    // D term (on velocity error, not derivative of position error)
    power += kV * velPlan;                                     // velocity feedforward
    power += kV_LOS * losRateDegPerSec;                        // LOS rate feedforward
    power += kS * Math.signum(velPlan);                        // static friction feedforward

    // Voltage compensation (from both reference teams)
    // double voltageScale = 12.0 / batteryVoltage;  // uncomment when voltage sensor is available
    // power *= voltageScale;

    // Settle region: cap max output to reduce overshoot
    if (settling) {
        power = Math.max(-SETTLE_MAX_OUTPUT, Math.min(SETTLE_MAX_OUTPUT, power));
    }

    // 7. LIMIT PROTECTION
    if ((turretAngleDeg >= HARD_LIMIT_DEG && power > 0) ||
        (turretAngleDeg <= -HARD_LIMIT_DEG && power < 0)) {
        power = 0;
    }

    // 8. APPLY
    power = Math.max(-1.0, Math.min(1.0, power));
    setServoPower(power);

    // 9. READINESS TRACKING
    boolean inTolerance = Math.abs(turretAngleDeg - targetAngleDeg) < AIM_TOLERANCE_DEG
                       && Math.abs(measuredVelDegPerSec) < AIM_VEL_TOLERANCE;
    if (inTolerance) {
        readyLoops++;
        notReadyLoops = 0;
    } else {
        if (notReadyLoops >= NOT_READY_LOOPS_REQUIRED) {
            readyLoops = 0;
        }
        notReadyLoops++;
    }
}

private void setServoPower(double power) {
    turretServo1.setPower(power);
    turretServo2.setPower(power);
}

public boolean isReadyToShoot() {
    return readyLoops >= READY_LOOPS_REQUIRED && state == TurretState.TRACKING;
}
```

### 3.5 LOS Rate Feedforward (Both Tracks — Adapted from 24064)

This is one of the most valuable pieces from the reference teams. It pre-compensates
turret rotation for the rate at which the goal bearing changes due to robot motion.

```java
/**
 * Compute the rate at which the turret must rotate (in deg/sec) to track the goal,
 * given the robot's current translational and angular velocity.
 *
 * Uses the derivative of atan2(dy, dx) where (dx, dy) is the vector from turret to goal.
 *
 * alphaDot = omega_robot - phiDot
 *   where phiDot = (dy * vx - dx * vy) / (dx^2 + dy^2)
 *
 * Returned in DEGREES/sec for consistency with our turret domain.
 */
private double computeLOSRate() {
    Pose botPose = PedroComponent.follower().getPose();
    Vector velocity = PedroComponent.follower().getVelocity();  // field-relative, confirmed

    double dx = shootingTarget.getX() - botPose.getX();
    double dy = shootingTarget.getY() - botPose.getY();
    double denom = dx * dx + dy * dy;
    if (denom < 1e-6) return 0.0;

    double vx = velocity.getXComponent();
    double vy = velocity.getYComponent();

    // Field-frame LOS angular rate (how fast the goal bearing changes) — rad/s
    double phiDot = (dy * vx - dx * vy) / denom;

    // Robot angular velocity — rad/s
    double omega = PedroComponent.follower().poseTracker.getAngularVelocity();

    // Turret must compensate for both: alphaDot = omega - phiDot
    double alphaDotRad = omega - phiDot;

    return Math.toDegrees(alphaDotRad);
}
```

**Student explanation:**
- `phiDot` = how fast the "line" from robot to goal is rotating in the field frame
  (caused by the robot *translating*).
- `omega` = how fast the robot body is spinning.
- `alphaDot` = how fast the turret must spin *relative to the robot* to keep
  the shooter pointed at the goal.
- If we feed `alphaDot` as a velocity command to the turret, the PID has almost
  nothing to correct — it just handles small errors.

### 3.6 Geometry and Wrap-Around Handling (Both Tracks)

```java
/**
 * Compute desired turret angle relative to robot, in degrees.
 * Returns value in (-180, +180].
 */
private double computeDesiredTurretAngle() {
    Pose botPose = PedroComponent.follower().getPose();

    double dx = shootingTarget.getX() - botPose.getX();
    double dy = shootingTarget.getY() - botPose.getY();

    // Field-frame angle to target
    double fieldAngleRad = Math.atan2(dy, dx);

    // Robot-relative turret angle (subtract robot heading + any mechanical offset)
    double turretDesiredRad = fieldAngleRad - botPose.getHeading() - Math.toRadians(TURRET_FORWARD_OFFSET_DEG);

    // Normalize to (-180, +180]
    double deg = Math.toDegrees(turretDesiredRad);
    while (deg > 180) deg -= 360;
    while (deg <= -180) deg += 360;

    return deg;
}

/**
 * Given a desired angle in (-180, +180] and the current continuous turret angle,
 * pick the equivalent target (desired + 360*k) that:
 *   1) Falls within [-CLAMP_LIMIT, +CLAMP_LIMIT]
 *   2) Is closest to the current angle
 * This prevents unnecessary 360° spins.
 */
private double selectBestTarget(double desiredDeg, double currentDeg) {
    double bestTarget = desiredDeg;
    double bestDist = Double.MAX_VALUE;

    for (int k = -2; k <= 2; k++) {
        double candidate = desiredDeg + 360.0 * k;
        if (candidate >= -CLAMP_LIMIT_DEG && candidate <= CLAMP_LIMIT_DEG) {
            double dist = Math.abs(candidate - currentDeg);
            if (dist < bestDist) {
                bestDist = dist;
                bestTarget = candidate;
            }
        }
    }
    return bestTarget;
}
```

### 3.7 Simple Trapezoidal Motion Planner (Track B Only)

Prevents setpoint jumps that cause jerk, overshoot, and derivative spikes.
Not needed in Track A — the positional servo handles its own motion profile.

```java
private double plannerPos = 0.0;   // current planned position
private double plannerVel = 0.0;   // current planned velocity

/**
 * Step the trapezoidal motion planner toward the target.
 * Returns [plannedPosition, plannedVelocity].
 */
private double[] motionPlanner(double target, double dt) {
    double error = target - plannerPos;
    double sign = Math.signum(error);

    // How far we'd travel if we started decelerating now
    double stoppingDist = (plannerVel * plannerVel) / (2.0 * MAX_TURRET_ACCEL);

    double desiredVel;
    if (Math.abs(error) <= stoppingDist + 1.0) {
        // Deceleration phase: ramp down
        desiredVel = sign * Math.sqrt(Math.max(0, 2.0 * MAX_TURRET_ACCEL * Math.abs(error)));
    } else {
        // Acceleration/cruise phase
        desiredVel = sign * MAX_TURRET_VEL;
    }

    // Apply acceleration limits
    double velError = desiredVel - plannerVel;
    double maxAccelStep = MAX_TURRET_ACCEL * dt;
    if (Math.abs(velError) > maxAccelStep) {
        plannerVel += Math.signum(velError) * maxAccelStep;
    } else {
        plannerVel = desiredVel;
    }

    // Cap velocity
    plannerVel = Math.max(-MAX_TURRET_VEL, Math.min(MAX_TURRET_VEL, plannerVel));

    // Integrate position
    plannerPos += plannerVel * dt;

    // Prevent overshoot past target
    if ((plannerPos - target) * sign > 0) {
        plannerPos = target;
        plannerVel = 0;
    }

    return new double[]{plannerPos, plannerVel};
}
```

### 3.8 Velocity Estimation (Track B Only — Filtered Quadrature Differentiation)

With our 4.8:1 ratio and 4096 CPR, we get ~54.6 counts per turret degree — this is
excellent resolution and means velocity estimation will be relatively clean. A small
moving average filter (3-5 samples) should be sufficient.

```java
private double[] velWindow = new double[5];  // moving average window
private int velWindowIdx = 0;
private int velWindowFill = 0;
private int lastQuadTicks = 0;
private boolean firstVelRead = true;

private double computeVelocity(double dt) {
    int ticks = quadEncoder.getCurrentPosition();
    if (firstVelRead) {
        lastQuadTicks = ticks;
        firstVelRead = false;
        return 0.0;
    }

    int deltaTicks = ticks - lastQuadTicks;
    lastQuadTicks = ticks;

    double rawVelDegPerSec = (deltaTicks * DEGREES_PER_TICK) / dt;
    if (REVERSE_QUAD_DIRECTION) rawVelDegPerSec = -rawVelDegPerSec;

    // Simple moving average filter (reduces noise from low CPR at 1:1)
    velWindow[velWindowIdx] = rawVelDegPerSec;
    velWindowIdx = (velWindowIdx + 1) % velWindow.length;
    if (velWindowFill < velWindow.length) velWindowFill++;

    double sum = 0;
    for (int i = 0; i < velWindowFill; i++) sum += velWindow[i];
    return sum / velWindowFill;
}
```

### 3.9 Sanity Check (Both Tracks — Absolute vs. Quadrature Mismatch)

Since both the absolute and quadrature signals come from the same Melonbotics encoder
at 4.8:1, we compare in the **encoder shaft domain** (not the turret domain) to avoid
the sector ambiguity problem.

```java
public static double MISMATCH_WARNING_DEG = 5.0;   // degrees in encoder-shaft space
public static double MISMATCH_FAULT_DEG = 15.0;
private boolean mismatchWarning = false;
private boolean mismatchFault = false;
private int mismatchConsecutiveLoops = 0;
public static int MISMATCH_LOOPS_THRESHOLD = 50; // ~1 second at 50Hz

public void checkEncoderHealth() {
    // Get Melonbotics absolute reading in encoder-shaft degrees (0-360)
    double absEncoderShaftDeg = getMelonAbsEncoderShaftDeg();

    // Get quadrature reading converted to encoder-shaft degrees, mod 360
    double quadTurretDeg = getTurretAngleDeg();
    double quadEncoderShaftDeg = (quadTurretDeg * ENCODER_TO_TURRET_RATIO) % 360.0;
    if (quadEncoderShaftDeg < 0) quadEncoderShaftDeg += 360.0;

    // Compare in encoder-shaft space (no sector ambiguity here)
    double diff = Math.abs(absEncoderShaftDeg - quadEncoderShaftDeg);
    if (diff > 180) diff = 360 - diff;

    if (diff > MISMATCH_FAULT_DEG) {
        mismatchConsecutiveLoops++;
    } else {
        mismatchConsecutiveLoops = 0;
    }

    mismatchWarning = diff > MISMATCH_WARNING_DEG;
    mismatchFault = mismatchConsecutiveLoops >= MISMATCH_LOOPS_THRESHOLD;
}

/** Raw Melonbotics absolute encoder reading in encoder-shaft degrees (0-360) */
private double getMelonAbsEncoderShaftDeg() {
    double voltage = absEncoder.getVoltage();
    double rawDeg = (voltage / ABS_ENCODER_MAX_VOLTAGE) * 360.0;
    return (rawDeg - ABS_ENCODER_OFFSET_DEG + 720.0) % 360.0;
}
```

---

## 4. Integration with Teleop

### 4.1 Adding TurretSubsystem to Pickles2025Teleop

```java
// In Pickles2025Teleop constructor:
public Pickles2025Teleop() {
    addComponents(
        new PedroComponent(Constants::createFollower),
        new SubsystemComponent(
            ShooterSubsystem.INSTANCE,
            IntakeWithSensorsSubsystem.INSTANCE,
            LEDControlSubsystem.INSTANCE,
            TurretSubsystem.INSTANCE       // <-- ADD THIS
        ),
        BulkReadComponent.INSTANCE
    );
}
```

### 4.2 Replacing Current Aiming Logic

Currently, `left_bumper` rotates the **entire robot** to face the goal. With the turret,
the robot stays driver-controlled and the turret aims independently.

**Current code (lines 375-380 of Pickles2025Teleop):**
```java
if (gamepad1.left_bumper) {
    rotate = angleErrorDeg * shooterTargetkP;   // <-- rotates chassis
    targetRPM = calculateShooterRPMOdoDistance(this.ODODistance);
    ShooterSubsystem.INSTANCE.spinUp(targetRPM);
    ...
}
```

**With turret (basic, no SOTM) — the turret handles aiming, chassis stays free:**
```java
if (gamepad1.left_bumper) {
    targetRPM = calculateShooterRPMOdoDistance(this.ODODistance);
    ShooterSubsystem.INSTANCE.spinUp(targetRPM);

    // Optional: near soft limits, add gentle chassis rotation assist
    if (TurretSubsystem.INSTANCE.isNearSoftLimit()) {
        rotate += TurretSubsystem.INSTANCE.getChassisAssistRotation() * chassisAssistGain;
    }
}
```

**With turret + SOTM (Stage 2+):** The `left_bumper` block expands significantly to
include lead-point calculation, turret angle command with angular compensation,
predictive RPM targeting, and hood compensation. See `SOTM_Plan.md` Section 7.3 for
the full integrated code block. The chassis assist logic above still applies on top.

### 4.3 Chassis Handoff Near Limits

When the turret approaches its soft limit, gradually shift some rotation
responsibility to the chassis (only when aim button is held, not during intake).

This is inspired by 23511's `angleToDriveTurretErrors()` but uses a smooth blend
instead of a hard cutover:

```java
/**
 * Returns a chassis rotation assist value in the range [-1, +1] that the teleop
 * can add to the rotate command. Magnitude scales from 0 at SOFT limit to full
 * at CLAMP limit.
 */
public double getChassisAssistRotation() {
    double absAngle = Math.abs(turretAngleDeg);
    if (absAngle < SOFT_LIMIT_DEG) return 0.0;

    // alpha goes from 0 (at SOFT) to 1 (at CLAMP)
    double alpha = (absAngle - SOFT_LIMIT_DEG) / (CLAMP_LIMIT_DEG - SOFT_LIMIT_DEG);
    alpha = Math.max(0, Math.min(1, alpha));

    // Smooth step for less abrupt transition
    alpha = alpha * alpha * (3 - 2 * alpha);

    // Direction: if turret is at +170, chassis should turn to reduce that
    double sign = Math.signum(turretAngleDeg);

    // The chassis rotation needed to center the turret
    double chassisError = computeDesiredTurretAngle();
    return -sign * alpha * Math.min(Math.abs(chassisError) / 90.0, 1.0);
}

public boolean isNearSoftLimit() {
    return Math.abs(turretAngleDeg) > SOFT_LIMIT_DEG;
}
```

---

## 5. Integration with SOTM

The turret and SOTM systems complement each other. See `SOTM_Plan.md` for the full
staged implementation.

**Stage 1 (no turret):** The robot heading aims at the virtual target. The turret
is either not installed or locked forward. No turret code is involved.

**Stage 2+ (turret active):** The turret tracks the virtual target computed by the
SOTM lead-point math (see `SOTM_Plan.md` Section 7.3 for the full code). The driver
drives freely while the turret aims. Specifically:

1. The SOTM helper computes a virtual target (real target minus velocity-based drift).
2. The turret angle is computed from robot pose to the virtual target, plus angular
   velocity compensation (Section 7.4 of the SOTM plan).
3. Predictive RPM targeting and hood compensation for rapid fire (Sections 6.6-6.7
   of the SOTM plan) handle flywheel/hood, independent of the turret.

The turret replaces the "rotate chassis to aim" step from Stage 1. The division is:
- **Turret** handles lateral aim compensation (turret angle tracks virtual target)
- **Shooter RPM** uses predictive distance from SOTM + flywheel response time
- **Hood angle** adjusts based on apparent distance (compensating for actual RPM)
- **Fire gating** requires turret ready + hood valid — no flywheel RPM wait

**Important:** Angular velocity compensation can be applied either in the teleop SOTM
code (as shown in `SOTM_Plan.md` Section 7.3-7.4) OR in the turret's own `periodic()`
via LOS feedforward (Section 3.0G Phase 2 below). Do not apply both — they compensate
for the same effect and would double-count.

---

## 6. Telemetry Recommendations

### 6A. Track A — Servo Position Mode

```java
// Essential (always show these 4)
telemetry.addData("T_target", targetAngleDeg);
telemetry.addData("T_current", turretAngleDeg);
telemetry.addData("T_error", targetAngleDeg - turretAngleDeg);
telemetry.addData("T_servoPos", lastCommandedServoPos);  // [0, 1] position sent to servo

// Readiness
telemetry.addData("T_readyLoops", readyLoops);
telemetry.addData("T_isReady", isReadyToShoot());

// Health
telemetry.addData("T_absAngle", getAbsoluteAngleDeg());
telemetry.addData("T_mismatch", mismatchWarning);
telemetry.addData("T_state", state);
```

### 6B. Track B — CR Mode (adds control loop internals)

```java
// Everything from 6A, plus:
telemetry.addData("T_power", lastServoPower);
telemetry.addData("T_velMeasured", measuredVelDegPerSec);
telemetry.addData("T_velPlanned", plannedVelDegPerSec);
telemetry.addData("T_losRate", lastLOSRate);
```

---

## 7. Tuning Order

### 7A. Track A — Servo Position Mode (Competition Robot)

See Section 3.0F for the full 8-step checklist. Summary:
1. Encoder validation
2. Calibration
3. Servo direction
4. SERVO_ROTATION_DEG
5. SERVO_CENTER_OFFSET
6. Limit values
7. Geometry check
8. Readiness tolerance

Estimated time: **1-2 sessions.**

### 7B. Track B — CR Mode (Test Stand)

This order is critical — do NOT skip steps.

| Step | What | How | Pass Criteria |
|------|------|-----|--------------|
| 1 | **Encoder validation** | Display quad and abs angles, manually rotate turret | Both track correctly, same direction, reasonable values |
| 2 | **Calibration** | Verify `calibrateTwoStage()` produces correct 0° | After init, turret "forward" reads ~0° |
| 3 | **kS (static friction)** | Set all other gains to 0. Increase kS until turret just barely starts moving | Smallest value that reliably starts motion |
| 4 | **Open-loop direction** | Send +0.2 power, verify turret rotates in expected positive direction | If backward, flip servo direction or sign convention |
| 5 | **kV (velocity FF)** | With planner running, set target 90° away. Tune kV until turret roughly tracks the planner velocity | Turret follows planned motion profile without PID |
| 6 | **kD (damping)** | Increase kD until turret stops smoothly without overshoot | No oscillation at target |
| 7 | **kP (position)** | Increase kP until final error is small without jitter | < 2° steady-state error, no oscillation |
| 8 | **Settle gains** | Tune `SETTLE_MAX_OUTPUT` so near-target behavior is smooth | No hunting/buzzing at target |
| 9 | **kV_LOS** | Drive robot past goal while turret tracks. Tune until turret maintains lock smoothly | Minimal position error while robot translates |
| 10 | **Voltage comp** | Add voltage sensor, verify consistent behavior at 13V vs 11.5V | Same response across voltage range |

Estimated time: **4-6 sessions.**

---

## 8. Borrow / Avoid Summary

### Borrow from 23511 master (CR mode — for Track B)
- Chassis/turret responsibility split architecture (`angleToDriveTurretErrors` concept)
- Voltage-compensated static friction and velocity FF
- Distance-dependent tolerance
- Absolute encoder sync offset pattern

### Borrow from 23511 v2-robot / virtual-goal (Servo mode — for Track A)
- `ServoExGroup` pattern with positional servos and `.setInverted(true)`
- `convertRadianToServoPos()` / `convertServoPoseToRadian()` conversion math
- `TURRET_SERVO_ROTATION = 320` and `TURRET_SERVO_OFFSET` constants pattern
- Encoder-based `readyToLaunch()` verification (compare encoder vs commanded position)
- `TURRET_VEL_LAG` velocity lookahead concept for the Phase 2 enhancement
- `VirtualGoalSolver` architecture for the Phase 4 enhancement
- `TURRET_PHYSICAL_OFFSET` for turret-not-at-center correction

### Borrow from 23521 Desoto Technix (NextFTC servo mode — for Track A)
- **NextFTC `ServoEx` usage** — position-caching servo wrapper from `dev.nextftc.hardware.impl`.
  Reduces I2C bus traffic when the target position hasn't changed. Use this instead of raw
  SDK `Servo` in our Java subsystem.
- **Per-servo offset pattern** (`SERVO2_OFFSET`) — compensates for mechanical misalignment
  between the two turret servos. Small value (23521 uses 0.028), tuned on-robot by listening
  for binding/grinding.
- **`SubsystemComponent` registration pattern** — how to wire subsystems into a NextFTC
  `OpMode` via `addComponents(SubsystemComponent(...))`. Directly applicable to our Java
  teleop and auto OpModes.
- **`BulkReadComponent`** — batch I2C reads for all sensors in a single bus transaction.
  23521 includes this in all their OpModes. We should do the same.
- **Heading lock mode as a driver aid** — PID on robot heading toward the alliance goal
  while a button is held. Complements turret tracking by keeping the turret near center
  where it has the most range and is most responsive. Consider adding this as a driver option.

### Borrow from 24064
- LOS rate feedforward formula (`alphaDot = omega - phiDot`)
- Tolerance counters for stability-gated shooting (`READY_TO_SHOOT_LOOPS`)
- Moving tolerance scaling
- Predicted pose concept for shooting while moving
- kA (acceleration feedforward) as a Phase 2 enhancement

### Avoid
- 24064's IDLE park-to-zero behavior — always track goal or hold last useful angle
- 23511 master's instant `atSetPoint()` readiness — use dwell-time gating instead
- 23511 master's hard `power = 0` cutoff when `atSetPoint()` is true — causes micro-hunting
- 23511 master's identical large/small PIDF coefficients — actually differentiate them in Track B
- 23511 master's commanded (not measured) velocity for feedforward — use actual velocity
- 24064's `CachedMotor` slew rate on output — be deliberate about whether you want it
- **23521's lack of readiness gating** — they fire without verifying turret has arrived.
  Always keep our encoder-based dwell-time counters (`isReadyToShoot()`).
- **23521's analog-only encoder approach** — their ~0.3° resolution is marginal. Our
  Melonbotics quadrature at 4.8:1 gives ~0.018° — use it for readiness verification.
- **Any team's exact gain values** for Track B — even though 23511 uses the same Axon Mini
  servos at the same 1:1 servo ratio, their turret has different mass, friction, inertia,
  and encoder gearing (3.75:1 vs our 4.8:1). Their gains (e.g. `kP=0.467`, `kS=0.029`,
  `VEL_FF=0.0267`) are a reasonable *starting neighborhood* but must be tuned from
  scratch on our hardware. 24064's gains are even less transferable since they use a motor.

---

## 9. Development Phases — Dual Track Schedule

### Track A: Competition Robot (Servo Position Mode)

**Goal:** Drivers practicing with turret as fast as possible.

**A1: Bench validation (1 session)**
- Wire servos as positional (`ServoEx`, not CRServo) and verify direction
- Validate encoders (quad + absolute), calibration, turret-forward = 0°
- Command +90°, -90°, 0° and verify with protractor / encoder
- Determine `SERVO_ROTATION_DEG` and `SERVO_CENTER_OFFSET`
- Tune `SERVO2_OFFSET` — listen for binding/grinding, adjust in ±0.005 increments
- Determine `MAX_TURRET_DEG` / `MIN_TURRET_DEG` from wire management

**A2: On-robot static (1 session)**
- Integrate TurretSubsystem with Pedro Pathing pose
- Place robot at 3-4 known positions, verify turret points at goal
- Tune `AIM_TOLERANCE_DEG` by shooting from various positions
- Verify `isReadyToShoot()` gating works correctly
- Test limit clamping — turret should stop before wires get tight

**A3: On-robot moving — driver practice begins (ongoing)**
- Drivers practice with turret tracking while driving
- Add LOS rate lookahead (Section 3.0G, enhancement #1) if tracking lags
- Add chassis handoff near soft limits (Section 4.3) if drivers hit limits
- Iterate on readiness tolerance based on shot accuracy data

**A4: Advanced enhancements (as time allows)**
- Velocity lag compensation (Section 3.0G, enhancement #2)
- Virtual goal solver (Section 3.0G, enhancement #3)
- Shooter integration with effective distance from virtual goal

### Track B: Test Stand (CR Mode)

**Goal:** Develop the full custom PID controller. If it outperforms servo mode,
swap it onto the competition robot.

**B1: Bench test (2-3 sessions)**
- Validate encoders, calibration, direction (same as A1 but with CRServos)
- Tune kS, kV, kD, kP in order (Section 7B)
- Test motion planner with 90° and 180° sweeps
- Verify limit clamping works

**B2: Bench refinement (1-2 sessions)**
- Tune settle behavior (acquire vs settle gain scheduling)
- Test rapid target changes (simulating moving-shot scenarios)
- Stress test: repeated fast sweeps, verify no missed counts
- Compare tracking quality against servo mode (A-B comparison)

**B3: Decision point**
- If CR mode tracks better than servo mode under dynamic conditions,
  plan a swap onto the competition robot
- If servo mode is "good enough," keep Track B as a backup and focus
  on Track A enhancements instead

### What's Shared Between Tracks

Both tracks share the same code for:
- Two-stage calibration (Section 3.3)
- Goal angle geometry (Section 3.6)
- Wrap-around / multi-turn selection (Section 3.6)
- LOS rate computation (Section 3.5)
- Readiness gating (dwell-time counters)
- Chassis handoff near limits (Section 4.3)
- Encoder health checks (Section 3.9)
- Teleop integration pattern (Section 4)

Only the `periodic()` method's output stage differs:
- Track A: `turretServo.setPosition(pos)` (one line)
- Track B: PID + feedforward -> `turretServo.setPower(power)` (many lines)

---

## 10. Open Questions for Team Discussion

### Questions for Both Tracks

1. **Melonbotics encoder CPR:** Confirm exact counts per revolution (4096? 2048? other?).
   This directly affects `DEGREES_PER_TICK` and velocity noise. With 4.8:1, even 2048 CPR
   gives ~27 counts/degree which is good. The code above assumes 4096 — verify this.

2. **Melonbotics absolute voltage range:** Confirm max voltage (3.2V? 3.3V?). Check datasheet.
   This affects the voltage-to-degrees conversion in calibration.

3. **Axon Mini built-in encoder voltage range:** Confirm max voltage and whether it's
   accessible via a separate analog port, or shares the servo signal wire.
   We need this for Stage 1 of calibration.

4. **Servo direction convention:** Which servo is "left" vs "right"? Which one gets reversed?
   Determine on bench before writing config.

5. **Hardware encoder port:** Confirm which Control Hub port (0 or 3) the Melonbotics
   quadrature will use. This must be hardware-decoded for reliability at our 4.8:1 ratio.
   At full turret speed, the encoder sees 4.8x the turret RPM — hardware decode is essential.

6. **Turret forward direction:** Which way is "0 degrees"? Typically forward (same as
   robot heading). Mechanical team needs to define this.

7. **Soft/hard limit values:** The plan says ±200° hard, ±160° soft. Are these confirmed
   by the mechanical team for wire safety? Note: 23511 uses ±149° (~2.6 radians).

8. **Battery voltage sensor:** Is one already wired and accessible via `hardwareMap`?
   Needed for voltage compensation. (Both reference teams use one.)

9. **Should we track goal during intake?** The plan recommends yes (and both reference
   teams effectively do this during ODOM_TRACKING). Confirm with drivers.

10. **Auto-fire when ready?** Both reference teams support this. Do drivers want the
    option of automatic ball feed when turret + shooter are both ready?

11. **Calibration fallback:** If Axon built-in encoder is too unreliable for Stage 1,
    are drivers OK with a startup requirement of "turret must start within ±37.5° of center"?
    This allows Melonbotics-only calibration with the assumption of sector k=0.

### Questions Specific to Track A (Servo Position Mode)

12. **Axon Mini rotation range:** What is the configured rotation range of our Axon Minis
    in positional mode? 23511 uses 320°. The Axon Mini MK2 supports configurable ranges
    (typically 300-355° depending on firmware). Check the Axon programmer tool or datasheet.
    This directly sets `SERVO_ROTATION_DEG`.

13. **Axon Mini servo mode configuration:** Do our Axon Minis need to be reconfigured
    from CR mode to positional mode? This is typically done via the Axon programmer
    hardware tool. Both servos on the competition robot need to be flashed to positional
    mode for Track A. The test stand servos stay in CR mode for Track B.

14. **Servo response speed:** How fast does the Axon Mini move in positional mode?
    If it's too slow for tracking while driving fast, we may need the LOS lookahead
    enhancement (Section 3.0G) sooner rather than later. Test this in session A2.

15. **Servo stall behavior:** What happens when a positional servo is commanded to a
    position but something is physically blocking it? Unlike CR mode (where we control
    power and can limit current implicitly), a positional servo will stall at full
    internal effort. Verify this doesn't cause overheating or brown-outs during limit
    approach scenarios.

16. **Per-servo offset magnitude:** 23521 uses `RIGHT_OFFSET = 0.028` (about 9° of servo
    travel on a 320° range). This seems large — it may reflect their specific servo/gear
    tolerances. Start with `SERVO2_OFFSET = 0.0` and only adjust if binding is observed.
    If a large offset is needed (>0.01), investigate whether the mounting is the real issue.

17. **Heading lock as a driver feature:** 23521 offers a button-held heading lock toward
    the alliance goal. Do our drivers want this option? It keeps the turret near center
    (where it has the most range) at the cost of restricting robot heading freedom.
    This is a teleop binding decision, not a turret code change.

### Decision Points

18. **When to evaluate Track B for competition swap:** Set a specific date/session count
    for the B3 decision point. Suggestion: if Track B isn't demonstrably better than
    Track A within 4 test-stand sessions, stick with Track A for competition.

19. **Parallel hardware:** Do we have enough Axon Minis for both tracks simultaneously?
    The competition robot needs 2 servos in positional mode, the test stand needs 2 in
    CR mode. That's 4 total Axon Minis (or the ability to reconfigure between sessions).
