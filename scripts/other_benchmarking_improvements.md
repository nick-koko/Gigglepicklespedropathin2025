# Other Possible Benchmarking Improvements

**Purpose:** Document non-turret improvements observed in reference team codebases
that could enhance our robot's performance. These are separate from turret control
(covered in `turret_analysis.md`) and focus on shooter, hood, and general system
integration.

**Review with your team and prioritize based on current needs.**

---

## 1. Distance-Based Shot Configuration (from FTC-23521)

**What they do:** 23521's teleop automatically adjusts flywheel speed and hood angle
based on the robot's distance to the alliance goal. A lookup table of `ShootingConfig`
entries maps distance ranges to specific flywheel RPM and hood position values.

**Their implementation (Kotlin, from teleop.kt):**
```kotlin
data class ShootingConfig(
    val minDistance: Double,
    val maxDistance: Double,
    val flywheelSpeed: Double,
    val hoodPosition: Double,
)

val shootingConfigs = listOf(
    ShootingConfig(60.0, 75.0, 1_400.0, 0.45),
    ShootingConfig(75.0, 85.0, 1_500.0, 0.55),
    ShootingConfig(85.0, 93.0, 1_600.0, 0.55),
    ShootingConfig(93.0, 98.0, 1_700.0, 0.70),
    ShootingConfig(98.0, 104.0, 1_700.0, 0.65),
    ShootingConfig(104.0, 110.0, 1_700.0, 0.65),
    ShootingConfig(110.0, 130.0, 1_700.0, 0.60),
)

fun getShootingConfigForDistance(distance: Double): ShootingConfig? {
    return shootingConfigs.firstOrNull {
        distance >= it.minDistance && distance < it.maxDistance
    }
}
```

**How it's used in their game loop:**
```kotlin
val distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY)
val config = getShootingConfigForDistance(distanceToTarget)
if (config != null && autoRangingEnabled) {
    Flywheel.setSpeed(config.flywheelSpeed)
    Hood.position = config.hoodPosition
}
```

**Comparison to our approach:** We already have our own distance-based flywheel speed
and hood angle adjustment formula. Points to compare:

| Aspect | 23521 (Lookup Table) | Our Approach (Formula) |
|--------|---------------------|----------------------|
| **Method** | Discrete range bands | Continuous function |
| **Tuning** | Add/adjust individual range entries | Adjust formula coefficients |
| **Interpolation** | None (step function between bands) | Smooth (continuous output) |
| **Edge cases** | Gaps between ranges return null (no adjustment) | Always produces a value |
| **Flexibility** | Easy to hand-tune specific distances | Better for untested distances |

**What to consider borrowing:**
- **Auto/manual toggle:** 23521 has a gamepad button to toggle between auto-ranging and
  manual flywheel/hood control. If we don't already have this, it's useful for debugging
  or when the driver wants to override the formula (e.g., for a specific shot they've
  practiced).
- **Null/out-of-range handling:** Their system returns null for distances outside all
  configured ranges, which means no adjustment at extreme distances. If our formula
  produces unreasonable values at very short or very long range, we should add clamps.
- **Telemetry of active config:** They display the current shooting mode (AUTO vs MANUAL)
  and flywheel target speed on the driver station. Useful for the driver to confirm
  the system is behaving as expected.

**What to avoid:**
- **Step discontinuities:** Their lookup table has abrupt jumps at range boundaries
  (e.g., at 75 inches, flywheel jumps from 1400 to 1500 RPM). A continuous formula
  avoids this. If we ever switch to a lookup table, add linear interpolation between
  entries.
- **Coverage gaps:** Their table only covers 60-130 inches. Shots from outside this
  range get no automatic adjustment. Our formula should handle the full expected range.

---

## 2. Flywheel Voltage Compensation (from FTC-23521)

**What they do:** 23521 wraps their flywheel motors in `VoltageCompensatingMotor`
from NextFTC's hardware library. This automatically scales motor power commands based
on the current battery voltage, maintaining consistent flywheel speed as the battery
drains during a match.

**Their implementation:**
```kotlin
private val upperShooterMotor = VoltageCompensatingMotor(MotorEx("leftShooter").brakeMode())
private val lowerShooterMotor = VoltageCompensatingMotor(MotorEx("rightShooter").brakeMode().reversed())
```

**Why this matters:** Without voltage compensation, a flywheel PID tuned at 13V will
behave differently at 11.5V near the end of a match. The PID will eventually correct,
but there will be transient speed drops after each shot as the battery sags. Voltage
compensation reduces these transients by proactively scaling the power command.

**Comparison to our approach:** Check if our flywheel subsystem already accounts for
battery voltage. If not, wrapping our flywheel motors in `VoltageCompensatingMotor`
is a one-line change using NextFTC's built-in implementation. The Java equivalent:
```java
private VoltageCompensatingMotor shooterMotor =
    new VoltageCompensatingMotor(new MotorEx("shooterMotor").brakeMode());
```

---

## 3. Flywheel Velocity PID (from FTC-23521)

**What they do:** 23521 runs a velocity PID on the flywheel using NextFTC's
`controlSystem` builder with a separate encoder for velocity measurement (not the
motor's built-in encoder).

```kotlin
val PID = controlSystem {
    velPid(0.01, 0.0, 0.0)
    basicFF(0.0004)
}
```

They also have a `waitForSpeed()` command that gates the shooting sequence until
the flywheel is within a tolerance of the target speed (`SPEED_TOLERANCE = 30`):

```kotlin
fun waitForSpeed() =
    LambdaCommand("waitForSpeed")
        .setIsDone { abs(targetSpeed - speed) <= SPEED_TOLERANCE }
        .requires(this)
```

**The shooting sequence chains these together:**
```kotlin
val shootAllLong = Flywheel.setSpeed(2_200.0)
    .then(Flywheel.waitForSpeed())
    .thenIfEnabled(Tube.shootAll())
```

**What to consider:** If our flywheel PID doesn't already use NextFTC's control system
builder, this is a clean pattern. The `waitForSpeed()` gating before `shootAll()` is
analogous to the turret's `isReadyToShoot()` — it prevents firing before the flywheel
has reached the correct speed.

---

## 4. Heading Lock Mode (from FTC-23521)

**What they do:** When the driver holds right bumper, a PID locks the robot heading
toward the alliance goal:

```kotlin
val headingPID = controlSystem { posPid(0.0085, 0.0, 0.0) }

// In onUpdate():
if (headingLocked) {
    if (alliance == Alliance.RED) {
        headingPID.goal = KineticState(135.0, 0.0)
    } else {
        headingPID.goal = KineticState(45.0, 0.0)
    }
    rotatedTurn = headingPID.calculate(
        KineticState(currentHeadingDeg, angularVelocityDeg)
    )
} else {
    rotatedTurn = -gamepad1.right_stick_x
}
```

**Why this is useful:** When the heading is locked toward the goal, the turret stays
near its center position (0°) where it has the most remaining range in both directions
and the servo response is most symmetric. This is a driver aid, not a turret feature.

**Comparison to our approach:** Our turret analysis already plans for chassis handoff
near turret soft limits (Section 4.3). A heading lock is a simpler, more direct version
of the same idea — instead of a smooth blend near limits, it always keeps the robot
pointed at the goal. The tradeoff is less freedom of movement for the driver.

**Consider offering both:** Heading lock as a hold-button option for when the driver
is setting up a shot, plus the smooth chassis handoff for general driving. These are
complementary, not competing features.

---

## 5. BulkReadComponent (from FTC-23521)

**What they do:** Every OpMode includes `BulkReadComponent` as one of the registered
components. This configures the Control Hub to batch all sensor reads into a single
I2C transaction per loop, rather than one transaction per sensor.

```kotlin
addComponents(
    BulkReadComponent,
    // ... other components
)
```

**Why this matters:** Without bulk reads, each `encoder.getVoltage()`, `motor.getCurrentPosition()`,
etc. triggers a separate I2C transaction (~3ms each). With 4+ sensors, this can add
12-15ms of dead time per loop. Bulk reads reduce this to a single ~3ms transaction.

**Action:** Confirm we're already using `BulkReadComponent` in all our OpModes. If not,
add it — it's a single line in the `addComponents()` call and benefits all subsystems.

Java:
```java
addComponents(BulkReadComponent.INSTANCE, /* ... other components */);
```

---

## 6. Auto/Manual Shooting Mode Toggle (from FTC-23521)

**What they do:** A gamepad button (`gamepad2.ps`) toggles between auto-ranging
(distance-based flywheel/hood adjustment) and manual control. In manual mode, the driver
can adjust flywheel speed in 100 RPM increments and bump the hood up/down.

```kotlin
val autoAimToggle = button { gamepad2.ps }
    .whenBecomesTrue { autoRangingEnabled = !autoRangingEnabled }
```

**Why this is useful:** During testing and tuning, the driver may want to override the
automatic settings. During a match, if the auto-ranging misbehaves (e.g., bad pose
estimate giving wrong distance), the driver can switch to manual and use a known-good
preset.

**Consider:** If we already have flywheel presets or manual override, this is covered.
If not, having a toggle between auto and manual shooter modes is a practical driver
feature, especially in the early stages when the auto-ranging formula is still being
dialed in.

---

## Summary: Priority Order

| # | Improvement | Effort | Impact | Priority |
|---|------------|--------|--------|----------|
| 5 | BulkReadComponent | Trivial (1 line) | Medium (faster loops) | Do immediately |
| 2 | Flywheel voltage compensation | Low (1 line per motor) | Medium (consistent shots) | Do soon |
| 6 | Auto/manual shooting toggle | Low (gamepad binding) | Medium (driver flexibility) | Do soon |
| 1 | Compare shot config approach | Medium (analysis) | Low (we have our own) | Review when tuning |
| 4 | Heading lock mode | Low (small PID addition) | Medium (driver aid) | Discuss with drivers |
| 3 | Flywheel velocity PID pattern | Medium (if restructuring) | Low (if already working) | Only if refactoring |
