# TurretServoTest Constant and Variable Guide

This document explains every constant and key runtime variable in:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretServoTest.java`

It is intended as a calibration and telemetry reference while tuning turret behavior.

---

## 1) Control model summary

`TurretServoTest` uses a hybrid control flow:

1. **Target angle** is set by gamepad presets.
2. A **rate-limited commanded angle** moves toward that target.
3. A small **outer-loop correction trim** is added from encoder error.
4. Final angle command is converted to servo position.
5. Quadrature encoder angle is treated as runtime truth.
6. Absolute encoder is used for startup alignment, continuous angle telemetry, and diagnostics.

---

## 2) Constants (all `private static final`)

## A. Servo and angle mapping

| Name | Value | Meaning | Tune notes |
|---|---:|---|---|
| `SERVO_CENTER_POSITION` | `0.5` | Servo midpoint position. | Keep at `0.5` unless servo API convention changes. |
| `TURRET_TRAVEL_DEGREES` | `355.0` | Total turret travel represented across servo position span. | Adjust if commanded angle scale does not match physical travel. |
| `INITIAL_ANGLE_DEGREES` | `0.0` | Startup target angle in turret logical frame. | Usually keep `0.0`. |
| `POSITIVE_LIMIT_ANGLE_DEGREES` | `0.5 * TURRET_TRAVEL_DEGREES` | Precomputed positive half-range. | Derived convenience constant. |
| `NEGATIVE_LIMIT_ANGLE_DEGREES` | `-0.5 * TURRET_TRAVEL_DEGREES` | Precomputed negative half-range. | Derived convenience constant. |
| `QUARTER_POSITIVE_ANGLE_DEGREES` | `0.25 * TURRET_TRAVEL_DEGREES` | Positive quarter-range convenience constant. | Used for quick presets/tests. |
| `QUARTER_NEGATIVE_ANGLE_DEGREES` | `-0.25 * TURRET_TRAVEL_DEGREES` | Negative quarter-range convenience constant. | Used for quick presets/tests. |
| `MIN_SERVO_POSITION` | `0.0` | Minimum servo command. | Safety clamp. |
| `MAX_SERVO_POSITION` | `1.0` | Maximum servo command. | Safety clamp. |
| `MIN_SERVO_ROTATION_DEGREES` | `-160.0` | Min allowed commanded turret angle to servos. | Mechanical/wire safety limit. |
| `MAX_SERVO_ROTATION_DEGREES` | `160.0` | Max allowed commanded turret angle to servos. | Mechanical/wire safety limit. |

## B. Motion/rate limiting

| Name | Value | Meaning | Tune notes |
|---|---:|---|---|
| `MIN_RATE_DEG_PER_SEC` | `0.01 * TURRET_TRAVEL_DEGREES` | Minimum rate limit for command motion. | Lower = slower fine movement. |
| `MAX_RATE_DEG_PER_SEC` | `2.0 * TURRET_TRAVEL_DEGREES` | Maximum rate limit for command motion. | Upper cap to avoid aggressive jumps. |
| `RATE_STEP_DEG_PER_LOOP` | `0.0005 * TURRET_TRAVEL_DEGREES` | Per-loop increment/decrement amount for rate tuning via stick. | Increase to change rate faster during tuning. |

## C. Outer-loop correction and directional bias

| Name | Value | Meaning | Tune notes |
|---|---:|---|---|
| `OUTER_LOOP_KP` | `0.12` | Gain from angle error to correction trim. | Too high can oscillate; too low leaves steady-state error. |
| `OUTER_LOOP_MAX_TRIM_DEGREES` | `8.0` | Max absolute correction trim. | Limits how much outer loop can override command. |
| `POSITIVE_TARGET_BIAS_DEGREES` | `0.0` | Extra bias added for positive commands. | Use for directional asymmetry (e.g., + side off by ~3 deg). |
| `NEGATIVE_TARGET_BIAS_DEGREES` | `0.0` | Extra bias added for negative commands. | Usually 0 unless negative side has its own bias. |
| `TARGET_BIAS_APPLY_THRESHOLD_DEGREES` | `1.0` | Deadband around zero where bias is not applied. | Prevents bias from affecting small near-zero commands. |

## D. Ready-to-shoot gating logic

| Name | Value | Meaning | Tune notes |
|---|---:|---|---|
| `READY_TOLERANCE_DEGREES` | `2.0` | Max allowed angle error to count as in-tolerance. | Tighten for precision; loosen for faster readiness. |
| `READY_VELOCITY_TOLERANCE_DEG_PER_SEC` | `15.0` | Max allowed turret speed to count as settled. | Raise if too strict while still stable. |
| `READY_LOOPS_REQUIRED` | `3` | Consecutive in-tolerance loops needed for ready=true. | Dwell filter against transient crossings. |
| `NOT_READY_LOOPS_REQUIRED` | `3` | Consecutive out-of-tolerance loops before resetting ready loops. | Adds hysteresis. |

## E. Init/startup stillness and zeroing

| Name | Value | Meaning | Tune notes |
|---|---:|---|---|
| `STARTUP_EXPECTED_TURRET_ANGLE_DEGREES` | `0.0` | Turret angle expected at match start. | Keep aligned with your real startup orientation procedure. |
| `INIT_STABLE_LOOPS_REQUIRED` | `10` | Number of stable init loops to report "ready to zero". | Informational in current implementation. |
| `INIT_MAX_QUAD_DELTA_DEGREES_PER_LOOP` | `0.35` | Max quad delta/loop allowed to count as still. | Raise if noisy but stationary. |
| `INIT_MAX_ABS_DELTA_TURRET_DEGREES_PER_LOOP` | `0.35` | Max abs-derived delta/loop to count as still. | Keep similar scale to quad threshold. |

## F. Quadrature encoder configuration

| Name | Value | Meaning | Tune notes |
|---|---:|---|---|
| `TURRET_ENCODER_NAME` | `"back_left"` | Hardware map name for repurposed quadrature channel. | Must match Robot Config. |
| `TURRET_ENCODER_CPR` | `4096.0` | Encoder counts per shaft revolution. | Verify from encoder datasheet/mode. |
| `ENCODER_TO_TURRET_RATIO` | `4.8` | Encoder shaft revolutions per 1 turret revolution. | Critical conversion constant. |
| `TURRET_ENCODER_COUNTS_PER_REV` | `CPR * ratio` | Counts per turret revolution. | Derived (~19660.8). |
| `TURRET_ENCODER_COUNTS_PER_DEGREE` | `countsPerRev / 360` | Counts per turret degree. | Derived (~54.6). |

## G. Absolute encoder configuration

| Name | Value | Meaning | Tune notes |
|---|---:|---|---|
| `ABSOLUTE_TURRET_ENCODER_NAME` | `"analog_turret_encoder"` | Hardware map name for analog absolute input. | Must match Robot Config. |
| `ABSOLUTE_TURRET_ENCODER_MAX_VOLTAGE` | `3.255` | Voltage corresponding to full raw 360-degree analog range. | Must match real max from device/hub measurement. |
| `ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES` | `0.0` | Absolute encoder angular offset used during conversion. | Main persistent offset to tune for absolute reference frame. |

---

## 3) Key runtime variables (inside `runOpMode`)

## A. Hardware handles

| Variable | Type | Purpose |
|---|---|---|
| `lurret`, `rurret` | `Servo` | Primary turret servos. |
| `hurret` | `Servo` | Additional servo (currently `shooter_hood`, disabled with A PWM kill). |
| `turretEncoder` | `DcMotor` | Repurposed quadrature input channel. |
| `absoluteTurretEncoder` | `AnalogInput` | Analog absolute encoder input. |

## B. Learned/command state

| Variable | Purpose |
|---|---|
| `learnedServoCommandOffsetDegrees` | Runtime-learned servo command frame shift from startup absolute error. |
| `targetAngleDegrees` | Driver-requested turret target angle. |
| `correctedTargetAngleDegrees` | Target after directional bias correction (+/- bias constants). |
| `commandedAngleDegrees` | Rate-limited internal command that moves toward corrected target. |
| `servoCommandAngleDegrees` | Final command angle after adding outer-loop trim. |
| `currentPosition` | Final servo position command [0,1] sent to servos. |

## C. Timing/rate

| Variable | Purpose |
|---|---|
| `rateLimitDegPerSec` | Current rate limit (adjusted by left stick X). |
| `lastTime`, `dt` | Loop timing for per-second calculations. |

## D. Quadrature tracking

| Variable | Purpose |
|---|---|
| `previousEncoderTicks` | Last quad tick count for delta calculations. |
| `encoderTicks` | Current quad ticks. |
| `encoderDelta` | Tick delta this loop. |
| `quadRawAngleDegrees` | Raw quad angle from counts conversion only. |
| `quadratureOffsetDegrees` | Offset captured at Start; defines runtime zero frame. |
| `measuredAngleDegrees` | Main runtime turret angle used for control (`quadRaw - quadOffset`). |
| `encoderDeltaDegrees` | Quad angle change this loop. |
| `encoderVelocityDegPerSec` | Quad-derived angular velocity estimate. |

## E. Absolute tracking

| Variable | Purpose |
|---|---|
| `previousAbsoluteEncoderRawDegrees` | Last raw absolute angle sample for unwrapping. |
| `unwrappedAbsoluteEncoderDegrees` | Continuous absolute shaft angle (raw unwrap). |
| `absoluteEncoderRawDegrees` | Current raw absolute angle from voltage conversion (0..360 wrap). |
| `absoluteEncoderDeltaRawDegrees` | Wrapped shortest delta raw angle this loop. |
| `absoluteEncoderTurretAngleDegrees` | Absolute-based turret angle in startup-referenced frame. |
| `absoluteEncoderTurretDeltaDegrees` | Absolute-based turret delta this loop (converted through ratio). |
| `absoluteTurretAtStartDegrees` | Startup absolute-based turret estimate near expected sector. |
| `absoluteTurretReferenceAtStartDegrees` | Startup reference frame target (currently expected startup angle). |
| `absoluteStartupErrorDegrees` | Startup mismatch absolute estimate vs expected startup angle. |
| `suggestedAbsoluteOffsetDegrees` | Suggested new value for `ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES`. |

## F. Init stability and readiness flags

| Variable | Purpose |
|---|---|
| `initStableLoops` | Number of consecutive stable init loops. |
| `initStableAtStart` | Whether init stability requirement was met before Start. |
| `readyLoops`, `notReadyLoops` | Dwell counters for ready/not-ready logic. |
| `inTolerance` | Loop-level settle check (angle+velocity thresholds). |
| `turretReady` | Final ready status (used for telemetry). |
| `servosEnabled` | True until A button disables PWM. |
| `aWasPressed` | Edge detection for one-shot A press action. |

---

## 4) Helper methods and what they do

| Method | Purpose |
|---|---|
| `angleDegreesToServoPosition(angle, servoCommandOffset)` | Converts turret angle command to servo [0,1], applying learned servo offset and clamping. |
| `absoluteVoltageToRawDegrees(voltage)` | Converts analog voltage to raw absolute angle [0..360], with voltage clamping. |
| `absoluteRawToNearestTurretAngleDegrees(raw, expected)` | Converts raw absolute angle into turret-space candidate and resolves sector by nearest expected startup angle. |
| `smallestWrappedDeltaDegrees(current, previous)` | Returns shortest signed wrapped delta across 0/360 boundaries. |
| `clamp(value, min, max)` | Generic clamp helper. |

---

## 5) Telemetry dictionary (what each line means)

## A. Command/control telemetry

| Telemetry key | Meaning |
|---|---|
| `Target angle (deg)` | Driver-requested target angle. |
| `Corrected target (deg)` | Target after directional bias correction. |
| `Commanded angle (deg)` | Rate-limited command moving toward corrected target. |
| `Servo cmd angle (deg)` | Commanded angle after outer-loop trim. |
| `Target servo pos` | Servo position corresponding to target angle + learned offset. |
| `Current servo pos` | Actual servo command being sent this loop. |
| `Angle error (deg)` | `targetAngle - measuredAngle`. |
| `Outer-loop trim (deg)` | Correction term from `OUTER_LOOP_KP * error` (clamped). |

## B. Encoder telemetry

| Telemetry key | Meaning |
|---|---|
| `Turret angle (deg)` | Primary runtime angle from quadrature with startup offset. |
| `Quad raw angle (deg)` | Raw quadrature angle before startup offset subtraction. |
| `Quad offset (deg)` | Startup-captured quadrature offset used to align frame. |
| `Turret encoder (ticks)` | Raw quadrature tick count. |
| `Encoder delta/loop` | Tick change this loop. |
| `Encoder delta/loop (deg)` | Quadrature angle change this loop. |
| `Encoder vel (deg/s)` | Quadrature angular velocity estimate. |
| `Absolute encoder (V)` | Raw analog voltage reading. |
| `Absolute encoder raw angle (deg)` | Raw 0..360 angle from voltage conversion. |
| `Absolute encoder turret angle (deg)` | Absolute-based continuous turret angle in startup frame. |
| `Absolute delta/loop (deg)` | Absolute-based turret delta this loop. |

## C. Startup calibration diagnostics

| Telemetry key | Meaning |
|---|---|
| `Init stable at start` | Whether startup stillness thresholds were met before Start. |
| `Abs turret ref @start (deg)` | Startup reference frame target (normally 0). |
| `Abs turret est @start (deg)` | Absolute-based startup angle estimate before frame anchoring. |
| `Abs startup error (deg)` | Difference between absolute startup estimate and expected startup angle. |
| `Suggested abs offset (deg)` | Suggested replacement for `ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES`. |
| `Learned servo cmd offset (deg)` | Runtime-learned servo offset applied to command conversion. |
| `Abs encoder turret offset (deg)` | Current configured absolute offset constant. |

## D. Status and behavior telemetry

| Telemetry key | Meaning |
|---|---|
| `Rate limit` | Current command slew-rate limit in deg/s. |
| `Ready loops` | Ready dwell counter state. |
| `Turret ready` | True when settle criteria met for required loops. |
| `Servos enabled` | False after A-button PWM disable. |
| `Target +bias / -bias (deg)` | Configured directional target bias values. |

---

## 6) Tuning order recommendation

1. Verify hardware names and direction (`TURRET_ENCODER_NAME`, analog input, servo directions).
2. Verify startup behavior is stable (`Init stable at start` true most runs).
3. Tune `ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES` using `Abs startup error` / `Suggested abs offset`.
4. Verify learned servo offset is reasonable and consistent run-to-run.
5. Tune `OUTER_LOOP_KP` and `OUTER_LOOP_MAX_TRIM_DEGREES` for settle quality.
6. Tune directional biases (`POSITIVE_TARGET_BIAS_DEGREES`, `NEGATIVE_TARGET_BIAS_DEGREES`) only if side-dependent residual error remains.
7. Tune readiness thresholds (`READY_TOLERANCE_DEGREES`, velocity tolerance, dwell loop counts).

---

## 7) Quick troubleshooting map

- **Turret reaches wrong angle on both sides:** check `TURRET_TRAVEL_DEGREES` and startup/offset calibration.
- **Only positive or only negative side is off:** use directional bias constants.
- **Jitter around setpoint:** lower `OUTER_LOOP_KP` or reduce max trim.
- **Never goes ready:** loosen tolerance/velocity thresholds or reduce dynamic aggressiveness.
- **Absolute and quad drift apart over time:** inspect encoder health, wiring noise, or ratio/CPR constants.

