package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Shooter subsystem with dt-aware PIDF + feedforward, burst caps,
 * slew-limited positive-only output for dual flywheels,
 * and time-based boost activation.
 *
 * This is based on the former TestingShooterSubsystem implementation.
 */
@Configurable
public class ShooterSubsystem implements Subsystem {

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
    private ShooterSubsystem() {}

    // =============================================
    // CONFIGURABLE GAINS (Panels tunable)
    // =============================================
//    public static double kP = 0.15;
    public static double kP = 0.0009;
//    public static double kP = 0.00075;
    //public static double kI = 0.0001;
    public static double kI = 0.000;
//    public static double kD = 0.00001;
    public static double kD = 0.0000;

    public static double kS = 0.183; //voltage needed to just barely move flywheel
    public static double kV = 0.000113; //check voltage needed at about 2500rpm and 4000rpm, and then determine the kV slope required with kS
    public static double kA = 0.00000;

    public static double HEADROOM = 0.10;
    public static double CONTACT_DROOP_RPM = 50.0;
    public static double SLEW_PER_SECOND = 1.5;
    public static boolean DISABLE_SLEW_DURING_BOOST = true;
    public static double I_ZONE = 500.0;

    // =============================================
    // TIME BASED BOOST SETTINGS
    // =============================================
    public static long BOOST_DELAY_MS = 8000;   // boost activates after spinUp()
    public static double BOOST_STAGE1_MULTIPLIER_NEAR = 2.0;
    public static double BOOST_STAGE2_MULTIPLIER_NEAR = 2.0;
    public static double BOOST_STAGE1_MULTIPLIER_FAR = 2.0;
    public static double BOOST_STAGE2_MULTIPLIER_FAR = 2.0;
    public static boolean ENABLE_HYBRID_SHOT_FEED_BOOST = true;
    public static double HYBRID_NEAR_FAR_DISTANCE_THRESHOLD_IN = 105.0;
    public static long HYBRID_CLOSE_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST = 135;
    public static long HYBRID_CLOSE_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST = 115;
    public static long HYBRID_CLOSE_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST = 155;
    public static long HYBRID_FAR_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST = 195;
    public static long HYBRID_FAR_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST = 125;
    public static long HYBRID_FAR_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST = 145;
    public static long HYBRID_PREBOOST1_LEAD_MS = 0;
    public static long HYBRID_PREBOOST2_LEAD_MS = 0;
    public static long HYBRID_PREBOOST3_LEAD_MS = 0;
    public static double HYBRID_PREBOOST1_AMOUNT = 0.0;
    public static double HYBRID_PREBOOST2_AMOUNT = 0.0;
    public static double HYBRID_PREBOOST3_AMOUNT = 0.0;
    public static long HYBRID_BALL1_CONTACT_WINDOW_HALF_WIDTH_MS = 60;
    public static long HYBRID_BALL2_CONTACT_WINDOW_HALF_WIDTH_MS = 45;
    public static long HYBRID_BALL3_CONTACT_WINDOW_HALF_WIDTH_MS = 80;
    public static long HYBRID_MIN_EDGE_GAP_MS = 25;
    public static long HYBRID_TIMER_FALLBACK_EXTRA_MS = 0;
    public static boolean HYBRID_USE_BB1_FALL_FOR_BALL2 = true;
    public static boolean HYBRID_USE_BB1_FALL_FOR_BALL3 = true;
    public static boolean HYBRID_USE_BB2_FALL_FOR_BALL3 = true;

    // Ball-1 composite trigger: motor-signal advance conditions for WAIT_BALL1_CONTACT
    // in addition to the existing bb2_fall_edge edge + timer fallback. Based on the
    // April 18, 2026 dataset (see scripts/ball1_contact_trigger_analysis_2026-04-18.md):
    // - bb2 fall alone fires in only ~75% of sequences (misses when ball is already
    //   past bb2 at feed start), so the motor-based branches restore 100% coverage.
    // - On sequences where bb2 DOES fire, bb2 beats the motor signals by ~60-90 ms
    //   and takes precedence.
    // - On sequences where bb2 misses, the RPM-drop branch fires ~60 ms before the
    //   timer fallback on average.
    public static boolean HYBRID_USE_RPM_DROP_FOR_BALL1 = true;
    public static boolean HYBRID_USE_CURRENT_SPIKE_FOR_BALL1 = true;
    // Minimum time since shot-feed-start before the motor-based ball-1 advance
    // conditions are allowed to fire. Prevents false positives during the feed-servo
    // transient and while the baseline EMA is still settling.
    public static long HYBRID_BALL1_MOTOR_TRIGGER_MIN_MS = 50L;
    public static boolean ENABLE_VOLTAGE_COMPENSATION = false;
    public static double VOLTAGE_COMP_NOMINAL_V = 12.5;
    public static double VOLTAGE_COMP_FILTER_TAU_SEC = 0.5;
    public static double VOLTAGE_COMP_MIN_GAIN = 1.00;
    public static double VOLTAGE_COMP_MAX_GAIN = 1.10;
    public static double VOLTAGE_COMP_MIN_VALID_V = 7.0;

    // =============================================
    // RPM-DROP OBSERVER (also drives ball-1 composite trigger when enabled)
    // =============================================
    // Maintains a filtered-RPM baseline plus a derivative signal and boolean "candidate"
    // flags. The flags are both logged (for post-match CSV analysis) and consumed by
    // the hybrid ball-1 advance logic when HYBRID_USE_RPM_DROP_FOR_BALL1 is true.
    public static boolean ENABLE_RPM_DROP_OBSERVER = true;
    // First-order low-pass time constant for the baseline RPM filter (seconds).
    public static double RPM_BASELINE_FILTER_TAU_SEC = 0.08;
    // Candidate trigger: measured RPM dropped this many RPM below the active target.
    // Not used by the ball-1 trigger (baseline delta is the primary RPM signal), but
    // kept for observability.
    public static double RPM_DROP_CANDIDATE_TARGET_DELTA_RPM = 150.0;
    // Candidate trigger: measured RPM dropped this many RPM below the baseline captured
    // at the moment the hybrid shot sequence started. Tuned from April 18 data: 150 RPM
    // is the cleanest separation from motor/controller noise in the stable pre-contact
    // window while still firing in the sample immediately after true contact.
    public static double RPM_DROP_CANDIDATE_BASELINE_DELTA_RPM = 150.0;
    // Candidate trigger: d(measured RPM)/dt more negative than this threshold (RPM/sec).
    // Tuned from April 18 data: -3000 rpm/s is roughly 2x the worst pre-contact noise
    // sample and consistently fires ~1 sample into real ball contact.
    public static double RPM_DROP_CANDIDATE_DERIVATIVE_RPM_PER_SEC = -3000.0;

    // =============================================
    // CURRENT-SPIKE OBSERVER (also drives ball-1 composite trigger when enabled)
    // =============================================
    // Maintains a filtered motor-current baseline plus a derivative signal and boolean
    // "candidate" flags. Both logged and consumed by the hybrid ball-1 advance logic
    // when HYBRID_USE_CURRENT_SPIKE_FOR_BALL1 is true. NextFTC's BulkReadComponent
    // makes getCurrent() essentially free because it reads from the same cached bulk
    // transaction as encoders and digital inputs.
    public static boolean ENABLE_CURRENT_SPIKE_OBSERVER = true;
    // First-order low-pass time constant for the baseline motor-current filter (seconds).
    // Somewhat longer than the RPM baseline because the raw per-loop current reading is
    // noisier than the RPM delta.
    public static double CURRENT_BASELINE_FILTER_TAU_SEC = 0.12;
    // Candidate trigger: avg-motor-current spiked this many amps above the captured
    // pre-shot baseline. Tuned from April 18 data: ball contact adds ~0.3-1.5A above
    // the stable pre-feed baseline on the near profile; 0.3A fires cleanly on the
    // first post-contact sample without false-positiving on spin-up noise.
    public static double CURRENT_SPIKE_CANDIDATE_BASELINE_DELTA_A = 0.3;
    // Candidate trigger: d(avg-motor-current)/dt above this threshold (A/sec). Tuned
    // from April 18 data: ball-contact samples reach 10-20 A/s; stable-state noise
    // stays under ~8 A/s. 10 A/s gives the earliest reliable fire.
    public static double CURRENT_SPIKE_CANDIDATE_DERIVATIVE_A_PER_SEC = 10.0;

    // =============================================
    // CONSTANTS
    // =============================================
    private static final double SHOOTER_GEAR_RATIO = (17.0 / 23.0);
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double TICKS_PER_REV = ENCODER_TICKS_PER_MOTOR_REV * SHOOTER_GEAR_RATIO;
    private static final double INTEGRAL_MIN = -1.0;
    private static final double INTEGRAL_MAX = 1.0;

    private double boost1 = BOOST_STAGE1_MULTIPLIER_NEAR;
    private double boost2 = BOOST_STAGE2_MULTIPLIER_NEAR;

    // =============================================
    // HARDWARE
    // =============================================
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private Servo shooterHood;

    // =============================================
    // STATE
    // =============================================
    private double targetRpm = 0.0;
    private double lastError = 0.0;
    private double integral = 0.0;
    private double lastMeasuredRpm = 0.0;
    private long lastTimestampNanos = 0L;
    private double lastOutput = 0.0;
    private boolean enabled = false;

    // BOOST STATE
    public boolean boostActive = false;
    public boolean boostOverride = false;
    public boolean secondBoostActive = false;
    private long boostStartTimeMs = 0;
    private boolean preBoostActive = false;
    private double preBoostAmountOverride = Double.NaN;
    private boolean contactWindowActive = false;
    private boolean recoveryWindowActive = false;
    private boolean hybridPrevBbInitialized = false;
    private boolean hybridPrevBb1 = false;
    private boolean hybridPrevBb2 = false;
    private boolean hybridShotFeedBoostActive = false;
    private boolean hybridShotFeedUsesFarProfile = false;
    private long hybridShotFeedStartMs = 0L;
    private long hybridBall1ContactStartMsEst = -1L;
    private long hybridBall2ContactStartMsEst = -1L;
    private long hybridBall3ContactStartMsEst = -1L;
    private long hybridLastAcceptedEdgeMs = -1L;
    private long hybridLastAdvanceAtRelMs = -1L;
    private String hybridLastAdvanceReason = "NONE";

    private enum HybridShotFeedBoostPhase {
        IDLE,
        WAIT_BALL1_CONTACT,
        WAIT_BALL2_CONTACT,
        WAIT_BALL3_CONTACT,
        COMPLETE
    }

    private HybridShotFeedBoostPhase hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.IDLE;

    // =============================================
    // ENCODER-DELTA RPM MEASUREMENT STATE
    // =============================================
    private int lastTicksShooter1 = 0;
    private int lastTicksShooter2 = 0;
    private double lastRpmDeltaShooter1 = 0.0;
    private double lastRpmDeltaShooter2 = 0.0;
    private double lastRpmDeltaAverage = 0.0;
    private double lastBatteryVoltageRaw = Double.NaN;
    private double lastBatteryVoltageFiltered = Double.NaN;
    private double lastVoltageCompGain = 1.0;
    private double lastCommandPreVoltageComp = 0.0;
    private double lastCommandPostVoltageComp = 0.0;
    private boolean lastCommandSaturated = false;

    // =============================================
    // RPM-DROP OBSERVER STATE
    // =============================================
    // Continuously-updated low-pass of the instantaneous average RPM. Useful as the
    // "what were we sitting at right before the shot sequence" baseline.
    private double rpmFilteredBaseline = 0.0;
    private boolean rpmBaselineFilterInitialized = false;
    // d(measured RPM)/dt in RPM/sec, computed once per update().
    private double rpmMeasuredDerivativeRpmPerSec = 0.0;
    // Snapshot of rpmFilteredBaseline captured at startHybridShotFeedBoostController();
    // NaN when no hybrid sequence is active (or before the first one since reset).
    private double rpmAtShotSequenceStart = Double.NaN;

    // =============================================
    // CURRENT-SPIKE OBSERVER STATE
    // =============================================
    // Raw amps from the most recent update() tick (one per motor).
    private double lastCurrentA1 = 0.0;
    private double lastCurrentA2 = 0.0;
    // EMA of average motor current (both motors averaged, then filtered).
    private double currentFilteredBaselineA = 0.0;
    private boolean currentBaselineFilterInitialized = false;
    // d(avg motor current)/dt in A/sec, computed once per update().
    private double currentDerivativeAPerSec = 0.0;
    // Previous-loop unfiltered avg-current, retained for the derivative.
    private double lastAverageCurrentA = 0.0;
    // Snapshot of currentFilteredBaselineA captured at startHybridShotFeedBoostController();
    // NaN when no hybrid sequence is active (or before the first one since reset).
    private double currentAtShotSequenceStartA = Double.NaN;

    // =============================================
    // NEXTFTC HOOKS
    // =============================================
    @Override
    public void initialize() {
        // Because ShooterSubsystem is a process-wide singleton (INSTANCE), every
        // non-static-final field persists across OpMode runs for the lifetime of the
        // robot-controller app. If we don't explicitly clear runtime state here, a
        // previous run that ended with enabled=true + targetRpm>0 will cause the motors
        // to spin up on the next init before any driver input. See also the parallel
        // reset in IntakeWithSensorsSubsystem.initialize().
        resetAllRuntimeState();

        shooter1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter_motor1");
        shooter2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter_motor2");
        shooterHood = ActiveOpMode.hardwareMap().get(Servo.class, "shooter_hood");

        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Belt-and-suspenders: ensure the motors are zeroed at the Lynx level even if
        // something else tries to read stale commanded power before update() runs.
        applyPower(0.0);
    }

    /**
     * Zeros every runtime field on the singleton so a fresh OpMode init looks identical
     * to an app-boot state. Does NOT touch the @Configurable public static tuning values;
     * those are knobs, not state.
     */
    private void resetAllRuntimeState() {
        targetRpm = 0.0;
        enabled = false;

        resetControllerState();
        resetHybridShotFeedBoostController();

        // Encoder-delta RPM state. lastTicks* get re-initialized from the live encoder
        // positions in update()'s first-call-init branch (guarded by lastTimestampNanos
        // == 0L, which resetControllerState() has just set).
        lastTicksShooter1 = 0;
        lastTicksShooter2 = 0;
        lastRpmDeltaShooter1 = 0.0;
        lastRpmDeltaShooter2 = 0.0;
        lastRpmDeltaAverage = 0.0;

        // Voltage-compensation scratch state.
        lastBatteryVoltageRaw = Double.NaN;
        lastBatteryVoltageFiltered = Double.NaN;
        lastVoltageCompGain = 1.0;
        lastCommandPreVoltageComp = 0.0;
        lastCommandPostVoltageComp = 0.0;
        lastCommandSaturated = false;

        // RPM-drop observer state. Marking the filter uninitialized forces re-seeding
        // from the first post-init measurement rather than carrying last run's EMA value.
        rpmFilteredBaseline = 0.0;
        rpmBaselineFilterInitialized = false;
        rpmMeasuredDerivativeRpmPerSec = 0.0;
        rpmAtShotSequenceStart = Double.NaN;

        // Current-spike observer state. Same reasoning as RPM observer.
        lastCurrentA1 = 0.0;
        lastCurrentA2 = 0.0;
        currentFilteredBaselineA = 0.0;
        currentBaselineFilterInitialized = false;
        currentDerivativeAPerSec = 0.0;
        lastAverageCurrentA = 0.0;
        currentAtShotSequenceStartA = Double.NaN;
    }

    @Override
    public void periodic() {
        update();
    }

    // =============================================
    // CONTROL LOOP
    // =============================================
    public void update() {
        long now = System.nanoTime();
        long nowMs = System.currentTimeMillis();
        double measuredRpm = getAverageRpmInstant();

        if (lastTimestampNanos == 0L) {
            updateBatteryVoltageEstimate(0.0);
            lastTimestampNanos = now;
            lastMeasuredRpm = measuredRpm;
            // Initialize encoder-delta baselines (hardware is expected to be non-null)
            lastTicksShooter1 = shooter1.getCurrentPosition();
            lastTicksShooter2 = shooter2.getCurrentPosition();
            return;
        }

        double dt = (now - lastTimestampNanos) / 1e9;
        if (dt <= 0.0) {
            updateBatteryVoltageEstimate(0.0);
            lastTimestampNanos = now;
            return;
        }
        updateBatteryVoltageEstimate(dt);

        // === ENCODER-DELTA RPM MEASUREMENT (uses same dt as PID) ===
        int ticks1 = shooter1.getCurrentPosition();
        int ticks2 = shooter2.getCurrentPosition();

        int deltaTicks1 = ticks1 - lastTicksShooter1;
        int deltaTicks2 = ticks2 - lastTicksShooter2;

        lastTicksShooter1 = ticks1;
        lastTicksShooter2 = ticks2;

        double tps1 = deltaTicks1 / dt;
        double tps2 = deltaTicks2 / dt;

        lastRpmDeltaShooter1 = ticksPerSecondToRpm(tps1);
        lastRpmDeltaShooter2 = ticksPerSecondToRpm(tps2);
        lastRpmDeltaAverage = 0.5 * (lastRpmDeltaShooter1 + lastRpmDeltaShooter2);

        // === RPM-DROP OBSERVER (logging-only; runs every loop, even when disabled) ===
        // Derivative of measured RPM in RPM/sec. Noisy on a per-loop basis but still
        // useful aggregated/smoothed during log analysis.
        rpmMeasuredDerivativeRpmPerSec = (measuredRpm - lastMeasuredRpm) / dt;
        // Single-pole low-pass on measured RPM; alpha derived from tau so the filter is
        // stable across variable loop times.
        double baselineTau = Math.max(1e-3, RPM_BASELINE_FILTER_TAU_SEC);
        double baselineAlpha = dt / (baselineTau + dt);
        if (!rpmBaselineFilterInitialized) {
            rpmFilteredBaseline = measuredRpm;
            rpmBaselineFilterInitialized = true;
        } else {
            rpmFilteredBaseline += baselineAlpha * (measuredRpm - rpmFilteredBaseline);
        }

        // === CURRENT-SPIKE OBSERVER (logging-only; runs every loop, even when disabled) ===
        // Bulk-caching (NextFTC BulkReadComponent) makes these two getCurrent() calls
        // essentially free since they come from the same cached read as the encoders.
        if (ENABLE_CURRENT_SPIKE_OBSERVER) {
            lastCurrentA1 = shooter1.getCurrent(CurrentUnit.AMPS);
            lastCurrentA2 = shooter2.getCurrent(CurrentUnit.AMPS);
            double avgCurrentA = 0.5 * (lastCurrentA1 + lastCurrentA2);
            currentDerivativeAPerSec = (avgCurrentA - lastAverageCurrentA) / dt;
            double currentTau = Math.max(1e-3, CURRENT_BASELINE_FILTER_TAU_SEC);
            double currentAlpha = dt / (currentTau + dt);
            if (!currentBaselineFilterInitialized) {
                currentFilteredBaselineA = avgCurrentA;
                currentBaselineFilterInitialized = true;
            } else {
                currentFilteredBaselineA += currentAlpha * (avgCurrentA - currentFilteredBaselineA);
            }
            lastAverageCurrentA = avgCurrentA;
        }

        if (!enabled) {
            lastVoltageCompGain = computeVoltageCompGain();
            lastCommandPreVoltageComp = 0.0;
            lastCommandPostVoltageComp = 0.0;
            lastCommandSaturated = false;
            applyPower(0.0);
            lastMeasuredRpm = measuredRpm;
            lastTimestampNanos = now;
            integral = 0.0;
            lastError = 0.0;
            lastOutput = 0.0;
            resetControllerState();
            return;
        }

        updateHybridShotFeedBoostController(nowMs);

        // === AUTO-BOOST ACTIVATION ===
        if (!boostActive && boostStartTimeMs > 0 && nowMs >= boostStartTimeMs + BOOST_DELAY_MS) {
            boostActive = true;
        }
        if (boostActive && boostStartTimeMs > 0 && nowMs >= boostStartTimeMs + BOOST_DELAY_MS * 2) {
            secondBoostActive = true;
        }

        // Effective RPM target
        double effectiveTarget = contactWindowActive
                ? Math.max(0.0, targetRpm - CONTACT_DROOP_RPM)
                : targetRpm;

        // PID calculations
        double error = effectiveTarget - measuredRpm;

        if (!contactWindowActive && Math.abs(error) < I_ZONE) {
            integral += error * dt;
            integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);
        }

        double derivative = (error - lastError) / dt;
        double pid = (kP * error) + (kI * integral) + (kD * derivative);

        // Feedforward
        double ffStatic = kS * Math.signum(effectiveTarget);
        double ffVel = kV * effectiveTarget;
        double accel = (measuredRpm - lastMeasuredRpm) / dt;
        double ffAccel = kA * accel;
        double ff = ffStatic + ffVel + ffAccel;

        // Base output
        double output = Math.max(ff + pid, 0.0);

        // === APPLY BOOST ===
        double activePreBoostAmount =
                Double.isFinite(preBoostAmountOverride)
                        ? Math.max(0.0, preBoostAmountOverride)
                        : 0.0;

        if (preBoostActive) {
            output = Math.min(output + activePreBoostAmount, 1.0);
        } else if (boostActive && !this.boostOverride) {
            output = Math.min(output * boost1, 1.0);
        }
        else if (secondBoostActive && !this.boostOverride) {
            output = Math.min(output * boost2, 1.0);
        }

        double cap = Math.max(0.0, ff + HEADROOM);
        if (contactWindowActive || recoveryWindowActive) {
            output = Math.min(output, cap);
        }

        boolean boostWindowActive =
                preBoostActive ||
                (boostActive && !this.boostOverride) ||
                (secondBoostActive && !this.boostOverride);

        // Keep distance-noise smoothing in normal tracking, but do not blunt
        // intentionally short boost windows used for burst recovery.
        if (!(DISABLE_SLEW_DURING_BOOST && boostWindowActive)) {
            double maxStep = SLEW_PER_SECOND * dt;
            double delta = output - lastOutput;
            if (Math.abs(delta) > maxStep) {
                output = lastOutput + Math.signum(delta) * maxStep;
            }
        }

        output = Range.clip(output, 0.0, 1.0);
        lastCommandPreVoltageComp = output;
        lastVoltageCompGain = computeVoltageCompGain();
        output *= lastVoltageCompGain;
        output = Range.clip(output, 0.0, 1.0);
        lastCommandPostVoltageComp = output;
        lastCommandSaturated = output >= 0.999;
        applyPower(output);

        lastOutput = output;
        lastError = error;
        lastMeasuredRpm = measuredRpm;
        lastTimestampNanos = now;
    }

    // =============================================
    // PUBLIC API
    // =============================================
    public void spinUp(double rpm) {
        targetRpm = Math.max(0.0, rpm);
        enabled = targetRpm > 0.0;
        // Caller can optionally start boost timer via setBoostOn()
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    // Backwards-compatible alias for old API
    public double getTargetShooterRPM() {
        return targetRpm;
    }

    public void stop() {
        targetRpm = 0.0;
        enabled = false;
        resetHybridShotFeedBoostController();
        resetControllerState();
        applyPower(0.0);
    }

    public double getCurrentRpm() {
        return getAverageRpmInstant();
    }

    public boolean isAtSpeed(double tolerance) {
        return Math.abs(targetRpm - getCurrentRpm()) < tolerance;
    }

    public double getShooter1RPM() {
        return ticksPerSecondToRpm(shooter1.getVelocity());
    }

    public double getShooter2RPM() {
        return ticksPerSecondToRpm(shooter2.getVelocity());
    }

    public double getShooter1Power() {
        return shooter1.getPower();
    }

    public double getShooter2Power() {
        return shooter2.getPower();
    }

    public double getBatteryVoltageRaw() {
        return lastBatteryVoltageRaw;
    }

    public double getBatteryVoltageFiltered() {
        return lastBatteryVoltageFiltered;
    }

    public double getVoltageCompGain() {
        return lastVoltageCompGain;
    }

    public double getCommandPreVoltageComp() {
        return lastCommandPreVoltageComp;
    }

    public double getCommandPostVoltageComp() {
        return lastCommandPostVoltageComp;
    }

    public boolean isCommandSaturated() {
        return lastCommandSaturated;
    }

    public double getActiveBoostMultiplier1() {
        return boost1;
    }

    public double getActiveBoostMultiplier2() {
        return boost2;
    }

    public double getShooterHoodPosition() {
        return shooterHood.getPosition();
    }

    public void driveShooterHood(double joystick) {
        double currentPosition = shooterHood.getPosition();
        double increment = -joystick * 0.01;
        double newPosition = Range.clip(currentPosition + increment, 0.0, 1.0);
        shooterHood.setPosition(newPosition);
    }

    public void shooterHoodDrive(double hoodPosition) {
        shooterHood.setPosition(Range.clip(hoodPosition, 0.0, 1.0));
    }

    public void increaseShooterHoodPosInc() {
        shooterHood.setPosition(Range.clip(shooterHood.getPosition() + 0.1, 0.0, 1.0));
    }

    public void decreaseShooterHoodPosInc() {
        shooterHood.setPosition(Range.clip(shooterHood.getPosition() - 0.1, 0.0, 1.0));
    }

    public void increaseShooterRPMBy10() {
        if (this.targetRpm == 0)
            this.targetRpm = 3400;
        this.targetRpm = this.targetRpm + 10;
        this.enabled = true;
    }

    public void decreaseShooterRPMBy10() {
        if (this.targetRpm == 0)
            this.targetRpm = 3300;
        this.targetRpm = this.targetRpm - 10;
        this.enabled = true;
    }

    public boolean isEnabled() {
        return enabled;
    }

    // Backwards-compatible alias for old API
    public boolean getEnabled() {
        return enabled;
    }

    public void setBoostOn(boolean farBoost) {
        this.boost1 = farBoost ? BOOST_STAGE1_MULTIPLIER_FAR : BOOST_STAGE1_MULTIPLIER_NEAR;
        this.boost2 = farBoost ? BOOST_STAGE2_MULTIPLIER_FAR : BOOST_STAGE2_MULTIPLIER_NEAR;
        boostActive = false;
        secondBoostActive = false;
        boostStartTimeMs = System.currentTimeMillis();
    }

    public void setFarPID() {
        this.kP = 0.175;
        this.kI = 0;
        this.kD = 0.02;
    }

    public void setClosePID() {
        this.kP = 0.1;
        this.kI = 0;
        this.kD = 0.01;
    }

    public void setPreBoostWindow(boolean active) {
        preBoostActive = active;
    }

    public void setPreBoostAmountOverride(double amount) {
        preBoostAmountOverride = Math.max(0.0, amount);
    }

    public void clearPreBoostAmountOverride() {
        preBoostAmountOverride = Double.NaN;
    }

    public double getActivePreBoostAmount() {
        return Double.isFinite(preBoostAmountOverride)
                ? Math.max(0.0, preBoostAmountOverride)
                : 0.0;
    }

    public void setContactWindow(boolean active) {
        contactWindowActive = active;
        if (active) {
            integral = 0.0;
        }
    }

    public void setRecoveryWindow(boolean active) {
        recoveryWindowActive = active;
    }

    public void startHybridShotFeedBoostController(double shooterDistanceInches) {
        startHybridShotFeedBoostController(
                System.currentTimeMillis(),
                shooterDistanceInches >= Math.max(0.0, HYBRID_NEAR_FAR_DISTANCE_THRESHOLD_IN)
        );
    }

    public void startHybridShotFeedBoostController(boolean useFarProfile) {
        startHybridShotFeedBoostController(System.currentTimeMillis(), useFarProfile);
    }

    public void resetHybridShotFeedBoostController() {
        hybridPrevBbInitialized = false;
        hybridPrevBb1 = false;
        hybridPrevBb2 = false;
        hybridShotFeedBoostActive = false;
        hybridShotFeedUsesFarProfile = false;
        hybridShotFeedStartMs = 0L;
        hybridBall1ContactStartMsEst = -1L;
        hybridBall2ContactStartMsEst = -1L;
        hybridBall3ContactStartMsEst = -1L;
        hybridLastAcceptedEdgeMs = -1L;
        hybridLastAdvanceAtRelMs = -1L;
        hybridLastAdvanceReason = "NONE";
        hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.IDLE;
        rpmAtShotSequenceStart = Double.NaN;
        currentAtShotSequenceStartA = Double.NaN;
        boostActive = false;
        secondBoostActive = false;
        boostStartTimeMs = 0L;
        boostOverride = false;
        clearPreBoostAmountOverride();
        setPreBoostWindow(false);
        setContactWindow(false);
        setRecoveryWindow(false);
    }

    public boolean isHybridShotFeedBoostActive() {
        return hybridShotFeedBoostActive;
    }

    public String getHybridShotFeedBoostPhaseName() {
        return hybridShotFeedBoostPhase.name();
    }

    public long getHybridTimeSinceShotFeedStartMs() {
        return hybridShotFeedBoostActive
                ? Math.max(0L, System.currentTimeMillis() - hybridShotFeedStartMs)
                : -1L;
    }

    public long getHybridExpectedContactMsForCurrentPhase() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return hybridBall1ContactStartMsEst;
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return hybridBall2ContactStartMsEst;
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return hybridBall3ContactStartMsEst;
        }
        return -1L;
    }

    public String getHybridLastAdvanceReason() {
        return hybridLastAdvanceReason;
    }

    public long getHybridLastAdvanceAtRelMs() {
        return hybridLastAdvanceAtRelMs;
    }

    // =============================================
    // RPM-DROP OBSERVER GETTERS (logging / analysis only)
    // =============================================
    /** Low-pass-filtered measured RPM; tracked every loop. */
    public double getMeasuredRpmFilteredBaseline() {
        return rpmFilteredBaseline;
    }

    /** d(measured RPM)/dt in RPM/sec, computed per-loop. Noisy; smooth in analysis. */
    public double getMeasuredRpmDerivativeRpmPerSec() {
        return rpmMeasuredDerivativeRpmPerSec;
    }

    /**
     * Filtered measured RPM captured at the moment the hybrid shot sequence started.
     * NaN when no hybrid sequence has been started since the last reset.
     */
    public double getRpmAtShotSequenceStart() {
        return rpmAtShotSequenceStart;
    }

    /** target - instantaneous-measured RPM. Positive = below target. */
    public double getRpmDeltaFromTarget() {
        return targetRpm - getAverageRpmInstant();
    }

    /**
     * baseline-at-shot-start - instantaneous-measured RPM. Positive = below baseline.
     * NaN when no baseline has been captured yet (no shot sequence started).
     */
    public double getRpmDeltaFromBaseline() {
        if (Double.isNaN(rpmAtShotSequenceStart)) return Double.NaN;
        return rpmAtShotSequenceStart - getAverageRpmInstant();
    }

    /**
     * Would a target-delta trigger have fired? True only while a hybrid sequence is
     * active AND measured RPM is at least RPM_DROP_CANDIDATE_TARGET_DELTA_RPM below the
     * active target. Analysis finds "first row where this went true" per sequence.
     */
    public boolean isRpmDropCandidateTargetDelta() {
        if (!ENABLE_RPM_DROP_OBSERVER) return false;
        if (!hybridShotFeedBoostActive) return false;
        return getRpmDeltaFromTarget() >= RPM_DROP_CANDIDATE_TARGET_DELTA_RPM;
    }

    /** Same semantics as above but against the captured shot-start baseline. */
    public boolean isRpmDropCandidateBaselineDelta() {
        if (!ENABLE_RPM_DROP_OBSERVER) return false;
        if (!hybridShotFeedBoostActive) return false;
        double delta = getRpmDeltaFromBaseline();
        if (Double.isNaN(delta)) return false;
        return delta >= RPM_DROP_CANDIDATE_BASELINE_DELTA_RPM;
    }

    /** Candidate trigger on the RPM derivative (sufficiently negative d(rpm)/dt). */
    public boolean isRpmDropCandidateDerivative() {
        if (!ENABLE_RPM_DROP_OBSERVER) return false;
        if (!hybridShotFeedBoostActive) return false;
        return rpmMeasuredDerivativeRpmPerSec <= RPM_DROP_CANDIDATE_DERIVATIVE_RPM_PER_SEC;
    }

    // =============================================
    // CURRENT-SPIKE OBSERVER GETTERS (logging / analysis only)
    // =============================================
    /** Raw amps from shooter1, most recent update() tick. 0.0 if observer disabled. */
    public double getShooter1CurrentA() {
        return lastCurrentA1;
    }

    /** Raw amps from shooter2, most recent update() tick. 0.0 if observer disabled. */
    public double getShooter2CurrentA() {
        return lastCurrentA2;
    }

    /** Instantaneous average of the two motors' current, in amps. */
    public double getAverageCurrentA() {
        return 0.5 * (lastCurrentA1 + lastCurrentA2);
    }

    /** Low-pass-filtered average motor current; tracked every loop. */
    public double getCurrentFilteredBaselineA() {
        return currentFilteredBaselineA;
    }

    /** d(avg motor current)/dt in A/sec, computed per-loop. Noisy; smooth in analysis. */
    public double getCurrentDerivativeAPerSec() {
        return currentDerivativeAPerSec;
    }

    /**
     * Filtered avg-current captured at the moment the hybrid shot sequence started.
     * NaN when no hybrid sequence has been started since the last reset or the observer
     * is disabled.
     */
    public double getCurrentAtShotSequenceStartA() {
        return currentAtShotSequenceStartA;
    }

    /**
     * measured - baseline-at-shot-start, in amps. Positive = above baseline.
     * NaN when no baseline has been captured yet (no shot sequence started).
     */
    public double getCurrentDeltaFromBaselineA() {
        if (Double.isNaN(currentAtShotSequenceStartA)) return Double.NaN;
        return getAverageCurrentA() - currentAtShotSequenceStartA;
    }

    /**
     * Candidate current-spike trigger: fires when, during an active hybrid sequence, the
     * avg motor current has risen at least CURRENT_SPIKE_CANDIDATE_BASELINE_DELTA_A above
     * the captured shot-start baseline.
     */
    public boolean isCurrentSpikeCandidateBaselineDelta() {
        if (!ENABLE_CURRENT_SPIKE_OBSERVER) return false;
        if (!hybridShotFeedBoostActive) return false;
        double delta = getCurrentDeltaFromBaselineA();
        if (Double.isNaN(delta)) return false;
        return delta >= CURRENT_SPIKE_CANDIDATE_BASELINE_DELTA_A;
    }

    /** Candidate trigger on a sufficiently positive current derivative (A/sec). */
    public boolean isCurrentSpikeCandidateDerivative() {
        if (!ENABLE_CURRENT_SPIKE_OBSERVER) return false;
        if (!hybridShotFeedBoostActive) return false;
        return currentDerivativeAPerSec >= CURRENT_SPIKE_CANDIDATE_DERIVATIVE_A_PER_SEC;
    }

    // =============================================
    // HELPERS
    // =============================================
    private double getAverageRpmInstant() {
        double leftRpm = ticksPerSecondToRpm(shooter1.getVelocity());
        double rightRpm = ticksPerSecondToRpm(shooter2.getVelocity());
        return 0.5 * (leftRpm + rightRpm);
    }

    /**
     * Last computed encoder-delta RPM for shooter1 (updated once per update() call).
     */
    public double getShooter1RpmDelta() {
        return lastRpmDeltaShooter1;
    }

    /**
     * Last computed encoder-delta RPM for shooter2 (updated once per update() call).
     */
    public double getShooter2RpmDelta() {
        return lastRpmDeltaShooter2;
    }

    /**
     * Last computed average encoder-delta RPM across both shooters.
     */
    public double getAverageRpmDelta() {
        return lastRpmDeltaAverage;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    private void applyPower(double power) {
        double clipped = Range.clip(power, 0.0, 1.0);
        shooter1.setPower(clipped);
        shooter2.setPower(clipped);
    }

    private void startHybridShotFeedBoostController(long nowMs, boolean useFarProfile) {
        resetHybridShotFeedBoostController();
        hybridShotFeedBoostActive = true;
        hybridShotFeedUsesFarProfile = useFarProfile;

        // RPM observer: snapshot the filtered baseline right now. If the filter has not
        // seen enough updates yet (very early in a match), fall back to the last-known
        // instantaneous measured RPM so the column is never NaN during a shot.
        if (rpmBaselineFilterInitialized) {
            rpmAtShotSequenceStart = rpmFilteredBaseline;
        } else {
            rpmAtShotSequenceStart = lastMeasuredRpm;
        }
        // Current observer: analogous snapshot. Falls back to the raw avg if the filter
        // has not warmed yet, leaving NaN if the observer is disabled.
        if (ENABLE_CURRENT_SPIKE_OBSERVER) {
            if (currentBaselineFilterInitialized) {
                currentAtShotSequenceStartA = currentFilteredBaselineA;
            } else {
                currentAtShotSequenceStartA = lastAverageCurrentA;
            }
        } else {
            currentAtShotSequenceStartA = Double.NaN;
        }

        long dtShotFeedStartToBall1ContactMsEst = hybridShotFeedUsesFarProfile
                ? Math.max(0L, HYBRID_FAR_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST)
                : Math.max(0L, HYBRID_CLOSE_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST);
        long dtBall1ToBall2ContactStartMsEst = hybridShotFeedUsesFarProfile
                ? Math.max(0L, HYBRID_FAR_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST)
                : Math.max(0L, HYBRID_CLOSE_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST);
        long dtBall2ToBall3ContactStartMsEst = hybridShotFeedUsesFarProfile
                ? Math.max(0L, HYBRID_FAR_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST)
                : Math.max(0L, HYBRID_CLOSE_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST);

        hybridShotFeedStartMs = nowMs;
        hybridBall1ContactStartMsEst = dtShotFeedStartToBall1ContactMsEst;
        hybridBall2ContactStartMsEst = hybridBall1ContactStartMsEst + dtBall1ToBall2ContactStartMsEst;
        hybridBall3ContactStartMsEst = hybridBall2ContactStartMsEst + dtBall2ToBall3ContactStartMsEst;
        hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT;
        hybridLastAdvanceReason = "SHOT_FEED_START";
        hybridLastAdvanceAtRelMs = 0L;
    }

    private void updateHybridShotFeedBoostController(long nowMs) {
        boolean bb1 = IntakeWithSensorsSubsystem.INSTANCE.isSensor1Broken();
        boolean bb2 = IntakeWithSensorsSubsystem.INSTANCE.isSensor2Broken();
        boolean bb1FallEdge = false;
        boolean bb2FallEdge = false;
        if (hybridPrevBbInitialized) {
            bb1FallEdge = hybridPrevBb1 && !bb1;
            bb2FallEdge = hybridPrevBb2 && !bb2;
        }
        hybridPrevBb1 = bb1;
        hybridPrevBb2 = bb2;
        hybridPrevBbInitialized = true;

        if (!ENABLE_HYBRID_SHOT_FEED_BOOST) {
            clearPreBoostAmountOverride();
            setPreBoostWindow(false);
            return;
        }
        if (!hybridShotFeedBoostActive) {
            clearPreBoostAmountOverride();
            setPreBoostWindow(false);
            return;
        }

        long sinceShotFeedStartMs = nowMs - hybridShotFeedStartMs;
        long expectedContactMs = getHybridExpectedContactMsForCurrentPhase();
        long preBoostLeadMs = getHybridPreBoostLeadMsForCurrentPhase();

        boolean preBoostActive = expectedContactMs >= 0 &&
                preBoostLeadMs > 0 &&
                sinceShotFeedStartMs >= Math.max(0L, expectedContactMs - preBoostLeadMs) &&
                sinceShotFeedStartMs < expectedContactMs;
        if (preBoostActive) {
            setPreBoostAmountOverride(getHybridPreBoostAmountForCurrentPhase());
        } else {
            clearPreBoostAmountOverride();
        }
        setPreBoostWindow(preBoostActive);

        if (expectedContactMs < 0) {
            return;
        }

        boolean edgeAllowed = hybridLastAcceptedEdgeMs < 0L ||
                nowMs - hybridLastAcceptedEdgeMs >= Math.max(0L, HYBRID_MIN_EDGE_GAP_MS);
        boolean inExpectedWindow = isInHybridWindow(
                sinceShotFeedStartMs,
                expectedContactMs,
                getHybridContactWindowHalfWidthMs()
        );

        if (edgeAllowed && inExpectedWindow) {
            if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
                if (bb2FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL1_BB2_FALL_EDGE", true);
                    return;
                }
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
                if (bb2FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL2_BB2_FALL_EDGE", true);
                    return;
                }
                if (HYBRID_USE_BB1_FALL_FOR_BALL2 && bb1FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL2_BB1_FALL_EDGE", true);
                    return;
                }
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
                if (HYBRID_USE_BB1_FALL_FOR_BALL3 && bb1FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL3_BB1_FALL_EDGE", true);
                    return;
                }
                if (HYBRID_USE_BB2_FALL_FOR_BALL3 && bb2FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL3_BB2_FALL_EDGE", true);
                    return;
                }
            }
        }

        // Ball-1 composite motor-signal triggers. These are NOT gated by the
        // bb-edge debouncing (they're sustained conditions, not edges) and NOT by
        // the expected-contact window (a real RPM drop or current spike is proof of
        // contact regardless of where in the window it happens). They fire only
        // after a short arming delay so the feed-servo transient and the baseline
        // EMA have time to settle. Precedence: bb2 fall above wins when present
        // (earlier signal); these fire in the "no bb2 edge" cases.
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT &&
                sinceShotFeedStartMs >= Math.max(0L, HYBRID_BALL1_MOTOR_TRIGGER_MIN_MS)) {
            if (HYBRID_USE_RPM_DROP_FOR_BALL1 && isRpmDropCandidateBaselineDelta()) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL1_RPM_DROP_BASELINE", false);
                return;
            }
            if (HYBRID_USE_RPM_DROP_FOR_BALL1 && isRpmDropCandidateDerivative()) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL1_RPM_DROP_DERIVATIVE", false);
                return;
            }
            if (HYBRID_USE_CURRENT_SPIKE_FOR_BALL1 && isCurrentSpikeCandidateBaselineDelta()) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL1_CURRENT_SPIKE_BASELINE", false);
                return;
            }
            if (HYBRID_USE_CURRENT_SPIKE_FOR_BALL1 && isCurrentSpikeCandidateDerivative()) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL1_CURRENT_SPIKE_DERIVATIVE", false);
                return;
            }
        }

        if (sinceShotFeedStartMs >= expectedContactMs + Math.max(0L, HYBRID_TIMER_FALLBACK_EXTRA_MS)) {
            if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL1_TIMER_FALLBACK", false);
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL2_TIMER_FALLBACK", false);
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL3_TIMER_FALLBACK", false);
            }
        }
    }

    private void advanceHybridShotFeedBoostPhase(long nowMs, String reason, boolean fromEdge) {
        if (!hybridShotFeedBoostActive) return;

        long sinceShotFeedStartMs = nowMs - hybridShotFeedStartMs;
        hybridLastAdvanceReason = reason;
        hybridLastAdvanceAtRelMs = sinceShotFeedStartMs;
        if (fromEdge) {
            hybridLastAcceptedEdgeMs = nowMs;
        }

        // setBoostOn() loads the stage-1 / stage-2 multipliers but leaves
        // boostActive = false and schedules it on the old time-based delay
        // (BOOST_DELAY_MS, default 8000ms). That delay exists for the old
        // non-hybrid boost path; the hybrid controller has already proven a
        // ball just contacted the flywheel, so we force boostActive = true
        // immediately. Without this the log shows boost_active = false for
        // the entire burst and the multiplier never actually multiplies the
        // PID output.
        setBoostOn(hybridShotFeedUsesFarProfile);
        boostActive = true;

        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT;
        } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT;
        } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.COMPLETE;
            hybridShotFeedBoostActive = false;
            setPreBoostWindow(false);
        }
    }

    private long getHybridPreBoostLeadMsForCurrentPhase() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return Math.max(0L, HYBRID_PREBOOST1_LEAD_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return Math.max(0L, HYBRID_PREBOOST2_LEAD_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return Math.max(0L, HYBRID_PREBOOST3_LEAD_MS);
        }
        return 0L;
    }

    private double getHybridPreBoostAmountForCurrentPhase() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return Math.max(0.0, HYBRID_PREBOOST1_AMOUNT);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return Math.max(0.0, HYBRID_PREBOOST2_AMOUNT);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return Math.max(0.0, HYBRID_PREBOOST3_AMOUNT);
        }
        return 0.0;
    }

    private long getHybridContactWindowHalfWidthMs() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return Math.max(0L, HYBRID_BALL1_CONTACT_WINDOW_HALF_WIDTH_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return Math.max(0L, HYBRID_BALL2_CONTACT_WINDOW_HALF_WIDTH_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return Math.max(0L, HYBRID_BALL3_CONTACT_WINDOW_HALF_WIDTH_MS);
        }
        return 0L;
    }

    private boolean isInHybridWindow(long valueMs, long centerMs, long halfWidthMs) {
        return Math.abs(valueMs - centerMs) <= Math.max(0L, halfWidthMs);
    }

    private double sampleBatteryVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : ActiveOpMode.hardwareMap().voltageSensor) {
            if (sensor == null) continue;
            double voltage = sensor.getVoltage();
            if (Double.isFinite(voltage) && voltage >= VOLTAGE_COMP_MIN_VALID_V && voltage < minVoltage) {
                minVoltage = voltage;
            }
        }
        return minVoltage == Double.POSITIVE_INFINITY ? Double.NaN : minVoltage;
    }

    private void updateBatteryVoltageEstimate(double dtSeconds) {
        double rawVoltage = sampleBatteryVoltage();
        lastBatteryVoltageRaw = rawVoltage;
        if (!Double.isFinite(rawVoltage)) {
            return;
        }
        if (!Double.isFinite(lastBatteryVoltageFiltered)) {
            lastBatteryVoltageFiltered = rawVoltage;
            return;
        }
        double tau = Math.max(0.01, VOLTAGE_COMP_FILTER_TAU_SEC);
        double alpha = (dtSeconds <= 0.0)
                ? 1.0
                : Range.clip(dtSeconds / (tau + dtSeconds), 0.0, 1.0);
        lastBatteryVoltageFiltered += alpha * (rawVoltage - lastBatteryVoltageFiltered);
    }

    private double computeVoltageCompGain() {
        if (!ENABLE_VOLTAGE_COMPENSATION) {
            return 1.0;
        }
        if (!Double.isFinite(lastBatteryVoltageFiltered) || lastBatteryVoltageFiltered < VOLTAGE_COMP_MIN_VALID_V) {
            return 1.0;
        }
        double nominal = Math.max(0.1, VOLTAGE_COMP_NOMINAL_V);
        return Range.clip(
                nominal / lastBatteryVoltageFiltered,
                Math.max(0.0, VOLTAGE_COMP_MIN_GAIN),
                Math.max(VOLTAGE_COMP_MIN_GAIN, VOLTAGE_COMP_MAX_GAIN)
        );
    }

    private void resetControllerState() {
        integral = 0.0;
        lastError = 0.0;
        lastMeasuredRpm = 0.0;
        lastTimestampNanos = 0L;
        lastOutput = 0.0;
    }
}


