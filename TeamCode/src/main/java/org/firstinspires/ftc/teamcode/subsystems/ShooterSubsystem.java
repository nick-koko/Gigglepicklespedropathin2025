package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

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

    public static double kS = 0.0; //voltage needed to just barely move flywheel
    public static double kV = 0.00019; //check voltage needed at about 2500rpm and 4000rpm, and then determine the kV slope required with kS
    public static double kA = 0.00000;

    public static double HEADROOM = 0.10;
    public static double CONTACT_DROOP_RPM = 50.0;
    public static double SLEW_PER_SECOND = 1.5;
    public static boolean DISABLE_SLEW_DURING_BOOST = true;
    public static double I_ZONE = 500.0;

    // =============================================
    // TIME BASED BOOST SETTINGS
    // =============================================
    public static long BOOST_DELAY_MS = 8;   // boost activates after spinUp()
    public static double BOOST_STAGE1_MULTIPLIER_NEAR = 2.66;
    public static double BOOST_STAGE2_MULTIPLIER_NEAR = 3.25;
    public static double BOOST_STAGE1_MULTIPLIER_FAR = 2.66;
    public static double BOOST_STAGE2_MULTIPLIER_FAR = 3.25;
    public static boolean ENABLE_VOLTAGE_COMPENSATION = false;
    public static double VOLTAGE_COMP_NOMINAL_V = 12.5;
    public static double VOLTAGE_COMP_FILTER_TAU_SEC = 0.5;
    public static double VOLTAGE_COMP_MIN_GAIN = 1.00;
    public static double VOLTAGE_COMP_MAX_GAIN = 1.10;
    public static double VOLTAGE_COMP_MIN_VALID_V = 7.0;

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
    // NEXTFTC HOOKS
    // =============================================
    @Override
    public void initialize() {
        shooter1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter_motor1");
        shooter2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter_motor2");
        shooterHood = ActiveOpMode.hardwareMap().get(Servo.class, "shooter_hood");

        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        resetControllerState();
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
        boostActive = false;
        secondBoostActive = false;
        boostStartTimeMs = 0;
        preBoostAmountOverride = Double.NaN;
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


