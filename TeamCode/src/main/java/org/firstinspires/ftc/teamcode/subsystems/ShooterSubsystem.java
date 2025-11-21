package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    public static double kP = 0.005;
    public static double kI = 0.0001;
    public static double kD = 0.00005;

    public static double kS = .1431;
    public static double kV = 0.000126;
    public static double kA = 0.00000;

    public static double HEADROOM = 0.10;
    public static double PRE_BOOST_AMOUNT = 0.05;
    public static double CONTACT_DROOP_RPM = 50.0;
    public static double SLEW_PER_SECOND = 1.5;
    public static double I_ZONE = 500.0;

    // =============================================
    // TIME BASED BOOST SETTINGS
    // =============================================
    public static double BOOST_AMOUNT = 0.20; // 20% extra power
    public static long BOOST_DELAY_MS = 10;   // boost activates after spinUp()

    // =============================================
    // CONSTANTS
    // =============================================
    private static final double SHOOTER_GEAR_RATIO = (17.0 / 23.0);
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double TICKS_PER_REV = ENCODER_TICKS_PER_MOTOR_REV * SHOOTER_GEAR_RATIO;
    private static final double INTEGRAL_MIN = -1.0;
    private static final double INTEGRAL_MAX = 1.0;

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
    private long boostStartTimeMs = 0;
    private boolean preBoostActive = false;
    private boolean contactWindowActive = false;
    private boolean recoveryWindowActive = false;

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

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

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
            lastTimestampNanos = now;
            lastMeasuredRpm = measuredRpm;
            return;
        }

        double dt = (now - lastTimestampNanos) / 1e9;
        if (dt <= 0.0) {
            lastTimestampNanos = now;
            return;
        }

        if (!enabled) {
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
        if (preBoostActive) {
            output = Math.min(output + PRE_BOOST_AMOUNT, 1.0);
        } else if (boostActive) {
            output = Math.min(output * 1.75, 1.0);
        }

        double cap = Math.max(0.0, ff + HEADROOM);
        if (contactWindowActive || recoveryWindowActive) {
            output = Math.min(output, cap);
        }

        // Slew limit
        double maxStep = SLEW_PER_SECOND * dt;
        double delta = output - lastOutput;

        if (Math.abs(delta) > maxStep) {
            output = lastOutput + Math.signum(delta) * maxStep;
        }

        output = Range.clip(output, 0.0, 1.0);
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
        boostStartTimeMs = 0;
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

    public boolean isEnabled() {
        return enabled;
    }

    // Backwards-compatible alias for old API
    public boolean getEnabled() {
        return enabled;
    }

    public void setBoostOn() {
        boostActive = false;
        boostStartTimeMs = System.currentTimeMillis();
    }

    public void setPreBoostWindow(boolean active) {
        preBoostActive = active;
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

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    private void applyPower(double power) {
        double clipped = Range.clip(power, 0.0, 1.0);
        shooter1.setPower(clipped);
        shooter2.setPower(clipped);
    }

    private void resetControllerState() {
        integral = 0.0;
        lastError = 0.0;
        lastMeasuredRpm = 0.0;
        lastTimestampNanos = 0L;
        lastOutput = 0.0;
    }
}


