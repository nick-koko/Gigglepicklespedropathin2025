package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Testing shooter subsystem with dt-aware PIDF + feedforward, burst caps,
 * and slew-limited positive-only output for dual flywheels.
 */
@Configurable
public class TestingShooterSubsystem implements Subsystem {

    public static final TestingShooterSubsystem INSTANCE = new TestingShooterSubsystem();
    private TestingShooterSubsystem() {}

    // =============================================
    // CONFIGURABLE GAINS (Panels tunable)
    // =============================================
    public static double kP = 0.0008;
    public static double kI = 0.0000;
    public static double kD = 0.00005;

    public static double kS = 0.15;
    public static double kV = 0.0001348;
    public static double kA = 0.00000;

    public static double HEADROOM = 0.10;
    public static double PRE_BOOST_AMOUNT = 0.05;
    public static double CONTACT_DROOP_RPM = 50.0;
    public static double SLEW_PER_SECOND = 1.5;
    public static double I_ZONE = 500.0;

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

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        if (!enabled && targetRpm <= 0.0) {
            applyPower(0.0);
            lastMeasuredRpm = measuredRpm;
            lastTimestampNanos = now;
            integral = 0.0;
            lastError = 0.0;
            lastOutput = 0.0;
            return;
        }

        double effectiveTarget = contactWindowActive
                ? Math.max(0.0, targetRpm - CONTACT_DROOP_RPM)
                : targetRpm;

        double error = effectiveTarget - measuredRpm;

        if (!contactWindowActive && Math.abs(error) < I_ZONE) {
            integral += error * dt;
            integral = Range.clip(integral, INTEGRAL_MIN, INTEGRAL_MAX);
        }

        double derivative = (error - lastError) / dt;
        double pid = (kP * error) + (kI * integral) + (kD * derivative);

        double ffStatic = kS * Math.signum(effectiveTarget);
        double ffVel = kV * effectiveTarget;
        double accel = (measuredRpm - lastMeasuredRpm) / dt;
        double ffAccel = kA * accel;
        double ff = ffStatic + ffVel + ffAccel;

        double output = Math.max(ff + pid, 0.0);

        if (preBoostActive) {
            output = Math.min(output + PRE_BOOST_AMOUNT, 1.0);
        }

        double cap = Math.max(0.0, ff + HEADROOM);
        if (contactWindowActive || recoveryWindowActive) {
            output = Math.min(output, cap);
        }

        double maxStep = Math.max(0.0, SLEW_PER_SECOND * dt);
        if (maxStep > 0.0) {
            double delta = output - lastOutput;
            if (delta > maxStep) {
                output = lastOutput + maxStep;
            } else if (delta < -maxStep) {
                output = lastOutput - maxStep;
            }
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
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void stop() {
        targetRpm = 0.0;
        enabled = false;
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

