package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Intake Subsystem with Break Beam Sensors
 *
 * Hardware:
 * - Motors: m1 (top front), m2 (bottom), m3 (top back)
 * - Servos: s2 (bottom continuous), s3 (top back continuous)
 * - Sensors: sensor0, sensor1, sensor2 (beam breaks)
 *
 * The subsystem automatically monitors sensors and stops motors as balls pass through.
 * Ball tracking logic:
 * - Ball 1: Stops m3 when it breaks sensor2
 * - Ball 2: Stops m2 when it breaks sensor1
 * - Ball 3: Stops m1 when it breaks sensor0
 *
 * This implementation is based on the former TestingIntakeWithSensorsSubsystem.
 */
@Configurable
public class IntakeWithSensorsSubsystem implements Subsystem {

    public static final IntakeWithSensorsSubsystem INSTANCE = new IntakeWithSensorsSubsystem();
    private IntakeWithSensorsSubsystem() {}

    // =============================================
    // CONFIGURABLE CONSTANTS
    // =============================================

    // Intake RPM targets
    public static double M1_TARGET_RPM = 1000.0;
    public static double M3_TARGET_RPM = 500.0;

    // Shoot RPM targets
    public static double M1_SHOOT_RPM = 400.0;
    public static double M3_SHOOT_RPM = 900.0;

    // Power levels for single-ball / multi-single-ball feeding
    public static double M1_SINGLE_SHOT_POWER = 0.8;
    public static double M3_SINGLE_SHOT_POWER = 1.0;
    public static double S2_SINGLE_SHOT_POWER = 0.85;
    public static double S3_SINGLE_SHOT_POWER = 0.85;

    // Delay between shots for multi-single-shot mode (ms)
    public static long MULTI_SINGLE_SHOT_DELAY_MS = 150;
    
    // Timeout for single-ball feed - if sensor doesn't clear within this time,
    // assume ball has passed (handles case where 2 balls pass together undetected)
    public static long SINGLE_BALL_FEED_TIMEOUT_MS = 1000;

    // Continuous servo speeds
    public static double S2_INTAKE_SPEED = 0.7;
    public static double S3_INTAKE_SPEED = 0.7;
    public static double S2_SHOOT_SPEED = 0.5;
    public static double S3_SHOOT_SPEED = 0.5;

    public static int MAX_SHOTS_PER_SEQUENCE = 3;     // Number of balls to shoot per button press

    // Motor constants
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double M1_GEAR_RATIO = 4;
    private static final double M3_GEAR_RATIO = 4;

    // =============================================
    // HARDWARE
    // =============================================

    private DcMotorEx m1, m3;
    private CRServo s2, s3;
    private DigitalChannel sensor0, sensor1, sensor2;

    // =============================================
    // STATE TRACKING
    // =============================================

    private int ballCount = 0;
    private boolean m1Enabled = true;
    private boolean m2Enabled = true;
    private boolean m3Enabled = true;

    private boolean shooting = false;
    private boolean shootSequenceActive = false;  // True during multi-shot sequence
    private int currentShot = 0;                  // Current shot number in sequence (0 = not shooting)
    private int shotsToFire = 0;                  // Number of shots to fire in this sequence
    private long shootEndTime = 0;                // When current individual shot ends
    private long nextShotTime = 0;                // When next shot in sequence should start
    private ElapsedTime shootTimer = new ElapsedTime();

    // Sensor state tracking for edge detection
    private boolean prevSensor0 = true;
    private boolean prevSensor1 = true;
    private boolean prevSensor2 = true;

    // Ticks per revolution calculations
    private double m1TicksPerRev;
    private double m3TicksPerRev;

    // Intake run state
    private boolean isIntaking = false;
    private double currentDirection = 0.0; // +1 forward, -1 reverse
    private boolean singleBallActive = false;

    private long SHOT_DELAY_MS = 300;
    private long SHOT_TIME = 10; // wait time between shots
    private boolean shotInProgress = false;

    // =============================================
    // SINGLE-BALL FULL-POWER FEED STATE
    // =============================================
    /**
     * True while we are advancing exactly one ball at full power using sensor2
     * (broken -> clear) as the stop condition, or timeout.
     */
    private boolean singleBallFeedActive = false;
    private boolean prevSensor2BrokenForSingleFeed = false;
    private long singleBallFeedStartTimeMs = 0L;

    // Multi-single-shot sequence state (looped single-ball feeds with delay)
    private boolean multiSingleShotActive = false;
    private int multiSingleShotRequested = 0;
    private int multiSingleShotCompleted = 0;
    private long nextMultiSingleShotStartTimeMs = 0L;

    // =============================================
    // INITIALIZATION
    // =============================================

    @Override
    public void initialize() {
        // Initialize motors
        m1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake_motor");
        m3 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake_motor2");

        // Initialize servos
        s2 = ActiveOpMode.hardwareMap().get(CRServo.class, "intake_servo1");
        s3 = ActiveOpMode.hardwareMap().get(CRServo.class, "intake_servo2");

        // Initialize sensors
        sensor0 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "breakbeam0");
        sensor1 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "breakbeam1");
        sensor2 = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "breakbeam2");

        // Configure motors
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure servos
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        s3.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure sensors
        sensor0.setMode(DigitalChannel.Mode.INPUT);
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        sensor2.setMode(DigitalChannel.Mode.INPUT);

        // Calculate ticks per revolution
        m1TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M1_GEAR_RATIO;
        m3TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M3_GEAR_RATIO;
    }

    // =============================================
    // PERIODIC - Runs automatically every loop!
    // =============================================

    @Override
    public void periodic() {
//        if (shootSequenceActive) {   //Commented out because of intake issues, and using just dumb shoot with multi-ball boost, so maybe not needed.
//            updateShooting();
//            return;
//        }

        // 1) Handle active single-ball feed (highest priority - used by single-shot and multi-single-shot modes)
        if (singleBallFeedActive) {
            updateSingleBallFeed();
            return;
        }
        
        // 2) Handle multi-single-shot sequence between feeds
        if (multiSingleShotActive) {
            long now = System.currentTimeMillis();

            // If it's time for the next shot in the sequence, start another single-ball feed
            if (nextMultiSingleShotStartTimeMs == 0L || now >= nextMultiSingleShotStartTimeMs) {
                boolean started = feedSingleBallFullPower();
                if (!started) {
                    // Could not start another shot (no balls or conflicting state) – end the sequence
                    multiSingleShotActive = false;
                } else {
                    // Shot is now running; delay for the next one will be scheduled
                    // in updateSingleBallFeed() when this shot completes.
                    nextMultiSingleShotStartTimeMs = 0L;
                }
            }
            return;
        }

        // 3) Normal intake behavior
        if (isIntaking) {
            currentShot = 0;
            
            // First, detect new balls via transitions
            checkSensorsAndUpdateMotors();
            
            // SAFETY: Enforce motor enables match ball count
            // This catches stuck states where a motor is on but shouldn't be
            // Key insight: if ballCount >= N, motor for position N must be OFF
            // This is safer - thinking we have MORE balls means motors stay off (safe)
            // vs thinking we have FEWER balls means motors push ball into flywheel (dangerous)
            if (ballCount >= 1 && m3Enabled) {
                m3Enabled = false;
            }
            if (ballCount >= 2 && m2Enabled) {
                m2Enabled = false;
            }
            if (ballCount >= 3 && m1Enabled) {
                m1Enabled = false;
            }
            
            setIntakeDirection(currentDirection, false);
        }

        // Legacy singleBallActive handling remains commented out
//        if (singleBallActive) {
//            currentShot -= 1;
//        }
    }

    /**
     * Updates state for a single-ball full-power feed.
     * Stops the intake once sensor2 transitions from broken -> clear,
     * OR if the timeout expires (handles case where 2 balls pass together).
     * Decrements ballCount by 1 (clamped to >= 0).
     */
    private void updateSingleBallFeed() {
        long now = System.currentTimeMillis();
        boolean currentBroken = isSensor2Broken();
        
        // Check for timeout - if ball hasn't cleared sensor within timeout,
        // assume it passed (handles 2 balls going through together undetected)
        boolean timedOut = (now - singleBallFeedStartTimeMs) >= SINGLE_BALL_FEED_TIMEOUT_MS;

        // We want: sensor2.getState() false (broken) -> true (clear)
        // isSensor2Broken() is !sensor2.getState(), so that becomes true -> false.
        // OR timeout has elapsed
        if ((prevSensor2BrokenForSingleFeed && !currentBroken) || timedOut) {
            // Ball has cleared sensor2 (or timed out) – stop everything
            stop();

            // Safely decrement ball count
            if (ballCount > 0) {
                ballCount--;
            }

            singleBallFeedActive = false;

            // If we're in multi-single-shot mode, schedule the next shot (or finish)
            if (multiSingleShotActive) {
                multiSingleShotCompleted++;

                if (multiSingleShotCompleted >= multiSingleShotRequested || ballCount <= 0) {
                    // Sequence complete
                    multiSingleShotActive = false;
                } else {
                    // Schedule the next shot after a delay
                    nextMultiSingleShotStartTimeMs = now + MULTI_SINGLE_SHOT_DELAY_MS;
                }
            }
        }

        prevSensor2BrokenForSingleFeed = currentBroken;
    }

    public void updateShooting() {
        if (!shootSequenceActive) return;

        long now = System.currentTimeMillis();

        // 1️⃣ Currently shooting a ball
        if (shotInProgress) {
            if (now >= shootEndTime) {
                stop(); // stop feeder motors
                shotInProgress = false;

                // Set pause period
                nextShotTime = now + SHOT_DELAY_MS;

                // Decrement ball count safely
                if (ballCount > 0) ballCount--;
                currentShot++;

                // Prevent immediate restart in same loop
                return;
            }
        }

        // 2️⃣ Waiting before next shot
        if (!shotInProgress && currentShot < shotsToFire && ballCount > 0) {
            // Still in cooldown period
            if (now < nextShotTime) {
                // Do nothing — we're pausing intentionally
                return;
            }

            // Done waiting — time for next shot
            startSingleShot();
        }

        // 3️⃣ All done
        if ((currentShot >= shotsToFire || ballCount <= 0) && !shotInProgress) {
            shootSequenceActive = false;
            shooting = false;
            stop();
        }
    }

    /**
     * Start feeding one ball into the shooter.
     */
    private void startSingleShot() {
        shooting = true;
        shotInProgress = true;

        // Start feeder/indexer motors
        m1.setVelocity(rpmToTicksPerSecond(M1_SHOOT_RPM, m1TicksPerRev));
        m3.setVelocity(rpmToTicksPerSecond(M3_SHOOT_RPM, m3TicksPerRev));
        s2.setPower(S2_SHOOT_SPEED);
        s3.setPower(S3_SHOOT_SPEED);

        shootEndTime = System.currentTimeMillis() + SHOT_TIME;
    }

    // =============================================
    // SENSOR MONITORING (runs automatically)
    // =============================================

    private void checkSensorsAndUpdateMotors() {
        // Read sensor states (false = beam broken, true = clear)
        boolean sensor0State = sensor0.getState();
        boolean sensor1State = sensor1.getState();
        boolean sensor2State = sensor2.getState();

        // Detect when a ball enters a sensor (transition from clear to broken)
        boolean sensor0Falling = !sensor0State && prevSensor0;
        boolean sensor1Falling = !sensor1State && prevSensor1;
        boolean sensor2Falling = !sensor2State && prevSensor2;

        // Ball detection logic (only when not shooting)
        if (!shooting) {
            if (sensor2Falling && ballCount == 0) {
                m3Enabled = false;  // Ball 1 broke sensor2, stop m3
                ballCount = 1;
            } else if (sensor1Falling && ballCount == 1) {
                m2Enabled = false;  // Ball 2 broke sensor1, stop m2
                ballCount = 2;
            } else if (sensor0Falling && ballCount == 2) {
                m1Enabled = false;  // Ball 3 broke sensor0, stop m1
                ballCount = 3;
            } else if (!sensor0State && !sensor1State && !sensor2State) {
                m1Enabled = false;
                m2Enabled = false;
                m3Enabled = false;
                ballCount = 3;
            }
        }

        // Update previous states
        prevSensor0 = sensor0State;
        prevSensor1 = sensor1State;
        prevSensor2 = sensor2State;
    }

    // =============================================
    // PUBLIC CONTROL METHODS
    // =============================================

    /**
     * Run intake forward at configured speeds.
     * Motors are enabled based on current ballCount to prevent re-enabling
     * motors for positions where balls already exist.
     */
    public void intakeForward() {
        // Cancel any shooting states
        singleBallFeedActive = false;
        multiSingleShotActive = false;
        shooting = false;
        shootSequenceActive = false;
        shotInProgress = false;
        
        // CRITICAL: Reset prevSensor states to "clear" (true)
        // This allows us to detect balls that are ALREADY present
        // (e.g., a ball that slipped in during shooting while isIntaking was false)
        prevSensor0 = true;
        prevSensor1 = true;
        prevSensor2 = true;
        
        // Enable motors ONLY for positions where we don't expect balls
        // This prevents re-enabling a motor that should be off
        m3Enabled = ballCount < 1;  // Only enable if 0 balls
        m2Enabled = ballCount < 2;  // Only enable if 0-1 balls
        m1Enabled = ballCount < 3;  // Only enable if 0-2 balls
        
        isIntaking = true;
        currentDirection = 1.0;
        setIntakeDirection(1.0, false);
    }

    public int getNumberOfBalls() {
        return this.ballCount;
    }

    /**
     * Run intake in reverse at configured speeds.
     * Does NOT enable all motors - only spins motors the robot thinks are empty.
     * This allows ejecting untracked balls while keeping tracked balls in place.
     */
    public void intakeReverse() {
        // Cancel shooting states but DON'T enable all motors
        singleBallFeedActive = false;
        multiSingleShotActive = false;
        shooting = false;
        
        // DON'T reset m*Enabled flags - we want to keep balls where we think they are
        // Only motors that are enabled will spin (where we think there's no ball)
        
        isIntaking = true;
        currentDirection = -1.0;
        setIntakeDirection(-1.0, false);
    }

    /**
     * Just run all motors/servos at shoot speed, ignoring ball counts or time limits.
     * Resets ball count and motor enables so next intakeForward() starts clean.
     */
    public void dumbShoot() {
        isIntaking = false;
        singleBallFeedActive = false;
        multiSingleShotActive = false;
        shooting = false;
        
        m1.setPower(1.0);
        m3.setPower(1.0);
        s2.setPower(1.0);
        s3.setPower(1.0);
        
        ballCount = 0;
        
        // Reset motor enables so next intakeForward() starts clean
        // (These will be properly set based on ballCount=0 in intakeForward)
        m1Enabled = true;
        m2Enabled = true;
        m3Enabled = true;
        
        // Reset prev sensor states so next intake detects any stuck balls
        prevSensor0 = true;
        prevSensor1 = true;
        prevSensor2 = true;
    }

    /**
     * Stop all intake motors and servos.
     */
    public void stop() {
        isIntaking = false;
        m1.setPower(0.0);
        m3.setPower(0.0);
        s2.setPower(0.0);
        s3.setPower(0.0);
    }

    /**
     * Call this after shooting is complete and motors are stopped.
     * Checks if balls are present in the CORRECT stacked positions.
     * Only INCREASES ball count (safe direction) - never decreases.
     * 
     * Requires proper stacking order to detect:
     * - 1 ball: sensor2 broken (position 1)
     * - 2 balls: sensor2 AND sensor1 broken (positions 1 and 2)
     * - 3 balls: all three sensors broken
     * 
     * If a ball is in the wrong position (e.g., sensor1 only), it won't be
     * detected here. When intake starts, all motors will enable and push
     * the ball to the correct position for normal detection to work.
     */
    public void validateBallCountAfterShoot() {
        // Only run when we're truly idle
        if (isIntaking || singleBallFeedActive || multiSingleShotActive || shooting) {
            return;
        }
        
        boolean s0 = isSensor0Broken();
        boolean s1 = isSensor1Broken();
        boolean s2 = isSensor2Broken();
        
        // Check for 3 balls in correct positions (all sensors broken)
        if (s0 && s1 && s2 && ballCount < 3) {
            ballCount = 3;
            m1Enabled = false;
            m2Enabled = false;
            m3Enabled = false;
        }
        // Check for 2 balls in correct positions (sensor2 AND sensor1 broken)
        else if (s1 && s2 && ballCount < 2) {
            ballCount = 2;
            m2Enabled = false;
            m3Enabled = false;
        }
        // Check for 1 ball in correct position (sensor2 broken)
        else if (s2 && ballCount < 1) {
            ballCount = 1;
            m3Enabled = false;
        }
        // If balls are in wrong positions (e.g., s1 only, or s0 only),
        // don't update ballCount - let intake push them to correct spots
    }

    /**
     * Start shooting sequence - automatically shoots all balls currently in the robot.
     * Uses ballCount to determine how many shots to fire (more efficient).
     *
     * @return true if shoot sequence started, false if already shooting or no balls to shoot
     */
    public boolean shoot(long shotDuration, long delay) {
        this.SHOT_DELAY_MS = delay;
        this.SHOT_TIME = shotDuration;
        if (shootSequenceActive) return false;
        if (ballCount <= 0) return false;

        shotsToFire = Math.min(ballCount, MAX_SHOTS_PER_SEQUENCE);

        m1Enabled = false;
        m2Enabled = false;
        m3Enabled = false;

        shootSequenceActive = true;
        shooting = false;
        shotInProgress = false; // start as false
        currentShot = 0;
        nextShotTime = System.currentTimeMillis() + shotDuration; // start immediately
        shootTimer.reset();

        // Enable motors again for safety
        m1Enabled = true;
        m2Enabled = true;
        m3Enabled = true;

        startSingleShot();

        return true;
    }

    /**
     * Begin advancing exactly one ball at 100% power.
     * This is intended for testing / tuning from TeleOp: the caller should
     * ensure the shooter is at speed before invoking.
     *
     * The feed will automatically stop once sensor2 transitions from broken to
     * clear, and ballCount will be decremented by 1.
     *
     * @return true if the single-ball feed was started, false if already busy
     *         or there are no balls to advance.
     */
    public boolean feedSingleBallFullPower() {
        // Don't interfere with existing shooting sequence or another single feed
        if (shootSequenceActive || singleBallFeedActive) {
            return false;
        }

        if (ballCount <= 0) {
            return false;
        }

        // Ensure regular intake logic is not running
        isIntaking = false;
        shooting = false;

        // Start full-power feed
        singleBallFeedActive = true;
        singleBallFeedStartTimeMs = System.currentTimeMillis();  // Record start time for timeout
        prevSensor2BrokenForSingleFeed = isSensor2Broken();

        m1.setPower(M1_SINGLE_SHOT_POWER);
        m3.setPower(M3_SINGLE_SHOT_POWER);
        s2.setPower(S2_SINGLE_SHOT_POWER);
        s3.setPower(S3_SINGLE_SHOT_POWER);

        return true;
    }

    /**
     * Begin a multi-single-shot sequence: feed up to {@code shots} balls using the
     * same beam-break-based stop logic as {@link #feedSingleBallFullPower()}, with
     * a configurable delay between each shot.
     *
     * This does NOT restart automatically once done; callers should ensure the
     * shooter is at speed before invoking.
     *
     * @param shots number of balls to attempt to feed (clamped to available balls)
     * @return true if the sequence was started, false otherwise
     */
    public boolean shootMultipleSingleShots(int shots) {
        if (shots <= 0) {
            return false;
        }

        // Don't interfere with other shooting/feeding modes
        if (shootSequenceActive || singleBallFeedActive || multiSingleShotActive) {
            return false;
        }

        if (ballCount <= 0) {
            return false;
        }
        isIntaking = false;
        multiSingleShotRequested = Math.min(shots, ballCount);
        multiSingleShotCompleted = 0;
        multiSingleShotActive = true;
        nextMultiSingleShotStartTimeMs = 0L;

        // Kick off the first shot immediately
        boolean started = feedSingleBallFullPower();
        if (!started) {
            multiSingleShotActive = false;
            return false;
        }

        return true;
    }

    /**
     * Manually toggle motor enable states (for testing).
     */
    public void toggleMotor1() { m1Enabled = !m1Enabled; }
    public void toggleMotor2() { m2Enabled = !m2Enabled; }
    public void toggleMotor3() { m3Enabled = !m3Enabled; }

    // =============================================
    // GETTERS
    // =============================================

    public int getBallCount() { return ballCount; }

    public void setBallCount(int count) {
        // Clamp between 0 and 3
        ballCount = Math.max(0, Math.min(3, count));
        // Reflect existing balls by disabling the corresponding stages
        // ball 1 present => disable m3; ball 2 => disable m2; ball 3 => disable m1
        m3Enabled = ballCount < 1;
        m2Enabled = ballCount < 2;
        m1Enabled = ballCount < 3;
    }

    private void attemptNextShotOrFinish() {
        if (currentShot < shotsToFire) {
            startSingleShot();
        } else {
            // All shots fired
            shootSequenceActive = false;
            currentShot = 0;
            shotsToFire = 0;
        }
    }

    public boolean isShooting() { return shooting; }
    public boolean isShootSequenceActive() { return shootSequenceActive; }
    public int getCurrentShot() { return currentShot; }
    public int getShotsToFire() { return shotsToFire; }  // Total shots planned for current sequence
    public int getMaxShots() { return MAX_SHOTS_PER_SEQUENCE; }
    public boolean isMotor1Enabled() { return m1Enabled; }
    public boolean isMotor2Enabled() { return m2Enabled; }
    public boolean isMotor3Enabled() { return m3Enabled; }

    // Debug getters for state tracking
    public boolean isIntakingActive() { return isIntaking; }
    public boolean isSingleBallFeedActive() { return singleBallFeedActive; }
    public boolean isMultiSingleShotActive() { return multiSingleShotActive; }
    public boolean isShotInProgress() { return shotInProgress; }
    public double getCurrentDirection() { return currentDirection; }
    public int getMultiSingleShotRequested() { return multiSingleShotRequested; }
    public int getMultiSingleShotCompleted() { return multiSingleShotCompleted; }

    public boolean isSensor0Broken() { return !sensor0.getState(); }
    public boolean isSensor1Broken() { return !sensor1.getState(); }
    public boolean isSensor2Broken() { return !sensor2.getState(); }

    /**
     * Check if shoot sequence is available.
     * @return true if shoot() can be called, false if already shooting
     */
    public boolean canShoot() {
        return !shootSequenceActive;
    }

    /**
     * Cancel the current shooting sequence (emergency stop).
     */
    public void cancelShootSequence() {
        shootSequenceActive = false;
        shooting = false;
        currentShot = 0;
        shotsToFire = 0;
        stop();
    }

    public double getMotor1RPM() { return m1.getVelocity() * 60.0 / m1TicksPerRev; }
    public double getMotor3RPM() { return m3.getVelocity() * 60.0 / m3TicksPerRev; }

    // =============================================
    // HELPER METHODS
    // =============================================

    private void setIntakeDirection(double direction, boolean isShootMode) {
        if (isShootMode) {
            // Shooting mode handled by shoot() method
            return;
        }

        // Calculate velocities
        double m1Velocity = direction * rpmToTicksPerSecond(M1_TARGET_RPM, m1TicksPerRev);
        double m3Velocity = direction * rpmToTicksPerSecond(M3_TARGET_RPM, m3TicksPerRev);

        // Set motors based on enabled state
        if (m1Enabled) {
            m1.setVelocity(m1Velocity);
        } else {
            m1.setPower(0.0);
        }

        if (m2Enabled) {
            s2.setPower(direction * S2_INTAKE_SPEED);
            s3.setPower(direction * S3_INTAKE_SPEED);
        } else {
            s2.setPower(0.0);
            s3.setPower(0.0);
        }

        if (m3Enabled) {
            m3.setVelocity(m3Velocity);
        } else {
            m3.setPower(0.0);
        }
    }

    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }

    private boolean isShooterAtSpeed() {
        double rpm = ShooterSubsystem.INSTANCE.getShooter1RPM();
        double target = ShooterSubsystem.INSTANCE.getTargetShooterRPM();
        return Math.abs(rpm - target) < 50; // tolerance in RPM
    }
}



