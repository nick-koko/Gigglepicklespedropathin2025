package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

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
    public static double M2_TARGET_RPM = 350.0;
    public static double M3_TARGET_RPM = 500.0;

    // Shoot RPM targets
    public static double M1_SHOOT_RPM = 500.0;
    //public static double M2_SHOOT_RPM = 200.0;
    public static double M3_SHOOT_RPM = 1000.0;

    // Continuous servo speeds
    public static double S2_INTAKE_SPEED = 0.7;
    public static double S3_INTAKE_SPEED = 0.7;
    public static double S2_SHOOT_SPEED = 0.5;
    public static double S3_SHOOT_SPEED = 0.5;

    // Shooting timing (in milliseconds)
    public static long SHOOT_DURATION_MS = 250;      // How long to run motors for each shot
    public static long DELAY_BETWEEN_SHOTS_MS = 200;  // Delay between sequential shots
    public static int MAX_SHOTS_PER_SEQUENCE = 3;     // Number of balls to shoot per button press

    // Motor constants
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double M1_GEAR_RATIO = 4;
    private static final double M2_GEAR_RATIO = 13.7;
    private static final double M3_GEAR_RATIO = 4;

    // =============================================
    // HARDWARE
    // =============================================

    private DcMotorEx m1, m2, m3;
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
    private double m2TicksPerRev;
    private double m3TicksPerRev;

    // Intake run state
    private boolean isIntaking = false;
    private double currentDirection = 0.0; // +1 forward, -1 reverse

    // Lights
    //private DigitalChannel light1, light2;

    // Shooter recovery threshold (% of target speed)
    private static final double SHOOTER_READY_THRESHOLD = 0.95;  // 95% of target speed
    private boolean waitingForShooterSpeed = false;
    private boolean hasBeenBelowSpeed = true;
    private double shooterRpm = 0;
    private double shooterGoal = 0;

    private static final long SINGLE_SHOT_DURATION_MS = 33;   // how long feeder runs to shoot one ball
    private static final long MAX_WAIT_FOR_DROP_MS = 100;// small grace window (tweak as needed)

    private long waitForDropStartTime = 0;

    // class-level fields
    private double rpmAtShotEnd = 0.0;

    private static final double DROP_THRESHOLD_PERCENT = 0.95; // how much RPM mustop

    private long spinCheckStartTime = 0;
    private static final long MIN_SPIN_WAIT_MS = 250;       // minimum time before we can fire next
    private static final long MAX_SPIN_WAIT_MS = 800;       // safety timeout to never stall
    private static final int SPEED_TOLERANCE_RPM = 50;      // acceptable speed window




    // =============================================
    // INITIALIZATION
    // =============================================

    public void initialize(HardwareMap hardwareMap) {
        // Initialize motors
        m1 = hardwareMap.get(DcMotorEx.class, "intake_motor");
        //m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "intake_motor2");

        // Initialize servos
        s2 = hardwareMap.get(CRServo.class, "intake_servo1");
        s3 = hardwareMap.get(CRServo.class, "intake_servo2");

        // Initialize sensors
        sensor0 = hardwareMap.get(DigitalChannel.class, "breakbeam0");
        sensor1 = hardwareMap.get(DigitalChannel.class, "breakbeam1");
        sensor2 = hardwareMap.get(DigitalChannel.class, "breakbeam2");

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
        //m2TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M2_GEAR_RATIO;
        m3TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M3_GEAR_RATIO;



        // class-level field


//        light1 = hardwareMap.get(DigitalChannel.class, "light1");
//        light2 = hardwareMap.get(DigitalChannel.class, "light2");
//
//        light1.setMode(DigitalChannel.Mode.OUTPUT);
//        light2.setMode(DigitalChannel.Mode.OUTPUT);
    }

    // =============================================
    // PERIODIC - Runs automatically every loop!
    // =============================================

    @Override
    public void periodic() {
        if (isIntaking) {
            checkSensorsAndUpdateMotors();
            setIntakeDirection(currentDirection, shooting);
        }
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
     */
    public void intakeForward() {
        isIntaking = true;
        currentDirection = 1.0;
        setIntakeDirection(1.0, false);
    }

    /**
     * Run intake in reverse at configured speeds.
     */
    public void intakeReverse() {
        isIntaking = true;
        currentDirection = -1.0;
        setIntakeDirection(-1.0, false);
    }

    /**
     * Stop all intake motors and servos.
     */
    public void stop() {
        isIntaking = false;
        m1.setPower(0.0);
        //m2.setPower(0.0);
        m3.setPower(0.0);
        s2.setPower(0.0);
        s3.setPower(0.0);
    }

    /**
     * Start shooting sequence - automatically shoots all balls currently in the robot.
     * Uses ballCount to determine how many shots to fire (more efficient).
     *
     * @return true if shoot sequence started, false if already shooting or no balls to shoot
     */
    public boolean shoot() {
        // Prevent starting new sequence if already shooting
        if (shootSequenceActive) {
            return false;
        }

        // Check if there are any balls to shoot
        if (ballCount <= 0) {
            return false;  // No balls to shoot
        }

        // Determine how many shots to fire based on ball count (max is MAX_SHOTS_PER_SEQUENCE)
        shotsToFire = Math.min(ballCount, MAX_SHOTS_PER_SEQUENCE);

        // Start the shooting sequence
        shootSequenceActive = true;
        currentShot = 0;
        shootTimer.reset();

        // Re-enable all motors for sequence
        m1Enabled = true;
        m2Enabled = true;
        m3Enabled = true;

        // Start first shot immediately
        startSingleShot();

        // Reset ball count since we're shooting them all
        ballCount--;

        return true;
    }

    /**
     * Internal method to start a single shot within the sequence.
     */
    private void startSingleShot() {
        currentShot++;
        shooting = true;

        // Run feeder/indexer motors only
        s2.setPower(S2_SHOOT_SPEED);
        s3.setPower(S3_SHOOT_SPEED);
        m3.setVelocity(rpmToTicksPerSecond(M3_SHOOT_RPM, m3TicksPerRev));
        m1.setVelocity(rpmToTicksPerSecond(M3_SHOOT_RPM, m3TicksPerRev));
        shootEndTime = System.currentTimeMillis() + SHOOT_DURATION_MS;

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
    //public double getMotor2RPM() { return m2.getVelocity() * 60.0 / m2TicksPerRev; }
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
        //double m2Velocity = direction * rpmToTicksPerSecond(M2_TARGET_RPM, m2TicksPerRev);
        double m3Velocity = direction * rpmToTicksPerSecond(M3_TARGET_RPM, m3TicksPerRev);

        // Set motors based on enabled state
        if (m1Enabled) m1.setVelocity(m1Velocity); else m1.setPower(0.0);

        if (m2Enabled) {
            //m2.setVelocity(m2Velocity);
            s2.setPower(direction * S2_INTAKE_SPEED);
            s3.setPower(direction * S3_INTAKE_SPEED);
        } else {
            //m2.setPower(0.0);
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

