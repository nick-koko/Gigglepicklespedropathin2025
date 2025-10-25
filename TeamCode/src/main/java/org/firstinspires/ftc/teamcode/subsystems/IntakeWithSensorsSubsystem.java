package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    // =============================================
    // CONFIGURABLE CONSTANTS
    // =============================================
    
    // Intake RPM targets
    public static double M1_TARGET_RPM = 800.0;
    public static double M2_TARGET_RPM = 350.0;
    public static double M3_TARGET_RPM = 400.0;

    // Shoot RPM targets
    public static double M1_SHOOT_RPM = 200.0;
    public static double M2_SHOOT_RPM = 200.0;
    public static double M3_SHOOT_RPM = 200.0;

    // Continuous servo speeds
    public static double S2_INTAKE_SPEED = 0.8;
    public static double S3_INTAKE_SPEED = 0.8;
    public static double S2_SHOOT_SPEED = 0.5;
    public static double S3_SHOOT_SPEED = 0.5;

    // Motor constants
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    private static final double M1_GEAR_RATIO = 3.7;
    private static final double M2_GEAR_RATIO = 13.7;
    private static final double M3_GEAR_RATIO = 13.7;

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
    private long shootEndTime = 0;
    
    // Sensor state tracking for edge detection
    private boolean prevSensor0 = true;
    private boolean prevSensor1 = true;
    private boolean prevSensor2 = true;
    
    // Ticks per revolution calculations
    private double m1TicksPerRev;
    private double m2TicksPerRev;
    private double m3TicksPerRev;

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
        sensor0 = hardwareMap.get(DigitalChannel.class, "sensor0");
        sensor1 = hardwareMap.get(DigitalChannel.class, "sensor1");
        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");

        // Configure motors
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        //m2.setDirection(DcMotorSimple.Direction.FORWARD);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure servos
        s2.setDirection(DcMotorSimple.Direction.FORWARD);
        s3.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure sensors
        sensor0.setMode(DigitalChannel.Mode.INPUT);
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        sensor2.setMode(DigitalChannel.Mode.INPUT);

        // Calculate ticks per revolution
        m1TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M1_GEAR_RATIO;
        //m2TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M2_GEAR_RATIO;
        m3TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M3_GEAR_RATIO;
    }

    // =============================================
    // PERIODIC - Runs automatically every loop!
    // =============================================
    
    @Override
    public void periodic() {
        // Check sensors and update motor states
        checkSensorsAndUpdateMotors();
        
        // Check if shooting time is over
        if (shooting && System.currentTimeMillis() >= shootEndTime) {
            shooting = false;
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
        setIntakeDirection(1.0, false);
    }

    /**
     * Run intake in reverse at configured speeds.
     */
    public void intakeReverse() {
        setIntakeDirection(-1.0, false);
    }

    /**
     * Stop all intake motors and servos.
     */
    public void stop() {
        m1.setPower(0.0);
        m2.setPower(0.0);
        m3.setPower(0.0);
        s2.setPower(0.0);
        s3.setPower(0.0);
    }

    /**
     * Start shooting mode - runs all motors at shoot speeds for 2 seconds.
     * Automatically resets ball count.
     */
    public void shoot() {
        shooting = true;
        shootEndTime = System.currentTimeMillis() + 2000;
        m1Enabled = true;
        m2Enabled = true;
        m3Enabled = true;
        ballCount = 0;
        
        // Set shoot velocities
        m1.setVelocity(rpmToTicksPerSecond(M1_SHOOT_RPM, m1TicksPerRev));
        m2.setVelocity(rpmToTicksPerSecond(M2_SHOOT_RPM, m2TicksPerRev));
        m3.setVelocity(rpmToTicksPerSecond(M3_SHOOT_RPM, m3TicksPerRev));
        s2.setPower(S2_SHOOT_SPEED);
        s3.setPower(S3_SHOOT_SPEED);
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
    public boolean isShooting() { return shooting; }
    public boolean isMotor1Enabled() { return m1Enabled; }
    public boolean isMotor2Enabled() { return m2Enabled; }
    public boolean isMotor3Enabled() { return m3Enabled; }
    
    public boolean isSensor0Broken() { return !sensor0.getState(); }
    public boolean isSensor1Broken() { return !sensor1.getState(); }
    public boolean isSensor2Broken() { return !sensor2.getState(); }
    
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
        } else {
            //m2.setPower(0.0);
            s2.setPower(0.0);
        }
        
        if (m3Enabled) {
            m3.setVelocity(m3Velocity);
            s3.setPower(direction * S3_INTAKE_SPEED);
        } else {
            m3.setPower(0.0);
            s3.setPower(0.0);
        }
    }

    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }
}

