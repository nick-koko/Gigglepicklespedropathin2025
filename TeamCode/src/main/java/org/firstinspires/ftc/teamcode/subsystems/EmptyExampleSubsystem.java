package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * ============================================================================
 * SUBSYSTEM TEMPLATE FOR STUDENTS
 * ============================================================================
 * 
 * This is a template to help you create your own subsystems!
 * Follow the steps and fill in the TODOs with your mechanism's details.
 * 
 * WHAT IS A SUBSYSTEM?
 * A subsystem is a collection of robot parts that work together for one purpose.
 * Examples: intake, shooter, lift, claw, etc.
 * 
 * SUBSYSTEM PARTS:
 * 1. CONSTANTS - Numbers that control how your mechanism works
 * 2. HARDWARE - The actual motors, servos, sensors on your robot
 * 3. INITIALIZE - Set up your hardware when the program starts
 * 4. PERIODIC - Code that runs automatically every loop (optional but powerful!)
 * 5. PUBLIC METHODS - Commands that tell your mechanism what to do
 * 6. HELPER METHODS - Private methods that help your public methods work
 * 
 * HOW TO USE THIS TEMPLATE:
 * 1. Copy this file and rename it (e.g., IntakeSubsystem.java or ShooterSubsystem.java)
 * 2. Replace "EmptyExampleSubsystem" with your new class name
 * 3. Fill in each TODO section with your mechanism's details
 * 4. Delete sections you don't need
 * 5. Test it!
 * 
 * ============================================================================
 */
public class EmptyExampleSubsystem implements Subsystem {

    // ============================================================================
    // SECTION 1: CONSTANTS - Numbers that control your mechanism
    // ============================================================================
    // These are values you can change to tune how your mechanism works.
    // Making them "public static" lets you change them from FTC Dashboard!
    
    // TODO: Add your motor speeds, servo positions, or other constants here
    // Examples:
    // public static double INTAKE_SPEED = 0.8;
    // public static double SHOOTER_RPM = 3000.0;
    // public static double SERVO_OPEN_POSITION = 0.5;
    
    // Motor encoder constants (if using velocity control)
    // private static final double TICKS_PER_REV = 28.0;  // For most FTC motors
    // private static final double GEAR_RATIO = 1.0;      // Your gear ratio (output/input)
    

    // ============================================================================
    // SECTION 2: HARDWARE - Your robot's motors, servos, sensors
    // ============================================================================
    // Declare your hardware here. Don't set them up yet - that happens in initialize()
    
    // TODO: Declare your motors, servos, sensors here
    // Examples:
    // private DcMotorEx intakeMotor;
    // private DcMotorEx shooterMotor;
    // private Servo clawServo;
    // private DigitalChannel beamBreakSensor;
    

    // ============================================================================
    // SECTION 3: STATE TRACKING (Optional but useful!)
    // ============================================================================
    // Variables that track what your mechanism is doing
    
    // TODO: Add any variables you need to track the state of your mechanism
    // Examples:
    // private boolean isRunning = false;
    // private int ballCount = 0;
    // private boolean isAtTarget = false;
    

    // ============================================================================
    // SECTION 4: INITIALIZATION - Set up your hardware
    // ============================================================================
    // This runs ONCE when your OpMode starts. Set up all your hardware here!
    
    public void initialize(HardwareMap hardwareMap) {
        // TODO: Get your hardware from the hardwareMap
        // Use the names you configured in your robot configuration
        // Example:
        // intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        
        // TODO: Configure your motors
        // Example for a motor:
        // intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // TODO: Configure your servos
        // Example:
        // clawServo.setDirection(Servo.Direction.FORWARD);
        
        // TODO: Configure your sensors (if you have any)
        // Example:
        // beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    // ============================================================================
    // SECTION 5: PERIODIC - Runs automatically every loop! (OPTIONAL BUT POWERFUL!)
    // ============================================================================
    // This method runs over and over automatically while your OpMode is running.
    // Use it for things that need to happen continuously, like:
    // - Checking sensors
    // - Updating telemetry
    // - Automatic safety checks
    // - PID control loops
    
    @Override
    public void periodic() {
        // TODO: Add any code that needs to run automatically every loop
        
        // Example: Check if a sensor detects something
        // if (beamBreakSensor.getState() == false) {
        //     // Ball detected! Do something...
        // }
        
        // Example: Update motor power based on a calculation
        // double power = calculatePower();
        // motor.setPower(power);
    }

    // ============================================================================
    // SECTION 6: PUBLIC METHODS - Commands to control your mechanism
    // ============================================================================
    // These are the methods you'll call from TeleOp or Autonomous to control
    // your mechanism. Make these simple and clear!
    
    // TODO: Add methods to control your mechanism
    
    // Example: Simple motor control
    /**
     * Start the intake motor
     */
    // public void start() {
    //     intakeMotor.setPower(INTAKE_SPEED);
    // }
    
    /**
     * Stop the intake motor
     */
    // public void stop() {
    //     intakeMotor.setPower(0.0);
    // }
    
    /**
     * Reverse the intake motor
     */
    // public void reverse() {
    //     intakeMotor.setPower(-INTAKE_SPEED);
    // }
    
    // Example: Velocity control (for flywheels)
    /**
     * Spin up the shooter to target RPM
     */
    // public void spinUp() {
    //     double ticksPerSecond = rpmToTicksPerSecond(SHOOTER_RPM);
    //     shooterMotor.setVelocity(ticksPerSecond);
    // }
    
    // Example: Servo control
    /**
     * Open the claw
     */
    // public void openClaw() {
    //     clawServo.setPosition(CLAW_OPEN_POSITION);
    // }
    
    /**
     * Close the claw
     */
    // public void closeClaw() {
    //     clawServo.setPosition(CLAW_CLOSED_POSITION);
    // }

    // ============================================================================
    // SECTION 7: GETTER METHODS - Get information about your mechanism
    // ============================================================================
    // These methods let other parts of your code check the state of your mechanism
    
    // TODO: Add methods to get information about your mechanism
    
    // Example: Check if motor is running
    // public boolean isRunning() {
    //     return intakeMotor.getPower() > 0.01;
    // }
    
    // Example: Get current RPM
    // public double getCurrentRPM() {
    //     return ticksPerSecondToRPM(shooterMotor.getVelocity());
    // }
    
    // Example: Check if at target speed
    // public boolean isAtTargetSpeed() {
    //     double currentRPM = getCurrentRPM();
    //     return Math.abs(currentRPM - SHOOTER_RPM) < 50; // Within 50 RPM
    // }
    
    // Example: Check sensor state
    // public boolean isBallDetected() {
    //     return !beamBreakSensor.getState(); // Beam break sensors are inverted
    // }

    // ============================================================================
    // SECTION 8: HELPER METHODS - Private methods to help your code
    // ============================================================================
    // These are methods that help your public methods work, but don't need to
    // be called from outside this class
    
    // Example: Convert RPM to encoder ticks per second
    // private double rpmToTicksPerSecond(double rpm) {
    //     double ticksPerRevolution = TICKS_PER_REV * GEAR_RATIO;
    //     return (rpm * ticksPerRevolution) / 60.0;
    // }
    
    // Example: Convert encoder ticks per second to RPM
    // private double ticksPerSecondToRPM(double ticksPerSecond) {
    //     double ticksPerRevolution = TICKS_PER_REV * GEAR_RATIO;
    //     return (ticksPerSecond * 60.0) / ticksPerRevolution;
    // }
    
    // Example: Calculate motor power with a formula
    // private double calculatePower() {
    //     // Your calculation here
    //     return 0.5;
    // }
}

/**
 * ============================================================================
 * QUICK TIPS FOR STUDENTS
 * ============================================================================
 * 
 * 1. START SIMPLE: Don't try to do everything at once! Start with just 
 *    controlling a motor with simple power commands.
 * 
 * 2. TEST OFTEN: Add one feature, test it, then add the next feature.
 * 
 * 3. USE CONSTANTS: Put your speeds and positions at the top as constants
 *    so you can easily change them without searching through your code.
 * 
 * 4. PERIODIC IS POWERFUL: Use periodic() for automatic behaviors like
 *    monitoring sensors or running control loops.
 * 
 * 5. GOOD METHOD NAMES: Use clear names like "start()", "stop()", "spinUp()"
 *    so it's obvious what each method does.
 * 
 * 6. COMMENTS HELP: Write comments to explain WHY you're doing something,
 *    not just WHAT you're doing.
 * 
 * 7. ASK FOR HELP: If you're stuck, ask! Subsystems can be tricky at first.
 * 
 * ============================================================================
 * COMMON PATTERNS
 * ============================================================================
 * 
 * PATTERN 1: Simple Motor Control
 * --------------------------------
 * public void forward() { motor.setPower(0.8); }
 * public void reverse() { motor.setPower(-0.8); }
 * public void stop() { motor.setPower(0.0); }
 * 
 * PATTERN 2: Velocity Control (Flywheel)
 * ---------------------------------------
 * public void spinUp() { motor.setVelocity(ticksPerSecond); }
 * public void stop() { motor.setPower(0.0); }
 * public boolean isReady() { return getCurrentRPM() >= TARGET_RPM - 50; }
 * 
 * PATTERN 3: Servo with Positions
 * --------------------------------
 * public void open() { servo.setPosition(OPEN_POS); }
 * public void close() { servo.setPosition(CLOSED_POS); }
 * 
 * PATTERN 4: Sensor-Based Automatic Control
 * ------------------------------------------
 * @Override
 * public void periodic() {
 *     if (sensor.getState() == false) {
 *         // Ball detected, stop motor
 *         motor.setPower(0.0);
 *     }
 * }
 * 
 * ============================================================================
 */

