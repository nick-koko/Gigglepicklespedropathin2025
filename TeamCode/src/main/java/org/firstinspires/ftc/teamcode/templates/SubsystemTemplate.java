package org.firstinspires.ftc.teamcode.templates;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Template for creating a new subsystem.
 * 
 * HOW TO USE THIS TEMPLATE:
 * 1. Copy this file and rename it (e.g., ClawSubsystem.java)
 * 2. Rename the class to match the filename
 * 3. Update the package if needed
 * 4. Declare your hardware devices as private fields
 * 5. Initialize hardware in initialize()
 * 6. Create public methods for controlling the mechanism
 * 7. Add any configurable constants as public static fields
 * 8. Optionally add telemetry in periodic()
 * 
 * EXAMPLE: See IntakeSubsystem.java and FlywheelSubsystem.java
 */
public class SubsystemTemplate implements Subsystem {

    // ====================
    // HARDWARE DECLARATION
    // ====================
    // Declare your motors, servos, sensors, etc. as private fields
    // Examples:
    private DcMotor motor;
    // private Servo servo;
    // private DistanceSensor sensor;
    // private ColorSensor colorSensor;

    // ====================
    // CONFIGURABLE CONSTANTS
    // ====================
    // Make these public static so they can be adjusted from your OpMode or Dashboard
    // Examples:
    public static double MOTOR_POWER = 1.0;
    public static double SERVO_OPEN_POSITION = 0.8;
    public static double SERVO_CLOSED_POSITION = 0.2;

    // ====================
    // STATE VARIABLES
    // ====================
    // Track internal state if needed
    // Examples:
    // private boolean isOpen = false;
    // private double targetPosition = 0.0;

    /**
     * Initialize hardware devices.
     * Call this method from your OpMode's onInit().
     * 
     * @param hardwareMap The hardware map from the OpMode
     */
    public void initialize(HardwareMap hardwareMap) {
        // Initialize your hardware here
        // Use the hardware names from your Robot Configuration
        motor = hardwareMap.get(DcMotor.class, "motor_name");
        // servo = hardwareMap.get(Servo.class, "servo_name");
        // sensor = hardwareMap.get(DistanceSensor.class, "sensor_name");

        // Configure your hardware
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.FORWARD);
        // servo.setDirection(Servo.Direction.FORWARD);
    }

    // ====================
    // CONTROL METHODS
    // ====================
    // Create public methods that commands will call to control the mechanism
    // Keep these simple - just control the hardware, don't implement complex logic

    /**
     * Example: Run the motor at full power.
     */
    public void run() {
        motor.setPower(MOTOR_POWER);
    }

    /**
     * Example: Stop the motor.
     */
    public void stop() {
        motor.setPower(0.0);
    }

    /**
     * Example: Set custom power.
     * @param power Power level between -1.0 and 1.0
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    // /**
    //  * Example: Open a servo.
    //  */
    // public void open() {
    //     servo.setPosition(SERVO_OPEN_POSITION);
    //     isOpen = true;
    // }
    //
    // /**
    //  * Example: Close a servo.
    //  */
    // public void close() {
    //     servo.setPosition(SERVO_CLOSED_POSITION);
    //     isOpen = false;
    // }

    // ====================
    // SENSOR/STATE GETTERS
    // ====================
    // Create methods to read sensors or get state information
    // Commands can use these to check conditions

    /**
     * Example: Get current motor power.
     * @return Current power level
     */
    public double getPower() {
        return motor.getPower();
    }

    // /**
    //  * Example: Check if mechanism is in the desired state.
    //  * @return True if ready
    //  */
    // public boolean isReady() {
    //     return Math.abs(motor.getCurrentPosition() - targetPosition) < 10;
    // }
    //
    // /**
    //  * Example: Get distance from sensor.
    //  * @return Distance in cm
    //  */
    // public double getDistance() {
    //     return sensor.getDistance(DistanceUnit.CM);
    // }

    /**
     * Periodic method called automatically by NextFTC every loop.
     * Use this for telemetry updates or continuous state monitoring.
     * 
     * Optional - you can leave this empty if not needed.
     */
    @Override
    public void periodic() {
        // Optional: Add telemetry for debugging
        // telemetry.addData(getName() + " Power", getPower());
        
        // Optional: Update state based on sensors
        // currentState = sensor.getValue();
    }
}

