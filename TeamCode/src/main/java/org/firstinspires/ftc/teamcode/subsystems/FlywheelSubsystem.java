package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * NextFTC Subsystem for controlling flywheel shooter mechanism.
 * This subsystem uses velocity control for consistent shooting performance.
 */
public class FlywheelSubsystem implements Subsystem {

    private DcMotorEx flywheelMotor;

    // Configurable constants
    public static double ENCODER_TICKS_PER_MOTOR_REV = 28.0; // Yellow Jacket default
    public static double GEAR_RATIO = 1.0; // Adjust based on your gear ratio
    public static double HIGH_TARGET_RPM = 3000.0;
    public static double LOW_TARGET_RPM = 2000.0;

    private double targetVelocity = 0.0;
    private double ticksPerRev;

    public void initialize(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        
        // Configure motor
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Calculate ticks per revolution based on gear ratio
        ticksPerRev = ENCODER_TICKS_PER_MOTOR_REV * GEAR_RATIO;
    }

    /**
     * Set the flywheel to a specific RPM using velocity control.
     * @param rpm Target RPM for the flywheel
     */
    public void setRPM(double rpm) {
        targetVelocity = rpmToTicksPerSecond(rpm);
        flywheelMotor.setVelocity(targetVelocity);
    }

    /**
     * Set the flywheel to the high shooting speed.
     */
    public void setHighSpeed() {
        setRPM(HIGH_TARGET_RPM);
    }

    /**
     * Set the flywheel to the low shooting speed.
     */
    public void setLowSpeed() {
        setRPM(LOW_TARGET_RPM);
    }

    /**
     * Set flywheel power directly (0.0 to 1.0).
     * Note: Using velocity control (setRPM) is preferred for consistent shooting.
     * @param power Power level between 0.0 and 1.0
     */
    public void setPower(double power) {
        flywheelMotor.setPower(power);
        targetVelocity = 0.0; // Clear target velocity when using power control
    }

    /**
     * Stop the flywheel.
     */
    public void stop() {
        flywheelMotor.setPower(0.0);
        targetVelocity = 0.0;
    }

    /**
     * Get the current RPM of the flywheel.
     * @return Current RPM
     */
    public double getCurrentRPM() {
        return flywheelMotor.getVelocity() * 60.0 / ticksPerRev;
    }

    /**
     * Get the target RPM.
     * @return Target RPM
     */
    public double getTargetRPM() {
        return targetVelocity * 60.0 / ticksPerRev;
    }

    /**
     * Check if the flywheel is at the target speed (within tolerance).
     * @param toleranceRPM Acceptable RPM difference
     * @return True if within tolerance
     */
    public boolean isAtTargetSpeed(double toleranceRPM) {
        return Math.abs(getCurrentRPM() - getTargetRPM()) < toleranceRPM;
    }

    /**
     * Convert RPM to ticks per second for velocity control.
     */
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * ticksPerRev) / 60.0;
    }

    @Override
    public void periodic() {
        // Optional: Add telemetry for debugging
        // telemetry.addData("Flywheel RPM", getCurrentRPM());
        // telemetry.addData("Target RPM", getTargetRPM());
    }
}

