package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Shooter Subsystem with Velocity Control
 *
 * Hardware:
 * - m1, m2: Shooter motors (flywheels)
 * - m3: Counter roller motor
 *
 * This subsystem controls shooting mechanism with velocity control for consistent performance.
 *
 * Future: Can be enhanced with PIDF control using NextFTC's built-in controllers.
 */
@Configurable
public class ShooterSubsystem implements Subsystem {

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
    private ShooterSubsystem() {}

    // =============================================
    // CONFIGURABLE CONSTANTS
    // =============================================

    // Shooter RPM targets
    public static double HIGH_TARGET_RPM = 2000.0;
    public static double LOW_TARGET_RPM = 1300.0;

    // Counter roller RPM targets
    public static double CR_HIGH_TARGET_RPM = 900.0;
    public static double CR_LOW_TARGET_RPM = 500.0;
    public static double SHOOTER_P = 0.005;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0.00005;
    public static double SHOOTER_F = 0.00006;

    // Motor constants
    private static double SHOOTER_GEAR_RATIO = (17.0/23.0);  // Yellow Jacket 6000 RPM geared to 17/23
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;

    private double targetRPM = 0;

    // =============================================
    // HARDWARE
    // =============================================

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private Servo shooterHood;
    //private DcMotorEx counterRoller;

    private double ticksPerRev;

    // =============================================
    // STATE TRACKING
    // =============================================

    private boolean enabled = false;
    private boolean highMode = true;  // true = high speed, false = low speed
    private boolean crHighMode = true;

    private PIDFController shooterPID1;
    private PIDFController shooterPID2;

    public static PIDFCoefficients SHOOTER_PID_COEFFS = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

//    @Configurable
//    public static double kP = 0.005;
//    @Configurable
//    public static double kI = 0.0;
//    @Configurable
//    public static double kD = 0.0005;
//
//    @Configurable
//    public static double kF = 0.0005;



    // =============================================
    // INITIALIZATION
    // =============================================
    @Override
    public void initialize() {
        // Initialize motors
        shooter1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter_motor1");
        shooter2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter_motor2");
        shooterHood = ActiveOpMode.hardwareMap().get(Servo.class, "shooter_hood");
        //counterRoller = hardwareMap.get(DcMotorEx.class, "m3");

        // Configure motors
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //counterRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //counterRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        //counterRoller.setDirection(DcMotorSimple.Direction.FORWARD);

        // Calculate ticks per revolution
        ticksPerRev = ENCODER_TICKS_PER_MOTOR_REV * SHOOTER_GEAR_RATIO;

//        shooter1.setVelocityPIDFCoefficients(5., 0, 0.005, 17.85);
//        shooter2.setVelocityPIDFCoefficients(5, 0, 0.0005, 17.85);

        shooterPID1 = new PIDFController(SHOOTER_PID_COEFFS);
        shooterPID2 = new PIDFController(SHOOTER_PID_COEFFS);

        shooterPID1.reset();
        shooterPID2.reset();
    }

    // =============================================
    // PERIODIC - Could add PIDF control here later
    // =============================================

    @Override
    public void periodic() {
        // Update motor velocities if enabled
        SHOOTER_PID_COEFFS.setCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        this.shooterPID1.updatePosition(this.shooter1.getVelocity());
        this.shooterPID2.updatePosition(this.shooter2.getVelocity());
        if (enabled) {
            updateVelocities();
        }
    }

    // =============================================
    // PUBLIC CONTROL METHODS
    // =============================================

    /**
     * Turn on the shooter at current speed mode.
     */
    public void spinUp() {
        enabled = true;
        updateVelocities();
    }

//    public void updatePIDCoeffs() {
//        shooterPID1.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
//        shooterPID2.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
//    }

    /**
     * Turn on the shooter at specific RPM.
     * shooterRPM Target RPM for shooter motors
     * counterRollerRPM Target RPM for counter roller
     */
    public void spinUp(double targetRPM) {
        this.targetRPM = targetRPM;
        enabled = true;

        updateVelocities();
    }

    /**
     * Stop all shooter motors.
     */
    public void stop() {
        enabled = false;
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
    }

    /**
     * Set to high speed mode.
     */
    public void setHighSpeed() {
        highMode = true;
        crHighMode = true;
        if (enabled) updateVelocities();
    }

    /**
     * Set to low speed mode.
     */
    public void setLowSpeed() {
        highMode = false;
        crHighMode = false;
        if (enabled) updateVelocities();
    }

    /**
     * Toggle shooter on/off.
     */
    public void toggle() {
        if (enabled) {
            stop();
        } else {
            spinUp();
        }
    }

    // =============================================
    // GETTERS
    // =============================================

    public boolean isEnabled() { return enabled; }
    public boolean isHighMode() { return highMode; }

    public double getShooter1Power() {return shooter1.getPower(); }
    public double getShooter2Power() {return shooter2.getPower(); }
    public double getShooter1RPM() { return shooter1.getVelocity() * 60.0 / ticksPerRev; }
    public double getShooter2RPM() { return shooter2.getVelocity() * 60.0 / ticksPerRev; }
    //public double getCounterRollerRPM() { return counterRoller.getVelocity() * 60.0 / ticksPerRev; }

    public double getShooterHoodPosition() { return this.shooterHood.getPosition(); }

    public void driveShooterHood(double joystick) {
        double currentPosition = shooterHood.getPosition();
        double increment = -joystick * 0.01;  // adjust sensitivity here
        double newPosition = Range.clip(currentPosition + increment, 0.0, 1.0);
        shooterHood.setPosition(newPosition);
    }
    public void shooterHoodDrive(double hoodPosition){
        this.shooterHood.setPosition(hoodPosition);
    }

    public double getTargetShooterRPM() {
        return this.targetRPM;
    }

    public double getTargetCounterRollerRPM() {
        return crHighMode ? CR_HIGH_TARGET_RPM : CR_LOW_TARGET_RPM;
    }

    public void increaseShooterRPMBy10() {
        enabled = true;
        updateVelocities();
        if (this.targetRPM < 3000) {
            this.targetRPM = 3000;
        }
        else {
            this.targetRPM = this.targetRPM + 10;
        }
    }
    public void decreaseShooterRPMBy10() {
        enabled = true;
        updateVelocities();
        if (this.targetRPM < 3000) {
            this.targetRPM = 3000;
        }
        else {
            this.targetRPM = this.targetRPM - 10;
        }
    }

    public void increaseShooterHoodPosInc() {
        double curPos = this.shooterHood.getPosition();
        this.shooterHood.setPosition(curPos + 0.1);
    }
    public void decreaseShooterHoodPosInc() {
        double curPos = this.shooterHood.getPosition();
        this.shooterHood.setPosition(curPos - 0.1);
    }

    /**
     * Check if shooters are at target speed (within tolerance).
     * @param toleranceRPM Acceptable RPM difference
     * @return True if both shooters are within tolerance
     */
    public boolean isAtTargetSpeed(double toleranceRPM) {
        double targetShooter = getTargetShooterRPM();
        boolean s1Good = Math.abs(getShooter1RPM() - targetShooter) < toleranceRPM;
        boolean s2Good = Math.abs(getShooter2RPM() - targetShooter) < toleranceRPM;
        return s1Good && s2Good;
    }

    // =============================================
    // HELPER METHODS
    // =============================================

//    private void updateVelocities() {
//        double shooterTarget = this.targetRPM;
//
//        double shooterTps = rpmToTicksPerSecond(shooterTarget);
//
//        shooter1.setVelocity(shooterTps);
//        shooter2.setVelocity(shooterTps);
//        //counterRoller.setVelocity(crTps);
//        shooter1.setPower(1.0);
//        shooter2.setPower(1.0);
//
//    }

    private void updateVelocities() {
        double shooterTargetRPM = targetRPM;

        // Current feedback (in RPM)
        double shooter1RPM = getShooter1RPM();
        double shooter2RPM = getShooter2RPM();
        double rpm = rpmToTicksPerSecond(shooterTargetRPM);

        // Compute PID output
        shooterPID1.setTargetPosition(rpm);
        shooterPID2.setTargetPosition(rpm);

        double power1 = shooterPID1.run();
        double power2 = shooterPID1.run();

        // Clip power to motor-safe range [-1, 1]
        power1 = Range.clip(power1, 0, 1);
        power2 = Range.clip(power2, 0, 1);

        shooter1.setPower(power1);
        shooter2.setPower(power2);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * ticksPerRev) / 60.0;
    }

    public boolean getEnabled() {
        return this.enabled;
    }


}

/*
 * NOTES FOR FUTURE PIDF ENHANCEMENT:
 *
 * To add NextFTC's PIDF control (future enhancement):
 *
 * 1. Add PIDFController field:
 *    private PIDFController shooterPID;
 *
 * 2. Initialize in initialize():
 *    shooterPID = new PIDFController(kP, kI, kD, kF);
 *    shooterPID.setTolerance(50); // RPM tolerance
 *
 * 3. Update periodic() to use PIDF:
 *    if (enabled) {
 *        double currentRPM = getShooter1RPM();
 *        double output = shooterPID.calculate(currentRPM, targetRPM);
 *        shooter1.setPower(output);
 *        // same for shooter2
 *    }
 *
 * 4. Add isAtTarget() check:
 *    return shooterPID.atSetpoint();
 *
 * This would give much better control than built-in velocity control!
 */

