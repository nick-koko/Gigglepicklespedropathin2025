package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Test TeleOp for Shooter Subsystem
 * 
 * This replaces ShootingTest.java but uses NextFTC subsystem architecture.
 * 
 * Controls:
 * - Left Trigger: Toggle shooter on/off
 * - X: Set high speed mode
 * - B: Set low speed mode
 * 
 * The subsystem automatically manages velocity control!
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends NextFTCOpMode {

    private ShooterSubsystem shooter;
    
    // Button state tracking
    private boolean prevX = false;
    private boolean prevB = false;
    private boolean prevLT = false;

    @Override
    public void onInit() {
        // Create and initialize subsystem
        shooter = new ShooterSubsystem();
        shooter.initialize(hardwareMap);
        
        telemetry.addLine("Shooter Test Ready!");
        telemetry.addLine("LT=toggle ON/OFF");
        telemetry.addLine("X=high speed, B=low speed");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        // =============================================
        // CONTROLS - Traditional gamepad access
        // =============================================
        
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;
        boolean ltPressed = gamepad1.left_trigger > 0.3;

        // Mode selection
        if (x && !prevX) {
            shooter.setHighSpeed();
        }
        if (b && !prevB) {
            shooter.setLowSpeed();
        }
        
        // Toggle on/off
        if (ltPressed && !prevLT) {
            shooter.toggle();
        }

        // =============================================
        // TELEMETRY
        // =============================================
        
        String mode = shooter.isEnabled() ? 
                     (shooter.isHighMode() ? "HIGH" : "LOW") : "OFF";
        
        telemetry.addData("Mode", mode);
        telemetry.addLine();
        
        telemetry.addData("Target RPM (shooters)", "%.0f", shooter.getTargetShooterRPM());
        telemetry.addData("Target RPM (counter roller)", "%.0f", shooter.getTargetCounterRollerRPM());
        telemetry.addLine();
        
        telemetry.addData("Shooter 1 RPM", "%.1f", shooter.getShooter1RPM());
        telemetry.addData("Shooter 2 RPM", "%.1f", shooter.getShooter2RPM());
        telemetry.addData("Counter Roller RPM", "%.1f", shooter.getCounterRollerRPM());
        telemetry.addLine();
        
        if (shooter.isEnabled()) {
            boolean ready = shooter.isAtTargetSpeed(50.0);
            telemetry.addData("Ready to Shoot?", ready ? "✅ YES" : "⏳ Spinning up...");
        }
        
        telemetry.addLine();
        telemetry.addLine("📝 Notice: Velocity control runs automatically in subsystem!");
        telemetry.update();

        // Update previous button states
        prevX = x;
        prevB = b;
        prevLT = ltPressed;
    }
}

/*
 * TEACHING POINTS FOR STUDENTS:
 * 
 * 1. The subsystem handles velocity control!
 *    - Just call shooter.spinUp() or shooter.stop()
 *    - The subsystem manages RPM automatically
 * 
 * 2. The TeleOp is much simpler!
 *    - OLD ShootingTest.java: 115 lines with manual velocity calculations
 *    - NEW ShooterTest.java: 100 lines with simple method calls
 *    - All the complexity is in the subsystem
 * 
 * 3. Easy to use in autonomous!
 *    - new InstantCommand(() -> shooter.spinUp())
 *    - new SpinUpShooterCommand(shooter, 3000) // Waits until ready
 *    - new InstantCommand(() -> shooter.stop())
 * 
 * 4. Ready for PIDF enhancement:
 *    - The subsystem structure makes it easy to add NextFTC's PIDF later
 *    - Just update the subsystem, TeleOp stays the same!
 * 
 * 5. Compare to ShootingTest.java:
 *    - OLD: All logic in OpMode
 *    - NEW: Logic in subsystem, TeleOp just calls methods
 *    - OLD: Hard to reuse
 *    - NEW: Works everywhere (TeleOp, Autonomous, testing)
 */

