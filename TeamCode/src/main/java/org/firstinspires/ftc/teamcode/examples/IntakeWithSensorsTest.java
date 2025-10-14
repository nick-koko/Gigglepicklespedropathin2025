package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Test TeleOp for Intake Subsystem with Break Beam Sensors
 * 
 * This replaces TeleOpSensorTest.java but uses NextFTC subsystem architecture.
 * 
 * Controls:
 * - Left Bumper: Run intake forward
 * - Right Bumper: Run intake reverse
 * - A: Shoot (auto 2-second cycle, resets ball count)
 * - X/Y/B: Manually toggle motors m1/m2/m3 on/off (for testing)
 * 
 * The subsystem automatically monitors sensors and stops motors as balls pass!
 */
@TeleOp(name = "Intake with Sensors Test", group = "Test")
public class IntakeWithSensorsTest extends NextFTCOpMode {

    private IntakeWithSensorsSubsystem intake;
    
    // Button state tracking
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevB = false;
    private boolean prevA = false;

    @Override
    public void onInit() {
        // Create and initialize subsystem
        intake = new IntakeWithSensorsSubsystem();
        intake.initialize(hardwareMap);
        
        telemetry.addLine("Intake with Sensors Test Ready!");
        telemetry.addLine("LB=forward, RB=reverse");
        telemetry.addLine("A=Shoot, X/Y/B=toggle motors");
        telemetry.addLine("Sensors automatically stop motors!");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        // =============================================
        // CONTROLS - Traditional gamepad access
        // =============================================
        
        boolean forward = gamepad1.left_bumper;
        boolean reverse = gamepad1.right_bumper;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean b = gamepad1.b;
        boolean a = gamepad1.a;

        // Manual toggle controls (for testing individual motors)
        if (x && !prevX) intake.toggleMotor1();
        if (y && !prevY) intake.toggleMotor2();
        if (b && !prevB) intake.toggleMotor3();
        
        // Shoot button
        if (a && !prevA) {
            intake.shoot();
        }

        // Intake control (only when not shooting - subsystem handles this)
        if (!intake.isShooting()) {
            if (forward && !reverse) {
                intake.intakeForward();
            } else if (reverse && !forward) {
                intake.intakeReverse();
            } else {
                intake.stop();
            }
        }

        // =============================================
        // TELEMETRY
        // =============================================
        
        String mode = intake.isShooting() ? "SHOOTING" : 
                     (forward ? "Forward" : (reverse ? "Reverse" : "Stopped"));
        
        telemetry.addData("Mode", mode);
        telemetry.addData("Ball Count", intake.getBallCount() + "/3");
        telemetry.addLine();
        
        telemetry.addData("Sensor 0 (m1)", intake.isSensor0Broken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Sensor 1 (m2)", intake.isSensor1Broken() ? "BROKEN" : "CLEAR");
        telemetry.addData("Sensor 2 (m3)", intake.isSensor2Broken() ? "BROKEN" : "CLEAR");
        telemetry.addLine();
        
        telemetry.addData("m1", "%s  rpm=%.1f", 
                intake.isMotor1Enabled() ? "EN" : "OFF", intake.getMotor1RPM());
        telemetry.addData("m2", "%s  rpm=%.1f", 
                intake.isMotor2Enabled() ? "EN" : "OFF", intake.getMotor2RPM());
        telemetry.addData("m3", "%s  rpm=%.1f", 
                intake.isMotor3Enabled() ? "EN" : "OFF", intake.getMotor3RPM());
        
        telemetry.addLine();
        telemetry.addLine("📝 Notice: Sensor logic runs automatically in subsystem!");
        telemetry.update();

        // Update previous button states
        prevX = x;
        prevY = y;
        prevB = b;
        prevA = a;
    }
}

/*
 * TEACHING POINTS FOR STUDENTS:
 * 
 * 1. The subsystem does the heavy lifting!
 *    - Sensor checking happens in periodic() automatically
 *    - Ball counting is handled internally
 *    - Motor stopping is automatic
 * 
 * 2. The TeleOp is simpler!
 *    - Just call intake.intakeForward() or intake.intakeReverse()
 *    - Just call intake.shoot()
 *    - The subsystem handles all the details
 * 
 * 3. This will work in autonomous too!
 *    - new InstantCommand(() -> intake.intakeForward())
 *    - The sensor logic will work exactly the same
 * 
 * 4. Compare to TeleOpSensorTest.java:
 *    - OLD: All sensor logic in OpMode (130+ lines)
 *    - NEW: Sensor logic in subsystem, TeleOp is simple (70 lines)
 *    - OLD: Can't reuse in autonomous
 *    - NEW: Works everywhere automatically!
 */

