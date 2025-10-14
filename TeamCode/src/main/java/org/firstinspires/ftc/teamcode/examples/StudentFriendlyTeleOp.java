package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;

/**
 * STUDENT-FRIENDLY TeleOp - This looks almost exactly like your old OpMode!
 * 
 * What changed from the old way:
 * 1. "extends NextFTCOpMode" instead of "extends OpMode"
 * 2. Added "intake.initialize(hardwareMap)" in onInit()
 * 3. "onInit()" instead of "init()" and "onUpdate()" instead of "loop()"
 * 4. That's it!
 * 
 * Everything else is THE SAME as before!
 */
@TeleOp(name = "Student Friendly TeleOp", group = "Examples")
public class StudentFriendlyTeleOp extends NextFTCOpMode {

    // Same as before - create our subsystems (mechanisms)
    private IntakeSubsystem intake;
    
    // Drive speed (same as before)
    private double drivePower = 1.0;

    @Override
    public void onInit() {
        // Create intake (same as before)
        intake = new IntakeSubsystem();
        
        // NEW: Initialize the subsystem
        // (This is the only new line!)
        intake.initialize(hardwareMap);

        telemetry.addLine("Robot Ready!");
        telemetry.addLine("Left Bumper = Intake");
        telemetry.addLine("Right Bumper = Outtake");
        telemetry.addLine("A = Stop Intake");
        telemetry.update();
    }

    @Override
    public void onUpdate() {  // Changed from loop() - but works the same!
        
        // ===============================================
        // DRIVING - EXACTLY THE SAME AS BEFORE
        // ===============================================
        
        double driving = (-gamepad1.right_stick_y) * drivePower;
        double strafe = (-gamepad1.right_stick_x) * drivePower;
        double rotate = (-gamepad1.left_stick_x) * 0.5;
        
        // Use these for your drive motors (same as before)
        // frontLeft.setPower(driving + strafe + rotate);
        // frontRight.setPower(driving - strafe - rotate);
        // etc.
        
        // ===============================================
        // INTAKE CONTROL - EXACTLY THE SAME AS BEFORE
        // ===============================================
        
        if (gamepad1.left_bumper) {
            intake.intake();  // Turn on intake
        } else if (gamepad1.right_bumper) {
            intake.outtake();  // Turn on outtake
        } else if (gamepad1.a) {
            intake.stop();  // Stop intake
        }
        
        // ===============================================
        // SPEED CONTROL - SAME AS BEFORE
        // ===============================================
        
        if (gamepad1.dpad_up) {
            drivePower = 1.0;  // Full speed
        } else if (gamepad1.dpad_down) {
            drivePower = 0.5;  // Half speed
        }
        
        // ===============================================
        // TELEMETRY - SAME AS BEFORE
        // ===============================================
        
        telemetry.addData("Drive Power", "%.0f%%", drivePower * 100);
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.update();
    }
}

