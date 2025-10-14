package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.commands.utility.InstantCommand;

/**
 * HYBRID TeleOp - Shows how to mix NextFTC bindings with traditional gamepad access.
 * 
 * KEY CONCEPT:
 * - Use NextFTC bindings for BUTTONS (discrete actions)
 * - Use traditional gamepad access for JOYSTICKS (continuous control)
 * 
 * This is the RECOMMENDED approach for most teams!
 */
@TeleOp(name = "Hybrid TeleOp (Recommended)", group = "Examples")
public class HybridTeleOp extends NextFTCOpMode {

    private IntakeSubsystem intake;
    private FlywheelSubsystem flywheel;
    
    // Drive power multiplier (adjust to your preference)
    private double drivePower = 1.0;

    @Override
    public void onInit() {
        // Initialize subsystems
        intake = new IntakeSubsystem();
        flywheel = new FlywheelSubsystem();
        intake.initialize(hardwareMap);
        flywheel.initialize(hardwareMap);

        // ==========================================
        // NEXTFTC BINDINGS - For buttons/triggers
        // ==========================================
        
        // Intake control with bumpers
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(new InstantCommand(() -> intake.intake()));
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(new InstantCommand(() -> intake.outtake()));
        Gamepads.gamepad1().a().whenBecomesTrue(new InstantCommand(() -> intake.stop()));
        
        // Flywheel control with face buttons
        Gamepads.gamepad1().x().whenBecomesTrue(new InstantCommand(() -> flywheel.setHighSpeed()));
        Gamepads.gamepad1().y().whenBecomesTrue(new InstantCommand(() -> flywheel.setLowSpeed()));
        Gamepads.gamepad1().b().whenBecomesTrue(new InstantCommand(() -> flywheel.stop()));
        
        // Drive speed toggle with D-pad
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(new InstantCommand(() -> drivePower = 1.0));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(new InstantCommand(() -> drivePower = 0.5));

        telemetry.addLine("Hybrid TeleOp Initialized");
        telemetry.addLine("Use joysticks to drive (traditional style)");
        telemetry.addLine("Use buttons for mechanisms (NextFTC bindings)");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        // ==========================================
        // TRADITIONAL GAMEPAD ACCESS - For driving
        // ==========================================
        
        // Read joystick values directly - NO NextFTC bindings needed!
        double driving = (-gamepad1.right_stick_y) * drivePower;
        double strafe = (-gamepad1.right_stick_x) * drivePower;
        double rotate = (-gamepad1.left_stick_x) * 0.5;
        
        // Use these values for your drive train
        // (This is just an example - adjust for your drive implementation)
        // driveSubsystem.drive(driving, strafe, rotate);
        
        // Or if using Pedro Pathing with teleop:
        // follower.setTeleOpMovementVectors(driving, strafe, rotate);
        
        // ==========================================
        // Optional: Manual control can override bindings
        // ==========================================
        
        // You can also mix manual control if needed
        // For example, variable speed intake with trigger:
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(gamepad1.left_trigger * 0.8);
        } else if (gamepad1.right_trigger > 0.1) {
            intake.setPower(-gamepad1.right_trigger * 0.8);
        }
        
        // ==========================================
        // Telemetry
        // ==========================================
        
        telemetry.addData("Drive Power", "%.0f%%", drivePower * 100);
        telemetry.addData("Driving", "%.2f", driving);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Rotate", "%.2f", rotate);
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Flywheel RPM", "%.0f", flywheel.getCurrentRPM());
        telemetry.update();
    }
}

