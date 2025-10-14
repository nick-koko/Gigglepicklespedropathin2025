package org.firstinspires.ftc.teamcode.templates;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.delays.Delay;

/**
 * Template showing how to create commands in NextFTC.
 * 
 * IMPORTANT: In NextFTC, you typically DON'T extend Command!
 * Instead, you create methods that RETURN Command objects.
 * 
 * THREE APPROACHES TO COMMANDS:
 * 
 * 1. INSTANT COMMANDS (Simplest - Recommended for most cases)
 *    - Use InstantCommand with lambdas
 *    - No separate file needed
 *    - Perfect for simple actions
 * 
 * 2. COMMAND METHODS (Good for reusable routines)
 *    - Create methods that return Command objects
 *    - Build complex sequences
 *    - Easy to test and reuse
 * 
 * 3. CUSTOM COMMAND CLASSES (Advanced - rarely needed)
 *    - Only for very complex logic
 *    - Usually not worth the extra files
 * 
 * See examples below for each approach.
 */
public class CommandTemplate {

    // ====================
    // APPROACH 1: INSTANT COMMANDS (Recommended for most cases)
    // ====================
    // Use these directly in your OpMode - no separate file needed!
    
    /**
     * Example: Direct use in TeleOp with bindings
     */
    public void exampleBindingUsage() {
        // In your OpMode's onInit():
        // IntakeSubsystem intake = new IntakeSubsystem();
        // 
        // gamepad1().a().onTrue(
        //     new InstantCommand(() -> intake.intake())
        // );
        // 
        // gamepad1().b().onTrue(
        //     new InstantCommand(() -> intake.stop())
        // );
    }
    
    /**
     * Example: Direct use in Autonomous
     */
    public void exampleAutonomousUsage() {
        // In your OpMode:
        // IntakeSubsystem intake = new IntakeSubsystem();
        // 
        // Command routine = new SequentialGroup(
        //     new InstantCommand(() -> intake.intake()),
        //     new Delay(1.0),
        //     new InstantCommand(() -> intake.stop())
        // );
        // 
        // routine.schedule();
    }

    // ====================
    // APPROACH 2: COMMAND METHODS (Good for complex routines)
    // ====================
    // Create methods that return Command objects
    
    /**
     * Example: Autonomous routine as a command method
     * Place this in your Autonomous OpMode
     */
    private Command scoreAndPickup() {
        // Assumes you have these subsystems in your OpMode:
        // private IntakeSubsystem intake;
        // private PathChain pathToScore, pathToPickup;
        
        return new SequentialGroup(
            // Score preload
            new ParallelGroup(
                // new FollowPath(pathToScore),
                // new InstantCommand(() -> outtake.raise())
            ),
            new Delay(0.5),
            // new InstantCommand(() -> outtake.dump()),
            
            new Delay(0.3),
            
            // Go pickup
            new ParallelGroup(
                // new FollowPath(pathToPickup),
                // new InstantCommand(() -> intake.intake())
            ),
            new Delay(1.0)
            // new InstantCommand(() -> intake.stop())
        );
    }
    
    /**
     * Example: Complex mechanism sequence
     */
    private Command deployIntakeSequence() {
        // Assumes subsystems in your OpMode:
        // private IntakeSubsystem intake;
        // private ArmSubsystem arm;
        
        return new SequentialGroup(
            // Step 1: Position arm
            // new InstantCommand(() -> arm.setAngle(45)),
            new Delay(0.5),  // Wait for arm to move
            
            // Step 2: Start intake
            // new InstantCommand(() -> intake.intake()),
            
            // Step 3: Wait for game piece
            new Delay(2.0),
            
            // Step 4: Stop and retract
            new ParallelGroup(
                // new InstantCommand(() -> intake.stop()),
                // new InstantCommand(() -> arm.setAngle(0))
            )
        );
    }

    // ====================
    // APPROACH 3: CUSTOM COMMAND CLASS (Advanced - Rarely Needed)
    // ====================
    // Only use this if you need:
    // - Complex state management
    // - Continuous sensor feedback
    // - Custom interruption logic
    
    /**
     * Example of when you MIGHT need a custom command class:
     * - Waiting for a sensor to detect something
     * - Running a control loop with feedback
     * - Complex timing with multiple conditions
     * 
     * For most cases, use InstantCommand or command methods instead!
     * 
     * If you really need a custom command, create a new file:
     * 
     * public class WaitForSensorCommand extends Command {
     *     private final DistanceSensor sensor;
     *     private final double targetDistance;
     *     
     *     public WaitForSensorCommand(DistanceSensor sensor, double targetDistance) {
     *         this.sensor = sensor;
     *         this.targetDistance = targetDistance;
     *     }
     *     
     *     @Override
     *     public void execute() {
     *         // Called repeatedly
     *     }
     *     
     *     @Override
     *     public boolean isFinished() {
     *         return sensor.getDistance() < targetDistance;
     *     }
     * }
     */

    // ====================
    // QUICK REFERENCE
    // ====================
    
    /**
     * SIMPLE ACTION?
     * → Use InstantCommand directly:
     *   new InstantCommand(() -> subsystem.doSomething())
     * 
     * COMPLEX SEQUENCE?
     * → Create a command method:
     *   private Command myRoutine() {
     *       return new SequentialGroup(...);
     *   }
     * 
     * NEED STATE/SENSORS?
     * → Create a custom command class (rare)
     *   public class MyCommand extends Command { ... }
     * 
     * MIDDLE SCHOOL STUDENTS?
     * → Start with InstantCommand
     * → Then teach command methods
     * → Skip custom command classes unless really needed
     */
}
