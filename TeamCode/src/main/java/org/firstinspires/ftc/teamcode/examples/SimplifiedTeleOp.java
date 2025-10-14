package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.utility.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.Gamepads;

/**
 * SIMPLIFIED TeleOp using inline commands instead of separate command files.
 * 
 * This approach uses lambdas to avoid creating separate command files for simple actions.
 * Much less verbose - no need for IntakeCommand.java, OuttakeCommand.java, etc!
 * 
 * Use this approach when commands are simple (just calling subsystem methods).
 * Use separate command files when commands need complex logic or state tracking.
 */
@TeleOp(name = "Simplified TeleOp (No Command Files)", group = "Examples")
public class SimplifiedTeleOp extends NextFTCOpMode {

    private IntakeSubsystem intake;
    private FlywheelSubsystem flywheel;

    @Override
    public void onInit() {
        // Initialize subsystems (still need these!)
        intake = new IntakeSubsystem();
        flywheel = new FlywheelSubsystem();
        intake.initialize(hardwareMap);
        flywheel.initialize(hardwareMap);

        // =====================================
        // INLINE COMMANDS - No separate files!
        // =====================================
        
        // Intake control - all in one place!
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
            new InstantCommand(() -> intake.intake())
        );
        
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
            new InstantCommand(() -> intake.outtake())
        );
        
        Gamepads.gamepad1().a().whenBecomesTrue(
            new InstantCommand(() -> intake.stop())
        );

        // Flywheel control - also inline!
        Gamepads.gamepad1().x().whenBecomesTrue(
            new InstantCommand(() -> flywheel.setHighSpeed())
        );
        
        Gamepads.gamepad1().y().whenBecomesTrue(
            new InstantCommand(() -> flywheel.setLowSpeed())
        );
        
        Gamepads.gamepad1().b().whenBecomesTrue(
            new InstantCommand(() -> flywheel.stop())
        );

        // That's it! No IntakeCommand.java, OuttakeCommand.java, etc. needed!

        telemetry.addLine("Simplified TeleOp Initialized");
        telemetry.addLine("All commands defined inline - no separate files!");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Flywheel RPM", "%.0f", flywheel.getCurrentRPM());
        telemetry.update();
    }
}

