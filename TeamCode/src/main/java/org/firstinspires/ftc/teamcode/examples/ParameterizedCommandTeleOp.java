package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.utility.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.Gamepads;

/**
 * TeleOp using inline InstantCommand instead of separate command files.
 * 
 * This shows the simplest approach - just call subsystem methods directly.
 * No need for IntakeCommand, OuttakeCommand, StopIntakeCommand files!
 */
@TeleOp(name = "Inline Commands (Simplest)", group = "Examples")
public class ParameterizedCommandTeleOp extends NextFTCOpMode {

    private IntakeSubsystem intake;

    @Override
    public void onInit() {
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);

        // Simple inline commands - no separate files needed!
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
            new InstantCommand(() -> intake.intake())
        );
        
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
            new InstantCommand(() -> intake.outtake())
        );
        
        Gamepads.gamepad1().a().whenBecomesTrue(
            new InstantCommand(() -> intake.stop())
        );

        telemetry.addLine("Using inline commands - no command files needed!");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.update();
    }
}

