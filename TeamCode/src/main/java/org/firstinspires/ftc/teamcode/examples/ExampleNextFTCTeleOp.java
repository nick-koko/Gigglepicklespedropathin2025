package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.utility.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Example TeleOp using NextFTC command-based framework.
 * This demonstrates how to bind gamepad buttons to commands that control subsystems.
 * 
 * Controls:
 * - Left Bumper: Run intake
 * - Right Bumper: Run outtake
 * - A button: Stop intake
 * - X button: Spin flywheel at high speed
 * - B button: Stop flywheel
 */
@TeleOp(name = "NextFTC TeleOp Example", group = "Examples")
public class ExampleNextFTCTeleOp extends NextFTCOpMode {

    private IntakeSubsystem intakeSubsystem;
    private FlywheelSubsystem flywheelSubsystem;

    @Override
    public void onInit() {
        // Initialize subsystems
        intakeSubsystem = new IntakeSubsystem();
        flywheelSubsystem = new FlywheelSubsystem();

        // Initialize subsystems with hardware map
        intakeSubsystem.initialize(hardwareMap);
        flywheelSubsystem.initialize(hardwareMap);

        // Bind gamepad buttons to commands for intake (using InstantCommand)
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(new InstantCommand(() -> intakeSubsystem.intake()));
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(new InstantCommand(() -> intakeSubsystem.outtake()));
        Gamepads.gamepad1().a().whenBecomesTrue(new InstantCommand(() -> intakeSubsystem.stop()));

        // Bind gamepad buttons to commands for flywheel (using InstantCommand)
        Gamepads.gamepad1().x().whenBecomesTrue(new InstantCommand(() -> flywheelSubsystem.setHighSpeed()));
        Gamepads.gamepad1().b().whenBecomesTrue(new InstantCommand(() -> flywheelSubsystem.stop()));

        telemetry.addLine("NextFTC TeleOp Initialized");
        telemetry.addLine("LB=Intake, RB=Outtake, A=Stop Intake");
        telemetry.addLine("X=Flywheel On, B=Flywheel Off");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        // Add telemetry for debugging
        telemetry.addData("Intake Power", "%.2f", intakeSubsystem.getPower());
        telemetry.addData("Flywheel RPM", "%.0f", flywheelSubsystem.getCurrentRPM());
        telemetry.addData("Flywheel Target", "%.0f", flywheelSubsystem.getTargetRPM());
        telemetry.update();
    }

    @Override
    public void onStop() {
    }

}