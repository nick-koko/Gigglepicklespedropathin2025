package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Example Autonomous using NextFTC with Pedro Pathing integration.
 * 
 * This demonstrates how to:
 * 1. Run commands in parallel (following a path while running intake)
 * 2. Run commands sequentially (spin up flywheel, wait, then shoot)
 * 3. Combine Pedro Pathing with mechanism control
 */
@Autonomous(name = "NextFTC Pedro Auto Example", group = "Examples")
public class ExampleNextFTCAutonomous extends NextFTCOpMode {

    public ExampleNextFTCAutonomous() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private IntakeSubsystem intakeSubsystem;
    private FlywheelSubsystem flywheelSubsystem;

    // Poses for the autonomous routine
    private final Pose startPose = new Pose(65, 135.7, Math.toRadians(180));
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135));
    private final Pose pickupPose = new Pose(37, 121, Math.toRadians(0));

    private PathChain pathToScore;
    private PathChain pathToPickup;

    @Override
    public void onInit() {
        // Initialize subsystems
        intakeSubsystem = new IntakeSubsystem();
        flywheelSubsystem = new FlywheelSubsystem();

        // Initialize subsystems with hardware map
        intakeSubsystem.initialize(hardwareMap);
        flywheelSubsystem.initialize(hardwareMap);

        // Initialize Pedro Pathing follower
        PedroComponent.follower().setStartingPose(startPose);

        // Build paths
        buildPaths();

        telemetry.addLine("NextFTC Pedro Autonomous Initialized");
        telemetry.update();
    }

    private void buildPaths() {
        // Path from start to scoring position
        pathToScore = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Path from scoring position to pickup position
        pathToPickup = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(50, 105, Math.toRadians(90)),
                        pickupPose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();
    }

    // Create autonomous routine as a reusable command method
    private Command autoRoutine() {
        // This routine:
        // 1. Drives to scoring position while spinning up the flywheel
        // 2. Waits for flywheel to reach speed
        // 3. Waits 1 second (simulating shooting)
        // 4. Stops flywheel
        // 5. Drives to pickup position while running intake
        // 6. Waits 1 second at pickup
        // 7. Stops intake

        return new SequentialGroup(
            // Step 1: Drive to score while spinning up flywheel
            new ParallelGroup(
                new FollowPath(pathToScore),
                new InstantCommand(() -> flywheelSubsystem.setRPM(FlywheelSubsystem.HIGH_TARGET_RPM))
            ),

            // Step 2: Wait to shoot (simulated)
            new Delay(1.0),

            // Step 3: Stop flywheel
            new InstantCommand(() -> flywheelSubsystem.stop()),

            // Step 4: Drive to pickup while running intake
            new ParallelGroup(
                new FollowPath(pathToPickup),
                new InstantCommand(() -> intakeSubsystem.intake())
            ),

            // Step 5: Wait at pickup
            new Delay(1.0),

            // Step 6: Stop intake
            new InstantCommand(() -> intakeSubsystem.stop())
        );
    }

    @Override
    public void onStartButtonPressed() {
        // Schedule the autonomous routine when START is pressed
        autoRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        // Add telemetry for debugging
        telemetry.addData("X", "%.2f", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", "%.2f", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("Intake Power", "%.2f", intakeSubsystem.getPower());
        telemetry.addData("Flywheel RPM", "%.0f", flywheelSubsystem.getCurrentRPM());
        telemetry.update();
    }
}

