package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * STUDENT-FRIENDLY Autonomous - Shows the new way to do autonomous!
 * 
 * This is where NextFTC really helps - it makes autonomous much easier!
 * 
 * Key concepts:
 * 1. SequentialGroup = do things ONE AFTER ANOTHER (like steps)
 * 2. ParallelGroup = do things AT THE SAME TIME (together)
 * 3. InstantCommand = do one quick action
 * 4. Delay = wait for some time
 */
@Autonomous(name = "Student Friendly Auto", group = "Examples")
public class StudentFriendlyAutonomous extends NextFTCOpMode {

    public StudentFriendlyAutonomous() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private IntakeSubsystem intake;

    // Starting position
    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose scorePose = new Pose(24, 0, 0);
    private final Pose pickupPose = new Pose(48, 0, 0);

    private PathChain pathToScore;
    private PathChain pathToPickup;
    private PathChain pathBack;

    @Override
    public void onInit() {
        // Create intake (same as TeleOp)
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);

        // Set up Pedro Pathing (SAME AS BEFORE!)
        PedroComponent.follower().setStartingPose(startPose);
        
        // Build paths (SAME AS BEFORE!)
        buildPaths();

        telemetry.addLine("Autonomous Ready!");
        telemetry.update();
    }

    private void buildPaths() {
        // Build paths just like before with Pedro Pathing
        pathToScore = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(0, 0)
                .build();

        pathToPickup = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPose))
                .setLinearHeadingInterpolation(0, 0)
                .build();

        pathBack = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    // Create a method that returns our autonomous routine as a Command
    private Command autoRoutine() {
        // Here's our autonomous routine:
        // 1. Drive to score
        // 2. Wait
        // 3. Drive to pickup WHILE running intake
        // 4. Wait to grab game piece
        // 5. Stop intake
        // 6. Drive back
        
        return new SequentialGroup(
            // ===========================================
            // STEP 1: Drive to scoring position
            // ===========================================
            new FollowPath(pathToScore),
            
            // ===========================================
            // STEP 2: Wait at scoring position (pretend we're scoring)
            // ===========================================
            new Delay(1.0),  // Wait 1 second
            
            // ===========================================
            // STEP 3: Drive to pickup WHILE running intake
            // ParallelGroup means "do both at the same time"
            // ===========================================
            new ParallelGroup(
                new FollowPath(pathToPickup),  // Drive
                new InstantCommand(() -> intake.intake())       // AND turn on intake
            ),
            
            // ===========================================
            // STEP 4: Wait to grab game piece
            // ===========================================
            new Delay(1.0),  // Wait 1 second
            
            // ===========================================
            // STEP 5: Stop intake
            // InstantCommand means "do this one thing quickly"
            // ===========================================
            new InstantCommand(() -> intake.stop()),
            
            // ===========================================
            // STEP 6: Drive back to score again
            // ===========================================
            new FollowPath(pathBack),
            
            // ===========================================
            // STEP 7: Wait (pretend we're scoring again)
            // ===========================================
            new Delay(1.0)
        );
    }

    @Override
    public void onStartButtonPressed() {
        // When START is pressed, schedule our autonomous routine
        autoRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        // Add telemetry to see what's happening
        telemetry.addData("X Position", "%.2f", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y Position", "%.2f", PedroComponent.follower().getPose().getY());
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.update();
    }
}

/*
 * EXPLAIN TO STUDENTS:
 * 
 * Think of SequentialGroup like a recipe:
 * 1. First, mix the dry ingredients
 * 2. Then, mix the wet ingredients
 * 3. Then, combine them
 * Each step happens ONE AFTER ANOTHER
 * 
 * Think of ParallelGroup like doing homework:
 * - Listen to music WHILE doing homework
 * Both things happen AT THE SAME TIME
 * 
 * In our autonomous:
 * - We drive to places in order (Sequential)
 * - But we can drive WHILE intaking (Parallel)
 * 
 * This was HARD with the old RoadRunner Actions!
 * Now it's EASY with NextFTC!
 */

