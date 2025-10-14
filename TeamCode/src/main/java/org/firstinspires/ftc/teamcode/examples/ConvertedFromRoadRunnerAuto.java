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

//import dev.nextftc.core.command.InstantCommand;
//import dev.nextftc.core.command.ParallelGroup;
//import dev.nextftc.core.command.SequentialGroup;
//import dev.nextftc.core.command.WaitCommand;
//import dev.nextftc.extensions.pedro.FollowPathCommand;
//import dev.nextftc.extensions.pedro.PedroOpMode;

/**
 * CONVERTED FROM ROADRUNNER ACTIONS
 *
 * This shows your exact autonomous pattern from last year, converted to NextFTC.
 *
 * Compare this to your FourOrMoreSampleAutoPedro.java to see the translation!
 *
 * OLD PATTERN (RoadRunner Actions):
 * Actions.runBlocking(
 *     new SequentialAction(
 *         new ParallelAction(followPath, intake.intakePosition()),
 *         new SleepAction(1.0)
 *     )
 * );
 *
 * NEW PATTERN (NextFTC):
 * schedule(
 *     new SequentialGroup(
 *         new ParallelGroup(new FollowPathCommand(...), new InstantCommand(() -> intake.intake())),
 *         new WaitCommand(1.0)
 *     )
 * );
 */
@Autonomous(name = "Converted From RoadRunner", group = "Examples")
public class ConvertedFromRoadRunnerAuto extends NextFTCOpMode {

    public ConvertedFromRoadRunnerAuto() {
        addComponents(
                /* existing components */
                new PedroComponent(Constants::createFollower)
        );
    }
    private IntakeSubsystem intake;
    
    // Same poses as you used before
    private final Pose startPose = new Pose(7.5, 79.5, Math.toRadians(-90));
    private final Pose scorePose = new Pose(16.06, 126.16, Math.toRadians(-45));
    private final Pose pickupPose = new Pose(45.544, 107.447, Math.toRadians(90));
    
    private PathChain pathToScore;
    private PathChain pathToPickup;

    @Override
    public void onInit() {
        // Initialize subsystems
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);
        
        // Pedro Pathing setup - SAME AS BEFORE
        PedroComponent.follower().setStartingPose(startPose);
        buildPaths();
        
        telemetry.addLine("Converted autonomous ready!");
        telemetry.update();
    }

    private void buildPaths() {
        // Build paths EXACTLY like before
        pathToScore = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        scorePose
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        
        pathToPickup = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        scorePose,
                        pickupPose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();
    }


    private Command autoRoutineOne() {
        // =================================================================
        // THIS IS YOUR OLD PATTERN FROM ROADRUNNER ACTIONS
        // CONVERTED TO NEXTFTC LINE-BY-LINE
        // =================================================================

        // OLD:
        // Actions.runBlocking(
        //     new SequentialAction(

        // NEW:
        return new SequentialGroup(

                // =================================================================
                // SECTION 1: Score preload (parallel actions)
                // =================================================================
                // OLD:
                //     new ParallelAction(
                //         followStartToBucket1,
                //         outtakeSlide.high(),
                //         new SequentialAction(
                //             new SleepAction(1),
                //             outtakeDump.bucketPosition()
                //         )
                //     ),

                // NEW:
                new ParallelGroup(
                        new FollowPath(pathToScore),
                        // If you had outtake: new InstantCommand(() -> outtakeSlide.setHigh()),
                        new SequentialGroup(
                                new Delay(1.0),
                                new InstantCommand(() -> {
                                    // outtakeDump.bucketPosition()
                                })
                        )
                ),

                // =================================================================
                // SECTION 2: Wait
                // =================================================================
                // OLD: new SleepAction(0.2),
                // NEW:
                new Delay(0.2),

                // =================================================================
                // SECTION 3: Go to pickup while deploying intake
                // =================================================================
                // OLD:
                //     new ParallelAction(
                //         followBucketToSample2,
                //         outtakeSlide.low(),
                //         new SequentialAction(
                //             new SleepAction(0.5),
                //             new ParallelAction(
                //                 intakeSlide.autonAction(),
                //                 intakeWrist.wristIntakeAbyss(),
                //                 intakeArm.armIntakeAbyss()
                //             )
                //         )
                //     ),

                // NEW:
                new ParallelGroup(
                        new FollowPath(pathToPickup),
                        // If you had slides: new InstantCommand(() -> outtakeSlide.setLow()),
                        new SequentialGroup(
                                new Delay(0.5),
                                new ParallelGroup(
                                        // If you had intake slide: new InstantCommand(() -> intakeSlide.extend()),
                                        // If you had wrist: new InstantCommand(() -> intakeWrist.intakePosition()),
                                        // If you had arm: new InstantCommand(() -> intakeArm.intakePosition())
                                )
                        )
                ),

                // =================================================================
                // SECTION 4: Run intake
                // =================================================================
                // OLD: intakeSpinner.intakePosition(),
                // NEW:
                new InstantCommand(() -> intake.intake()),

                // =================================================================
                // SECTION 5: Wait to grab
                // =================================================================
                // OLD: new SleepAction(0.4),
                // NEW:
                new Delay(0.4),

                // =================================================================
                // SECTION 6: Stop intake and transfer
                // =================================================================
                // OLD:
                //     new ParallelAction(
                //         intakeSpinner.stopPosition(),
                //         intakeWrist.wristTransfer(),
                //         intakeArm.armTransfer()
                //     )

                // NEW:
                new ParallelGroup(
                        new InstantCommand(() -> intake.stop())
                        // If you had wrist: new InstantCommand(() -> intakeWrist.transfer()),
                        // If you had arm: new InstantCommand(() -> intakeArm.transfer())
                ),

                // =================================================================
                // SECTION 7: Drive back to score
                // =================================================================
                // OLD: followSampleToBucket3,
                // NEW:
                new FollowPath(pathToScore)
        );
    }
        
        // That's it! Same structure, just different names!

    @Override
    public void onStartButtonPressed() {
        autoRoutineOne().schedule();
    }


    @Override
    public void onUpdate() {
        // Add telemetry
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }
}

/*
 * SUMMARY FOR STUDENTS:
 * 
 * What changed:
 * 1. Actions.runBlocking() → Create Command method + .schedule()
 * 2. SequentialAction → SequentialGroup
 * 3. ParallelAction → ParallelGroup
 * 4. SleepAction → Delay
 * 5. intakeSpinner.intakePosition() → new InstantCommand(() -> intake.intake())
 * 6. followPath → new FollowPath(path)
 * 7. init() → onInit(), start() → onStartButtonPressed(), loop() → onUpdate()
 * 
 * What stayed the same:
 * - Path building
 * - Sequential/Parallel structure
 * - Timing
 * - Overall logic
 * 
 * What's better:
 * - No separate Actions files needed!
 * - Works with Pedro Pathing (no bugs!)
 * - Less code to write
 * - Reusable command methods!
 */

