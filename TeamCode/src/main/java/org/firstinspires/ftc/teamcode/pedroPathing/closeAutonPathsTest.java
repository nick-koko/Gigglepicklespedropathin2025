package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
public class closeAutonPathsTest extends NextFTCOpMode {
    public static double autonShooterRPM = 2825.0;
    public static double autonShooterHoodServoPos = -0.1;
    public static double pickupBrakingStrength = 1.0;

    // Start and scoring poses
    public final Pose startPoseBlue = new Pose(32.5, 134.375, Math.toRadians(180));
    private final Pose scorePoseClose = new Pose(33.0, 107.0, Math.toRadians(128));
    private final Pose scorePoseFar = new Pose(56.730, 81.502, Math.toRadians(130));

    // Second pickup poses and control points
    private final Pose pickup2CP1 = new Pose(47.000, 91.500, Math.toRadians(180));
    private final Pose pickup2CP2 = new Pose(52.500, 66.000, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(40.000, 60.000, Math.toRadians(180));
    private final Pose pickup2EndPose = new Pose(23.500, 60.000, Math.toRadians(180));
    private final Pose pickup2ReturnCP1 = new Pose(41.052, 61.274, Math.toRadians(180));
    private final Pose pickup2ReturnCP2 = new Pose(50.927, 73.802, Math.toRadians(180));

    // Gate poses and control points
    private final Pose gateCP1 = new Pose(53.576, 72.048, Math.toRadians(180));
    private final Pose gateCP2 = new Pose(35.985, 63.624, Math.toRadians(180));
    private final Pose gatePose = new Pose(17.010, 66.268, Math.toRadians(167));

    // Intake from gate poses
    private final Pose intakeCP1 = new Pose(13.708, 51.629, Math.toRadians(180));
    private final Pose intakePose = new Pose(9.481, 54.984, Math.toRadians(123));

    // Intake from gate return to shoot
    private final Pose intakeFromGateShootCP1 = new Pose(44.217, 64.852, Math.toRadians(180));

    private PathChain firstshootpath;
    private PathChain secondPickup;
    private PathChain secondPickupEnd;
    private PathChain hitGateBeforePickup;
    private PathChain intakeFromGate;
    private PathChain intakeFromGateShoot;

    public void buildPaths() {
        // Path 1: Start to score position
        firstshootpath = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPoseBlue, scorePoseClose))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseClose.getHeading())
                .build();

        // Path 2: Score to second pickup position
        secondPickup = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePoseClose, pickup2CP1, pickup2CP2, pickup2Pose))
                .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup2Pose.getHeading())
                .setNoDeceleration()
                .build();

        // Paths 3+4 combined: Pickup sweep across line + return to score area
        secondPickupEnd = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())
                .addPath(new BezierCurve(pickup2EndPose, pickup2ReturnCP1, pickup2ReturnCP2, scorePoseFar))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePoseFar.getHeading())
                .setBrakingStrength(pickupBrakingStrength)
                .build();

        // Path 5: From score area, hit gate before pickup
        hitGateBeforePickup = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePoseFar, gateCP1, gateCP2, gatePose))
                .setLinearHeadingInterpolation(scorePoseFar.getHeading(), gatePose.getHeading())
                .setBrakingStrength(pickupBrakingStrength)
                .build();

        // Path 6: Intake from gate position
        intakeFromGate = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(gatePose, intakeCP1, intakePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), intakePose.getHeading())
                .setBrakingStrength(pickupBrakingStrength)
                .build();

        // Path 7: Return from intake to far scoring position
        intakeFromGateShoot = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(intakePose, intakeFromGateShootCP1, scorePoseFar))
                .setLinearHeadingInterpolation(intakePose.getHeading(), scorePoseFar.getHeading())
                .build();
    }

    /** Shoot the preloaded balls: drive to score position while spinning up shooter, then shoot */
    public Command TestShootPreload() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpath),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.startHybridShotFeedBoostController(false)),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    /** Drive to second pickup line with intake enabled partway through */
    public Command TestGoToSecondPickup() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(secondPickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    /** Pickup second row (sweep across + return to score area) and shoot */
    public Command TestPickupAndShootSecondRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(secondPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.2),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.startHybridShotFeedBoostController(true)),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    /** Hit the gate while enabling intake - intake starts when we begin following this path */
    public Command TestHitGateWithIntake() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(hitGateBeforePickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    /** Follow the intake-from-gate path (intake stays running to keep collecting balls) */
    public Command TestIntakeFromGate() {
        return new SequentialGroup(
                new FollowPath(intakeFromGate)
        );
    }

    /** Return from gate intake to far scoring position, spin up shooter on the way, then shoot */
    public Command TestIntakeFromGateShoot() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(intakeFromGateShoot),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.2),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.startHybridShotFeedBoostController(true)),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    /** The full test sequence */
    public Command TestRun() {
        return new SequentialGroup(
                TestShootPreload(),
                TestGoToSecondPickup(),
                TestPickupAndShootSecondRow(),
                TestHitGateWithIntake(),
                TestIntakeFromGate(),
                TestIntakeFromGateShoot()
        );
    }
}
