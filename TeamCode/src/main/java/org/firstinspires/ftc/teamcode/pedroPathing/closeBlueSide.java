package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.fateweaver.FateComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
@Autonomous(name = "closeBlueSide", group = "Comp")
public class closeBlueSide extends NextFTCOpMode{
    public closeBlueSide() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE),
                FateComponent.INSTANCE
        );
    }
    public static double autonShooterRPM = 2775.0;
    public static double autonShooterHoodServoPos = 0.0;
    public static double pickupBrakingStrength = 1.0;

    public double intAmount = 12;

    private Pose finalStartPose = new Pose();
    private final Pose startPoseBlue = new Pose(32.5, 134.375, Math.toRadians(180)); // Start Pose of our robot
    private final Pose scorePoseCloseBlue = new Pose(33, 107, Math.toRadians(128)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1PoseBlue = new Pose(40, 86.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose pickup1CP1Blue = new Pose(46.5, 97, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP2Blue = new Pose( 48.5, 90, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup1EndPoseBlue = new Pose(24, 86.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right

    private final Pose pickup2PoseBlue = new Pose(40, 64.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP1Blue = new Pose(47, 91.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP2Blue = new Pose( 52.5, 66, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2EndPoseBlue = new Pose(23.5, 64.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup2CPPathBlue = new Pose(44, 57, Math.toRadians(180));

    private final Pose pickup3PoseBlue = new Pose(40, 39.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP1Blue = new Pose(37,90  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP2Blue = new Pose( 57, 39, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3EndPoseBlue = new Pose(20, 35.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup3CPPathBlue = new Pose(40, 45, Math.toRadians(180));

    private final Pose offLineLeverBlue = new Pose(30, 80, Math.toRadians(270)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose offLineCloseBlue = new Pose(50, 117, Math.toRadians(135));

    private final Pose startPoseRed = startPoseBlue.mirror();
    private final Pose scorePoseCloseRed = scorePoseCloseBlue.mirror();

    private final Pose pickup1PoseRed = pickup1PoseBlue.mirror();
    private final Pose pickup1CP1Red = pickup1CP1Blue.mirror();
    private final Pose pickup1CP2Red = pickup1CP2Blue.mirror();
    private final Pose pickup1EndPoseRed = pickup1EndPoseBlue.mirror();
    private final Pose pickup2PoseRed = pickup2PoseBlue.mirror();
    private final Pose pickup2CP1Red = pickup2CP1Blue.mirror();
    private final Pose pickup2CP2Red = pickup2CP2Blue.mirror();
    private final Pose pickup2EndPoseRed = pickup2EndPoseBlue.mirror();
    private final Pose getPickup2CPPathRed = getPickup2CPPathBlue.mirror();
    private final Pose pickup3PoseRed = pickup3PoseBlue.mirror();
    private final Pose pickup3CP1Red = pickup3CP1Blue.mirror();
    private final Pose pickup3CP2Red = pickup3CP2Blue.mirror();
    private final Pose pickup3EndPoseRed = pickup3EndPoseBlue.mirror();
    private final Pose getPickup3CPPathRed = getPickup3CPPathBlue.mirror();



    private final Pose offLineLeverRed = offLineLeverBlue.mirror();
    private final Pose offLineCloseRed = offLineCloseBlue.mirror();

    private PathChain firstshootpathBlue, firstPickupBlue,firstPickupEndBlue, secondPickupBlue, secondPickupEndBlue, thirdPickupBlue, thirdPickupEndBlue, moveOffLineLeverBlue, moveOffLineCloseBlue;
    private PathChain firstshootpathRed, firstPickupRed, firstPickupEndRed, secondPickupRed, secondPickupEndRed, thirdPickupRed, thirdPickupEndRed, moveOffLineLeverRed, moveOffLineCloseRed;

        public void buildPaths() {

            // if we want to change braking for a pathchain, then use .setNoDeceleration() or .setBrakingStrength(double set) after a path is added
            firstshootpathBlue=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(startPoseBlue,scorePoseCloseBlue)

                    )
                    .setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
                    .build();

            firstPickupBlue= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseCloseBlue,
                                    pickup1CP1Blue,
                                    pickup1CP2Blue,
                                    pickup1PoseBlue
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), pickup1PoseBlue.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            firstPickupEndBlue = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1PoseBlue, pickup1EndPoseBlue))
                    .setLinearHeadingInterpolation(pickup1PoseBlue.getHeading(), pickup1EndPoseBlue.getHeading())

                    .addPath(new BezierLine(pickup1EndPoseBlue, scorePoseCloseBlue))
                    .setLinearHeadingInterpolation(pickup1EndPoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupBlue= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseCloseBlue,
                                    pickup2CP1Blue,
                                    pickup2CP2Blue,
                                    pickup2PoseBlue
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), pickup2PoseBlue.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            secondPickupEndBlue = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2PoseBlue, pickup2EndPoseBlue))
                    .setLinearHeadingInterpolation(pickup2PoseBlue.getHeading(), pickup2EndPoseBlue.getHeading())

                    .addPath(new BezierCurve(pickup2EndPoseBlue, getPickup2CPPathBlue, scorePoseCloseBlue))
                    .setLinearHeadingInterpolation(pickup2EndPoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickupBlue= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseCloseBlue,
                                    pickup3CP1Blue,
                                    pickup3CP2Blue,
                                    pickup3PoseBlue
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), pickup3PoseBlue.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            thirdPickupEndBlue = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3PoseBlue, pickup3EndPoseBlue))
                    .setLinearHeadingInterpolation(pickup2PoseBlue.getHeading(), pickup3EndPoseBlue.getHeading())

                    .addPath(new BezierCurve(pickup3EndPoseBlue, getPickup3CPPathBlue, scorePoseCloseBlue))
                    .setLinearHeadingInterpolation(pickup3EndPoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            moveOffLineLeverBlue=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseCloseBlue,offLineLeverBlue)

                    )
                    .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), offLineLeverBlue.getHeading())
                    .build();

            moveOffLineCloseBlue=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseCloseBlue,offLineCloseBlue)

                    )
                    .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), offLineCloseBlue.getHeading())
                    .build();

            firstshootpathRed=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(startPoseRed,scorePoseCloseRed)

                    )
                    .setLinearHeadingInterpolation(startPoseRed.getHeading(), scorePoseCloseRed.getHeading())
                    .build();

            firstPickupRed= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseCloseRed,
                                    pickup1CP1Red,
                                    pickup1CP2Red,
                                    pickup1PoseRed
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), pickup1PoseRed.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            firstPickupEndRed = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1PoseRed, pickup1EndPoseRed))
                    .setLinearHeadingInterpolation(pickup1PoseRed.getHeading(), pickup1EndPoseRed.getHeading())

                    .addPath(new BezierLine(pickup1EndPoseRed, scorePoseCloseRed))
                    .setLinearHeadingInterpolation(pickup1EndPoseRed.getHeading(), scorePoseCloseRed.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupRed= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseCloseRed,
                                    pickup2CP1Red,
                                    pickup2CP2Red,
                                    pickup2PoseRed
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), pickup2PoseRed.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            secondPickupEndRed = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2PoseRed, pickup2EndPoseRed))
                    .setLinearHeadingInterpolation(pickup2PoseRed.getHeading(), pickup2EndPoseRed.getHeading())

                    .addPath(new BezierCurve(pickup2EndPoseRed, getPickup2CPPathRed, scorePoseCloseRed))
                    .setLinearHeadingInterpolation(pickup2EndPoseRed.getHeading(), scorePoseCloseRed.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickupRed= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseCloseRed,
                                    pickup3CP1Red,
                                    pickup3CP2Red,
                                    pickup3PoseRed
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), pickup3PoseRed.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            thirdPickupEndRed = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3PoseRed, pickup3EndPoseRed))
                    .setLinearHeadingInterpolation(pickup2PoseRed.getHeading(), pickup3EndPoseRed.getHeading())

                    .addPath(new BezierCurve(pickup3EndPoseRed, getPickup3CPPathRed, scorePoseCloseRed))
                    .setLinearHeadingInterpolation(pickup3EndPoseRed.getHeading(), scorePoseCloseRed.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            moveOffLineLeverRed=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseCloseRed,offLineLeverRed)

                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), offLineLeverRed.getHeading())
                    .build();

            moveOffLineCloseRed=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseCloseRed,offLineCloseRed)

                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), offLineCloseRed.getHeading())
                    .build();

        }
    // if you want to reduce power for a path, use FollowPath(pathchain, holdend, maxpower)
    private Command CloseRed3BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathRed),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineCloseRed)
        );
    }

    private Command CloseRed6BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathRed),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(firstPickupEndRed),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),

                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineCloseRed)
        );
    }

    private Command CloseRed9BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathRed),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(firstPickupEndRed),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(secondPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(secondPickupEndRed),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineLeverRed)
        );
    }

    private Command CloseRed12BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathRed),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(firstPickupEndRed),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(secondPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(secondPickupEndRed),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(thirdPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(thirdPickupEndRed),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                new FollowPath(moveOffLineLeverRed)
            );
        }

    private Command CloseBlue3BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathBlue),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineCloseBlue)
        );
    }

    private Command CloseBlue6BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathBlue),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(firstPickupEndBlue),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),

                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineCloseBlue)
        );
    }

    private Command CloseBlue9BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathBlue),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.75),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(firstPickupEndBlue),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(secondPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(secondPickupEndBlue),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineLeverBlue)
        );
    }

    private Command CloseBlue12BallAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpathBlue),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(1.5),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(firstPickupEndBlue),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(secondPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(secondPickupEndBlue),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(thirdPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                ),
                new ParallelGroup(
                        new FollowPath(thirdPickupEndBlue),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                new FollowPath(moveOffLineLeverBlue)
        );
    }





        /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
        @Override
        public void onUpdate() {

            // These loop the movements of the robot, these must be called continuously in order to work

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", PedroComponent.follower().getPose().getX());
            telemetry.addData("y", PedroComponent.follower().getPose().getY());
            telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
            telemetry.addData("shooter 1 power", ShooterSubsystem.INSTANCE.getShooter1Power());
            telemetry.addData("shooter 2 power", ShooterSubsystem.INSTANCE.getShooter2Power());
            telemetry.update();
        }

        /** This method is called once at the init of the OpMode. **/
        @Override
        public void onInit() {
            ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos);
            ShooterSubsystem.INSTANCE.stop();

            //ShooterSubsystem.INSTANCE.initialize(hardwareMap);
            //IntakeWithSensorsSubsystem.INSTANCE.initialize(hardwareMap);

            GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
            PedroComponent.follower().setStartingPose(startPoseBlue);

            // Seed ball count for auton: assume robot starts loaded with 3
            IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3);

            // Build Pedro paths and set starting pose during init per docs
            //buildPaths();
            //PedroComponent.follower().setStartingPose(startPoseBlue);
            buildPaths();

        }

        /** This method is called continuously after Init while waiting for "play". **/
        @Override
        public void onWaitForStart() {

            /*if (gamepad1.x) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
                finalStartPose = startPoseBlue.copy();
            } else if (gamepad1.b) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
                finalStartPose = startPoseRed.copy();
            }*/

            if ((gamepad1.dpadUpWasPressed()) && (intAmount < 12)) {
              intAmount = intAmount + 3;
            } else if ((gamepad1.dpadDownWasPressed()) && (intAmount > 3)) {
                intAmount = intAmount - 3;
            }

            //if up on dpad
            //Go back an option
            //int option goes up by 1
            //if down on dpad
            //Go forward option
            //int option goes down by 1

            telemetry.addLine("Hello Pickle of the robot");
            telemetry.addLine("This is an Mr. Todone Speaking,");
            telemetry.addLine("----------------------------------------------");
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                telemetry.addLine("Favorite fruit: Blueberries!!! (Blue)");
            } else {
                telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
            }
            telemetry.addLine();
            telemetry.addData("Eating this number of balls: ", intAmount);

            telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

            telemetry.update();

        }// dont know what to do here

        /** We do not use this because everything should automatically disable **/
        @Override
        public void onStop() {
            // Persist ball count (and optionally pose) for TeleOp
            ShooterSubsystem.INSTANCE.stop();
            GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
            GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
            GlobalRobotData.hasAutonRun = true;
        }

        /** This method is called once at the start of the OhhpMode.
         * It runs all the setup actions, including building paths and starting the path system **/
        @Override
        public void onStartButtonPressed()
        {
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                if (intAmount == 3){
                    CloseBlue3BallAuto().schedule();
                }
                else if (intAmount == 6){
                    CloseBlue6BallAuto().schedule();
                }
                else if (intAmount == 9){
                    CloseBlue9BallAuto().schedule();
                }
                else {
                    CloseBlue12BallAuto().schedule();
                }


            } else {
                if (intAmount == 3){
                    CloseRed3BallAuto().schedule();
                }
                else if (intAmount == 6){
                    CloseRed6BallAuto().schedule();
                }
                else if (intAmount == 9){
                    CloseRed9BallAuto().schedule();
                }
                else {
                    CloseRed12BallAuto().schedule();
                }
            }

            // Persist ball count (and optionally pose) for TeleOp
            GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
            GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
            GlobalRobotData.hasAutonRun = true;
        }

    }

