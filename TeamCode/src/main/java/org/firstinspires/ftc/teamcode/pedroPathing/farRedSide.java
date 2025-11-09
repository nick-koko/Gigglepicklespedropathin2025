package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
@Autonomous(name = "farRedSide", group = "Comp")
public class farRedSide extends NextFTCOpMode{
    public farRedSide() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE)
        );
    }
    public static double autonShooterRPM = 4600.0;
    public static double autonShooterHoodServoPos = 0.4;

    public double intAmount = 12;

    private Pose finalStartPose = new Pose();
    private final Pose startPoseBlue = new Pose(64, 8.5, Math.toRadians(90)); // Start Pose of our robot
    private final Pose scorePoseCloseBlue = new Pose(60.5, 23, Math.toRadians(117)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //our robot is peak :D
    private final Pose pickup1PoseBlue = new Pose(20, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP1Blue = new Pose(51, 40, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP2Blue = new Pose( 41, 35, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    //our robot is very peak :D
    private final Pose pickup2PoseBlue = new Pose(7, 8.5, Math.toRadians(-90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP1Blue = new Pose(42.5, 25, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP2Blue = new Pose( 13, 42, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    //our robot is himalayas :D
    private final Pose pickup3PoseBlue = new Pose(19, 57, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP1Blue = new Pose(52,64  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP2Blue = new Pose( 46, 61, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    //our robot is crystal peak :D
    private final Pose offLineBlue = new Pose(47, 27, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    //our robot is absolute crystallization of the himalayas peak :D
    private final Pose startPoseRed = startPoseBlue.mirror();
    private final Pose scorePoseCloseRed = scorePoseCloseBlue.mirror();

    private final Pose pickup1PoseRed = pickup1PoseBlue.mirror();
    private final Pose pickup1CP1Red = pickup1CP1Blue.mirror();
    private final Pose pickup1CP2Red = pickup1CP2Blue.mirror();
    private final Pose pickup2PoseRed = pickup2PoseBlue.mirror();
    private final Pose pickup2CP1Red = pickup2CP1Blue.mirror();
    private final Pose pickup2CP2Red = pickup2CP2Blue.mirror();
    private final Pose pickup3PoseRed = pickup3PoseBlue.mirror();
    private final Pose pickup3CP1Red = pickup3CP1Blue.mirror();
    private final Pose pickup3CP2Red = pickup3CP2Blue.mirror();

    private final Pose offLineRed = offLineBlue.mirror();

    private PathChain firstshootpathBlue, firstPickupBlue, secondPickupBlue, thirdPickupBlue, moveOffLineBlue;
    private PathChain firstshootpathRed, firstPickupRed, secondPickupRed, thirdPickupRed, moveOffLineRed;

        public void buildPaths() {

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

                    .addPath(new BezierLine(pickup1PoseBlue, scorePoseCloseBlue))
                    .setLinearHeadingInterpolation(pickup1PoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
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

                    .addPath(new BezierLine(pickup2PoseBlue, scorePoseCloseBlue))
                    .setLinearHeadingInterpolation(pickup2PoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
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

                    .addPath(new BezierLine(pickup3PoseBlue, scorePoseCloseBlue))
                    .setLinearHeadingInterpolation(pickup3PoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
                    .build();

            moveOffLineBlue=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseCloseBlue,offLineBlue)

                    )
                    .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), offLineBlue.getHeading())
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

                    .addPath(new BezierLine(pickup1PoseRed, scorePoseCloseRed))
                    .setLinearHeadingInterpolation(pickup1PoseRed.getHeading(), scorePoseCloseRed.getHeading())
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

                    .addPath(new BezierLine(pickup2PoseRed, scorePoseCloseRed))
                    .setLinearHeadingInterpolation(pickup2PoseRed.getHeading(), scorePoseCloseRed.getHeading())
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

                    .addPath(new BezierLine(pickup3PoseRed, scorePoseCloseRed))
                    .setLinearHeadingInterpolation(pickup3PoseRed.getHeading(), scorePoseCloseRed.getHeading())
                    .build();

            moveOffLineRed=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseCloseRed,offLineRed)

                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), offLineRed.getHeading())
                    .build();

        }

    private Command CloseRed3BallAuto() {
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
                new FollowPath(moveOffLineRed)
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
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineRed)
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
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupRed),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineRed)
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                new FollowPath(moveOffLineRed)
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
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineBlue)
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
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineBlue)
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
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new ParallelGroup(
                        new FollowPath(firstPickupBlue),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineBlue)
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
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
                                new Delay(0.25),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(1.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                new FollowPath(moveOffLineBlue)
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
            super.onInit();
            ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos);
            ShooterSubsystem.INSTANCE.stop();

            //ShooterSubsystem.INSTANCE.initialize(hardwareMap);
            //IntakeWithSensorsSubsystem.INSTANCE.initialize(hardwareMap);

            GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
            PedroComponent.follower().setStartingPose(startPoseRed);

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
            super.onStop();
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
            super.onStartButtonPressed();

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

//a
//b
//c
//d
//e
//f
//g
//h
//i
//j
//k
//l
//m
//n
//o
//p
//q
//r
//s
//t
//u
//v
//w
//x
//y
//and
//z
//now
//i
//know
//my
//a
//b
//c's
//next
//time
//wont
//you
//sing
//with
//me