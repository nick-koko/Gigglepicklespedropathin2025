package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
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

/* notes:
    - Start up shooter immediately
    - (save .75) right now, first shot at 2.75 s, thinking we can reduce to 2.0s?
    - (save 1.0) first set of shots done at about 3.35s, we don't start moving until 4.4s, so can either cut a second off here, or maybe use break beam sensor as an indication that we are done?
    - (save 1.5) we are at back at the shooting point at about 8.0s, but don't shoot until about 9.48s, assuming we start the shooter when driving, we could shoot earlier - maybe the timers are off, or maybe need to stop path earlier?
    - (save 1.1) done shooting at 9.82s, don't start moving until 11.0s
    - (save ??) path to go back to shooter is wide (a bit on purpose to avoid gate) not needed if we are hitting the gate already, and can save time by going straight
    - (save 0.7) back to shooting point about 16.1s, don't shoot until 16.9s
    - (save 1.2) done shooting at 17.25s, don't start moving until 18.49s
    - (??) maybe can take a less wide path to the last spike mark to possibly avoid robots that are just driving off the line?
    - (save 1.0) back to shooting spot at 24.25, don't shoot until 25.25
    - (save 1.0) done shooting at 25.62s, don't move until 26.7s
    - I wonder after, if we should have the robot turned 180deg, and ready to intake as soon as teleop starts and they hit the gate?
    - after we are done, watch time says 2s left, but match timer says 3
    - (total possible perfect savings of 8.25s for a total of 10.25 ) Need minimum 7s

 */
@Configurable
//@Autonomous(name = "closeBlueSide", group = "Comp")
public class closeAutonPaths extends NextFTCOpMode{
   /* public closeAutonPaths() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE),
                FateComponent.INSTANCE
        );
    } */
    public static double autonShooterRPM = 2775.0;
    public static double autonShooterHoodServoPos = 0.0;
    public static double pickupBrakingStrength = 1.0;

    public final Pose startPoseBlue = new Pose(32.5, 134.375, Math.toRadians(180)); // Start Pose of our robot
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

    public final Pose startPoseRed = startPoseBlue.mirror();
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


    public Pose startPose;
    public Pose scorePoseClose;

    public Pose pickup1Pose;
    public Pose pickup1CP1;
    public Pose pickup1CP2;

    public Pose pickup1EndPose;

    public Pose pickup2Pose;
    public Pose pickup2CP1;
    public Pose pickup2CP2;

    public Pose pickup2EndPose;
    public Pose getPickup2CPPath;

    public Pose pickup3Pose;
    public Pose pickup3CP1;
    public Pose pickup3CP2;

    public Pose pickup3EndPose;
    public Pose getPickup3CPPath;

    public Pose offLineLever;
    public Pose offLineClose;

    private PathChain firstshootpath, firstPickup,firstPickupEnd, secondPickup, secondPickupEnd, thirdPickup, thirdPickupEnd, moveOffLineLever, moveOffLineClose;

        public void buildPaths() {

            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                startPose = startPoseBlue;
                scorePoseClose = scorePoseCloseBlue;
                pickup1Pose = pickup1PoseBlue;
                pickup1CP1 = pickup1CP1Blue;
                pickup1CP2 = pickup1CP2Blue;
                pickup1EndPose = pickup1EndPoseBlue;
                pickup2Pose = pickup2PoseBlue;
                pickup2CP1 = pickup2CP1Blue;
                pickup2CP2 = pickup2CP2Blue;
                pickup2EndPose = pickup2EndPoseBlue;
                getPickup2CPPath = getPickup2CPPathBlue;
                pickup3Pose = pickup3PoseBlue;
                pickup3CP1 = pickup3CP1Blue;
                pickup3CP2 = pickup3CP2Blue;
                pickup3EndPose = pickup3EndPoseBlue;
                getPickup3CPPath = getPickup3CPPathBlue;
                offLineLever = offLineLeverBlue;
                offLineClose = offLineCloseBlue;
            } else {
                startPose = startPoseRed;
                scorePoseClose = scorePoseCloseRed;
                pickup1Pose = pickup1PoseRed;
                pickup1CP1 = pickup1CP1Red;
                pickup1CP2 = pickup1CP2Red;
                pickup1EndPose = pickup1EndPoseRed;
                pickup2Pose = pickup2PoseRed;
                pickup2CP1 = pickup2CP1Red;
                pickup2CP2 = pickup2CP2Red;
                pickup2EndPose = pickup2EndPoseRed;
                getPickup2CPPath = getPickup2CPPathRed;
                pickup3Pose = pickup3PoseRed;
                pickup3CP1 = pickup3CP1Red;
                pickup3CP2 = pickup3CP2Red;
                pickup3EndPose = pickup3EndPoseRed;
                getPickup3CPPath = getPickup3CPPathRed;
                offLineLever = offLineLeverRed;
                offLineClose = offLineCloseRed;

            }

            // if we want to change braking for a pathchain, then use .setNoDeceleration() or .setBrakingStrength(double set) after a path is added
            firstshootpath=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(startPose,scorePoseClose)

                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePoseClose.getHeading())
                    .build();

            firstPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup1CP1,
                                    pickup1CP2,
                                    pickup1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup1Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            firstPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())

                    .addPath(new BezierLine(pickup1EndPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup2CP1,
                                    pickup2CP2,
                                    pickup2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup2Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            secondPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())

                    .addPath(new BezierCurve(pickup2EndPose, getPickup2CPPath, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup3CP1,
                                    pickup3CP2,
                                    pickup3Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup3Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            thirdPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3EndPose.getHeading())

                    .addPath(new BezierCurve(pickup3EndPose, getPickup3CPPath, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            moveOffLineLever=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseClose,offLineLever)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineLever.getHeading())
                    .build();

            moveOffLineClose=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseClose,offLineClose)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineClose.getHeading())
                    .build();


        }
    // if you want to reduce power for a path, use FollowPath(pathchain, holdend, maxpower)

    public Command CloseShootPreload() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpath),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM))
                ),
                new Delay(0.75),  //Could replace this with shooting a ball
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoToFirstPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }
    public Command ClosePickupAndShootFirstRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),

                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.2),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoTo2ndPickupLine() {
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

    public Command ClosePickupGateLeverShoot2ndRow() {
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
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoTo3rdPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    public Command ClosePickupAndShoot3rdRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.5),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    public Command CloseMoveOffLine() {
        return new SequentialGroup(
                new FollowPath(moveOffLineClose)
        );
    }

    public Command CloseMoveOffLineToLever() {
        return new SequentialGroup(
                new FollowPath(moveOffLineLever)
        );
    }

}

