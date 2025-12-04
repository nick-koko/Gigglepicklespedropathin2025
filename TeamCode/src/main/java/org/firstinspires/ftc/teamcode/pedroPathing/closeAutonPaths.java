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
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;

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

    public static double autonShooterRPM = 2825.0;
    public static double autonShooterHoodServoPos = 0.0;
    public static double pickupBrakingStrength = 1.0;
    public static double pickupCornerBrakingStrength = 0.5;

    public final Pose startPoseBlue = new Pose(32.5, 134.375, Math.toRadians(180)); // Start Pose of our robot
    private final Pose scorePoseCloseBlue = new Pose(33, 107, Math.toRadians(128)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1PoseBlue = new Pose(40, 86.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose pickup1CP1Blue = new Pose(46.5, 97, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP2Blue = new Pose( 48.5, 90, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup1EndPoseBlue = new Pose(24, 86.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right

    private final Pose pickup1GatePoseBlue = new Pose(14.5, 76.0, Math.toRadians(180));
    private final Pose pickup1GateCP1Blue = new Pose(32.5, 80.5, Math.toRadians(180));
    private final Pose pickup1GateCP2Blue = new Pose( 25.5, 72, Math.toRadians(180));

    private final Pose pickup2PoseBlue = new Pose(40, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP1Blue = new Pose(47, 91.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP2Blue = new Pose( 52.5, 66, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2EndPoseBlue = new Pose(23.5, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup2CPPathBlue = new Pose(44, 57, Math.toRadians(180));

    private final Pose pickup2GateLeverEndPoseBlue = new Pose(17, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2GateLeverPushPoseBlue = new Pose(19, 75.0, Math.toRadians(270)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2GateLeverPushCP1Blue = new Pose(22.5, 57, Math.toRadians(270)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2GateLeverPushCP2Blue = new Pose(28.5, 69.5, Math.toRadians(270)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2GateLeverPushHoldPoseBlue = new Pose(12, 76.0, Math.toRadians(270));

    private final Pose pickup3PoseBlue = new Pose(40, 37.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP1Blue = new Pose(37,90  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP2Blue = new Pose( 57, 39, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3EndPoseBlue = new Pose(20, 35.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup3CPPathBlue = new Pose(40, 45, Math.toRadians(180));

    private final Pose pickupZonePoseBlue = new Pose(9, 37, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupZoneCP1Blue = new Pose(32,48, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupZoneCP2Blue = new Pose( 5.5, 66, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickupZoneEndPoseBlue = new Pose(8, 13.0, Math.toRadians(255)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickupZoneCPPathBlue = new Pose(44, 59, Math.toRadians(235));

    private final Pose offLineLeverBlue = new Pose(30, 80, Math.toRadians(270)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose offLineCloseBlue = new Pose(50, 117, Math.toRadians(135));

    public final Pose startPoseRed = startPoseBlue.mirror();
    private final Pose scorePoseCloseRed = scorePoseCloseBlue.mirror();

    private final Pose pickup1PoseRed = pickup1PoseBlue.mirror();
    private final Pose pickup1CP1Red = pickup1CP1Blue.mirror();
    private final Pose pickup1CP2Red = pickup1CP2Blue.mirror();
    private final Pose pickup1EndPoseRed = pickup1EndPoseBlue.mirror();
    private final Pose pickup1GatePoseRed = pickup1GatePoseBlue.mirror();
    private final Pose pickup1GateCP1Red = pickup1GateCP1Blue.mirror();
    private final Pose pickup1GateCP2Red = pickup1GateCP2Blue.mirror();

    private final Pose pickup2PoseRed = pickup2PoseBlue.mirror();
    private final Pose pickup2CP1Red = pickup2CP1Blue.mirror();
    private final Pose pickup2CP2Red = pickup2CP2Blue.mirror();
    private final Pose pickup2EndPoseRed = pickup2EndPoseBlue.mirror();
    private final Pose getPickup2CPPathRed = getPickup2CPPathBlue.mirror();
    private final Pose pickup2GateLeverEndPoseRed = pickup2GateLeverEndPoseBlue.mirror();
    private final Pose pickup2GateLeverPushPoseRed = pickup2GateLeverPushPoseBlue.mirror();
    private final Pose pickup2GateLeverPushHoldPoseRed = pickup2GateLeverPushHoldPoseBlue.mirror();
    private final Pose pickup2GateLeverPushCP1Red = pickup2GateLeverPushCP1Blue.mirror();
    private final Pose pickup2GateLeverPushCP2Red = pickup2GateLeverPushCP2Blue.mirror();
    private final Pose pickup3PoseRed = pickup3PoseBlue.mirror();
    private final Pose pickup3CP1Red = pickup3CP1Blue.mirror();
    private final Pose pickup3CP2Red = pickup3CP2Blue.mirror();
    private final Pose pickup3EndPoseRed = pickup3EndPoseBlue.mirror();
    private final Pose getPickup3CPPathRed = getPickup3CPPathBlue.mirror();
    private final Pose pickupZonePoseRed = pickupZonePoseBlue.mirror();
    private final Pose pickupZoneCP1Red = pickupZoneCP1Blue.mirror();
    private final Pose pickupZoneCP2Red = pickupZoneCP2Blue.mirror();
    private final Pose pickupZoneEndPoseRed = pickupZoneEndPoseBlue.mirror();
    private final Pose getPickupZoneCPPathRed = getPickupZoneCPPathBlue.mirror();

    private final Pose offLineLeverRed = offLineLeverBlue.mirror();
    private final Pose offLineCloseRed = offLineCloseBlue.mirror();


    public Pose startPose;
    public Pose scorePoseClose;

    public Pose pickup1Pose;
    public Pose pickup1CP1;
    public Pose pickup1CP2;

    public Pose pickup1EndPose;

    public Pose pickup1GatePose;
    public Pose pickup1GateCP1;
    public Pose pickup1GateCP2;

    public Pose pickup2Pose;
    public Pose pickup2CP1;
    public Pose pickup2CP2;

    public Pose pickup2EndPose;
    public Pose getPickup2CPPath;

    public Pose pickup2GateLeverEndPose;

    public Pose pickup2GateLeverPushPose;
    public Pose pickup2GateLeverPushHoldPose;
    public Pose pickup2GateLeverPushCP1;
    public Pose pickup2GateLeverPushCP2;

    public Pose pickup3Pose;
    public Pose pickup3CP1;
    public Pose pickup3CP2;

    public Pose pickup3EndPose;
    public Pose getPickup3CPPath;

    public Pose pickupZonePose;
    public Pose pickupZoneCP1;
    public Pose pickupZoneCP2;

    public Pose pickupZoneEndPose;
    public Pose getPickupZoneCPPath;

    public Pose offLineLever;
    public Pose offLineClose;

    private PathChain firstshootpath, firstPickup,firstPickupEnd, firstPickupGateLeverEnd, firstPickupGateLeverShootEnd, secondPickup, secondPickupEnd, secondPickupGateLeverEnd, secondPickupGateLeverShootEnd, thirdPickup, thirdPickupEnd, zonePickup, zonePickupEnd, zonePickupShoot, moveOffLineLever, moveOffLineClose;

        public void buildPaths() {

            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                startPose = startPoseBlue;
                scorePoseClose = scorePoseCloseBlue;
                pickup1Pose = pickup1PoseBlue;
                pickup1CP1 = pickup1CP1Blue;
                pickup1CP2 = pickup1CP2Blue;
                pickup1EndPose = pickup1EndPoseBlue;
                pickup1GatePose = pickup1GatePoseBlue;
                pickup1GateCP1 = pickup1GateCP1Blue;
                pickup1GateCP2 = pickup1GateCP2Blue;
                pickup2Pose = pickup2PoseBlue;
                pickup2CP1 = pickup2CP1Blue;
                pickup2CP2 = pickup2CP2Blue;
                pickup2EndPose = pickup2EndPoseBlue;
                pickup2GateLeverEndPose = pickup2GateLeverEndPoseBlue;
                pickup2GateLeverPushPose = pickup2GateLeverPushPoseBlue;
                pickup2GateLeverPushHoldPose = pickup2GateLeverPushHoldPoseBlue;
                pickup2GateLeverPushCP1 = pickup2GateLeverPushCP1Blue;
                pickup2GateLeverPushCP2 = pickup2GateLeverPushCP2Blue;
                getPickup2CPPath = getPickup2CPPathBlue;
                pickup3Pose = pickup3PoseBlue;
                pickup3CP1 = pickup3CP1Blue;
                pickup3CP2 = pickup3CP2Blue;
                pickup3EndPose = pickup3EndPoseBlue;
                getPickup3CPPath = getPickup3CPPathBlue;
                pickupZonePose = pickupZonePoseBlue;
                pickupZoneCP1 = pickupZoneCP1Blue;
                pickupZoneCP2 = pickupZoneCP2Blue;
                pickupZoneEndPose = pickupZoneEndPoseBlue;
                getPickupZoneCPPath = getPickupZoneCPPathBlue;
                offLineLever = offLineLeverBlue;
                offLineClose = offLineCloseBlue;
            } else {
                startPose = startPoseRed;
                scorePoseClose = scorePoseCloseRed;
                pickup1Pose = pickup1PoseRed;
                pickup1CP1 = pickup1CP1Red;
                pickup1CP2 = pickup1CP2Red;
                pickup1EndPose = pickup1EndPoseRed;
                pickup1GatePose = pickup1GatePoseRed;
                pickup1GateCP1 = pickup1GateCP1Red;
                pickup1GateCP2 = pickup1GateCP2Red;
                pickup2Pose = pickup2PoseRed;
                pickup2CP1 = pickup2CP1Red;
                pickup2CP2 = pickup2CP2Red;
                pickup2EndPose = pickup2EndPoseRed;
                pickup2GateLeverEndPose = pickup2GateLeverEndPoseRed;
                pickup2GateLeverPushPose = pickup2GateLeverPushPoseRed;
                pickup2GateLeverPushHoldPose = pickup2GateLeverPushHoldPoseRed;
                pickup2GateLeverPushCP1 = pickup2GateLeverPushCP1Red;
                pickup2GateLeverPushCP2 = pickup2GateLeverPushCP2Red;
                getPickup2CPPath = getPickup2CPPathRed;
                pickup3Pose = pickup3PoseRed;
                pickup3CP1 = pickup3CP1Red;
                pickup3CP2 = pickup3CP2Red;
                pickup3EndPose = pickup3EndPoseRed;
                getPickup3CPPath = getPickup3CPPathRed;
                pickupZonePose = pickupZonePoseRed;
                pickupZoneCP1 = pickupZoneCP1Red;
                pickupZoneCP2 = pickupZoneCP2Red;
                pickupZoneEndPose = pickupZoneEndPoseRed;
                getPickupZoneCPPath = getPickupZoneCPPathRed;
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

            firstPickupGateLeverEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())

                    .addPath(
                            new BezierCurve(
                                    pickup1EndPose,
                                    pickup1GateCP1,
                                    pickup1GateCP2,
                                    pickup1GatePose
                            )
                    )
                    .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), pickup1GatePose.getHeading(),.75)

                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            firstPickupGateLeverShootEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1GatePose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup1GatePose.getHeading(), scorePoseClose.getHeading())
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

            secondPickupGateLeverEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, pickup2GateLeverEndPose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2GateLeverEndPose.getHeading())

                    .addPath(new BezierCurve(
                            pickup2GateLeverEndPose,
                            pickup2GateLeverPushCP1,
                            pickup2GateLeverPushCP2,
                            pickup2GateLeverPushPose))
                    .setLinearHeadingInterpolation(pickup2GateLeverEndPose.getHeading(), pickup2GateLeverPushPose.getHeading())

                    .addPath(new BezierLine(pickup2GateLeverPushPose, pickup2GateLeverPushHoldPose))
                    .setLinearHeadingInterpolation(pickup2GateLeverPushPose.getHeading(), pickup2GateLeverPushHoldPose.getHeading())

                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupGateLeverShootEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2GateLeverPushPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup2GateLeverPushPose.getHeading(), scorePoseClose.getHeading())
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
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3EndPose.getHeading())

                    .addPath(new BezierCurve(pickup3EndPose, getPickup3CPPath, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            zonePickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickupZoneCP1,
                                    pickupZoneCP2,
                                    pickupZonePose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickupZonePose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            zonePickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickupZonePose, pickupZoneEndPose))
                    .setLinearHeadingInterpolation(pickupZonePose.getHeading(), pickupZoneEndPose.getHeading())
                    .setBrakingStrength(pickupCornerBrakingStrength)
                    .build();

            zonePickupShoot = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierCurve(pickupZoneEndPose, getPickupZoneCPPath, scorePoseClose))
                    .setLinearHeadingInterpolation(pickupZoneEndPose.getHeading(), scorePoseClose.getHeading())
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

    public Command ClosePickupAndGateLeverFirstRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickupGateLeverEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                )
        );
    }

    public Command ClosePickupShootAfterGateLever1stRow() {
        return new SequentialGroup(
                new FollowPath(firstPickupGateLeverShootEnd),
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

    public Command ClosePickupAndShoot2ndRow() {
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
                new Delay(0.8),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command ClosePickupAndGateLever2ndRow() {
        return new SequentialGroup(
                    new ParallelGroup(
                        new FollowPath(secondPickupGateLeverEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                )
       );
    }

    public Command ClosePickupShootAfterGateLever2ndRow() {
        return new SequentialGroup(
                new FollowPath(secondPickupGateLeverShootEnd),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new Delay(0.8),
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
                new Delay(0.7),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(0.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    public Command CloseGoToZonePickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(zonePickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }

    /**
     * Follows the zonePickupEnd path and terminates when EITHER:
     * - The path completes, OR
     * - The intake has collected 3 balls
     *
     * Uses ParallelRaceGroup which stops when the first command finishes.
     */
    /*public Command ClosePickupZoneRow() {
        return new SequentialGroup(
             new ParallelRaceGroup(
                     //new FollowPath(zonePickupEnd).raceWith(new Delay(2.0))'
                     new Delay(2.0),
                    IntakeUntilFull()
            )
        );
    } */ // Parallel Race Group was not working as expected, so we are using the IntakeUntilFull command instead

    /**
     * Follows zonePickupEnd path and finishes when EITHER:
     * - The path completes, OR
     * - The intake has collected 3 balls
     */
    public Command FollowZonePickupEndUntilFull() {
        return new LambdaCommand()
                .setStart(() -> PedroComponent.follower().followPath(zonePickupEnd))
                .setIsDone(() -> (IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) || (!PedroComponent.follower().isBusy()))
                .setInterruptible(true)
                .named("FollowZonePickupEndUntilFull");
    }

    /**
     * Follows zonePickup path and finishes when EITHER:
     * - The path completes, OR
     * - The intake has collected 3 balls
     */
    public Command FollowZonePickupUntilFull() {
        return new LambdaCommand()
                .setStart(() -> PedroComponent.follower().followPath(zonePickup))
                .setIsDone(() -> (IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) || (!PedroComponent.follower().isBusy()))
                .setInterruptible(true)
                .named("FollowZonePickupUntilFull");
    }

    public Command CloseShootZoneRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(zonePickupShoot),
                        new SequentialGroup(
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(0.5),
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

