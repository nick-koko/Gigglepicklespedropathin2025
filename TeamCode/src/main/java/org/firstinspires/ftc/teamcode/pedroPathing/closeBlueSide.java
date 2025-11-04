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
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
@Autonomous(name = "closeBothSide", group = "Comp")
public class closeBlueSide extends NextFTCOpMode{
    public closeBlueSide() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE)
        );
    }
    public static double autonShooterRPM = 3000.0;
    public static double autonShooterHoodServoPos = 0.06;

    private Pose finalStartPose = new Pose();
    private final Pose startPoseBlue = new Pose(18, 117.5, Math.toRadians(144)); // Start Pose of our robot
    private final Pose scorePoseCloseBlue = new Pose(37, 104, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1PoseBlue = new Pose(24, 82.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP1Blue = new Pose(60.1, 92.1, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP2Blue = new Pose( 50, 81.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2PoseBlue = new Pose(20, 57.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP1Blue = new Pose(70, 90.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP2Blue = new Pose( 65.5, 64, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3PoseBlue = new Pose(20, 35.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP1Blue = new Pose(44,75.6  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP2Blue = new Pose( 72, 40.09, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose offLineBlue = new Pose(40, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose startPoseRed = startPoseBlue.mirror();
    private final Pose scorePoseCloseRed = scorePoseCloseBlue.mirror();

    private final Pose pickup1PoseRed = pickup1PoseBlue.mirror();
    private final Pose pickup1CP1Red = pickup1CP1Blue.mirror();
    private final Pose pickup1CP2Red = pickup1CP2Blue.mirror();
    private final Pose pickup2PoseRed = pickup2PoseBlue.mirror();
    private final Pose pickup2CP1Red = pickup2CP1Blue.mirror();
    private final Pose pickup2CP2Red = pickup2CP2Blue.mirror();
    private final Pose pickup3PoseRed = pickup2PoseBlue.mirror();
    private final Pose pickup3CP1Red = pickup2CP1Blue.mirror();
    private final Pose pickup3CP2Red = pickup2CP2Blue.mirror();

    private final Pose offLineRed = offLineBlue.mirror();

    private PathChain firstshootpathBlue, firstPickupBlue, secondPickupBlue, thirdPickupBlue, moveOffLineBlue;
    private PathChain firstshootpathRed, firstPickupRed, moveOffLineRed;

        public void buildPaths() {

            firstshootpathBlue=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(startPoseBlue,scorePoseCloseBlue)

                    )
                    .setLinearHeadingInterpolation(startPoseBlue.getHeading(), scorePoseCloseBlue.getHeading())
                    .build();

            firstshootpathRed=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(startPoseRed,scorePoseCloseRed)

                    )
                    .setLinearHeadingInterpolation(startPoseRed.getHeading(), scorePoseCloseRed.getHeading())
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

            firstPickupRed= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseCloseRed,
                                    pickup1CP1Red,
                                    pickup1CP2Red,
                                    pickup1PoseRed
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), pickup1PoseRed.getHeading())
                    .addPath(new BezierLine(pickup1PoseRed, scorePoseCloseRed))
                    .setLinearHeadingInterpolation(pickup1PoseRed.getHeading(), scorePoseCloseRed.getHeading())
                    .build();

            moveOffLineRed=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseCloseRed,offLineRed)

                    )
                    .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), offLineRed.getHeading())
                    .build();

        }

        private Command BlueCloseAuton() {
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
                                    new Delay(0.5),
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
                                    new Delay(0.5),
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
                                    new Delay(0.5),
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

    private Command RedCloseAuton() {
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
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward()),
                                new Delay(2.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(2.0)
                        )
                ),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3)),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                new Delay(1.5),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new FollowPath(moveOffLineRed)
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

            // Seed ball count for auton: assume robot starts loaded with 3
            IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3);

            // Build Pedro paths and set starting pose during init per docs
            //buildPaths();
            //PedroComponent.follower().setStartingPose(startPoseBlue);
            buildPaths();
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                finalStartPose = startPoseBlue.copy();
            }
            else {
                finalStartPose = startPoseRed.copy();
            }
        }

        /** This method is called continuously after Init while waiting for "play". **/
        @Override
        public void onWaitForStart() {

            if (gamepad1.x) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
                finalStartPose = startPoseBlue.copy();
            } else if (gamepad1.b) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
                finalStartPose = startPoseRed.copy();
            }

            telemetry.addLine("Hello Pickle of the robot");
            telemetry.addLine("This is an Mr. Todone Speaking,");
            telemetry.addLine("----------------------------------------------");
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                telemetry.addLine("Favorite fruit: Blueberries!!! (Blue)");
            } else {
                telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
            }
            telemetry.addLine();
            telemetry.addData("x", PedroComponent.follower().getPose().getX());
            telemetry.addData("y", PedroComponent.follower().getPose().getY());
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
            PedroComponent.follower().setStartingPose(finalStartPose);
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                BlueCloseAuton().schedule();
            } else {
                RedCloseAuton().schedule();
            }

            // Persist ball count (and optionally pose) for TeleOp
            GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
            GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
            GlobalRobotData.hasAutonRun = true;
        }

    }

