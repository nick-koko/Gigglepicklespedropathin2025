package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "closeBlueSide", group = "Examples")
public class closeBlueSide extends NextFTCOpMode{
    public closeBlueSide() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE)
        );
    }
        private Timer pathTimer, actionTimer, opmodeTimer;
        private int pathState;

        //private final Pose startPose = new Pose(55, 9.5, Math.toRadians(90)); // Start Pose of our robotprivate Path scorePreload;
        private final Pose startPose = new Pose(18, 117.5, Math.toRadians(140)); // Start Pose of our robotprivate Path scorePreload;
        private final Pose scorePose = new Pose(50, 98.33, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose scorePoseClose = new Pose(37, 104, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
        private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
        private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

        private Path scorePreload;
        private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, curvedLineBlue, firstshootpath;

        public void buildPaths() {
            /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
            scorePreload = new Path(new BezierLine(startPose, scorePose));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
            firstshootpath=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(startPose,scorePoseClose)

                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePoseClose.getHeading())
                    .build();

            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            curvedLineBlue= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.027, 135.767),
                                    new Pose(69.060, 122.660),
                                    new Pose(68.051, 90.735),
                                    new Pose(48.224, 94.936)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(new BezierLine(new Pose(48.224, 94.936), new Pose(41.167, 84.182)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(new BezierLine(new Pose(41.167, 84.182), new Pose(19.827, 84.014)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.827, 84.014),
                                    new Pose(29.237, 84.518),
                                    new Pose(39.991, 85.022),
                                    new Pose(48.224, 94.936)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.224, 94.936),
                                    new Pose(53.769, 80.989),
                                    new Pose(49.736, 58.138),
                                    new Pose(41.503, 59.986)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(new BezierLine(new Pose(41.503, 59.986), new Pose(19.827, 59.986)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.827, 59.986),
                                    new Pose(39.655, 61.330),
                                    new Pose(47.720, 60.322),
                                    new Pose(48.224, 94.936)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.224, 94.936),
                                    new Pose(52.929, 64.019),
                                    new Pose(56.289, 34.446),
                                    new Pose(41.335, 35.790)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(new BezierLine(new Pose(41.335, 35.790), new Pose(19.995, 35.622)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.995, 35.622),
                                    new Pose(43.183, 36.630),
                                    new Pose(47.384, 47.888),
                                    new Pose(48.224, 94.936)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            grabPickup1 = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                    .build();

            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup1 = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                    .build();

            /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup2 = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .build();

            /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup2 = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                    .build();

            /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup3 = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup3Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                    .build();

            /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup3 = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                    .build();
        }

        private Command BlueCloseAuton() {
            return new SequentialGroup(
                    new ParallelGroup(
                            new FollowPath(firstshootpath),
                            new Delay(1.5)  //Would add or replace this with spinning up shooter while driving
                    ),
                    new Delay(1.0)  //Could replace this with shooting a ball
            );
        }



        /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
        @Override
        public void onUpdate() {

            // These loop the movements of the robot, these must be called continuously in order to work

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", PedroComponent.follower().getPose().getX());
            telemetry.addData("y", PedroComponent.follower().getPose().getY());
            telemetry.addData("heading", PedroComponent.follower().getPose().getHeading());
            telemetry.update();
        }

        /** This method is called once at the init of the OpMode. **/
        @Override
        public void onInit() {
            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();

        // Seed ball count for auton: assume robot starts loaded with 3
        IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3);

        // Build Pedro paths and set starting pose during init per docs
        buildPaths();
        PedroComponent.follower().setStartingPose(startPose);

        }

        /** This method is called continuously after Init while waiting for "play". **/
        @Override
        public void onWaitForStart() {}// dont know what to do here

        /** We do not use this because everything should automatically disable **/
        @Override
    public void onStop() {
        // Persist ball count (and optionally pose) for TeleOp
        GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
    }

        /** This method is called once at the start of the OhhpMode.
         * It runs all the setup actions, including building paths and starting the path system **/
        @Override
        public void onStartButtonPressed()

        {
        BlueCloseAuton().schedule();
        }

    }

