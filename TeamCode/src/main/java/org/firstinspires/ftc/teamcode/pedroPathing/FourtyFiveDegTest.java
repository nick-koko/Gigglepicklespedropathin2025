package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "45 Deg Test", group = "Examples")
public class FourtyFiveDegTest extends NextFTCOpMode{
    public FourtyFiveDegTest() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }
        int testDirection = 1;

        //private final Pose startPose = new Pose(55, 9.5, Math.toRadians(90)); // Start Pose of our robotprivate Path scorePreload;
        private final Pose startPose = new Pose(72, 72, Math.toRadians(0)); // Start Pose of our robotprivate Path scorePreload;
        private final Pose fourtyFive1 = new Pose(96, 96, Math.toRadians(0));
        private final Pose fourtyFive2 = new Pose(48, 48, Math.toRadians(0));
        private final Pose fourtyFive3 = new Pose(96, 48, Math.toRadians(0));
        private final Pose fourtyFive4 = new Pose(48, 96, Math.toRadians(0));
        private Path fourtyFivePath1, fourtyFivePath2, fourtyFivePath3, fourtyFivePath4;

        public void buildPaths() {
            /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
            fourtyFivePath1 = new Path(new BezierLine(startPose, fourtyFive1));
            fourtyFivePath1.setLinearHeadingInterpolation(startPose.getHeading(), fourtyFive1.getHeading());

            fourtyFivePath2 = new Path(new BezierLine(startPose, fourtyFive2));
            fourtyFivePath2.setLinearHeadingInterpolation(startPose.getHeading(), fourtyFive2.getHeading());

            fourtyFivePath3 = new Path(new BezierLine(startPose, fourtyFive3));
            fourtyFivePath3.setLinearHeadingInterpolation(startPose.getHeading(), fourtyFive3.getHeading());

            fourtyFivePath4 = new Path(new BezierLine(startPose, fourtyFive4));
            fourtyFivePath4.setLinearHeadingInterpolation(startPose.getHeading(), fourtyFive4.getHeading());

        }

        private Command TestForward() {
            return new SequentialGroup(
                    new ParallelGroup(
                            new FollowPath(fourtyFivePath1),
                            new Delay(1.5)  //Would add or replace this with spinning up shooter while driving
                    ),
                    new Delay(1.0)  //Could replace this with shooting a ball
            );
        }
    private Command TestBackward() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(fourtyFivePath2),
                        new Delay(1.5)  //Would add or replace this with spinning up shooter while driving
                ),
                new Delay(1.0)  //Could replace this with shooting a ball
        );
    }
    private Command TestRight() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(fourtyFivePath3),
                        new Delay(1.5)  //Would add or replace this with spinning up shooter while driving
                ),
                new Delay(1.0)  //Could replace this with shooting a ball
        );
    }
    private Command TestLeft() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(fourtyFivePath4),
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
            telemetry.addData("x", PedroComponent.follower().getPose().getX());
            telemetry.addData("y", PedroComponent.follower().getPose().getY());
            telemetry.addData("heading", PedroComponent.follower().getPose().getHeading());
            telemetry.update();
        }

        /** This method is called once at the init of the OpMode. **/
        @Override
        public void onInit() {

        // Build Pedro paths and set starting pose during init per docs
        buildPaths();
        PedroComponent.follower().setStartingPose(startPose);

        }

        /** This method is called continuously after Init while waiting for "play". **/
        @Override
        public void onWaitForStart() {
            if (gamepad1.dpadUpWasPressed()) {
                testDirection = 1;
            } else if (gamepad1.dpadDownWasPressed()) {
                testDirection = 2;
            } else if (gamepad1.dpadRightWasPressed()) {
                testDirection = 3;
            } else if (gamepad1.dpadLeftWasPressed()) {
                testDirection = 4;
            }

            telemetry.addLine("Hello Pickle of the robot");
            telemetry.addLine("This is an Mr. Todone Speaking,");
            telemetry.addLine("----------------------------------------------");
            if (testDirection == 1) {
                telemetry.addLine("Favorite Direction: Forward!");
            } else if (testDirection == 2) {
                telemetry.addLine("Favorite Direction: Backward!");
            } else if (testDirection == 3) {
                telemetry.addLine("Favorite Direction: Right!");
            } else {
                telemetry.addLine("Favorite Direction: Left!");
            }

            telemetry.update();
        }
        /** We do not use this because everything should automatically disable **/
        @Override
    public void onStop() {
     }
        /** This method is called once at the start of the OhhpMode.
         * It runs all the setup actions, including building paths and starting the path system **/
        @Override
        public void onStartButtonPressed()

        {
            if (testDirection == 1) {
                TestForward().schedule();
            } else if (testDirection == 2) {
                TestBackward().schedule();
            } else if (testDirection == 3) {
                TestRight().schedule();
            } else {
                TestLeft().schedule();
            }

        }

    }

