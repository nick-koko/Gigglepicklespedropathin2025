package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleNextFTCAuto extends NextFTCOpMode {
    public ExampleNextFTCAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private final Pose startPose = new Pose(55, 9.5, Math.toRadians(90)); // Start Pose of our robotprivate Path scorePreload;
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private PathChain scorePreload, curvedLineBlue;

    public void buildPaths() {
        /* This is our scorePreload path. */
        scorePreload = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    /** Creates a Command that can be used to score and shoot **/
    private Command driveToScoreAndShoot() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(scorePreload),
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

        buildPaths();
        PedroComponent.follower().setStartingPose(startPose);

    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed()
    {
        driveToScoreAndShoot().schedule();
    }

}
