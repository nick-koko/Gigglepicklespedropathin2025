package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.fateweaver.FateComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

@Configurable
@Autonomous(name = "closeBlueSideClassTest", group = "Comp")
public class closeBlueSideClassTest extends closeAutonPathsTest {
    public closeBlueSideClassTest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE),
                FateComponent.INSTANCE
        );
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos);
        ShooterSubsystem.INSTANCE.stop();

        GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
        PedroComponent.follower().setStartingPose(startPoseBlue);

        // Seed ball count for auton: assume robot starts loaded with 3
        IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3);

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {
        telemetry.addLine("Hello Pickle of the robot");
        telemetry.addLine("This is an Mr. Todone Speaking,");
        telemetry.addLine("----------------------------------------------");
        telemetry.addLine("Test Mode - Single run (shoot, pickup, gate+intake)");
        telemetry.addLine();
        telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs the single test sequence. **/
    @Override
    public void onStartButtonPressed() {
        TestRun().schedule();

        // Persist ball count (and optionally pose) for TeleOp
        GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
        GlobalRobotData.endAutonTurretAngleDegrees = Double.NaN;
        GlobalRobotData.hasAutonRun = true;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void onUpdate() {
        telemetry.addData("x", PedroComponent.follower().getPose().getX());
        telemetry.addData("y", PedroComponent.follower().getPose().getY());
        telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("shooter 1 power", ShooterSubsystem.INSTANCE.getShooter1Power());
        telemetry.addData("shooter 2 power", ShooterSubsystem.INSTANCE.getShooter2Power());
        telemetry.update();
    }

    /** We shouldn't need this because everything should automatically disable **/
    @Override
    public void onStop() {
        ShooterSubsystem.INSTANCE.stop();
        GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
        GlobalRobotData.endAutonTurretAngleDegrees = Double.NaN;
        GlobalRobotData.hasAutonRun = true;
    }
}
