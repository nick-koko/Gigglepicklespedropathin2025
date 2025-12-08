package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.fateweaver.FateComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

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
@Autonomous(name = "closeRedSide", group = "Comp")
public class closeRedSide extends closeAutonPaths{
    public closeRedSide() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE),
                FateComponent.INSTANCE
        );
    }
    public double intAmount = 15;
    public double pushLever = 2;

    private Pose finalStartPose = new Pose();

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos);
        ShooterSubsystem.INSTANCE.stop();

        GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
        PedroComponent.follower().setStartingPose(startPoseRed);

        // Seed ball count for auton: assume robot starts loaded with 3
        IntakeWithSensorsSubsystem.INSTANCE.setBallCount(3);

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

        // If dpad Up/Down is pressed, increase or decrease ball count
        if ((gamepad1.dpadUpWasPressed()) && (intAmount < 15)) {
            intAmount = intAmount + 3;
        } else if ((gamepad1.dpadDownWasPressed()) && (intAmount > 3)) {
            intAmount = intAmount - 3;
        }
        // If dpad left/right is pressed add or subtract a row until push lever
        if ((gamepad1.dpadRightWasPressed()) && (pushLever < 3)) {
            pushLever = pushLever + 1;
        } else if ((gamepad1.dpadLeftWasPressed()) && (pushLever > 0)) {
            pushLever = pushLever - 1;
        }

        //TODO Add dpad Left/Right to set when to hit gate lever (after 1st pickup, second pickup, or both)

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
        telemetry.addData("Opening basket at: ", pushLever);

        telemetry.addData("heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));

        telemetry.update();

    }


    public Command Close3Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseMoveOffLine()
        );
    }

    public Command Close6Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseMoveOffLine()
        );

    }

    public Command Close9Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseMoveOffLineToLever()
        );

    }
    public Command Close12Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close12BallLeverAfter3() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close12BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close12BallLeverBoth() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15Ball() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverAfter3() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndShoot2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverAfter6() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndShootFirstRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(.1),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }

    public Command Close15BallLeverBoth() {
        return new SequentialGroup(
                CloseShootPreload(),
                CloseGoToFirstPickupLine(),
                ClosePickupAndGateLeverFirstRow(),
                new Delay(.5),
                ClosePickupShootAfterGateLever1stRow(),
                CloseGoTo2ndPickupLine(),
                ClosePickupAndGateLever2ndRow(),
                new Delay(1.5),
                ClosePickupShootAfterGateLever2ndRow(),
                CloseGoTo3rdPickupLine(),
                ClosePickupAndShoot3rdRow(),
                CloseGoToZonePickupLine(),
                FollowZonePickupEndUntilFull(),
                CloseShootZoneRow(),
                CloseMoveOffLineToLever()
        );
    }


    /** This method is called once at the start of the OhhpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {
        if (intAmount == 3) {
                Close3Ball().schedule();
            }
            else if (intAmount == 6){
                Close6Ball().schedule();
            }
            else if (intAmount == 9){
                Close9Ball().schedule();
            }
            else if (intAmount == 12){
                if (pushLever == 0) {
                    Close12Ball().schedule();
                } else if (pushLever == 1){
                    Close12BallLeverAfter3().schedule();
                }  else if (pushLever == 2){
                    Close12BallLeverAfter6().schedule();
                }  else if (pushLever == 3){
                    Close12BallLeverBoth().schedule();
                }
            }
            else if (intAmount == 15){
                if (pushLever == 0) {
                    Close15Ball().schedule();
                } else if (pushLever == 1){
                    Close15BallLeverAfter3().schedule();
                }  else if (pushLever == 2){
                    Close15BallLeverAfter6().schedule();
                }  else if (pushLever == 3){
                    Close15BallLeverBoth().schedule();
                }
        }

        // Persist ball count (and optionally pose) for TeleOp
        GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
        GlobalRobotData.hasAutonRun = true;
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

        /** We shouldn't need this because everything should automatically disable **/
        @Override
        public void onStop() {
            // Persist ball count (and optionally pose) for TeleOp
            ShooterSubsystem.INSTANCE.stop();
            GlobalRobotData.endAutonBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
            GlobalRobotData.endAutonPose = PedroComponent.follower().getPose();
            GlobalRobotData.hasAutonRun = true;
        }


    }

