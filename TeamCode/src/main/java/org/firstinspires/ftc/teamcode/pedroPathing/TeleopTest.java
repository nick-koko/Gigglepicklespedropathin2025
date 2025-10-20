package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleopTest extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(71,8,Math.toRadians(270)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    boolean goToTargetAngle;
    double targetAngleDeg = -135.0;
    double targetAngleRad;
    double propAngleGain = -0.5;
    double minAnglePower = 0.075;
    double maxRotate = 0.8;
    double angleAllianceOffset = 0.0;
    double drivePower = 1.0;
    public static double normDrivePower = 1;
    public static double slowedDrivePower = 0.5;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();


        double driving = (-gamepad1.left_stick_y) * drivePower;
        double strafe = (-gamepad1.left_stick_x) * drivePower;
        double rotate = (-gamepad1.right_stick_x) * 0.5;

        double botHeadingRad = follower.getPose().getHeading();
        double botxvalue = follower.getPose().getX(); //gettingxvalue :D
        double botyvalue = follower.getPose().getY(); //gettingyvalue :D
        double angletangent=0;
        double shootingangle=0;
        //double shootingangle = Math.toDegrees(Math.atan2(144-botyvalue,botxvalue));
        if (gamepad1.right_bumper) {
            angletangent = (144-botyvalue)/(144-botxvalue);
            shootingangle = Math.toDegrees(Math.atan(angletangent));
            shootingangle = shootingangle;
            targetAngleDeg = shootingangle + angleAllianceOffset;
            goToTargetAngle = true;
            //drivePower = slowedDrivePower;
        } else if (gamepad1.left_bumper) {
            angletangent = (144-botyvalue)/botxvalue;
            shootingangle = Math.toDegrees(Math.atan(angletangent));
            shootingangle = 180-shootingangle;
            targetAngleDeg = shootingangle + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_down) {
            targetAngleDeg = 180.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_right) {
            targetAngleDeg = -90.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_left) {
            targetAngleDeg = 90.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_up) {
            targetAngleDeg = 0.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else {
            goToTargetAngle = false;
        }

        targetAngleRad = Math.toRadians(targetAngleDeg);


//mR. TODONE 😎👌👌
        if (goToTargetAngle) {
            double targetAngleDiff = botHeadingRad - targetAngleRad;
            if (targetAngleDiff > Math.PI) {
                targetAngleDiff = (targetAngleDiff - 2 * (Math.PI));
            } else if (targetAngleDiff < -Math.PI) {
                targetAngleDiff = (2 * (Math.PI) + targetAngleDiff);
            }
            rotate = targetAngleDiff * propAngleGain;
            if (rotate > 0.0) {
                rotate = rotate + minAnglePower;
            } else if (rotate < 0.0) {
                rotate = rotate - minAnglePower;
            }
            rotate = Math.max(Math.min(rotate, maxRotate), -maxRotate);
        }
        if (!automatedDrive) {
            //Make the last parameter false for field-centric

            //This is the normal version to use in the TeleOp

            follower.setTeleOpDrive(
                    driving,
                    strafe,
                    rotate,
                    false // field Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetryM.debug("Triangle Side 1", botxvalue);
        telemetryM.debug("Triangle Side 1", 144-botyvalue);
        telemetryM.debug("Shootin Gangle", shootingangle);
        telemetryM.debug("Target Angle", targetAngleDeg);
        telemetryM.debug("Use Special Angle?", goToTargetAngle);
        telemetryM.debug("Final Rotate Power", rotate);
        telemetryM.debug("Final Rotate Drive", rotate);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}