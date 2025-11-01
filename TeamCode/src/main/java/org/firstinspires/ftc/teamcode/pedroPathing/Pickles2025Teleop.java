package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
@TeleOp(name = "Pickles 2025 Teleop", group = "Comp")
public class Pickles2025Teleop extends NextFTCOpMode {
    public Pickles2025Teleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE, LEDControlSubsystem.INSTANCE)
        );
    }
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
    private Limelight3A limelight;

    private double xOffset;
    private double yOffset;
    private double yDesired = 17;

    private double areaOffset;

    private double targetRPM = 0;
    private double shooterHoodPos = 0;
    private boolean hasResults = false;



    @Override
    public void onInit() {
        super.onInit();
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        if ((GlobalRobotData.endAutonPose != null) && (GlobalRobotData.hasAutonRun)) {
            startingPose = GlobalRobotData.endAutonPose;
            GlobalRobotData.hasAutonRun = false;
        }

        PedroComponent.follower().setStartingPose(startingPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);

        // Carry over ball count from auton if available
        if (GlobalRobotData.endAutonBallCount >= 0) {
            IntakeWithSensorsSubsystem.INSTANCE.setBallCount(GlobalRobotData.endAutonBallCount);
        }

        pathChain = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        PedroComponent.follower().startTeleopDrive();
    }

    @Override
    public void onUpdate() {
        //Call this once per loop
        telemetryM.update();
        telemetry.update();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            xOffset = result.getTx();
            yOffset = result.getTy();
            areaOffset = result.getTa();
        }


        double driving = (-gamepad1.left_stick_y) * drivePower;
        double strafe = (-gamepad1.left_stick_x) * drivePower;
        double rotate = (-gamepad1.right_stick_x) * 0.5;

        if(GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            driving *= -1;
            strafe *= -1;
        }

        double botHeadingRad = PedroComponent.follower().getPose().getHeading();
        double botxvalue = PedroComponent.follower().getPose().getX(); //gettingxvalue :D
        double botyvalue = PedroComponent.follower().getPose().getY(); //gettingyvalue :D
        double angletangent = 0;
        double shootingangle = 0;
        //double shootingangle = Math.toDegrees(Math.atan2(144-botyvalue,botxvalue)
        if(gamepad1.y){
            PedroComponent.follower().setPose(new Pose(71,8,Math.toRadians(270)));
        }

        if (gamepad1.right_bumper) {
            if (result != null && result.isValid()) {
                if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED) {
                    List<LLResultTypes.FiducialResult> tag24Results = result.getFiducialResults().stream()
                            .filter(r -> r.getFiducialId() == 24)
                            .collect(Collectors.toList());

                    if (!tag24Results.isEmpty()) {
                        hasResults = true;
                        double targetX = tag24Results.get(0).getTargetXDegrees();
                        if (targetX != 0) {
                            rotate = -targetX * 0.025;
                            goToTargetAngle = false;
                        }
                    }
                    else{
                        hasResults = false;
                    }
                }
                else{
                    List<LLResultTypes.FiducialResult> tag20Results = result.getFiducialResults().stream()
                        .filter(r -> r.getFiducialId() == 20)
                        .collect(Collectors.toList());

                    if (!tag20Results.isEmpty()) {
                        hasResults = true;
                        double targetX = tag20Results.get(0).getTargetXDegrees();
                        if (targetX != 0) {
                            rotate = -targetX * 0.025;
                            goToTargetAngle = false;
                        }
                    }
                    else{
                        hasResults = false;
                    }
                }


//                List<LLResultTypes.FiducialResult> tag20Results = result.getFiducialResults().stream()
//                        .filter(r -> r.getFiducialId() == 20)
//                        .collect(Collectors.toList());
//                if (!tag24Results.isEmpty())
        } else {
//            hasResults = false;
//            angletangent = (144 - botyvalue) / (144 - botxvalue);
//            shootingangle = Math.toDegrees(Math.atan(angletangent));
//            targetAngleDeg = shootingangle + angleAllianceOffset;
//            goToTargetAngle = true;
//                angletangent = (135-botyvalue)/botxvalue;
//                shootingangle = Math.toDegrees(Math.atan(angletangent));
//                shootingangle = shootingangle;
//                targetAngleDeg = 180 - shootingangle;
//                goToTargetAngle = true;
        }
//            else {
//                angletangent = (144-botyvalue)/botxvalue;
//                shootingangle = Math.toDegrees(Math.atan(angletangent));
//                shootingangle = 180-shootingangle;
//                targetAngleDeg = shootingangle;
//                goToTargetAngle = true

            //drivePower = slowedDrivePower;
//        } else if (gamepad1.left_bumper) {
//            if (!tag20Results.isEmpty() && tag20Results.get(0).getTargetXDegrees() > 0 || tag20Results.get(0).getTargetXDegrees() < 0) {
//                rotate = -xOffset * 0.025;
//                driving = (17 - yOffset) * 0.025;
//                strafe = (17 - yOffset) * 0.025;
//                goToTargetAngle = false;
//            }
//            else{
//                angletangent = (144-botyvalue)/(144-botxvalue);
//                shootingangle = Math.toDegrees(Math.atan(angletangent));
//                shootingangle = shootingangle;
//                targetAngleDeg = shootingangle + angleAllianceOffset;
//                goToTargetAngle = true;
//            }
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

            PedroComponent.follower().setTeleOpDrive(
                    driving,
                    strafe,
                    rotate,
                    false // field Centric
            );
        }

        //Automated PathFollowing
       /* if (gamepad1.aWasPressed()) {
            PedroComponent.follower().followPath(pathChain.get());
            automatedDrive = true;
        }*/


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !PedroComponent.follower().isBusy())) {
            PedroComponent.follower().startTeleopDrive();
            automatedDrive = false;
        }

        telemetryM.debug("Triangle Side 1", botxvalue);
        telemetryM.debug("Triangle Side 1", 144-botyvalue);
        telemetryM.debug("Shootin Gangle", shootingangle);
        telemetryM.debug("Target Angle", targetAngleDeg);
        telemetryM.debug("Use Special Angle?", goToTargetAngle);
        telemetryM.debug("Final Rotate Power", rotate);
        telemetryM.debug("Final Rotate Drive", rotate);
        telemetryM.debug("position", PedroComponent.follower().getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("ballCount", IntakeWithSensorsSubsystem.INSTANCE.getBallCount());
        telemetry.addData("sensor0", IntakeWithSensorsSubsystem.INSTANCE.isSensor0Broken());
        telemetry.addData("sensor1", IntakeWithSensorsSubsystem.INSTANCE.isSensor1Broken());
        telemetry.addData("sensor2", IntakeWithSensorsSubsystem.INSTANCE.isSensor2Broken());
        telemetry.addData("Shooter Hood Value", ShooterSubsystem.INSTANCE.getShooterHoodPosition());
        telemetry.addData("Shooter Speed", ShooterSubsystem.INSTANCE.getShooter1RPM());
        telemetry.addData("Target Shooter Speed", ShooterSubsystem.INSTANCE.getTargetShooterRPM());


        if (result == null) {
            telemetry.addData("Limelight", "No result object");
        } else if (!result.isValid()) {
            telemetry.addData("Limelight", "No valid target");
        } else {
            telemetry.addData("Limelight", "Target seen!");
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("ty", result.getTy());
        }
/* My controls
        if(gamepad2.right_trigger > 0.1)
        {
            IntakeWithSensorsSubsystem.INSTANCE.intake();
            //ShooterSubsystem.INSTANCE.stop();
        }
        else if(gamepad2.left_trigger > 0.1)
        {
            IntakeWithSensorsSubsystem.INSTANCE.outtake();
            //ShooterSubsystem.INSTANCE.stop();
        }
        else{
            IntakeWithSensorsSubsystem.INSTANCE.stop();
        }


        if(gamepad1.right_trigger > 0.1)
        {
            ShooterSubsystem.INSTANCE.spinUp(3500);
        }
        if(gamepad1.left_trigger > 0.1) {
            ShooterSubsystem.INSTANCE.spinUp(5500);
        }
        if(gamepad1.bWasPressed()){
            ShooterSubsystem.INSTANCE.stop();
        }
 End My controls */
//Start Ian's controls
        if (!hasResults) {
            targetRPM = 3000;
        } else {
            targetRPM = 33 * Math.pow(yOffset, 2) - 1038 * yOffset + 11618;
        }
        
        if (gamepad2.rightBumperWasPressed()) {
            telemetry.addData("Target Shooter Speed", targetRPM);
            ShooterSubsystem.INSTANCE.spinUp(targetRPM);
        }
        else if (gamepad2.leftBumperWasPressed()) {
            ShooterSubsystem.INSTANCE.stop();
            //ShooterSubsystem.INSTANCE.decreaseShooterRPMBy10();
        }

//        if (gamepad2.xWasPressed()) {
//            ShooterSubsystem.INSTANCE.decreaseShooterHoodPosInc();
//        }
//        if(gamepad2.yWasPressed()) {
//            ShooterSubsystem.INSTANCE.increaseShooterHoodPosInc();
//        }

        if (gamepad2.right_trigger > 0.1) {
            IntakeWithSensorsSubsystem.INSTANCE.shoot();
        }
        else if (gamepad2.aWasPressed()) {
            IntakeWithSensorsSubsystem.INSTANCE.intakeForward();  //Hoping Forward is Intake (maybe change the method name)
            ShooterSubsystem.INSTANCE.stop();
        }
        else if (gamepad2.b) {
            IntakeWithSensorsSubsystem.INSTANCE.intakeReverse();
        }
        if (!hasResults) {
            this.shooterHoodPos = 0.20;
        } else if (yOffset <= 13.0) {
            this.shooterHoodPos = 0.43;
        } else {
            this.shooterHoodPos = 0.40;
        }
        ShooterSubsystem.INSTANCE.shooterHoodDrive(this.shooterHoodPos);


        if(IntakeWithSensorsSubsystem.INSTANCE.getNumberOfBalls() == 3) {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
        }
        else if(ShooterSubsystem.INSTANCE.isAtTargetSpeed(100) && ShooterSubsystem.INSTANCE.isEnabled()){
            LEDControlSubsystem.INSTANCE.startStrobe(LEDControlSubsystem.LedColor.GREEN , LEDControlSubsystem.LedColor.OFF ,500);
        }
        else{
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.RED);
        }

    }
}