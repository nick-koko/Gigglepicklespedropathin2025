package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "Pickles 2025 Teleop", group = "Comp")
public class Pickles2025Teleop extends NextFTCOpMode {
    public Pickles2025Teleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeWithSensorsSubsystem.INSTANCE, LEDControlSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE
                //FateComponent.INSTANCE
        );
    }

    public static Pose startingPose = new Pose(32.5, 134.375, Math.toRadians(180)); //See ExampleAuto to understand how to use this

    //public static Pose redShootingTarget = new Pose(127.63, 130.35, Math.toRadians(36));
    public static Pose redShootingTarget = new Pose(144, 144, Math.toRadians(36));
    public static Pose blueShootingTarget = redShootingTarget.mirror();

    // Adjust these from Panels at runtime
    public static boolean hold = false;
    public static boolean SHOW_SMOOTHED = false;
    public static int SMOOTH_WINDOW = 8;           // samples for moving average
    private final LoopTimer timer = new LoopTimer();
    private double rpmShooter1Smoothed = 0.0;
    private double rpmShooter2Smoothed = 0.0;
    private double rpmOuttakeSmoothed = 0.0;
    private double[] windowShooter1;
    private int wIdxShooter1 = 0;
    private int wFillShooter1 = 0;
    private double[] windowShooter2;
    private int wIdxShooter2 = 0;
    private int wFillShooter2 = 0;
    private double[] windowOuttake;
    private int wIdxOuttake = 0;
    private int wFillOuttake = 0;

    private Pose shootingTargetLocation;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    boolean goToTargetAngle;
    double targetAngleDeg = -135.0;
    double targetAngleRad;
    double propAngleGain = -0.5;
    public static double shooterTargetkP = 0.0185;
    double minAnglePower = 0.075;
    double maxRotate = 0.8;
    double angleAllianceOffset = 0.0;
    double drivePower = 1.0;
    public static double normDrivePower = 1;
    public static double slowedDrivePower = 0.5;
    private Limelight3A limelight;
    private double cameraHeightFromTags = 18.25;

    private double xOffset;
    private double yOffset;
    private double distanceLL;
    private double ODODistance;
    private double yDesired = 17;

    private double areaOffset;
    public static boolean testShooter = false;
    public static double targetRPM = 0;
    public static double shooterHoodPos = 0;
    private boolean hasResults = false;
    private boolean selectAllianceSide = false;

    private boolean shoot = false;


    @Override
    public void onInit() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        if ((GlobalRobotData.endAutonPose != null) && (GlobalRobotData.hasAutonRun)) {
            startingPose = GlobalRobotData.endAutonPose;
            GlobalRobotData.hasAutonRun = false;
        } else {
            selectAllianceSide = true;
        }

        PedroComponent.follower().setStartingPose(startingPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        windowShooter1 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowShooter2 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowOuttake = new double[Math.max(1, SMOOTH_WINDOW)];

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

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {

        if (selectAllianceSide) {
            if (gamepad1.xWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
            } else if (gamepad1.bWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
            }

            telemetry.addLine("Hello Pickle of the robot");
            telemetry.addLine("This is an Mr. Todone Speaking,");
            telemetry.addLine("----------------------------------------------");
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                telemetry.addLine("Favorite fruit: Blueberries!!! (Blue)");
            } else {
                telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
            }

            telemetry.update();

        }
    }

    @Override
    public void onStartButtonPressed() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        PedroComponent.follower().startTeleopDrive();

        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED) {
            shootingTargetLocation = redShootingTarget;
        } else {
            shootingTargetLocation = blueShootingTarget;
        }

    }

    @Override
    public void onUpdate() {
        //Call this once per loop
        timer.start();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            xOffset = result.getTx();
            yOffset = result.getTy();
            areaOffset = result.getTa();
            distanceLL = cameraHeightFromTags / (Math.tan(Math.toRadians(yOffset)));
        }
        ODODistance = PedroComponent.follower().getPose().distanceFrom(shootingTargetLocation);

        double driving = (-gamepad1.left_stick_y) * drivePower;
        double strafe = (-gamepad1.left_stick_x) * drivePower;
        double rotate = (-gamepad1.right_stick_x) * 0.5;

        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            driving *= -1;
            strafe *= -1;
        }

        double botHeadingRad = PedroComponent.follower().getPose().getHeading();
        double botxvalue = PedroComponent.follower().getPose().getX(); //gettingxvalue :D
        double botyvalue = PedroComponent.follower().getPose().getY(); //gettingyvalue :D

        double shootTargetX = shootingTargetLocation.getX();
        double shootTargetY = shootingTargetLocation.getY();

        // Vector from robot -> target
        double dx = shootTargetX - botxvalue;
        double dy = shootTargetY - botyvalue;

        // 1) Field-relative angle to the target (0 rad along +X, CCW positive)
        double fieldAngleRad = Math.atan2(dy, dx);
        // - fieldAngleDeg  -> "absolute field heading I want the robot to face"
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);

        // 2) Robot-relative angle error (how much you need to turn or point the turret)
        double angleErrorRad = normalizeRadians(fieldAngleRad - botHeadingRad);
        // - angleErrorDeg  -> "how many degrees left/right from current heading"
        double angleErrorDeg = Math.toDegrees(angleErrorRad);


        double angletangent = 0;
        double shootingangle = 0;
        // Add location based shooting angle here eventually
        //double shootingangle = Math.toDegrees(Math.atan2(144-botyvalue,botxvalue)
        if (gamepad1.y) {
            PedroComponent.follower().setPose(new Pose(71, 8, Math.toRadians(270)));
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
                        if (targetX != 3. && tag24Results.get(0).getTargetYDegrees() > 10) {
                            rotate = (-targetX + 2.25) * shooterTargetkP;
                            goToTargetAngle = false;
                        }
                        else if(targetX != 0){
                            rotate = (-targetX) * shooterTargetkP;
                            goToTargetAngle = false;
                        }
                    } else {
                        hasResults = false;
                    }
                } else {
                    List<LLResultTypes.FiducialResult> tag20Results = result.getFiducialResults().stream()
                            .filter(r -> r.getFiducialId() == 20)
                            .collect(Collectors.toList());

                    if (!tag20Results.isEmpty()) {
                        hasResults = true;
                        double targetX = tag20Results.get(0).getTargetXDegrees();
                        if (targetX != -3. && tag20Results.get(0).getTargetYDegrees() > 10) {
                            rotate = (-targetX) * shooterTargetkP;
                            goToTargetAngle = false;
                        }
                        else if(targetX != 0) {
                            rotate = (-targetX) * shooterTargetkP;
                            goToTargetAngle = false;
                        }
                    } else {
                        hasResults = false;
                    }
                }

                if (hasResults) {
                    LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.WHITE);
                    if (shoot) {
                        if (hasResults && yOffset > 9.) {
                            ShooterSubsystem.INSTANCE.setClosePID();
                            targetRPM = calculateShooterRPM(yOffset);
                        } else if (hasResults && yOffset <= 9.) {
                            ShooterSubsystem.INSTANCE.setFarPID();
                            targetRPM = 4330;
                        }
                        ShooterSubsystem.INSTANCE.spinUp(targetRPM);
                    }
                } else {
                    LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
                }

            }
        } else {
            // no valid result or bumper not held
        }

        if (gamepad1.left_bumper) {
            rotate = angleErrorDeg * shooterTargetkP;
            goToTargetAngle = false;
        } else if (gamepad1.right_trigger > 0.1) {
            targetAngleDeg = 180.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.left_trigger > 0.1) {
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
            if (!hold) {
                PedroComponent.follower().setTeleOpDrive(
                        driving,
                        strafe,
                        rotate,
                        false // field Centric
                );
            }
        }

        //Automated PathFollowing
       /* if (gamepad1.aWasPressed()) {
            PedroComponent.follower().followPath(pathChain.get());
            automatedDrive = true;
        }*/

        if (gamepad1.dpadDownWasPressed()) {
            hold = !hold;

            if (hold) {
                PedroComponent.follower().holdPoint(new BezierPoint(PedroComponent.follower().getPose()), PedroComponent.follower().getHeading(), false);
            } else {
                PedroComponent.follower().startTeleopDrive();
            }
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !PedroComponent.follower().isBusy())) {
            PedroComponent.follower().startTeleopDrive();
            automatedDrive = false;
        }

        double rpmShooter1 = ShooterSubsystem.INSTANCE.getShooter1RPM();
        double rpmShooter2 = ShooterSubsystem.INSTANCE.getShooter2RPM();
        double rpmOuttake = IntakeWithSensorsSubsystem.INSTANCE.getMotor3RPM();

        // Optional simple moving average
        if (SHOW_SMOOTHED) {
            if (windowShooter1.length != Math.max(1, SMOOTH_WINDOW)) {
                // if you change SMOOTH_WINDOW live, re-init the buffer on the fly
                windowShooter1 = new double[Math.max(1, SMOOTH_WINDOW)];
                wIdxShooter1 = 0;
                wFillShooter1 = 0;
                rpmShooter1Smoothed = 0.0;
            }
            windowShooter1[wIdxShooter1] = rpmShooter1;
            wIdxShooter1 = (wIdxShooter1 + 1) % windowShooter1.length;
            if (wFillShooter1 < windowShooter1.length) wFillShooter1++;

            double sum = 0.0;
            for (int i = 0; i < wFillShooter1; i++) sum += windowShooter1[i];
            rpmShooter1Smoothed = sum / wFillShooter1;

            if (windowShooter2.length != Math.max(1, SMOOTH_WINDOW)) {
                // if you change SMOOTH_WINDOW live, re-init the buffer on the fly
                windowShooter2 = new double[Math.max(1, SMOOTH_WINDOW)];
                wIdxShooter2 = 0;
                wFillShooter2 = 0;
                rpmShooter2Smoothed = 0.0;
            }
            windowShooter2[wIdxShooter2] = rpmShooter2;
            wIdxShooter2 = (wIdxShooter2 + 1) % windowShooter2.length;
            if (wFillShooter2 < windowShooter2.length) wFillShooter2++;

            sum = 0.0;
            for (int i = 0; i < wFillShooter2; i++) sum += windowShooter2[i];
            rpmShooter2Smoothed = sum / wFillShooter2;

            if (windowOuttake.length != Math.max(1, SMOOTH_WINDOW)) {
                // if you change SMOOTH_WINDOW live, re-init the buffer on the fly
                windowOuttake = new double[Math.max(1, SMOOTH_WINDOW)];
                wIdxOuttake = 0;
                wFillOuttake = 0;
                rpmOuttakeSmoothed = 0.0;
            }
            windowOuttake[wIdxOuttake] = rpmOuttake;
            wIdxOuttake = (wIdxOuttake + 1) % windowOuttake.length;
            if (wFillOuttake < windowOuttake.length) wFillOuttake++;

            sum = 0.0;
            for (int i = 0; i < wFillOuttake; i++) sum += windowOuttake[i];
            rpmOuttakeSmoothed = sum / wFillOuttake;

        }

        // Graph
        telemetryM.addData("Shooter1_RPM", rpmShooter1);
        telemetryM.addData("Calc Shooter1_RPM", ShooterSubsystem.INSTANCE.getShooter1RpmDelta());
        telemetryM.addData("Shooter2_RPM", rpmShooter2);
        telemetryM.addData("Calc Shooter2_RPM", ShooterSubsystem.INSTANCE.getShooter1RpmDelta());
        telemetryM.addData("Calc Avg_RPM", ShooterSubsystem.INSTANCE.getAverageRpmDelta());
        telemetryM.addData("Outtake_RPM", rpmOuttake);

        if (SHOW_SMOOTHED) {
            telemetryM.addData("Shooter1_RPM (smoothed)", rpmShooter1Smoothed);
            telemetryM.addData("Shooter2_RPM (smoothed)", rpmShooter2Smoothed);
            telemetryM.addData("Outtake_RPM (smoothed)", rpmOuttakeSmoothed);
        }

        boolean bb0 = IntakeWithSensorsSubsystem.INSTANCE.isSensor0Broken();
        boolean bb1 = IntakeWithSensorsSubsystem.INSTANCE.isSensor1Broken();
        boolean bb2 = IntakeWithSensorsSubsystem.INSTANCE.isSensor2Broken();

        telemetryM.addData("ballCount", IntakeWithSensorsSubsystem.INSTANCE.getBallCount());
        telemetryM.addData("BB_sensor0", bb0 ? 1 : 0);
        telemetryM.addData("BB_sensor1", bb1 ? 1 : 0);
        telemetryM.addData("BB_sensor2", bb2 ? 1 : 0);

        telemetryM.addData("LoopTime_ms", timer.getMs());
        telemetry.addData("rotate", rotate);
        telemetry.addData("boostActive", ShooterSubsystem.INSTANCE.boostActive);
        telemetryM.addData("targetRPM", ShooterSubsystem.INSTANCE.getTargetRpm());

        if (testShooter) {
            telemetry.addData("kP", ShooterSubsystem.kP);
            telemetry.addData("kI", ShooterSubsystem.kI);
            telemetry.addData("kD", ShooterSubsystem.kD);
            telemetry.addData("kS", ShooterSubsystem.kS);
            telemetry.addData("kV", ShooterSubsystem.kV);
            telemetry.addData("kA", ShooterSubsystem.kA);
            telemetry.addData("HEADROOM", ShooterSubsystem.HEADROOM);
            telemetry.addData("SLEW_PER_SECOND", ShooterSubsystem.SLEW_PER_SECOND);
            telemetry.addData("I_ZONE", ShooterSubsystem.I_ZONE);
        }

        if (result == null) {
            telemetry.addData("Limelight", "No result object");
        } else if (!result.isValid()) {
            telemetry.addData("Limelight", "No valid target");
        } else {
            telemetry.addData("Limelight", "Target seen!");
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("LL distance", distanceLL);
        }
        telemetry.addData("ODO distance", ODODistance);
        telemetry.addData("ODO X-Location", botxvalue);
        telemetry.addData("ODO Y-Location", botyvalue);



        //Start Ian's control

        if (gamepad2.rightBumperWasPressed()) {
            this.shoot = true;
            if (testShooter) {

            }else if (!hasResults) {
                ShooterSubsystem.INSTANCE.setClosePID();
//                targetRPM = calculateShooterRPMFromDistance(this.ODODistance);
                targetRPM = 2950;
            } else if (hasResults && yOffset > 9.) {
                ShooterSubsystem.INSTANCE.setClosePID();
                targetRPM = calculateShooterRPM(yOffset);
            } else if (hasResults && yOffset <= 9.) {
                ShooterSubsystem.INSTANCE.setFarPID();
                targetRPM = 4330;
            }
            //ShooterSubsystem.INSTANCE.increaseShooterRPMBy10();
            telemetry.addData("Target Shooter Speed", targetRPM);
            ShooterSubsystem.INSTANCE.spinUp(targetRPM);
        } else if (gamepad2.leftBumperWasPressed()) {
            ShooterSubsystem.INSTANCE.stop();
            //ShooterSubsystem.INSTANCE.decreaseShooterRPMBy10();
        }

        if (gamepad2.xWasPressed()) {
            ShooterSubsystem.INSTANCE.decreaseShooterHoodPosInc();
        }
        if (gamepad2.yWasPressed()) {
            ShooterSubsystem.INSTANCE.increaseShooterHoodPosInc();
        }

        if (gamepad2.right_trigger > 0.1) {
            long delay = 0;
            long shotTime = 250;
//            if (hasResults && yOffset >= 10) {
//                delay = 100;
//                IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();
//                ShooterSubsystem.INSTANCE.setBoostOn();
//            } else if (hasResults && yOffset < 11) {
//                delay = 700;
//                IntakeWithSensorsSubsystem.INSTANCE.shoot(shotTime, delay);
//                ShooterSubsystem.INSTANCE.setBoostOn();
//            }
            if (yOffset < 11){
                IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();
                ShooterSubsystem.INSTANCE.setBoostOn(true);
            }
            else{
                IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();
                ShooterSubsystem.INSTANCE.setBoostOn(false);

            }
            //IntakeWithSensorsSubsystem.INSTANCE.shoot(shotTime, delay);
        } else if (gamepad2.aWasPressed()) {
            this.shoot = false;
            IntakeWithSensorsSubsystem.INSTANCE.intakeForward();  //Hoping Forward is Intake (maybe change the method name)
            ShooterSubsystem.INSTANCE.stop();
            this.hasResults = false;
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.RED);
        } else if (gamepad2.b) {
            IntakeWithSensorsSubsystem.INSTANCE.intakeReverse();
        }

        if (testShooter) {

        } else if (hasResults) {  //if limelight doesn't have results then use ODO Distance - Thinking that it would be better to always use ODO distance unless pressing a button to use limelight?
            //this.shooterHoodPos = getHoodPositionFromDistance(this.ODODistance);
            //this.shooterHoodPos = 0.05;
//        } else{
            this.shooterHoodPos = getHoodPosition(yOffset);

        }
//        }
        ShooterSubsystem.INSTANCE.shooterHoodDrive(this.shooterHoodPos);


        // LED status based on number of balls
        int balls = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
        if (balls >= 3) {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
        } else if (balls == 2) {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.YELLOW);
        } else if (balls == 1) {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.ORANGE);
        } else {
            LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.RED);
        }

        telemetryM.update(telemetry);
        telemetry.update();
        timer.end();
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public static double calculateShooterRPM(double yOffset) {
        if (yOffset > 17)
            return 3350;
        //BEST ONE TO FALL BACK TO
//        double rpm = -13746
//                + 6315 * yOffset
//                - 805 * Math.pow(yOffset, 2)
//                + 43.7 * Math.pow(yOffset, 3)
//                - 0.867 * Math.pow(yOffset, 4);

        double rpm = 0;
        if (yOffset >= 17 && yOffset < 18) {
            //3350
            rpm = -117.65 * yOffset + 5385.3;
        }
        else if (yOffset >= 16 && yOffset < 17) {
            //rpm = -140.85 * yOffset + 5823.3;
//            rpm = -111.06 * yOffset + 5236.4;
//            rpm = -110.23 * Math.pow(yOffset, 2) +
//                    3562.9 * yOffset - 25383;
            rpm = 23.685 * yOffset + 3022.5;
        }
        else if (yOffset >= 15 && yOffset < 16) {
            //rpm = -87.719 * yOffset + 4883.3;
//            rpm = -24.878 * Math.pow(yOffset, 2) +
//                    671.77 * yOffset - 917.08;
//            rpm = -94.202 * yOffset + 4905.1;
            rpm = 26.838 * Math.pow(yOffset, 2) -
                    871.7 * yOffset + 10466;
        }
        else if (yOffset >= 14 && yOffset < 15) {
//            rpm = 551.15 * Math.pow(yOffset, 2) -
//                    15981 * yOffset + 119441;
//            rpm = 325.12 * Math.pow(yOffset, 3) - 13857 * Math.pow(yOffset, 2) +
//                    196702 * yOffset - 926207;
//            rpm = -31.719 * Math.pow(yOffset, 2) +
//                    789.01 * yOffset - 1158;
            rpm = -127.72 * yOffset + 5433.5;

        }
        else if (yOffset >= 13 && yOffset < 14) {
//            rpm = -68.493 * yOffset + 4597.9;
//            rpm = -86.153 * yOffset + 4847;
//            rpm = -20.44 * yOffset + 3954.2;
            rpm = -152.89 * yOffset + 5697.3;
        }
        else if (yOffset >= 12 && yOffset < 13) {
//            rpm = 196.87 * Math.pow(yOffset, 2) -
//                    5160.3 * yOffset + 37487;
//            rpm = 237.83 * Math.pow(yOffset, 2) -
//                    6155.5 * yOffset + 43528;
            rpm = -167.96 * yOffset + 5830.5;
        }
        else if (yOffset >= 11 && yOffset < 12) {
//            rpm = 288.35 * Math.pow(yOffset, 2) -
//                    6679.9 * yOffset + 42487;
//            rpm = 291.25 * Math.pow(yOffset, 2) -
//                    6746.5 * yOffset + 42869;
            rpm = -14.425 * yOffset + 3956.9;
        }
        else if (yOffset >= 10 && yOffset < 11) {
//            rpm = 3870;
            rpm = -89.495 * yOffset + 4836.9;
        }
        else if (yOffset >= 9 && yOffset < 10) {
            rpm = 4000;
        }
        else{
            rpm = 4250;
        }

        return rpm;
    }

    public static double calculateShooterRPMFromDistance(double distanceIn) {
        double distanceMm = distanceIn * 25.4;
        return 1616
                + 1.34 * distanceMm
                - 0.000168 * distanceMm * distanceMm;
    }

    public static double getHoodPosition(double yOffset) {
        // Default hood position if yOffset is out of known range

        if (yOffset >= 17.08) {
            return 0.3;
        }

        // Small 0.3 dip around 16.14
        if (yOffset >= 16.3 && yOffset <= 16.) {
            return 0.3;
        }

        // Small 0.3 dip around 14.18
        if (yOffset >= 14.3 && yOffset <= 14.) {
            return 0.3;
        }

        // --- 0.5 region (12.3 → 12.0 generalized) ---
        if (yOffset > 12.0 && yOffset < 12.9) {
            return 0.5;
        }

        // --- Main 0.4 region (everything else inside your measured range) ---
        if (yOffset >= 9.6) {
            return 0.4;
        }

        if (yOffset >= 10 && yOffset < 11) {
            return 0.5;
        }

        if (yOffset >= 11 && yOffset < 12) {
            return 0.5;
        }

        if (yOffset <= 10) {
            return 0.5;
        }

        // --- Below recorded range, assume last known ---
        return 0.4;
    }

    public static double getHoodPositionFromDistance(double distanceIn) {

        double distanceMm = distanceIn * 25.4;
        // Default hood position if distance is outside known range
        double hoodPos = 0.35;

        if (distanceMm <= 1379) {          // 1282–1379 mm
            hoodPos = 0.2;
        } else if (distanceMm <= 1637) {   // 1626–1637 mm
            hoodPos = 0.3;
        } else if (distanceMm <= 1910) {   // 1850–1910 mm
            hoodPos = 0.4;
        } else if (distanceMm <= 2299) {   // 2105–2299 mm
            hoodPos = 0.3;
        } else if (distanceMm <= 2552) {   // 2552 mm
            hoodPos = 0.4;
        } else if (distanceMm > 2552) {    // 2945 mm
            hoodPos = 0.5;
        }

        return hoodPos;
    }
}



