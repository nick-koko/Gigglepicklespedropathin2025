package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;

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
    private IntakeWithSensorsSubsystem intakeSubsystem;

    private ShooterSubsystem shooterSubsystem;

    private Limelight3A limelight;

    private double xOffset;
    private double yOffset;
    private double yDesired = 17;

    private double areaOffset;


    @Override
    public void init() {
        intakeSubsystem = new IntakeWithSensorsSubsystem();
        intakeSubsystem.initialize(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        shooterSubsystem = new ShooterSubsystem();
        shooterSubsystem.initialize(hardwareMap);
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
        intakeSubsystem.periodic(); // Need this if not extending NextFTCOpmode
        shooterSubsystem.periodic();
        LLResult result = limelight.getLatestResult();
        if (result !=null && result.isValid()){
                xOffset = result.getTx();
                yOffset = result.getTy();
                areaOffset = result.getTa();
        }


        double driving = (-gamepad1.left_stick_y) * drivePower;
        double strafe = (-gamepad1.left_stick_x) * drivePower;
        double rotate = (-gamepad1.right_stick_x) * 0.5;

        double botHeadingRad = follower.getPose().getHeading();
        double botxvalue = follower.getPose().getX(); //gettingxvalue :D
        double botyvalue = follower.getPose().getY(); //gettingyvalue :D
        double angletangent=0;
        double shootingangle=0;
        //double shootingangle = Math.toDegrees(Math.atan2(144-botyvalue,botxvalue));
        List<LLResultTypes.FiducialResult> tag24Results = result.getFiducialResults().stream()
                .filter(r -> r.getFiducialId() == 24)
                .collect(Collectors.toList());

        List<LLResultTypes.FiducialResult> tag20Results = result.getFiducialResults().stream()
                .filter(r -> r.getFiducialId() == 20)
                .collect(Collectors.toList());

        if (gamepad1.right_bumper) {
//            if (!tag24Results.isEmpty() && tag24Results.get(0).getTargetXDegrees() > 0 || tag24Results.get(0).getTargetXDegrees() < 0) {
//                rotate = -xOffset * 0.025;
//                driving = (17 - yOffset) * 0.025;
//                strafe = (17 - yOffset) * 0.025;
//                goToTargetAngle = false;
//            }
//            else {
//                angletangent = (144-botyvalue)/botxvalue;
//                shootingangle = Math.toDegrees(Math.atan(angletangent));
//                shootingangle = 180-shootingangle;
//                targetAngleDeg = shootingangle;
//                goToTargetAngle = true;
                  angletangent = (144-botyvalue)/(144-botxvalue);
                  shootingangle = Math.toDegrees(Math.atan(angletangent));
                  shootingangle = shootingangle;
                  targetAngleDeg = shootingangle + angleAllianceOffset;
                  goToTargetAngle = true;

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
            this.intakeSubsystem.intake();
            //this.shooterSubsystem.stop();
        }
        else if(gamepad2.left_trigger > 0.1)
        {
            this.intakeSubsystem.outtake();
            //this.shooterSubsystem.stop();
        }
        else{
            this.intakeSubsystem.stop();
        }


        if(gamepad1.right_trigger > 0.1)
        {
            shooterSubsystem.spinUp(3500);
        }
        if(gamepad1.left_trigger > 0.1) {
            shooterSubsystem.spinUp(5500);
        }
        if(gamepad1.bWasPressed()){
            shooterSubsystem.stop();
        }
 End My controls */
//Start Ian's controls
        if (gamepad2.rightBumperWasPressed()) {
            this.shooterSubsystem.spinUp(4000);
        }
        else if (gamepad2.leftBumperWasPressed()) {
            this.shooterSubsystem.stop();
        }

        if (gamepad2.right_trigger > 0.1) {
            this.intakeSubsystem.shoot();
        }
        else if (gamepad2.aWasPressed()) {
            this.intakeSubsystem.intakeForward();  //Hoping Forward is Intake (maybe change the method name)
            this.shooterSubsystem.stop();
        }
        else if (gamepad2.b) {
            this.intakeSubsystem.intakeReverse();
        }

    }
}