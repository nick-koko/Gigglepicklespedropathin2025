package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.CsvLogger;

import java.io.File;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "Turret Subsystem Servo Test", group = "Test")
public class TurretSubsystemServoTestOpMode extends NextFTCOpMode {
    public TurretSubsystemServoTestOpMode() {
        addComponents(
                new SubsystemComponent(TurretSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public static boolean ENABLE_TURRET_TEST_LOGGING = true;
    public static boolean ENABLE_TURRET_TEST_ABSOLUTE_LOGGING = false;
    public static long TURRET_TEST_LOG_PERIOD_MS = 25;

    public static double TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC = 650.0;
    public static double FAKE_ROBOT_MAX_TURN_RATE_DEG_PER_SEC = 650.0;
    public static double STICK_DEADBAND = 0.05;
    public static double INITIAL_FAKE_ROBOT_HEADING_DEG = 0.0;
    public static double INITIAL_DESIRED_FIELD_HEADING_DEG = 180.0;

    private CsvLogger turretLogger;
    private long logStartMs = 0L;
    private long lastTurretLogMs = 0L;
    private double lastLoopTimeSec = 0.0;

    private double fakeRobotHeadingDeg = INITIAL_FAKE_ROBOT_HEADING_DEG;
    private double desiredFieldHeadingDeg = INITIAL_DESIRED_FIELD_HEADING_DEG;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;

    @Override
    public void onInit() {
        TurretSubsystem.INSTANCE.setPeriodicAbsoluteEncoderReadEnabled(ENABLE_TURRET_TEST_ABSOLUTE_LOGGING);
        TurretSubsystem.INSTANCE.setServoEnabledStates(true, true);

        if (ENABLE_TURRET_TEST_LOGGING) {
            turretLogger = new CsvLogger("pickles2025_turret_servo_test");
            turretLogger.start(
                    "t_ms," +
                            "match_t_ms," +
                            "fake_robot_heading_deg," +
                            "desired_field_heading_deg," +
                            "robot_relative_aim_deg," +
                            "left_stick_x," +
                            "right_stick_x," +
                            "left_servo_enabled," +
                            "right_servo_enabled," +
                            "turret_target_deg," +
                            "turret_target_corrected_deg," +
                            "turret_commanded_deg," +
                            "turret_servo_command_deg," +
                            "turret_measured_deg," +
                            "turret_error_deg," +
                            "turret_outer_trim_deg," +
                            "turret_command_diff_deg," +
                            "turret_rate_step_deg," +
                            "turret_left_cmd_deg," +
                            "turret_right_cmd_deg," +
                            "turret_servo_pos," +
                            "turret_left_servo_pos," +
                            "turret_right_servo_pos," +
                            "turret_quad_raw_deg," +
                            "turret_quad_offset_deg," +
                            "turret_encoder_vel_deg_s," +
                            "turret_abs_raw_deg," +
                            "turret_abs_deg," +
                            "turret_abs_delta_deg," +
                            "turret_learned_servo_offset_deg," +
                            "turret_abs_startup_error_deg," +
                            "turret_encoder_ticks," +
                            "turret_ready"
            );
        } else {
            turretLogger = null;
        }
    }

    @Override
    public void onStartButtonPressed() {
        logStartMs = System.currentTimeMillis();
        lastTurretLogMs = 0L;
        lastLoopTimeSec = nowSeconds();

        fakeRobotHeadingDeg = INITIAL_FAKE_ROBOT_HEADING_DEG;
        desiredFieldHeadingDeg = INITIAL_DESIRED_FIELD_HEADING_DEG;

        TurretSubsystem.INSTANCE.setPeriodicAbsoluteEncoderReadEnabled(ENABLE_TURRET_TEST_ABSOLUTE_LOGGING);
        TurretSubsystem.INSTANCE.setServoEnabledStates(true, true);
        TurretSubsystem.INSTANCE.center();
    }

    @Override
    public void onUpdate() {
        double nowSec = nowSeconds();
        double dt = 0.0;
        if (lastLoopTimeSec > 0.0) {
            dt = Math.max(0.0, nowSec - lastLoopTimeSec);
        }
        lastLoopTimeSec = nowSec;

        double leftStickX = applyDeadband(gamepad1.left_stick_x, STICK_DEADBAND);
        double rightStickX = applyDeadband(gamepad1.right_stick_x, STICK_DEADBAND);

        if (dt > 0.0) {
            desiredFieldHeadingDeg += leftStickX * TURRET_MANUAL_MAX_SPEED_DEG_PER_SEC * dt;
            fakeRobotHeadingDeg += rightStickX * FAKE_ROBOT_MAX_TURN_RATE_DEG_PER_SEC * dt;
        }

        boolean xPressed = gamepad1.x;
        boolean bPressed = gamepad1.b;
        boolean yPressed = gamepad1.y;
        boolean aPressed = gamepad1.a;
        boolean leftBumperPressed = gamepad1.left_bumper;
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (xPressed && !prevX) {
            TurretSubsystem.INSTANCE.setLeftServoEnabled(!TurretSubsystem.INSTANCE.isLeftServoEnabled());
        }
        if (bPressed && !prevB) {
            TurretSubsystem.INSTANCE.setRightServoEnabled(!TurretSubsystem.INSTANCE.isRightServoEnabled());
        }
        if (yPressed && !prevY) {
            TurretSubsystem.INSTANCE.setServoEnabledStates(true, true);
        }
        if (aPressed && !prevA) {
            desiredFieldHeadingDeg = fakeRobotHeadingDeg + 180.0;
        }
        if (leftBumperPressed && !prevLeftBumper) {
            fakeRobotHeadingDeg = 0.0;
        }
        if (rightBumperPressed && !prevRightBumper) {
            desiredFieldHeadingDeg = INITIAL_DESIRED_FIELD_HEADING_DEG;
        }

        prevA = aPressed;
        prevB = bPressed;
        prevX = xPressed;
        prevY = yPressed;
        prevLeftBumper = leftBumperPressed;
        prevRightBumper = rightBumperPressed;

        double robotRelativeAimDeg = wrapDegrees(desiredFieldHeadingDeg - fakeRobotHeadingDeg);
        TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(robotRelativeAimDeg);

        double turretMeasuredDeg = TurretSubsystem.INSTANCE.getMeasuredAngleDegrees();
        double turretTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();
        double turretErrorDeg = turretTargetDeg - turretMeasuredDeg;
        while (turretErrorDeg > 180.0) turretErrorDeg -= 360.0;
        while (turretErrorDeg < -180.0) turretErrorDeg += 360.0;

        telemetry.addData("fakeRobotHeadingDeg", "%.2f", fakeRobotHeadingDeg);
        telemetry.addData("desiredFieldHeadingDeg", "%.2f", desiredFieldHeadingDeg);
        telemetry.addData("robotRelativeAimDeg", "%.2f", robotRelativeAimDeg);
        telemetry.addData("turretTargetDeg", "%.2f", turretTargetDeg);
        telemetry.addData("turretMeasuredDeg", "%.2f", turretMeasuredDeg);
        telemetry.addData("turretErrorDeg", "%.2f", turretErrorDeg);
        telemetry.addData("turretServoCmdDeg", "%.2f", TurretSubsystem.INSTANCE.getServoCommandAngleDegrees());
        telemetry.addData("turretVelDegS", "%.2f", TurretSubsystem.INSTANCE.getMeasuredVelocityDegPerSec());
        telemetry.addData("leftServoEnabled", TurretSubsystem.INSTANCE.isLeftServoEnabled());
        telemetry.addData("rightServoEnabled", TurretSubsystem.INSTANCE.isRightServoEnabled());
        telemetry.addData("leftServoPos", "%.4f", TurretSubsystem.INSTANCE.getCurrentLeftServoPosition());
        telemetry.addData("rightServoPos", "%.4f", TurretSubsystem.INSTANCE.getCurrentRightServoPosition());
        telemetry.addData("turretReady", TurretSubsystem.INSTANCE.isTurretReady());
        telemetry.addLine("g1 left stick X: desired field heading at +/-650 deg/s");
        telemetry.addLine("g1 right stick X: fake robot turn rate at +/-650 deg/s");
        telemetry.addLine("g1 X: toggle left servo, B: toggle right servo, Y: enable both");
        telemetry.addLine("g1 A: center turret target, LB: reset fake heading, RB: reset field heading");
        telemetry.update();

        if (ENABLE_TURRET_TEST_LOGGING && turretLogger != null) {
            long nowLogMs = System.currentTimeMillis();
            if (lastTurretLogMs == 0L || nowLogMs - lastTurretLogMs >= TURRET_TEST_LOG_PERIOD_MS) {
                lastTurretLogMs = nowLogMs;
                long matchT = (logStartMs == 0L) ? 0L : (nowLogMs - logStartMs);

                turretLogger.addRow(
                        nowLogMs,
                        matchT,
                        fakeRobotHeadingDeg,
                        desiredFieldHeadingDeg,
                        robotRelativeAimDeg,
                        leftStickX,
                        rightStickX,
                        TurretSubsystem.INSTANCE.isLeftServoEnabled(),
                        TurretSubsystem.INSTANCE.isRightServoEnabled(),
                        TurretSubsystem.INSTANCE.getTargetAngleDegrees(),
                        TurretSubsystem.INSTANCE.getCorrectedTargetAngleDegrees(),
                        TurretSubsystem.INSTANCE.getCommandedAngleDegrees(),
                        TurretSubsystem.INSTANCE.getServoCommandAngleDegrees(),
                        turretMeasuredDeg,
                        turretErrorDeg,
                        TurretSubsystem.INSTANCE.getOuterLoopTrimDegrees(),
                        TurretSubsystem.INSTANCE.getCommandDiffDegrees(),
                        TurretSubsystem.INSTANCE.getRateLimitedStepDegrees(),
                        TurretSubsystem.INSTANCE.getLeftTurretAngleCommandDegrees(),
                        TurretSubsystem.INSTANCE.getRightTurretAngleCommandDegrees(),
                        TurretSubsystem.INSTANCE.getCurrentServoPosition(),
                        TurretSubsystem.INSTANCE.getCurrentLeftServoPosition(),
                        TurretSubsystem.INSTANCE.getCurrentRightServoPosition(),
                        TurretSubsystem.INSTANCE.getQuadRawAngleDegrees(),
                        TurretSubsystem.INSTANCE.getQuadratureOffsetDegrees(),
                        TurretSubsystem.INSTANCE.getMeasuredVelocityDegPerSec(),
                        TurretSubsystem.INSTANCE.getAbsoluteEncoderRawDegrees(),
                        TurretSubsystem.INSTANCE.getAbsoluteEncoderTurretAngleDegrees(),
                        TurretSubsystem.INSTANCE.getAbsoluteEncoderTurretDeltaDegrees(),
                        TurretSubsystem.INSTANCE.getLearnedServoCommandOffsetDegrees(),
                        TurretSubsystem.INSTANCE.getAbsoluteStartupErrorDegrees(),
                        TurretSubsystem.INSTANCE.getCurrentEncoderTicks(),
                        TurretSubsystem.INSTANCE.isTurretReady()
                );
            }
        }
    }

    @Override
    public void onStop() {
        super.onStop();
        if (ENABLE_TURRET_TEST_LOGGING && turretLogger != null) {
            File savedFile = turretLogger.save();
            if (savedFile != null) {
                RobotLog.ii("TurretSubsystemServoTest", "Turret test CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("TurretSubsystemServoTest", "Turret test CSV was not saved.");
            }
        }
    }

    private static double applyDeadband(double value, double deadband) {
        return Math.abs(value) >= Math.max(0.0, deadband) ? value : 0.0;
    }

    private static double wrapDegrees(double angleDegrees) {
        double wrapped = angleDegrees;
        while (wrapped > 180.0) wrapped -= 360.0;
        while (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }

    private static double nowSeconds() {
        return System.nanoTime() / 1e9;
    }
}
