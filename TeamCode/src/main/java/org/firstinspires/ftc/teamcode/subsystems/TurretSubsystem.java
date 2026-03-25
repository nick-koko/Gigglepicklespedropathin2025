package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Turret subsystem derived from TurretServoTest.
 * Uses dual servos with an absolute encoder-based outer loop.
 */
@Configurable
public class TurretSubsystem implements Subsystem {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    private TurretSubsystem() {}

    // =========================
    // Hardware names
    // =========================
    public static String LEFT_TURRET_SERVO_NAME = "Lurret";
    public static String RIGHT_TURRET_SERVO_NAME = "Rurret";
    public static String TURRET_ENCODER_NAME = "back_left";
    public static String ABSOLUTE_TURRET_ENCODER_NAME = "analog_turret_encoder";

    // =========================
    // Configurable constants
    // =========================
    public static double SERVO_CENTER_POSITION = 0.5;
    public static double TURRET_TRAVEL_DEGREES = 355.0;
    public static double INITIAL_ANGLE_DEGREES = 0.0;
    public static double MIN_SERVO_ROTATION_DEGREES = -130.0;
    public static double MAX_SERVO_ROTATION_DEGREES = 130.0;
    public static double MIN_SERVO_POSITION = 0.0;
    public static double MAX_SERVO_POSITION = 1.0;

    public static double RATE_LIMIT_DEG_PER_SEC = 1.5 * TURRET_TRAVEL_DEGREES;
    public static double OUTER_LOOP_KP = 0.12;
    public static double OUTER_LOOP_MAX_TRIM_DEGREES = 8.0;

    public static double POSITIVE_TARGET_BIAS_DEGREES = 0.0;
    public static double NEGATIVE_TARGET_BIAS_DEGREES = 0.0;
    public static double TARGET_BIAS_APPLY_THRESHOLD_DEGREES = 1.0;
    public static double ROBOT_FRONT_RELATIVE_SIGN = 1.0;
    public static double ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES = 180.0;

    public static double READY_TOLERANCE_DEGREES = 2.0;
    public static double READY_VELOCITY_TOLERANCE_DEG_PER_SEC = 15.0;
    public static int READY_LOOPS_REQUIRED = 3;
    public static int NOT_READY_LOOPS_REQUIRED = 3;

    public static double STARTUP_EXPECTED_TURRET_ANGLE_DEGREES = 0.0;
    public static long STARTUP_CENTER_SETTLE_MS = 300;
    public static double ABSOLUTE_TURRET_ENCODER_MAX_VOLTAGE = 3.255;
    public static double ENCODER_TO_TURRET_RATIO = 4.8;
    public static double TURRET_ENCODER_CPR = 4096.0;
    public static double TURRET_ENCODER_COUNTS_PER_REV = TURRET_ENCODER_CPR * ENCODER_TO_TURRET_RATIO;
    public static double TURRET_ENCODER_COUNTS_PER_DEGREE = TURRET_ENCODER_COUNTS_PER_REV / 360.0;
    public static double ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES = 0.0;

    // =========================
    // Hardware
    // =========================
    private Servo leftTurret;
    private Servo rightTurret;
    private DcMotor turretEncoder;
    private AnalogInput absoluteTurretEncoder;

    // =========================
    // Runtime state
    // =========================
    private double targetAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double commandedAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double measuredAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double measuredVelocityDegPerSec = 0.0;
    private double quadratureOffsetDegrees = 0.0;
    private double quadRawAngleDegrees = 0.0;
    private int currentEncoderTicks = 0;
    private double currentServoPosition = SERVO_CENTER_POSITION;

    private double learnedServoCommandOffsetDegrees = 0.0;
    private double lastLoopTimeSeconds = 0.0;
    private int previousEncoderTicks = 0;

    private double previousAbsoluteEncoderRawDegrees = 0.0;
    private double unwrappedAbsoluteEncoderDegrees = 0.0;
    private double absoluteRawAtStartDegrees = 0.0;
    private double absoluteTurretReferenceAtStartDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
    private double absoluteEncoderTurretAngleDegrees = 0.0;
    private double absoluteEncoderTurretDeltaDegrees = 0.0;

    private int readyLoops = 0;
    private int notReadyLoops = 0;
    private boolean turretReady = false;

    @Override
    public void initialize() {
        leftTurret = ActiveOpMode.hardwareMap().get(Servo.class, LEFT_TURRET_SERVO_NAME);
        rightTurret = ActiveOpMode.hardwareMap().get(Servo.class, RIGHT_TURRET_SERVO_NAME);
        turretEncoder = ActiveOpMode.hardwareMap().get(DcMotor.class, TURRET_ENCODER_NAME);

        leftTurret.setDirection(Servo.Direction.FORWARD);
        rightTurret.setDirection(Servo.Direction.FORWARD);

        absoluteTurretEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, ABSOLUTE_TURRET_ENCODER_NAME);

        targetAngleDegrees = INITIAL_ANGLE_DEGREES;
        commandedAngleDegrees = INITIAL_ANGLE_DEGREES;
        measuredAngleDegrees = INITIAL_ANGLE_DEGREES;
        measuredVelocityDegPerSec = 0.0;
        quadratureOffsetDegrees = 0.0;
        quadRawAngleDegrees = 0.0;
        readyLoops = 0;
        notReadyLoops = 0;
        turretReady = false;

        // Force the turret to mechanical "center" first so startup references
        // are captured in a consistent sector even if drivers forgot to pre-center.
        leftTurret.setPosition(SERVO_CENTER_POSITION);
        rightTurret.setPosition(SERVO_CENTER_POSITION);
        currentServoPosition = SERVO_CENTER_POSITION;
        if (STARTUP_CENTER_SETTLE_MS > 0) {
            long settleStartMs = System.currentTimeMillis();
            while (System.currentTimeMillis() - settleStartMs < STARTUP_CENTER_SETTLE_MS) {
                // Intentionally waiting for turret to settle before startup references.
            }
        }

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentEncoderTicks = turretEncoder.getCurrentPosition();
        previousEncoderTicks = currentEncoderTicks;

        double quadRawAtStartDegrees = currentEncoderTicks / turretCountsPerDegree();
        double absoluteTurretAtStartDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
        absoluteTurretReferenceAtStartDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
        absoluteRawAtStartDegrees = absoluteVoltageToRawDegrees(absoluteTurretEncoder.getVoltage());
        previousAbsoluteEncoderRawDegrees = absoluteRawAtStartDegrees;
        unwrappedAbsoluteEncoderDegrees = absoluteRawAtStartDegrees;

        absoluteTurretAtStartDegrees = absoluteRawToNearestTurretAngleDegrees(
                absoluteRawAtStartDegrees,
                STARTUP_EXPECTED_TURRET_ANGLE_DEGREES
        );

        double absoluteStartupErrorDegrees =
                absoluteTurretAtStartDegrees - STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
        learnedServoCommandOffsetDegrees = absoluteStartupErrorDegrees;

        absoluteEncoderTurretAngleDegrees = absoluteTurretReferenceAtStartDegrees;
        absoluteEncoderTurretDeltaDegrees = 0.0;

        quadratureOffsetDegrees = quadRawAtStartDegrees - absoluteTurretReferenceAtStartDegrees;
        quadRawAngleDegrees = quadRawAtStartDegrees;
        measuredAngleDegrees = quadRawAtStartDegrees - quadratureOffsetDegrees;

        applyServoAngle(INITIAL_ANGLE_DEGREES);
        lastLoopTimeSeconds = nowSeconds();
    }

    @Override
    public void periodic() {
        double now = nowSeconds();
        if (lastLoopTimeSeconds <= 0.0) {
            lastLoopTimeSeconds = now;
            return;
        }

        double dt = now - lastLoopTimeSeconds;
        lastLoopTimeSeconds = now;
        if (dt <= 1e-6) return;

        updateMeasuredAngle(dt);

        double correctedTargetAngleDegrees = applyTargetBias(targetAngleDegrees);

        double maxMoveDegrees = Math.max(0.0, RATE_LIMIT_DEG_PER_SEC) * dt;
        double commandDiff = correctedTargetAngleDegrees - commandedAngleDegrees;
        if (Math.abs(commandDiff) <= maxMoveDegrees) {
            commandedAngleDegrees = correctedTargetAngleDegrees;
        } else {
            commandedAngleDegrees += Math.signum(commandDiff) * maxMoveDegrees;
        }

        double angleErrorDegrees = targetAngleDegrees - measuredAngleDegrees;
        double outerLoopTrimDegrees = Range.clip(
                OUTER_LOOP_KP * angleErrorDegrees,
                -OUTER_LOOP_MAX_TRIM_DEGREES,
                OUTER_LOOP_MAX_TRIM_DEGREES
        );

        double servoCommandAngleDegrees = Range.clip(
                commandedAngleDegrees + outerLoopTrimDegrees,
                MIN_SERVO_ROTATION_DEGREES,
                MAX_SERVO_ROTATION_DEGREES
        );

        applyServoAngle(servoCommandAngleDegrees);
        updateReadyState(angleErrorDegrees);
    }

    public void setTargetAngleDegrees(double angleDegrees) {
        targetAngleDegrees = Range.clip(
                angleDegrees,
                MIN_SERVO_ROTATION_DEGREES,
                MAX_SERVO_ROTATION_DEGREES
        );
    }

    public void center() {
        setTargetAngleDegrees(0.0);
    }

    /**
     * Converts a robot-front-relative target angle into the turret angle frame.
     * In this robot, turret 0 deg points to the rear, so front-relative 0 deg maps to 180 deg.
     */
    public double convertRobotFrontRelativeToTurretDegrees(double robotFrontRelativeDegrees) {
        double converted = wrapDegrees(
                (ROBOT_FRONT_RELATIVE_SIGN * robotFrontRelativeDegrees) +
                ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES
        );
        return Range.clip(converted, MIN_SERVO_ROTATION_DEGREES, MAX_SERVO_ROTATION_DEGREES);
    }

    public void setTargetAngleFromRobotFrontRelativeDegrees(double robotFrontRelativeDegrees) {
        setTargetAngleDegrees(convertRobotFrontRelativeToTurretDegrees(robotFrontRelativeDegrees));
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    public double getMeasuredAngleDegrees() {
        return measuredAngleDegrees;
    }

    public double getMeasuredVelocityDegPerSec() {
        return measuredVelocityDegPerSec;
    }

    public double getCurrentServoPosition() {
        return currentServoPosition;
    }

    public int getCurrentEncoderTicks() {
        return currentEncoderTicks;
    }

    public double getQuadRawAngleDegrees() {
        return quadRawAngleDegrees;
    }

    public double getQuadratureOffsetDegrees() {
        return quadratureOffsetDegrees;
    }

    public double getAbsoluteEncoderTurretAngleDegrees() {
        return absoluteEncoderTurretAngleDegrees;
    }

    public double getAbsoluteEncoderTurretDeltaDegrees() {
        return absoluteEncoderTurretDeltaDegrees;
    }

    public boolean isTurretReady() {
        return turretReady;
    }

    public boolean isUsingAbsoluteEncoder() {
        return true;
    }

    private void updateMeasuredAngle(double dt) {
        currentEncoderTicks = turretEncoder.getCurrentPosition();
        int encoderDeltaTicks = currentEncoderTicks - previousEncoderTicks;
        previousEncoderTicks = currentEncoderTicks;

        double countsPerDegree = turretCountsPerDegree();
        quadRawAngleDegrees = currentEncoderTicks / countsPerDegree;
        measuredAngleDegrees = quadRawAngleDegrees - quadratureOffsetDegrees;
        double encoderDeltaDegrees = encoderDeltaTicks / countsPerDegree;
        measuredVelocityDegPerSec = encoderDeltaDegrees / dt;

        double absoluteEncoderRawDegrees = absoluteVoltageToRawDegrees(absoluteTurretEncoder.getVoltage());
        double absoluteEncoderDeltaRawDegrees = smallestWrappedDeltaDegrees(
            absoluteEncoderRawDegrees,
            previousAbsoluteEncoderRawDegrees
        );

        unwrappedAbsoluteEncoderDegrees += absoluteEncoderDeltaRawDegrees;
        previousAbsoluteEncoderRawDegrees = absoluteEncoderRawDegrees;

        absoluteEncoderTurretAngleDegrees =
            ((unwrappedAbsoluteEncoderDegrees - absoluteRawAtStartDegrees) / ENCODER_TO_TURRET_RATIO)
                + absoluteTurretReferenceAtStartDegrees;
        absoluteEncoderTurretDeltaDegrees = absoluteEncoderDeltaRawDegrees / ENCODER_TO_TURRET_RATIO;
    }

    private void updateReadyState(double angleErrorDegrees) {
        boolean inTolerance =
                Math.abs(angleErrorDegrees) <= READY_TOLERANCE_DEGREES &&
                Math.abs(measuredVelocityDegPerSec) <= READY_VELOCITY_TOLERANCE_DEG_PER_SEC;

        if (inTolerance) {
            readyLoops++;
            notReadyLoops = 0;
        } else {
            notReadyLoops++;
            if (notReadyLoops >= NOT_READY_LOOPS_REQUIRED) {
                readyLoops = 0;
            }
        }

        turretReady = readyLoops >= READY_LOOPS_REQUIRED;
    }

    private void applyServoAngle(double turretAngleDegrees) {
        currentServoPosition = angleDegreesToServoPosition(
                turretAngleDegrees,
                learnedServoCommandOffsetDegrees
        );
        leftTurret.setPosition(currentServoPosition);
        rightTurret.setPosition(currentServoPosition);
    }

    private double applyTargetBias(double targetDegrees) {
        if (targetDegrees > TARGET_BIAS_APPLY_THRESHOLD_DEGREES) {
            return targetDegrees + POSITIVE_TARGET_BIAS_DEGREES;
        } else if (targetDegrees < -TARGET_BIAS_APPLY_THRESHOLD_DEGREES) {
            return targetDegrees + NEGATIVE_TARGET_BIAS_DEGREES;
        }
        return targetDegrees;
    }

    private static double angleDegreesToServoPosition(double angleDegrees, double servoCommandOffsetDegrees) {
        double correctedAngleDegrees = angleDegrees - servoCommandOffsetDegrees;
        double unclippedPosition = SERVO_CENTER_POSITION - (correctedAngleDegrees / TURRET_TRAVEL_DEGREES);
        return Range.clip(unclippedPosition, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
    }

    private static double absoluteVoltageToRawDegrees(double voltage) {
        double clippedVoltage = Range.clip(voltage, 0.0, ABSOLUTE_TURRET_ENCODER_MAX_VOLTAGE);
        return (clippedVoltage / ABSOLUTE_TURRET_ENCODER_MAX_VOLTAGE) * 360.0;
    }

    private static double absoluteRawToNearestTurretAngleDegrees(double absoluteRawDegrees, double expectedTurretAngleDegrees) {
        double ambiguousTurretAngleDegrees =
                (absoluteRawDegrees / ENCODER_TO_TURRET_RATIO) + ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES;
        double sectorWidthDegrees = 360.0 / ENCODER_TO_TURRET_RATIO;

        double bestCandidate = ambiguousTurretAngleDegrees;
        double bestDistance = Double.MAX_VALUE;
        for (int sector = -3; sector <= 3; sector++) {
            double candidate = ambiguousTurretAngleDegrees + (sector * sectorWidthDegrees);
            double distance = Math.abs(smallestWrappedDeltaDegrees(candidate, expectedTurretAngleDegrees));
            if (distance < bestDistance) {
                bestDistance = distance;
                bestCandidate = candidate;
            }
        }
        return bestCandidate;
    }

    private static double smallestWrappedDeltaDegrees(double currentDegrees, double previousDegrees) {
        double delta = currentDegrees - previousDegrees;
        if (delta > 180.0) {
            delta -= 360.0;
        } else if (delta < -180.0) {
            delta += 360.0;
        }
        return delta;
    }

    private static double nowSeconds() {
        return System.nanoTime() / 1e9;
    }

    private static double turretCountsPerDegree() {
        return Math.max(1e-6, TURRET_ENCODER_COUNTS_PER_DEGREE);
    }

    private static double wrapDegrees(double angleDegrees) {
        double wrapped = angleDegrees;
        while (wrapped > 180.0) wrapped -= 360.0;
        while (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }
}

