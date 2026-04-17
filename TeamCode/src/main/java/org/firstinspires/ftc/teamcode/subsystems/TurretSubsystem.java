package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
    // Split commanded turret angle across servos to add slight opposing preload.
    // Set to 0.0 to disable this behavior.
    public static double SERVO_DIFFERENTIAL_DEGREES = 1.0;

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
    public static long STARTUP_CENTER_SETTLE_MS = 2000;
    public static boolean STARTUP_SEND_CENTER_COMMAND = true;
    public static double ABSOLUTE_TURRET_ENCODER_MAX_VOLTAGE = 3.255;
    public static double ENCODER_TO_TURRET_RATIO = 4.8;
    public static double TURRET_ENCODER_CPR = 4096.0;
    public static double TURRET_ENCODER_COUNTS_PER_REV = TURRET_ENCODER_CPR * ENCODER_TO_TURRET_RATIO;
    public static double TURRET_ENCODER_COUNTS_PER_DEGREE = TURRET_ENCODER_COUNTS_PER_REV / 360.0;
    // Set to -1.0 if quadrature angle moves opposite commanded turret-angle sign.
    public static double QUAD_DIRECTION_SIGN = -1.0;
    public static double ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES = 0.0;
    // Disable periodic analog reads unless explicitly needed for diagnostics/logging.
    public static boolean READ_ABSOLUTE_ENCODER_IN_PERIODIC = false;

    // =========================
    // Hardware
    // =========================
    private ServoImplEx leftTurret;
    private ServoImplEx rightTurret;
    private DcMotor turretEncoder;
    private AnalogInput absoluteTurretEncoder;

    // =========================
    // Runtime state
    // =========================
    private double targetAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double correctedTargetAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double commandedAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double measuredAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double measuredVelocityDegPerSec = 0.0;
    private double quadratureOffsetDegrees = 0.0;
    private double quadRawAngleDegrees = 0.0;
    private int currentEncoderTicks = 0;
    private double currentServoPosition = SERVO_CENTER_POSITION;
    private double currentLeftServoPosition = SERVO_CENTER_POSITION;
    private double currentRightServoPosition = SERVO_CENTER_POSITION;
    private double lastServoCommandAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double lastOuterLoopTrimDegrees = 0.0;
    private double lastCommandDiffDegrees = 0.0;
    private double lastRateLimitedStepDegrees = 0.0;
    private double lastLeftTurretAngleDegrees = INITIAL_ANGLE_DEGREES;
    private double lastRightTurretAngleDegrees = INITIAL_ANGLE_DEGREES;

    private double learnedServoCommandOffsetDegrees = 0.0;
    private double absoluteStartupErrorDegrees = 0.0;
    private double lastLoopTimeSeconds = 0.0;
    private int previousEncoderTicks = 0;

    private double previousAbsoluteEncoderRawDegrees = 0.0;
    private double absoluteEncoderRawDegrees = 0.0;
    private double unwrappedAbsoluteEncoderDegrees = 0.0;
    private double absoluteRawAtStartDegrees = 0.0;
    private double absoluteTurretReferenceAtStartDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
    private double absoluteEncoderTurretAngleDegrees = 0.0;
    private double absoluteEncoderTurretDeltaDegrees = 0.0;
    private boolean periodicAbsoluteEncoderReadEnabled = READ_ABSOLUTE_ENCODER_IN_PERIODIC;
    private boolean wasPeriodicAbsoluteReadEnabled = false;

    private int readyLoops = 0;
    private int notReadyLoops = 0;
    private boolean turretReady = false;
    private boolean leftServoEnabled = true;
    private boolean rightServoEnabled = true;
    private long startupCenterCommandTimeMs = 0L;
    private double startupExpectedTurretAngleDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;

    private enum StartupCalibrationState {
        UNCALIBRATED,
        WAITING_FOR_SETTLE,
        CALIBRATED
    }

    private StartupCalibrationState startupCalibrationState = StartupCalibrationState.UNCALIBRATED;

    @Override
    public void initialize() {
        leftTurret = ActiveOpMode.hardwareMap().get(ServoImplEx.class, LEFT_TURRET_SERVO_NAME);
        rightTurret = ActiveOpMode.hardwareMap().get(ServoImplEx.class, RIGHT_TURRET_SERVO_NAME);
        turretEncoder = ActiveOpMode.hardwareMap().get(DcMotor.class, TURRET_ENCODER_NAME);

        leftTurret.setDirection(Servo.Direction.FORWARD);
        rightTurret.setDirection(Servo.Direction.FORWARD);

        absoluteTurretEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, ABSOLUTE_TURRET_ENCODER_NAME);

        targetAngleDegrees = INITIAL_ANGLE_DEGREES;
        correctedTargetAngleDegrees = INITIAL_ANGLE_DEGREES;
        commandedAngleDegrees = INITIAL_ANGLE_DEGREES;
        measuredAngleDegrees = INITIAL_ANGLE_DEGREES;
        measuredVelocityDegPerSec = 0.0;
        quadratureOffsetDegrees = 0.0;
        quadRawAngleDegrees = 0.0;
        lastServoCommandAngleDegrees = INITIAL_ANGLE_DEGREES;
        lastOuterLoopTrimDegrees = 0.0;
        lastCommandDiffDegrees = 0.0;
        lastRateLimitedStepDegrees = 0.0;
        lastLeftTurretAngleDegrees = INITIAL_ANGLE_DEGREES;
        lastRightTurretAngleDegrees = INITIAL_ANGLE_DEGREES;
        learnedServoCommandOffsetDegrees = 0.0;
        absoluteStartupErrorDegrees = 0.0;
        readyLoops = 0;
        notReadyLoops = 0;
        turretReady = false;
        leftServoEnabled = true;
        rightServoEnabled = true;
        periodicAbsoluteEncoderReadEnabled = READ_ABSOLUTE_ENCODER_IN_PERIODIC;
        wasPeriodicAbsoluteReadEnabled = false;
        startupCenterCommandTimeMs = 0L;
        startupExpectedTurretAngleDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
        startupCalibrationState = StartupCalibrationState.UNCALIBRATED;

        currentLeftServoPosition = leftTurret.getPosition();
        currentRightServoPosition = rightTurret.getPosition();
        currentServoPosition = 0.5 * (currentLeftServoPosition + currentRightServoPosition);

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentEncoderTicks = turretEncoder.getCurrentPosition();
        previousEncoderTicks = currentEncoderTicks;

        double quadRawAtStartDegrees = (currentEncoderTicks / turretCountsPerDegree()) * QUAD_DIRECTION_SIGN;
        absoluteTurretReferenceAtStartDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
        absoluteRawAtStartDegrees = absoluteVoltageToRawDegrees(absoluteTurretEncoder.getVoltage());
        absoluteEncoderRawDegrees = absoluteRawAtStartDegrees;
        previousAbsoluteEncoderRawDegrees = absoluteRawAtStartDegrees;
        unwrappedAbsoluteEncoderDegrees = absoluteRawAtStartDegrees;
        absoluteEncoderTurretAngleDegrees = Double.NaN;
        absoluteEncoderTurretDeltaDegrees = 0.0;

        quadratureOffsetDegrees = 0.0;
        quadRawAngleDegrees = quadRawAtStartDegrees;
        measuredAngleDegrees = quadRawAtStartDegrees;
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
        if (!isStartupCalibrationComplete()) {
            turretReady = false;
            return;
        }

        correctedTargetAngleDegrees = applyTargetBias(targetAngleDegrees);

        double maxMoveDegrees = Math.max(0.0, RATE_LIMIT_DEG_PER_SEC) * dt;
        double previousCommandedAngleDegrees = commandedAngleDegrees;
        double commandDiff = correctedTargetAngleDegrees - commandedAngleDegrees;
        lastCommandDiffDegrees = commandDiff;
        if (Math.abs(commandDiff) <= maxMoveDegrees) {
            commandedAngleDegrees = correctedTargetAngleDegrees;
        } else {
            commandedAngleDegrees += Math.signum(commandDiff) * maxMoveDegrees;
        }
        lastRateLimitedStepDegrees = commandedAngleDegrees - previousCommandedAngleDegrees;

        double angleErrorDegrees = targetAngleDegrees - measuredAngleDegrees;
        double outerLoopTrimDegrees = Range.clip(
                OUTER_LOOP_KP * angleErrorDegrees,
                -OUTER_LOOP_MAX_TRIM_DEGREES,
                OUTER_LOOP_MAX_TRIM_DEGREES
        );
        lastOuterLoopTrimDegrees = outerLoopTrimDegrees;

        double servoCommandAngleDegrees = Range.clip(
                commandedAngleDegrees + outerLoopTrimDegrees,
                MIN_SERVO_ROTATION_DEGREES,
                MAX_SERVO_ROTATION_DEGREES
        );
        lastServoCommandAngleDegrees = servoCommandAngleDegrees;

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

    public void beginStartupCentering() {
        if (!STARTUP_SEND_CENTER_COMMAND) {
            beginStartupCalibrationWithoutCentering();
            return;
        }
        double centerPosition = Range.clip(SERVO_CENTER_POSITION, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        currentLeftServoPosition = centerPosition;
        currentRightServoPosition = centerPosition;
        currentServoPosition = centerPosition;
        // Keep startup centering minimal: position-only command, no PWM state toggles.
        // This matches the old stable initialization behavior.
        leftTurret.setPosition(currentLeftServoPosition);
        rightTurret.setPosition(currentRightServoPosition);
        startupCenterCommandTimeMs = System.currentTimeMillis();
        startupCalibrationState = StartupCalibrationState.WAITING_FOR_SETTLE;
    }

    public void beginStartupCalibrationWithoutCentering() {
        startupCenterCommandTimeMs = System.currentTimeMillis();
        startupCalibrationState = StartupCalibrationState.WAITING_FOR_SETTLE;
    }

    public boolean updateStartupCalibrationFromExpected(double expectedTurretAngleDegrees) {
        if (startupCalibrationState == StartupCalibrationState.CALIBRATED) {
            return true;
        }
        if (startupCalibrationState == StartupCalibrationState.UNCALIBRATED) {
            beginStartupCalibrationWithoutCentering();
        }

        long elapsedMs = System.currentTimeMillis() - startupCenterCommandTimeMs;
        if (elapsedMs < Math.max(0L, STARTUP_CENTER_SETTLE_MS)) {
            return false;
        }

        completeStartupCalibrationFromExpected(expectedTurretAngleDegrees);
        return true;
    }

    public void forceStartupCalibrationFromExpected(double expectedTurretAngleDegrees) {
        completeStartupCalibrationFromExpected(expectedTurretAngleDegrees);
    }

    public boolean isStartupCalibrationComplete() {
        return startupCalibrationState == StartupCalibrationState.CALIBRATED;
    }

    public String getStartupCalibrationStateName() {
        return startupCalibrationState.name();
    }

    /**
     * Converts a robot-front-relative target angle into the turret angle frame.
     * In this robot, turret 0 deg points to the rear, so front-relative 0 deg maps to 180 deg.
     */
    public double convertRobotFrontRelativeToTurretDegrees(double robotFrontRelativeDegrees) {
        double unwrappedTurretTarget =
                (ROBOT_FRONT_RELATIVE_SIGN * robotFrontRelativeDegrees) +
                ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES;

        // Choose the equivalent target nearest the previous target, but only from values that are
        // already inside the legal turret range. This avoids wrap+clip sign flips near 180 deg.
        return chooseNearestEquivalentInRangeDegrees(
                unwrappedTurretTarget,
                targetAngleDegrees,
                MIN_SERVO_ROTATION_DEGREES,
                MAX_SERVO_ROTATION_DEGREES
        );
    }

    public void setTargetAngleFromRobotFrontRelativeDegrees(double robotFrontRelativeDegrees) {
        setTargetAngleDegrees(convertRobotFrontRelativeToTurretDegrees(robotFrontRelativeDegrees));
    }

    public double getTargetAngleDegrees() {
        return Range.clip(targetAngleDegrees, MIN_SERVO_ROTATION_DEGREES, MAX_SERVO_ROTATION_DEGREES);
    }

    public double getCorrectedTargetAngleDegrees() {
        return Range.clip(correctedTargetAngleDegrees, MIN_SERVO_ROTATION_DEGREES, MAX_SERVO_ROTATION_DEGREES);
    }

    public double getMeasuredAngleDegrees() {
        return Range.clip(measuredAngleDegrees, MIN_SERVO_ROTATION_DEGREES, MAX_SERVO_ROTATION_DEGREES);
    }

    public double getCommandedAngleDegrees() {
        return Range.clip(commandedAngleDegrees, MIN_SERVO_ROTATION_DEGREES, MAX_SERVO_ROTATION_DEGREES);
    }

    public double getServoCommandAngleDegrees() {
        return Range.clip(lastServoCommandAngleDegrees, MIN_SERVO_ROTATION_DEGREES, MAX_SERVO_ROTATION_DEGREES);
    }

    public double getOuterLoopTrimDegrees() {
        return lastOuterLoopTrimDegrees;
    }

    public double getCommandDiffDegrees() {
        return lastCommandDiffDegrees;
    }

    public double getRateLimitedStepDegrees() {
        return lastRateLimitedStepDegrees;
    }

    public double getMeasuredVelocityDegPerSec() {
        return measuredVelocityDegPerSec;
    }

    public double getCurrentServoPosition() {
        return currentServoPosition;
    }

    public double getCurrentLeftServoPosition() {
        return currentLeftServoPosition;
    }

    public double getCurrentRightServoPosition() {
        return currentRightServoPosition;
    }

    public double getLeftTurretAngleCommandDegrees() {
        return lastLeftTurretAngleDegrees;
    }

    public double getRightTurretAngleCommandDegrees() {
        return lastRightTurretAngleDegrees;
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

    public double getAbsoluteEncoderRawDegrees() {
        return absoluteEncoderRawDegrees;
    }

    public double getAbsoluteEncoderTurretDeltaDegrees() {
        return absoluteEncoderTurretDeltaDegrees;
    }

    public double getLearnedServoCommandOffsetDegrees() {
        return learnedServoCommandOffsetDegrees;
    }

    public double getAbsoluteStartupErrorDegrees() {
        return absoluteStartupErrorDegrees;
    }

    public boolean isTurretReady() {
        return turretReady;
    }

    public void setLeftServoEnabled(boolean enabled) {
        leftServoEnabled = enabled;
        if (enabled) {
            leftTurret.setPwmEnable();
            leftTurret.setPosition(currentLeftServoPosition);
        } else {
            leftTurret.setPwmDisable();
        }
    }

    public void setRightServoEnabled(boolean enabled) {
        rightServoEnabled = enabled;
        if (enabled) {
            rightTurret.setPwmEnable();
            rightTurret.setPosition(currentRightServoPosition);
        } else {
            rightTurret.setPwmDisable();
        }
    }

    public void setServoEnabledStates(boolean leftEnabled, boolean rightEnabled) {
        setLeftServoEnabled(leftEnabled);
        setRightServoEnabled(rightEnabled);
    }

    public boolean isLeftServoEnabled() {
        return leftServoEnabled;
    }

    public boolean isRightServoEnabled() {
        return rightServoEnabled;
    }

    public void SetServoCenter() {
        beginStartupCentering();
    }
    public boolean isUsingAbsoluteEncoder() {
        return true;
    }

    public void setPeriodicAbsoluteEncoderReadEnabled(boolean enabled) {
        periodicAbsoluteEncoderReadEnabled = enabled;
    }

    public boolean isPeriodicAbsoluteEncoderReadEnabled() {
        return periodicAbsoluteEncoderReadEnabled;
    }

    private void completeStartupCalibrationFromExpected(double expectedTurretAngleDegrees) {
        startupExpectedTurretAngleDegrees = expectedTurretAngleDegrees;

        double rawDegrees = absoluteVoltageToRawDegrees(absoluteTurretEncoder.getVoltage());
        absoluteRawAtStartDegrees = rawDegrees;
        absoluteEncoderRawDegrees = rawDegrees;
        previousAbsoluteEncoderRawDegrees = rawDegrees;
        unwrappedAbsoluteEncoderDegrees = rawDegrees;

        double resolvedTurretAngleDegrees =
                absoluteRawToNearestTurretAngleDegrees(rawDegrees, expectedTurretAngleDegrees);
        absoluteTurretReferenceAtStartDegrees = resolvedTurretAngleDegrees;
        absoluteEncoderTurretAngleDegrees = resolvedTurretAngleDegrees;
        absoluteEncoderTurretDeltaDegrees = 0.0;

        absoluteStartupErrorDegrees = resolvedTurretAngleDegrees - expectedTurretAngleDegrees;
        learnedServoCommandOffsetDegrees = absoluteStartupErrorDegrees;

        currentEncoderTicks = turretEncoder.getCurrentPosition();
        previousEncoderTicks = currentEncoderTicks;
        quadRawAngleDegrees = (currentEncoderTicks / turretCountsPerDegree()) * QUAD_DIRECTION_SIGN;
        quadratureOffsetDegrees = quadRawAngleDegrees - resolvedTurretAngleDegrees;
        measuredAngleDegrees = quadRawAngleDegrees - quadratureOffsetDegrees;

        targetAngleDegrees = measuredAngleDegrees;
        correctedTargetAngleDegrees = measuredAngleDegrees;
        commandedAngleDegrees = measuredAngleDegrees;
        lastServoCommandAngleDegrees = measuredAngleDegrees;
        lastOuterLoopTrimDegrees = 0.0;
        lastCommandDiffDegrees = 0.0;
        lastRateLimitedStepDegrees = 0.0;
        readyLoops = 0;
        notReadyLoops = 0;
        turretReady = false;

        startupCenterCommandTimeMs = 0L;
        startupCalibrationState = StartupCalibrationState.CALIBRATED;
    }

    private void updateMeasuredAngle(double dt) {
        currentEncoderTicks = turretEncoder.getCurrentPosition();
        int encoderDeltaTicks = currentEncoderTicks - previousEncoderTicks;
        previousEncoderTicks = currentEncoderTicks;

        double countsPerDegree = turretCountsPerDegree();
        quadRawAngleDegrees = (currentEncoderTicks / countsPerDegree) * QUAD_DIRECTION_SIGN;
        measuredAngleDegrees = quadRawAngleDegrees - quadratureOffsetDegrees;
        double encoderDeltaDegrees = (encoderDeltaTicks / countsPerDegree) * QUAD_DIRECTION_SIGN;
        measuredVelocityDegPerSec = encoderDeltaDegrees / dt;

        if (periodicAbsoluteEncoderReadEnabled) {
            double currentAbsoluteRawDegrees = absoluteVoltageToRawDegrees(absoluteTurretEncoder.getVoltage());

            if (!wasPeriodicAbsoluteReadEnabled) {
                // Re-prime unwrap state when periodic sampling is re-enabled.
                previousAbsoluteEncoderRawDegrees = currentAbsoluteRawDegrees;
                unwrappedAbsoluteEncoderDegrees = currentAbsoluteRawDegrees;
            }

            absoluteEncoderRawDegrees = currentAbsoluteRawDegrees;
            double absoluteEncoderDeltaRawDegrees = smallestWrappedDeltaDegrees(
                    currentAbsoluteRawDegrees,
                    previousAbsoluteEncoderRawDegrees
            );
            if (!wasPeriodicAbsoluteReadEnabled) {
                absoluteEncoderDeltaRawDegrees = 0.0;
            }

            unwrappedAbsoluteEncoderDegrees += absoluteEncoderDeltaRawDegrees;
            previousAbsoluteEncoderRawDegrees = currentAbsoluteRawDegrees;

            absoluteEncoderTurretAngleDegrees =
                    ((unwrappedAbsoluteEncoderDegrees - absoluteRawAtStartDegrees) / ENCODER_TO_TURRET_RATIO)
                            + absoluteTurretReferenceAtStartDegrees;
            absoluteEncoderTurretDeltaDegrees = absoluteEncoderDeltaRawDegrees / ENCODER_TO_TURRET_RATIO;
            wasPeriodicAbsoluteReadEnabled = true;
        } else {
            absoluteEncoderRawDegrees = Double.NaN;
            absoluteEncoderTurretAngleDegrees = Double.NaN;
            absoluteEncoderTurretDeltaDegrees = 0.0;
            wasPeriodicAbsoluteReadEnabled = false;
        }
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
        double halfDifferentialDegrees = SERVO_DIFFERENTIAL_DEGREES * 0.5;

        double leftTurretAngleDegrees = Range.clip(
                turretAngleDegrees + halfDifferentialDegrees,
                MIN_SERVO_ROTATION_DEGREES,
                MAX_SERVO_ROTATION_DEGREES
        );
        double rightTurretAngleDegrees = Range.clip(
                turretAngleDegrees - halfDifferentialDegrees,
                MIN_SERVO_ROTATION_DEGREES,
                MAX_SERVO_ROTATION_DEGREES
        );
        lastLeftTurretAngleDegrees = leftTurretAngleDegrees;
        lastRightTurretAngleDegrees = rightTurretAngleDegrees;

        currentLeftServoPosition = angleDegreesToServoPosition(
                leftTurretAngleDegrees,
                learnedServoCommandOffsetDegrees
        );
        currentRightServoPosition = angleDegreesToServoPosition(
                rightTurretAngleDegrees,
                learnedServoCommandOffsetDegrees
        );
        currentServoPosition = angleDegreesToServoPosition(
                turretAngleDegrees,
                learnedServoCommandOffsetDegrees
        );

        if (leftServoEnabled) {
            leftTurret.setPosition(currentLeftServoPosition);
        } else {
            leftTurret.setPwmDisable();
        }

        if (rightServoEnabled) {
            rightTurret.setPosition(currentRightServoPosition);
        } else {
            rightTurret.setPwmDisable();
        }
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

    private static double chooseNearestEquivalentDegrees(double baseAngleDegrees, double referenceAngleDegrees) {
        double bestCandidate = baseAngleDegrees;
        double bestDistance = Math.abs(baseAngleDegrees - referenceAngleDegrees);
        for (int turns = -2; turns <= 2; turns++) {
            double candidate = baseAngleDegrees + (360.0 * turns);
            double distance = Math.abs(candidate - referenceAngleDegrees);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestCandidate = candidate;
            }
        }
        return bestCandidate;
    }

    private static double chooseNearestEquivalentInRangeDegrees(
            double baseAngleDegrees,
            double referenceAngleDegrees,
            double minDegrees,
            double maxDegrees
    ) {
        double bestCandidate = Double.NaN;
        double bestDistance = Double.MAX_VALUE;

        for (int turns = -3; turns <= 3; turns++) {
            double candidate = baseAngleDegrees + (360.0 * turns);
            if (candidate < minDegrees || candidate > maxDegrees) continue;
            double distance = Math.abs(candidate - referenceAngleDegrees);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestCandidate = candidate;
            }
        }

        // If no wrapped candidate lands in range, fall back to normal clamp.
        if (Double.isNaN(bestCandidate)) {
            double minDistance = Math.abs(minDegrees - referenceAngleDegrees);
            double maxDistance = Math.abs(maxDegrees - referenceAngleDegrees);
            return (minDistance <= maxDistance) ? minDegrees : maxDegrees;
        }
        return bestCandidate;
    }
}

