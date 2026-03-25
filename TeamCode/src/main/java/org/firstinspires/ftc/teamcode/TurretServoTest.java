package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Test opmode for turret Axon mini servos (Left and Right).
 * Both run in servo mode, position 0–1.
 * Position preset buttons: X=0, Y=0.5, B=1, left bumper=0.25, right bumper=0.75.
 * Left stick X: left = speed up rate limit, right = slow down rate limit.
 */
@TeleOp(name = "Turret Servo Test", group = "Test")
public class TurretServoTest extends LinearOpMode {

	private static final double SERVO_CENTER_POSITION = 0.5;
	private static final double TURRET_TRAVEL_DEGREES = 355.0; // Change this to calibrate real turret travel.
	private static final double INITIAL_ANGLE_DEGREES = 0.0;
	private static final double POSITIVE_LIMIT_ANGLE_DEGREES = 0.5 * TURRET_TRAVEL_DEGREES;
	private static final double NEGATIVE_LIMIT_ANGLE_DEGREES = -0.5 * TURRET_TRAVEL_DEGREES;
	private static final double QUARTER_POSITIVE_ANGLE_DEGREES = 0.25 * TURRET_TRAVEL_DEGREES;
	private static final double QUARTER_NEGATIVE_ANGLE_DEGREES = -0.25 * TURRET_TRAVEL_DEGREES;
	private static final double MIN_SERVO_POSITION = 0.0;
	private static final double MAX_SERVO_POSITION = 1.0;
    private static final double MIN_SERVO_ROTATION_DEGREES = -130.0;
    private static final double MAX_SERVO_ROTATION_DEGREES = 130.0;
    private static final double MIN_RATE_DEG_PER_SEC = 0.01 * TURRET_TRAVEL_DEGREES;
	private static final double MAX_RATE_DEG_PER_SEC = 2.0 * TURRET_TRAVEL_DEGREES;
	private static final double RATE_STEP_DEG_PER_LOOP = 0.0005 * TURRET_TRAVEL_DEGREES;
	private static final double OUTER_LOOP_KP = 0.12;
	private static final double OUTER_LOOP_MAX_TRIM_DEGREES = 8.0;
	private static final double POSITIVE_TARGET_BIAS_DEGREES = 0.0;
	private static final double NEGATIVE_TARGET_BIAS_DEGREES = 0.0;
	private static final double TARGET_BIAS_APPLY_THRESHOLD_DEGREES = 1.0;
	private static final double READY_TOLERANCE_DEGREES = 2.0;
	private static final double READY_VELOCITY_TOLERANCE_DEG_PER_SEC = 15.0;
	private static final int READY_LOOPS_REQUIRED = 3;
	private static final int NOT_READY_LOOPS_REQUIRED = 3;
	private static final double STARTUP_EXPECTED_TURRET_ANGLE_DEGREES = 0.0;
	private static final int INIT_STABLE_LOOPS_REQUIRED = 10;
	private static final double INIT_MAX_QUAD_DELTA_DEGREES_PER_LOOP = 0.35;
	private static final double INIT_MAX_ABS_DELTA_TURRET_DEGREES_PER_LOOP = 0.35;
	private static final String TURRET_ENCODER_NAME = "back_left";
	private static final double TURRET_ENCODER_CPR = 4096.0;
	private static final double ENCODER_TO_TURRET_RATIO = 4.8;
	private static final double TURRET_ENCODER_COUNTS_PER_REV =
			TURRET_ENCODER_CPR * ENCODER_TO_TURRET_RATIO; // 19660.8 counts/rev
	private static final double TURRET_ENCODER_COUNTS_PER_DEGREE =
			TURRET_ENCODER_COUNTS_PER_REV / 360.0; // ~54.6 counts/deg
	private static final String ABSOLUTE_TURRET_ENCODER_NAME = "analog_turret_encoder";
	private static final double ABSOLUTE_TURRET_ENCODER_MAX_VOLTAGE = 3.255;
	private static final double ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES = 0.0;

	@Override
	public void runOpMode() throws InterruptedException {
		Servo lurret = hardwareMap.get(Servo.class, "Lurret");
		Servo rurret = hardwareMap.get(Servo.class, "Rurret");
        Servo hurret = hardwareMap.get(Servo.class, "shooter_hood");
		DcMotor turretEncoder = hardwareMap.get(DcMotor.class, TURRET_ENCODER_NAME);
		AnalogInput absoluteTurretEncoder = hardwareMap.get(AnalogInput.class, ABSOLUTE_TURRET_ENCODER_NAME);

		// This motor port is repurposed as turret encoder input.
		turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Servo mode (position mode) – default for standard Servo
		lurret.setDirection(Servo.Direction.FORWARD);
		rurret.setDirection(Servo.Direction.FORWARD);
        hurret.setDirection(Servo.Direction.FORWARD);

		double learnedServoCommandOffsetDegrees = 0.0;
		lurret.setPosition(angleDegreesToServoPosition(INITIAL_ANGLE_DEGREES, learnedServoCommandOffsetDegrees));
		rurret.setPosition(angleDegreesToServoPosition(INITIAL_ANGLE_DEGREES, learnedServoCommandOffsetDegrees));
        hurret.setPosition(0.0);

		double targetAngleDegrees = INITIAL_ANGLE_DEGREES;
		double commandedAngleDegrees = INITIAL_ANGLE_DEGREES;
		double rateLimitDegPerSec = 1.5 * TURRET_TRAVEL_DEGREES;
		double lastTime = 0.0;
		int previousEncoderTicks = turretEncoder.getCurrentPosition();
		double previousAbsoluteEncoderRawDegrees = 0.0;
		double unwrappedAbsoluteEncoderDegrees = 0.0;
		boolean servosEnabled = true;
		boolean aWasPressed = false;
		int readyLoops = 0;
		int notReadyLoops = 0;
		int initStableLoops = 0;
		boolean initStableAtStart = false;

		telemetry.addLine("Turret Servo Test ready.");
		telemetry.addLine("X=0, Y=0.5, B=1, LB=0.25, RB=0.75");
		telemetry.addLine("Left stick: lurret=speed up rate, rurret=slow down rate");
		telemetry.addData("Turret travel (deg)", "%.1f", TURRET_TRAVEL_DEGREES);
		telemetry.addData("Outer-loop kP", "%.2f", OUTER_LOOP_KP);
		telemetry.addData("Outer-loop max trim (deg)", "%.1f", OUTER_LOOP_MAX_TRIM_DEGREES);
		telemetry.addLine("Init: holding turret at 0 deg and checking stillness");
		telemetry.addLine("Press Start to capture quad offset from absolute encoder");
		telemetry.update();

		double initHoldPosition = angleDegreesToServoPosition(STARTUP_EXPECTED_TURRET_ANGLE_DEGREES, learnedServoCommandOffsetDegrees);
		double initLastTime = getRuntime();
		int initPreviousQuadTicks = turretEncoder.getCurrentPosition();
		double initPreviousAbsoluteRawDegrees = absoluteVoltageToRawDegrees(absoluteTurretEncoder.getVoltage());

		while (!isStarted() && !isStopRequested()) {
			double initNow = getRuntime();
			double initDt = initNow - initLastTime;
			initLastTime = initNow;

			int initQuadTicks = turretEncoder.getCurrentPosition();
			int initQuadDelta = initQuadTicks - initPreviousQuadTicks;
			initPreviousQuadTicks = initQuadTicks;
			double initQuadDeltaDegrees = initQuadDelta / TURRET_ENCODER_COUNTS_PER_DEGREE;

			double initAbsVoltage = absoluteTurretEncoder.getVoltage();
			double initAbsRawDegrees = absoluteVoltageToRawDegrees(initAbsVoltage);
			double initAbsDeltaRawDegrees = smallestWrappedDeltaDegrees(initAbsRawDegrees, initPreviousAbsoluteRawDegrees);
			initPreviousAbsoluteRawDegrees = initAbsRawDegrees;
			double initAbsDeltaTurretDegrees = initAbsDeltaRawDegrees / ENCODER_TO_TURRET_RATIO;

			boolean initStill =
					Math.abs(initQuadDeltaDegrees) <= INIT_MAX_QUAD_DELTA_DEGREES_PER_LOOP &&
					Math.abs(initAbsDeltaTurretDegrees) <= INIT_MAX_ABS_DELTA_TURRET_DEGREES_PER_LOOP;
			if (initStill) {
				initStableLoops++;
			} else {
				initStableLoops = 0;
			}
			initStableAtStart = initStableLoops >= INIT_STABLE_LOOPS_REQUIRED;

			lurret.setPosition(initHoldPosition);
			rurret.setPosition(initHoldPosition);

			double initQuadVelocityDegPerSec = initDt > 1e-6 ? (initQuadDeltaDegrees / initDt) : 0.0;
			double initAbsTurretAngleGuess = absoluteRawToNearestTurretAngleDegrees(
					initAbsRawDegrees,
					STARTUP_EXPECTED_TURRET_ANGLE_DEGREES
			);

			telemetry.addLine("Init: hold still near turret 0 deg");
			telemetry.addData("Init stable loops", "%d/%d", initStableLoops, INIT_STABLE_LOOPS_REQUIRED);
			telemetry.addData("Init still", initStill);
			telemetry.addData("Init ready to zero", initStableAtStart);
			telemetry.addData("Init quad vel (deg/s)", "%.2f", initQuadVelocityDegPerSec);
			telemetry.addData("Init abs V", "%.3f", initAbsVoltage);
			telemetry.addData("Init abs turret guess (deg)", "%.2f", initAbsTurretAngleGuess);
			telemetry.update();
			idle();
		}
		if (isStopRequested()) return;

		// Capture quadrature offset from absolute encoder once Start is pressed.
		double quadRawAtStartDegrees = turretEncoder.getCurrentPosition() / TURRET_ENCODER_COUNTS_PER_DEGREE;
		double absoluteRawAtStartDegrees = absoluteVoltageToRawDegrees(absoluteTurretEncoder.getVoltage());
		double absoluteTurretAtStartDegrees = absoluteRawToNearestTurretAngleDegrees(
				absoluteRawAtStartDegrees,
				STARTUP_EXPECTED_TURRET_ANGLE_DEGREES
		);
		double absoluteStartupErrorDegrees =
				absoluteTurretAtStartDegrees - STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
		double suggestedAbsoluteOffsetDegrees =
				ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES - absoluteStartupErrorDegrees;
		double absoluteTurretReferenceAtStartDegrees = STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
		learnedServoCommandOffsetDegrees = absoluteStartupErrorDegrees;
		double quadratureOffsetDegrees = quadRawAtStartDegrees - absoluteTurretReferenceAtStartDegrees;

		// Start absolute encoder unwrapping from the startup sample so telemetry is in the same frame.
		previousAbsoluteEncoderRawDegrees = absoluteRawAtStartDegrees;
		unwrappedAbsoluteEncoderDegrees = absoluteRawAtStartDegrees;

		lastTime = getRuntime();

		while (opModeIsActive()) {
			double now = getRuntime();
			double dt = now - lastTime;
			lastTime = now;

			// Preset positions (last pressed wins if multiple)
			if (gamepad1.x) targetAngleDegrees = MAX_SERVO_ROTATION_DEGREES;
			if (gamepad1.y) targetAngleDegrees = 0.0;
			if (gamepad1.b) targetAngleDegrees = MIN_SERVO_ROTATION_DEGREES;
			if (gamepad1.left_bumper) targetAngleDegrees = 90;
			if (gamepad1.right_bumper) targetAngleDegrees = -90;
			if (gamepad1.a && !aWasPressed) {
				servosEnabled = false;
				lurret.getController().pwmDisable();
				rurret.getController().pwmDisable();
				hurret.getController().pwmDisable();
			}
			aWasPressed = gamepad1.a;

			double correctedTargetAngleDegrees = targetAngleDegrees;
			if (targetAngleDegrees > TARGET_BIAS_APPLY_THRESHOLD_DEGREES) {
				correctedTargetAngleDegrees += POSITIVE_TARGET_BIAS_DEGREES;
			} else if (targetAngleDegrees < -TARGET_BIAS_APPLY_THRESHOLD_DEGREES) {
				correctedTargetAngleDegrees += NEGATIVE_TARGET_BIAS_DEGREES;
			}

			// Left stick X: lurret = speed up rate, rurret = slow down rate
			float stickX = gamepad1.left_stick_x;
			if (stickX < -0.2) {
				rateLimitDegPerSec = Math.min(MAX_RATE_DEG_PER_SEC, rateLimitDegPerSec + RATE_STEP_DEG_PER_LOOP * Math.abs(.02));
			} else if (stickX > 0.2) {
				rateLimitDegPerSec = Math.max(MIN_RATE_DEG_PER_SEC, rateLimitDegPerSec - RATE_STEP_DEG_PER_LOOP * .02);
			}

			// Rate-limited command move toward target (this is the nominal servo target angle).
			double maxMove = rateLimitDegPerSec * dt;
			double commandDiff = correctedTargetAngleDegrees - commandedAngleDegrees;
			if (Math.abs(commandDiff) <= maxMove) {
				commandedAngleDegrees = correctedTargetAngleDegrees;
			} else {
				commandedAngleDegrees += Math.signum(commandDiff) * maxMove;
			}

			int encoderTicks = turretEncoder.getCurrentPosition();
			int encoderDelta = encoderTicks - previousEncoderTicks;
			double quadRawAngleDegrees = encoderTicks / TURRET_ENCODER_COUNTS_PER_DEGREE;
			double measuredAngleDegrees = quadRawAngleDegrees - quadratureOffsetDegrees;
			double encoderDeltaDegrees = encoderDelta / TURRET_ENCODER_COUNTS_PER_DEGREE;
			double encoderVelocityDegPerSec = dt > 1e-6 ? (encoderDeltaDegrees / dt) : 0.0;
			double absoluteEncoderVoltage = absoluteTurretEncoder.getVoltage();
			double absoluteEncoderRawDegrees = absoluteVoltageToRawDegrees(absoluteEncoderVoltage);
			double absoluteEncoderDeltaRawDegrees = smallestWrappedDeltaDegrees(
					absoluteEncoderRawDegrees,
					previousAbsoluteEncoderRawDegrees
			);
			unwrappedAbsoluteEncoderDegrees += absoluteEncoderDeltaRawDegrees;
			previousAbsoluteEncoderRawDegrees = absoluteEncoderRawDegrees;
			double absoluteEncoderTurretAngleDegrees =
					((unwrappedAbsoluteEncoderDegrees - absoluteRawAtStartDegrees) / ENCODER_TO_TURRET_RATIO)
							+ absoluteTurretReferenceAtStartDegrees;
			double absoluteEncoderTurretDeltaDegrees = absoluteEncoderDeltaRawDegrees / ENCODER_TO_TURRET_RATIO;
			previousEncoderTicks = encoderTicks;

			// Hybrid control: servo gets commanded angle + small correction from measured encoder error.
			double angleErrorDegrees = targetAngleDegrees - measuredAngleDegrees;
			double outerLoopTrimDegrees = clamp(
					OUTER_LOOP_KP * angleErrorDegrees,
					-OUTER_LOOP_MAX_TRIM_DEGREES,
					OUTER_LOOP_MAX_TRIM_DEGREES
			);
			double servoCommandAngleDegrees = clamp(
					commandedAngleDegrees + outerLoopTrimDegrees,
					MIN_SERVO_ROTATION_DEGREES,
					MAX_SERVO_ROTATION_DEGREES
			);
			double currentPosition = angleDegreesToServoPosition(servoCommandAngleDegrees, learnedServoCommandOffsetDegrees);

			boolean inTolerance =
					Math.abs(angleErrorDegrees) <= READY_TOLERANCE_DEGREES &&
					Math.abs(encoderVelocityDegPerSec) <= READY_VELOCITY_TOLERANCE_DEG_PER_SEC;
			if (servosEnabled && inTolerance) {
				readyLoops++;
				notReadyLoops = 0;
			} else {
				if (notReadyLoops >= NOT_READY_LOOPS_REQUIRED) {
					readyLoops = 0;
				}
				notReadyLoops++;
			}
			boolean turretReady = servosEnabled && readyLoops >= READY_LOOPS_REQUIRED;

			if (servosEnabled) {
				lurret.setPosition(currentPosition);
				rurret.setPosition(currentPosition);
			}

			telemetry.addData("Target servo pos", "%.3f", angleDegreesToServoPosition(targetAngleDegrees, learnedServoCommandOffsetDegrees));
			telemetry.addData("Current servo pos", "%.3f", currentPosition);
			telemetry.addData("Target angle (deg)", "%.1f", targetAngleDegrees);
			telemetry.addData("Corrected target (deg)", "%.1f", correctedTargetAngleDegrees);
			telemetry.addData("Commanded angle (deg)", "%.1f", commandedAngleDegrees);
			telemetry.addData("Servo cmd angle (deg)", "%.1f", servoCommandAngleDegrees);
			telemetry.addData("Turret angle (deg)", "%.1f", measuredAngleDegrees);
			telemetry.addData("Quad raw angle (deg)", "%.2f", quadRawAngleDegrees);
			telemetry.addData("Quad offset (deg)", "%.2f", quadratureOffsetDegrees);
			telemetry.addData("Init stable at start", initStableAtStart);
			telemetry.addData("Angle error (deg)", "%.2f", angleErrorDegrees);
			telemetry.addData("Outer-loop trim (deg)", "%.2f", outerLoopTrimDegrees);
			telemetry.addData("Turret encoder (ticks)", encoderTicks);
			telemetry.addData("Turret encoder angle (deg)", "%.2f", measuredAngleDegrees);
			telemetry.addData("Absolute encoder (V)", "%.3f", absoluteEncoderVoltage);
			telemetry.addData("Absolute encoder raw angle (deg)", "%.2f", absoluteEncoderRawDegrees);
			telemetry.addData("Absolute encoder turret angle (deg)", "%.2f", absoluteEncoderTurretAngleDegrees);
			telemetry.addData("Abs turret ref @start (deg)", "%.2f", absoluteTurretReferenceAtStartDegrees);
			telemetry.addData("Abs turret est @start (deg)", "%.2f", absoluteTurretAtStartDegrees);
			telemetry.addData("Abs startup error (deg)", "%.2f", absoluteStartupErrorDegrees);
			telemetry.addData("Suggested abs offset (deg)", "%.2f", suggestedAbsoluteOffsetDegrees);
			telemetry.addData("Learned servo cmd offset (deg)", "%.2f", learnedServoCommandOffsetDegrees);
			telemetry.addData("Target +bias / -bias (deg)", "%.1f / %.1f", POSITIVE_TARGET_BIAS_DEGREES, NEGATIVE_TARGET_BIAS_DEGREES);
			telemetry.addData("Abs encoder turret offset (deg)", "%.1f", ABSOLUTE_ENCODER_TURRET_OFFSET_DEGREES);
			telemetry.addData("Encoder delta/loop", encoderDelta);
			telemetry.addData("Encoder delta/loop (deg)", "%.2f", encoderDeltaDegrees);
			telemetry.addData("Encoder vel (deg/s)", "%.2f", encoderVelocityDegPerSec);
			telemetry.addData("Absolute delta/loop (deg)", "%.2f", absoluteEncoderTurretDeltaDegrees);
			telemetry.addData("Rate limit", "%.1f deg/s", rateLimitDegPerSec); // EDINO!
			telemetry.addData("Ready loops", "%d/%d", readyLoops, READY_LOOPS_REQUIRED);
			telemetry.addData("Turret ready", turretReady);
			telemetry.addData("Servos enabled", servosEnabled);
			telemetry.addLine("A = disable servo PWM (latched)");
			telemetry.addLine("X=0 Y=0.5 B=1 LB=0.25 RB=0.75 | Stick: rate"); // HAW!
			telemetry.update();

			idle();
		}
	}

	private static double angleDegreesToServoPosition(double angleDegrees, double servoCommandOffsetDegrees) {
		// Learned runtime offset keeps servo command frame aligned to the absolute-based turret frame.
		double correctedAngleDegrees = angleDegrees - servoCommandOffsetDegrees;
		double unclippedPosition = SERVO_CENTER_POSITION - (correctedAngleDegrees / TURRET_TRAVEL_DEGREES);
		return clamp(unclippedPosition, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
	}

	private static double absoluteVoltageToRawDegrees(double voltage) {
		double clippedVoltage = clamp(voltage, 0.0, ABSOLUTE_TURRET_ENCODER_MAX_VOLTAGE);
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

	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}
