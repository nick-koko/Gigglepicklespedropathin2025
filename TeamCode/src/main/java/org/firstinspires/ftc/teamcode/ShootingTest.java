package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Configurable
@TeleOp(name = "Shooting Test", group = "Test")
public class ShootingTest extends LinearOpMode {

	// Live-tunable with Panels
	public static double HIGH_TARGET_RPM = 5000.0; // X button
	public static double LOW_TARGET_RPM = 1300.0;  // B button
	public static double CR_HIGH_TARGET_RPM = 900.0; // Y button (counter roller)
	public static double CR_LOW_TARGET_RPM = 500.0;  // A button (counter roller)
	private static double SHOOTER_GEAR_RATIO = 1.0; // Yellow Jacket default used here

	private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0; // Yellow Jacket

	@Override
	public void runOpMode() throws InterruptedException {
		DcMotorEx s1 = hardwareMap.get(DcMotorEx.class, "m1");
		DcMotorEx s2 = hardwareMap.get(DcMotorEx.class, "m2");
		DcMotorEx cr = hardwareMap.get(DcMotorEx.class, "m3");


		s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	    cr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		s1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		s2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		cr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		s1.setDirection(DcMotorSimple.Direction.REVERSE);
		s2.setDirection(DcMotorSimple.Direction.FORWARD);
		cr.setDirection(DcMotorSimple.Direction.FORWARD);

		telemetry.addLine("Shooting Test ready. X/B=shooters high/low, Y/A=CR high/low, LT=toggle ON/OFF");
		telemetry.addLine("s1/s2 reversed direction, CR forward");
		telemetry.update();

		waitForStart();
		if (isStopRequested()) return;

		final double ticksPerRev = ENCODER_TICKS_PER_MOTOR_REV * SHOOTER_GEAR_RATIO;

		boolean enabled = false;
		boolean highMode = true; // X selects high, B selects low
		boolean crHighMode = true; // Y selects high, A selects low for counter roller
		boolean prevX = false;
		boolean prevB = false;
		boolean prevLT = false;
		boolean prevY = false;
		boolean prevA = false;

		while (opModeIsActive()) {
			boolean x = gamepad1.x;
			boolean b = gamepad1.b;
			boolean ltPressed = gamepad1.left_trigger > 0.3;
			boolean y = gamepad1.y;
			boolean a = gamepad1.a;

			if (x && !prevX) highMode = true;
			if (b && !prevB) highMode = false;
			if (ltPressed && !prevLT) enabled = !enabled; // toggle on left trigger edge
			if (y && !prevY) crHighMode = true;
			if (a && !prevA) crHighMode = false;

			double targetRpm = highMode ? HIGH_TARGET_RPM : LOW_TARGET_RPM;
			double crTargetRpm = crHighMode ? CR_HIGH_TARGET_RPM : CR_LOW_TARGET_RPM;
			double targetTps = rpmToTicksPerSecond(targetRpm, ticksPerRev);
			double crTargetTps = rpmToTicksPerSecond(crTargetRpm, ticksPerRev);

			if (enabled) {
				// Shooters use selected target; counter roller uses its own target
				s1.setVelocity(targetTps);
				s2.setVelocity(targetTps);
				cr.setVelocity(crTargetTps);
			} else {
				s1.setPower(0.0);
				s2.setPower(0.0);
				cr.setPower(0.0);
			}

			double s1Rpm = s1.getVelocity() * 60.0 / ticksPerRev;
			double s2Rpm = s2.getVelocity() * 60.0 / ticksPerRev;
			double crRpm = cr.getVelocity() * 60.0 / ticksPerRev;

			telemetry.addData("Mode", enabled ? (highMode ? "HIGH" : "LOW") : "OFF");
			telemetry.addData("Target RPM shooters", "%.0f", targetRpm);
			telemetry.addData("Target RPM CR", "%.0f", crTargetRpm);
			telemetry.addData("s1 rpm", "%.1f", s1Rpm);
			telemetry.addData("s2 rpm", "%.1f", s2Rpm);
			telemetry.addData("cr rpm", "%.1f", crRpm);
			telemetry.update();

			prevX = x;
			prevB = b;
			prevLT = ltPressed;
			prevY = y;
			prevA = a;
			idle();
		}
	}

	private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
		return (rpm * ticksPerRev) / 60.0;
	}
}


