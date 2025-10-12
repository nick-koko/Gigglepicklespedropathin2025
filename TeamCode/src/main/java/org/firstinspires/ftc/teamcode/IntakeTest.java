package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Intake test TeleOp.
 *
 * Assumptions:
 * - Hardware names: m1 (front), m2 (back), m3 (back)
 * - Uses RUN_USING_ENCODER velocity control with DcMotorEx
 * - Left bumper spins forward, Right bumper spins backward
 */
@Configurable
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    public static double M1_TARGET_RPM = 800.0; // top front
    public static double M2_TARGET_RPM = 350.0; // bottom
    public static double M3_TARGET_RPM = 400.0; // top back

    // goBILDA Yellow Jacket integrated encoder has 28 ticks per motor revolution
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    // Gear ratios provided by user
    private static final double M1_GEAR_RATIO = 3.7;   // 3.7:1
    private static final double M2_GEAR_RATIO = 13.7;  // 13.7:1
    private static final double M3_GEAR_RATIO = 13.7;  // 13.7:1

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "m1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "m2");
        DcMotorEx m3 = hardwareMap.get(DcMotorEx.class, "m3");

        // Configure motors
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Intake Test ready. LB=forward, RB=reverse");
        telemetry.addData("m1 target RPM", M1_TARGET_RPM);
        telemetry.addData("m2 target RPM", M2_TARGET_RPM);
        telemetry.addData("m3 target RPM", M3_TARGET_RPM);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Compute ticks-per-rev from gear ratios for accurate velocity control
        final double m1TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M1_GEAR_RATIO; // 28 * 3.7 = 103.6
        final double m2TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M2_GEAR_RATIO; // 28 * 13.7 = 383.6
        final double m3TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M3_GEAR_RATIO; // 28 * 13.7 = 383.6

        boolean m1Enabled = true;
        boolean m2Enabled = true;
        boolean m3Enabled = true;

        boolean prevX = false;
        boolean prevY = false;
        boolean prevB = false;

        while (opModeIsActive()) {
            boolean forward = gamepad1.left_bumper;
            boolean reverse = gamepad1.right_bumper;

            // Toggle enable/disable per motor on button rising edge
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean b = gamepad1.b;
            if (x && !prevX) m1Enabled = !m1Enabled;
            if (y && !prevY) m2Enabled = !m2Enabled;
            if (b && !prevB) m3Enabled = !m3Enabled;

            // Recompute velocities from current (possibly updated) target RPMs
            final double m1TicksPerSec = rpmToTicksPerSecond(M1_TARGET_RPM, m1TicksPerRev);
            final double m2TicksPerSec = rpmToTicksPerSecond(M2_TARGET_RPM, m2TicksPerRev);
            final double m3TicksPerSec = rpmToTicksPerSecond(M3_TARGET_RPM, m3TicksPerRev);

            if (forward ^ reverse) { // exactly one pressed
                double direction = forward ? 1.0 : -1.0;
                if (m1Enabled) m1.setVelocity(direction * m1TicksPerSec); else m1.setPower(0.0);
                if (m2Enabled) m2.setVelocity(direction * m2TicksPerSec); else m2.setPower(0.0);
                if (m3Enabled) m3.setVelocity(direction * m3TicksPerSec); else m3.setPower(0.0);
            } else {
                // Neither or both pressed: stop
                m1.setPower(0.0);
                m2.setPower(0.0);
                m3.setPower(0.0);
            }

            // Compute current RPMs from measured ticks/sec and gear-adjusted ticks/rev
            double m1Rpm = m1.getVelocity() * 60.0 / m1TicksPerRev;
            double m2Rpm = m2.getVelocity() * 60.0 / m2TicksPerRev;
            double m3Rpm = m3.getVelocity() * 60.0 / m3TicksPerRev;

            telemetry.addData("Mode", forward ? "Forward" : (reverse ? "Reverse" : "Stopped"));
            telemetry.addData("targets rpm", "m1=%.0f m2=%.0f m3=%.0f", M1_TARGET_RPM, M2_TARGET_RPM, M3_TARGET_RPM);
            telemetry.addData("m1", "%s  rpm=%.1f", m1Enabled ? "EN" : "OFF", m1Rpm);
            telemetry.addData("m2", "%s  rpm=%.1f", m2Enabled ? "EN" : "OFF", m2Rpm);
            telemetry.addData("m3", "%s  rpm=%.1f", m3Enabled ? "EN" : "OFF", m3Rpm);
            telemetry.update();

            prevX = x;
            prevY = y;
            prevB = b;
            idle();
        }
    }

    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }
}


