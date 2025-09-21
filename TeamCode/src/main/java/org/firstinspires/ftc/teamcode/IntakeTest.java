package org.firstinspires.ftc.teamcode;

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
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    private static final double M1_TARGET_RPM = 435.0; // top front
    private static final double M2_TARGET_RPM = 350.0; // bottom
    private static final double M3_TARGET_RPM = 300.0; // top back

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
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

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

        final double m1TicksPerSec = rpmToTicksPerSecond(M1_TARGET_RPM, m1TicksPerRev);
        final double m2TicksPerSec = rpmToTicksPerSecond(M2_TARGET_RPM, m2TicksPerRev);
        final double m3TicksPerSec = rpmToTicksPerSecond(M3_TARGET_RPM, m3TicksPerRev);

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

            telemetry.addData("Mode", forward ? "Forward" : (reverse ? "Reverse" : "Stopped"));
            telemetry.addData("m1", "%s  vel=%.1f tps", m1Enabled ? "EN" : "OFF", m1.getVelocity());
            telemetry.addData("m2", "%s  vel=%.1f tps", m2Enabled ? "EN" : "OFF", m2.getVelocity());
            telemetry.addData("m3", "%s  vel=%.1f tps", m3Enabled ? "EN" : "OFF", m3.getVelocity());
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


