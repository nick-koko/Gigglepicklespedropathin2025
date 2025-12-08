package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * TeleOp Sensor Test - combines intake control with beam break sensors.
 * 
 * Hardware:
 * - Motors: m1 (top front), m2 (bottom), m3 (top back)
 * - Servos: s2 (bottom continuous), s3 (top back continuous)
 * - Sensors: sensor0, sensor1, sensor2 (beam breaks)
 * 
 * Logic:
 * - Each sensor corresponds to a motor: sensor0->m1, sensor1->m2, sensor2->m3
 * - Ball 1: passes through sensor0 and sensor1, stops motor m3 when it passes sensor2
 * - Ball 2: passes through sensor0, stops motor m2 when it passes sensor1
 * - Ball 3: stops motor m1 when it passes sensor0
 * 
 * Controls:
 * - Left bumper: run intake forward
 * - Right bumper: run intake reverse
 * - X/Y/B: manually toggle motors m1/m2/m3 on/off
 * - A: Shoot (spin all motors for 2 seconds, reset ball count)
 */
@Disabled
@Configurable
@TeleOp(name = "TeleOp Sensor Test", group = "Test")
public class TeleOpSensorTest extends LinearOpMode {

    // Intake RPM targets
    public static double M1_TARGET_RPM = 800.0; // top front
    public static double M2_TARGET_RPM = 350.0; // bottom
    public static double M3_TARGET_RPM = 400.0; // top back

    // Shoot RPM targets (used when A button is pressed)
    public static double M1_SHOOT_RPM = 200.0; // top front
    public static double M2_SHOOT_RPM = 200.0; // bottom
    public static double M3_SHOOT_RPM = 200.0; // top back

    // Continuous servo speeds (range -1.0 to 1.0)
    public static double S2_INTAKE_SPEED = 0.8;  // s2 intake speed
    public static double S3_INTAKE_SPEED = 0.8;  // s3 intake speed
    public static double S2_SHOOT_SPEED = 0.5;   // s2 shoot speed
    public static double S3_SHOOT_SPEED = 0.5;   // s3 shoot speed

    // goBILDA Yellow Jacket integrated encoder has 28 ticks per motor revolution
    private static final double ENCODER_TICKS_PER_MOTOR_REV = 28.0;
    // Gear ratios
    private static final double M1_GEAR_RATIO = 3.7;   // 3.7:1
    private static final double M2_GEAR_RATIO = 13.7;  // 13.7:1
    private static final double M3_GEAR_RATIO = 13.7;  // 13.7:1

    // Ball counting
    private int ballCount = 0; // Tracks how many balls have been detected

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "m1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "m2");
        DcMotorEx m3 = hardwareMap.get(DcMotorEx.class, "m3");

        // Initialize continuous servos
        CRServo s2 = hardwareMap.get(CRServo.class, "s2");
        CRServo s3 = hardwareMap.get(CRServo.class, "s3");

        // Initialize sensors
        DigitalChannel sensor0 = hardwareMap.get(DigitalChannel.class, "sensor0");
        DigitalChannel sensor1 = hardwareMap.get(DigitalChannel.class, "sensor1");
        DigitalChannel sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");

        // Configure motors
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setDirection(DcMotorSimple.Direction.FORWARD);
        m2.setDirection(DcMotorSimple.Direction.FORWARD);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure servos
        s2.setDirection(DcMotorSimple.Direction.FORWARD);
        s3.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure sensors
        sensor0.setMode(DigitalChannel.Mode.INPUT);
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        sensor2.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("TeleOp Sensor Test Ready");
        telemetry.addLine("LB=forward, RB=reverse, X/Y/B=toggle motors");
        telemetry.addLine("Sensors will automatically stop motors as balls pass");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Compute ticks-per-rev from gear ratios
        final double m1TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M1_GEAR_RATIO;
        final double m2TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M2_GEAR_RATIO;
        final double m3TicksPerRev = ENCODER_TICKS_PER_MOTOR_REV * M3_GEAR_RATIO;

        boolean m1Enabled = true;
        boolean m2Enabled = true;
        boolean m3Enabled = true;

        boolean prevX = false;
        boolean prevY = false;
        boolean prevB = false;
        boolean prevA = false;

        // Sensor state tracking for edge detection
        boolean prevSensor0 = sensor0.getState();
        boolean prevSensor1 = sensor1.getState();
        boolean prevSensor2 = sensor2.getState();

        boolean shooting = false;
        long shootEndTime = 0;

        while (opModeIsActive()) {
            boolean forward = gamepad1.left_bumper;
            boolean reverse = gamepad1.right_bumper;

            // Manual toggle controls
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean b = gamepad1.b;
            boolean a = gamepad1.a;
            
            if (x && !prevX) m1Enabled = !m1Enabled;
            if (y && !prevY) m2Enabled = !m2Enabled;
            if (b && !prevB) m3Enabled = !m3Enabled;
            
            // Shoot button - A
            if (a && !prevA && !shooting) {
                shooting = true;
                shootEndTime = System.currentTimeMillis() + 2000; // 2 seconds from now
                m1Enabled = true;
                m2Enabled = true;
                m3Enabled = true;
                ballCount = 0; // Reset ball count
            }
            
            // Check if shooting time is over
            if (shooting && System.currentTimeMillis() >= shootEndTime) {
                shooting = false;
            }

            // Read sensor states (false = beam broken, true = clear)
            boolean sensor0State = sensor0.getState();
            boolean sensor1State = sensor1.getState();
            boolean sensor2State = sensor2.getState();

            // Detect when a ball enters a sensor (transition from clear to broken)
            // This means a ball has just arrived at that sensor
            boolean sensor0Falling = !sensor0State && prevSensor0; // ball just broke sensor0
            boolean sensor1Falling = !sensor1State && prevSensor1; // ball just broke sensor1
            boolean sensor2Falling = !sensor2State && prevSensor2; // ball just broke sensor2

            // Sequential ball detection logic:
            // Ball 1: when it breaks sensor2, stop m3
            // Ball 2: when it breaks sensor1, stop m2
            // Ball 3: when it breaks sensor0, stop m1
            // Students: Notice how we count balls based on WHICH sensor is broken, not the order
            // Only detect balls when NOT shooting (so shooting can reset the count properly)
            if (!shooting) {
                if (sensor2Falling && ballCount == 0) {
                    m3Enabled = false; // Ball 1 broke sensor2, stop motor m3
                    ballCount = 1;
                } else if (sensor1Falling && ballCount == 1) {
                    m2Enabled = false; // Ball 2 broke sensor1, stop motor m2
                    ballCount = 2;
                } else if (sensor0Falling && ballCount == 2) {
                    m1Enabled = false; // Ball 3 broke sensor0, stop motor m1
                    ballCount = 3;
                }
            }

            // Recompute velocities from current target RPMs
            final double m1TicksPerSec = rpmToTicksPerSecond(M1_TARGET_RPM, m1TicksPerRev);
            final double m2TicksPerSec = rpmToTicksPerSecond(M2_TARGET_RPM, m2TicksPerRev);
            final double m3TicksPerSec = rpmToTicksPerSecond(M3_TARGET_RPM, m3TicksPerRev);

            // Compute shoot velocities
            final double m1ShootTps = rpmToTicksPerSecond(M1_SHOOT_RPM, m1TicksPerRev);
            final double m2ShootTps = rpmToTicksPerSecond(M2_SHOOT_RPM, m2TicksPerRev);
            final double m3ShootTps = rpmToTicksPerSecond(M3_SHOOT_RPM, m3TicksPerRev);

            // Control motors and servos based on shooting mode or bumper input
            if (shooting) {
                // Shooting mode: run all motors and servos forward at SHOOT speeds regardless of enable state
                m1.setVelocity(m1ShootTps);
                m2.setVelocity(m2ShootTps);
                m3.setVelocity(m3ShootTps);
                s2.setPower(S2_SHOOT_SPEED);
                s3.setPower(S3_SHOOT_SPEED);
            } else if (forward ^ reverse) { // exactly one pressed
                double direction = forward ? 1.0 : -1.0;
                if (m1Enabled) m1.setVelocity(direction * m1TicksPerSec); else m1.setPower(0.0);
                if (m2Enabled) {
                    m2.setVelocity(direction * m2TicksPerSec);
                    s2.setPower(direction * S2_INTAKE_SPEED);
                } else {
                    m2.setPower(0.0);
                    s2.setPower(0.0);
                }
                if (m3Enabled) {
                    m3.setVelocity(direction * m3TicksPerSec);
                    s3.setPower(direction * S3_INTAKE_SPEED);
                } else {
                    m3.setPower(0.0);
                    s3.setPower(0.0);
                }
            } else {
                // Neither or both pressed: stop
                m1.setPower(0.0);
                m2.setPower(0.0);
                m3.setPower(0.0);
                s2.setPower(0.0);
                s3.setPower(0.0);
            }

            // Compute current RPMs
            double m1Rpm = m1.getVelocity() * 60.0 / m1TicksPerRev;
            double m2Rpm = m2.getVelocity() * 60.0 / m2TicksPerRev;
            double m3Rpm = m3.getVelocity() * 60.0 / m3TicksPerRev;

            // Telemetry
            String mode = shooting ? "SHOOTING" : (forward ? "Forward" : (reverse ? "Reverse" : "Stopped"));
            telemetry.addData("Mode", mode);
            if (shooting) {
                long timeLeft = (shootEndTime - System.currentTimeMillis()) / 1000;
                telemetry.addData("Shoot time left", timeLeft + "s");
            }
            telemetry.addData("Ball Count", ballCount + "/3");
            telemetry.addLine();
            
            telemetry.addData("Sensor 0 (m1)", sensor0State ? "CLEAR" : "BROKEN");
            telemetry.addData("Sensor 1 (m2)", sensor1State ? "CLEAR" : "BROKEN");
            telemetry.addData("Sensor 2 (m3)", sensor2State ? "CLEAR" : "BROKEN");
            telemetry.addLine();
            
            telemetry.addData("m1", "%s  rpm=%.1f", m1Enabled ? "EN" : "OFF", m1Rpm);
            telemetry.addData("m2", "%s  rpm=%.1f", m2Enabled ? "EN" : "OFF", m2Rpm);
            telemetry.addData("m3", "%s  rpm=%.1f", m3Enabled ? "EN" : "OFF", m3Rpm);
            telemetry.update();

            // Update previous states
            prevX = x;
            prevY = y;
            prevB = b;
            prevA = a;
            prevSensor0 = sensor0State;
            prevSensor1 = sensor1State;
            prevSensor2 = sensor2State;
            
            idle();
        }
    }

    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }
}

