package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestingIntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestingShooterSubsystem;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "Shooter Tuning Teleop", group = "Comp")
public class PicklesTuningTeleop extends NextFTCOpMode {
    public PicklesTuningTeleop() {
        addComponents(
                new SubsystemComponent(TestingShooterSubsystem.INSTANCE, TestingIntakeWithSensorsSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    // Adjust these from Panels at runtime
    public static double targetRPM = 3000.0;
    public static double shooterHoodPos = 0.5;
    public static boolean SHOW_SMOOTHED = true;
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

    private TelemetryManager telemetryM;
    double angleAllianceOffset = 0.0;
    double drivePower = 1.0;


    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        windowShooter1 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowShooter2 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowOuttake = new double[Math.max(1, SMOOTH_WINDOW)];

    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {
        //Call this once per loop
        timer.start();

        double rpmShooter1 = TestingShooterSubsystem.INSTANCE.getShooter1RPM();
        double rpmShooter2 = TestingShooterSubsystem.INSTANCE.getShooter2RPM();
        double rpmOuttake = TestingIntakeWithSensorsSubsystem.INSTANCE.getMotor3RPM();

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
        telemetryM.addData("Shooter2_RPM", rpmShooter2);
        telemetryM.addData("Outtake_RPM", rpmOuttake);

        if (SHOW_SMOOTHED) {
            telemetryM.addData("Shooter1_RPM (smoothed)", rpmShooter1Smoothed);
            telemetryM.addData("Shooter2_RPM (smoothed)", rpmShooter2Smoothed);
            telemetryM.addData("Outtake_RPM (smoothed)", rpmOuttakeSmoothed);
        }

        boolean bb0 = TestingIntakeWithSensorsSubsystem.INSTANCE.isSensor0Broken();
        boolean bb1 = TestingIntakeWithSensorsSubsystem.INSTANCE.isSensor1Broken();
        boolean bb2 = TestingIntakeWithSensorsSubsystem.INSTANCE.isSensor2Broken();

        telemetryM.addData("ballCount", TestingIntakeWithSensorsSubsystem.INSTANCE.getBallCount());
        telemetryM.addData("BB_sensor0", bb0 ? 1 : 0);
        telemetryM.addData("BB_sensor1", bb1 ? 1 : 0);
        telemetryM.addData("BB_sensor2", bb2 ? 1 : 0);

        telemetryM.addData("LoopTime_ms", timer.getMs());


        if (gamepad2.rightBumperWasPressed()) {
            TestingShooterSubsystem.INSTANCE.spinUp(targetRPM);
        }
        else if (gamepad2.leftBumperWasPressed()) {
            TestingShooterSubsystem.INSTANCE.stop();
            TestingIntakeWithSensorsSubsystem.INSTANCE.stop();
        }

        if (gamepad2.right_trigger > 0.1) {
            TestingIntakeWithSensorsSubsystem.INSTANCE.dumbShoot();
            TestingIntakeWithSensorsSubsystem.INSTANCE.setBallCount(0);

        }
        else if (gamepad2.aWasPressed()) {
            TestingIntakeWithSensorsSubsystem.INSTANCE.intakeForward();  //Hoping Forward is Intake (maybe change the method name)
            TestingShooterSubsystem.INSTANCE.stop();
        }
        else if (gamepad2.b) {
            TestingIntakeWithSensorsSubsystem.INSTANCE.intakeReverse();
        }

        TestingShooterSubsystem.INSTANCE.shooterHoodDrive(shooterHoodPos);


        telemetryM.update(telemetry);

        timer.end();
    }
}