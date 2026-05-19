package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Demo teleop: full driving + intake, manual turret/shooter controls via Gamepad 2.
 *
 * Gamepad 2 controls:
 *   Left  stick X   → pan turret left/right (clamped to hardware limits ±130°)
 *   Left  stick Y   → adjust flywheel target RPM (up = faster, down = slower)
 *   Right stick Y   → adjust hood position (clamped to HOOD_MIN/MAX)
 *   Left  trigger   → toggle flywheel on / off
 *   Right trigger   → fire one ball (requires flywheel running)
 *   Right bumper    → dumbShoot burst + boost (same logic as main teleop fire)
 *   A               → intake forward / reset
 *   B (held)        → intake reverse
 *
 * Gamepad 1 controls (driving — identical to main teleop):
 *   Left  stick     → translate
 *   Right stick X   → rotate
 *   Right trigger   → dumbShoot burst + boost  (locks robot position while held)
 *   Right bumper    → dumbShoot burst + boost
 */
@Configurable
@TeleOp(name = "Pickles 2025 Demo", group = "Demo")
public class Pickles2025DemoTeleop extends NextFTCOpMode {

    public Pickles2025DemoTeleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem.INSTANCE,
                        IntakeWithSensorsSubsystem.INSTANCE,
                        LEDControlSubsystem.INSTANCE,
                        TurretSubsystem.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    // ── Starting pose (same reference as the main teleop) ──────────────────────
    public static Pose startingPoseBlue = new Pose(31.5, 134.0, Math.toRadians(270));
    public static Pose startingPoseRed  = startingPoseBlue.mirror(144);
    public static Pose startingPose     = startingPoseBlue;

    // ── Demo tuning constants ──────────────────────────────────────────────────
    /** Degrees-per-second the turret moves at full left-stick X deflection. */
    public static double DEMO_TURRET_RATE_DEG_PER_SEC = 90.0;
    /** RPM change per second at full left-stick Y deflection. */
    public static double DEMO_RPM_RATE_PER_SEC = 400.0;
    /** Lowest RPM the operator can dial in with the stick. */
    public static double DEMO_RPM_MIN = 500.0;
    /** Highest RPM the operator can dial in with the stick. */
    public static double DEMO_RPM_MAX = 5500.0;
    /** Starting RPM when the match begins. */
    public static double DEMO_RPM_DEFAULT = 3200.0;
    /** Joystick deadband applied to all sticks. */
    public static double DEMO_STICK_DEADBAND = 0.05;
    /** Drive power (normal). */
    public static double DEMO_DRIVE_POWER = 1.0;

    // ── DumbShoot / boost constants (mirrors main teleop) ─────────────────────
    /** How long to let the burst sequence run before auto-stopping the shooter. */
    public static long   DUMBSHOOT_SHOOTER_TIMEOUT_MS    = 1200L;
    /** Settle window after stopping so a mid-transit ball can fully clear. */
    public static long   DUMBSHOOT_POST_SHOT_SETTLE_MS   = 250L;
    /**
     * RPM threshold above which the far no-boost profile is used.
     * Above this threshold, boost is skipped and a fixed inter-shot delay is used.
     */
    public static double FAR_NO_BOOST_RPM_THRESHOLD      = 4000.0;
    /** Inter-shot delay used when FAR_NO_BOOST path is active. */
    public static long   FAR_NO_BOOST_BETWEEN_SHOTS_MS   = 150L;
    /**
     * Fixed inter-shot delay (ms) used for normal (non-far-no-boost) dumbShoot
     * in demo mode. Replaces the distance-based delay that is unavailable without
     * a real field target.  ~200 ms matches a mid-range competition shot.
     */
    public static long   DEMO_BETWEEN_SHOTS_MS           = 200L;
    /**
     * Effective "distance to goal" forwarded to the hybrid boost controller.
     * This is demo-only; tune it to match the boost profile you want to demonstrate.
     */
    public static double DEMO_SHOOT_DISTANCE_IN          = 60.0;
    /** RPM tolerance used for the at-speed gate before allowing a burst. */
    public static double SHOOT_GATE_AT_SPEED_TOLERANCE_RPM = 150.0;
    /** Minimum target RPM before the at-speed gate can be considered satisfied. */
    public static double SHOOT_GATE_MIN_TARGET_RPM       = 500.0;

    // Assumed loop period for stick integration (seconds).
    private static final double LOOP_TIME_SEC = 0.020;

    // ── Startup calibration constants (mirrored from main teleop) ─────────────
    public static long TELEOP_TURRET_STARTUP_SETTLE_MS       = 250L;
    public static int  TELEOP_TURRET_START_SAMPLE_COUNT      = 5;
    public static long TELEOP_TURRET_START_SAMPLE_INTERVAL_MS = 10L;

    // ── Instance state ─────────────────────────────────────────────────────────
    private boolean matchHasStarted       = false;
    private double  demoTurretTargetDeg   = 0.0;
    private double  demoTargetRpm         = DEMO_RPM_DEFAULT;
    private boolean demoFlywheelRunning   = false;

    // DumbShoot burst timer state
    private boolean dumbShootTimerActive  = false;
    private long    dumbShootStartTimeMs  = 0L;
    private boolean dumbShootSettleActive = false;
    private long    dumbShootSettleStartMs = 0L;

    // Hold-point state (right trigger locks robot while held, same as main teleop)
    private boolean hold     = false;
    private boolean prevHold = false;

    // Previous-frame edge detectors
    private boolean prevLeftTrigger  = false;
    private boolean prevRightTrigger = false;

    // ── Lifecycle ──────────────────────────────────────────────────────────────

    @Override
    public void onInit() {
        matchHasStarted      = false;
        demoFlywheelRunning  = false;
        demoTargetRpm        = DEMO_RPM_DEFAULT;
        demoTurretTargetDeg  = 0.0;
        dumbShootTimerActive  = false;
        dumbShootSettleActive = false;
        hold     = false;
        prevHold = false;
        prevLeftTrigger  = false;
        prevRightTrigger = false;

        ShooterSubsystem.INSTANCE.stop();
        ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();

        TurretSubsystem.INSTANCE.inTeleop = true;
        TurretSubsystem.INSTANCE.turretCenterDelayComplete = false;
    }

    @Override
    public void onWaitForStart() {
        // Alliance selection: X = blue, B = red  (same as main teleop)
        if (gamepad1.xWasPressed()) {
            GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
            startingPose = startingPoseBlue;
        } else if (gamepad1.bWasPressed()) {
            GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
            startingPose = startingPoseRed;
        }

        telemetry.addLine("=== DEMO MODE ===");
        telemetry.addLine("Press X (Blue) or B (Red) to choose alliance.");
        telemetry.addData("Alliance", GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        matchHasStarted = true;

        // Turret calibration — same pattern as main teleop
        TurretSubsystem.INSTANCE.beginStartupCalibrationWithoutCentering();
        TurretSubsystem.INSTANCE.turretResetDelayTotalTime = 0.0;

        if (TurretSubsystem.INSTANCE.turretCenterDelayComplete) {
            if (!TurretSubsystem.INSTANCE.isStartupCalibrationComplete()) {
                TurretSubsystem.INSTANCE.forceStartupCalibrationFromExpectedSampled(
                        TurretSubsystem.STARTUP_EXPECTED_TURRET_ANGLE_DEGREES,
                        TELEOP_TURRET_START_SAMPLE_COUNT,
                        TELEOP_TURRET_START_SAMPLE_INTERVAL_MS
                );
            }
        }

        // Seed turret target from current measured position so it doesn't jump on first loop
        demoTurretTargetDeg = TurretSubsystem.INSTANCE.getMeasuredAngleDegrees();

        PedroComponent.follower().setStartingPose(startingPose);
        PedroComponent.follower().startTeleopDrive();

        ShooterSubsystem.INSTANCE.shooterHoodDrive(0.0);
    }

    // ── Main loop ──────────────────────────────────────────────────────────────

    @Override
    public void onUpdate() {
        long nowMs = System.currentTimeMillis();

        // ── Phase 1: dumbShoot burst timeout ──────────────────────────────────
        // Once the burst timer elapses, stop the shooter/intake and start the
        // settle window. Mirrors the same two-phase pattern in the main teleop.
        if (dumbShootTimerActive && nowMs - dumbShootStartTimeMs >= DUMBSHOOT_SHOOTER_TIMEOUT_MS) {
            ShooterSubsystem.INSTANCE.stop();
            ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();
            IntakeWithSensorsSubsystem.INSTANCE.stop();
            demoFlywheelRunning  = false;
            dumbShootTimerActive = false;
            dumbShootSettleActive    = true;
            dumbShootSettleStartMs   = nowMs;
        }

        // ── Phase 2: settle window elapsed ────────────────────────────────────
        // Re-validate ball count and re-arm intake now that any mid-transit ball
        // has had time to either exit or settle at a sensor.
        if (dumbShootSettleActive && nowMs - dumbShootSettleStartMs >= DUMBSHOOT_POST_SHOT_SETTLE_MS) {
            IntakeWithSensorsSubsystem.INSTANCE.validateBallCountAfterShoot();
            IntakeWithSensorsSubsystem.INSTANCE.intakeForward();
            dumbShootSettleActive = false;
        }

        // ── Driving (Gamepad 1) ────────────────────────────────────────────────
        double driving = (-gamepad1.left_stick_y) * DEMO_DRIVE_POWER;
        double strafe  = (-gamepad1.left_stick_x) * DEMO_DRIVE_POWER;
        double rotate  = (-gamepad1.right_stick_x) * 0.5;

        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            driving *= -1;
            strafe  *= -1;
        }

        if (!hold) {
            PedroComponent.follower().setTeleOpDrive(driving, strafe, rotate, false);
        }

        // ── Intake (Gamepad 2 A / B — same as main teleop) ────────────────────
        if (gamepad2.aWasPressed()) {
            hold = false;
            dumbShootTimerActive  = false;
            dumbShootSettleActive = false;
            ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();
            IntakeWithSensorsSubsystem.INSTANCE.intakeForward();
        } else if (gamepad2.b) {
            hold = false;
            IntakeWithSensorsSubsystem.INSTANCE.intakeReverse();
        }

        // ── Demo: Turret (Gamepad 2 left stick X) ─────────────────────────────
        // Integrate stick position into a target angle each loop.
        // setTargetAngleDegrees() clamps to [-130°, +130°]; read-back prevents
        // the accumulator from winding past the end-stops.
        double turretStickX = gamepad2.left_stick_x;
        if (Math.abs(turretStickX) > DEMO_STICK_DEADBAND) {
            demoTurretTargetDeg += turretStickX * DEMO_TURRET_RATE_DEG_PER_SEC * LOOP_TIME_SEC;
        }
        TurretSubsystem.INSTANCE.setTargetAngleDegrees(demoTurretTargetDeg);
        demoTurretTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();

        // ── Demo: Flywheel RPM adjust (Gamepad 2 left stick Y, up = faster) ───
        double rpmStickY = -gamepad2.left_stick_y;
        if (Math.abs(rpmStickY) > DEMO_STICK_DEADBAND) {
            demoTargetRpm += rpmStickY * DEMO_RPM_RATE_PER_SEC * LOOP_TIME_SEC;
            demoTargetRpm  = Math.max(DEMO_RPM_MIN, Math.min(DEMO_RPM_MAX, demoTargetRpm));
        }

        // Keep flywheel at current target RPM while running
        if (matchHasStarted && demoFlywheelRunning) {
            ShooterSubsystem.INSTANCE.spinUp(demoTargetRpm);
        }

        // ── Demo: Hood (Gamepad 2 right stick Y) ──────────────────────────────
        // driveShooterHood() applies a -joystick * 0.01 increment and clamps to
        // [HOOD_MIN_POS, HOOD_MAX_POS] internally.
        ShooterSubsystem.INSTANCE.driveShooterHood(gamepad2.right_stick_y);

        // ── Demo: Flywheel toggle (Gamepad 2 left trigger — rising edge) ──────
        boolean leftTriggerNow = gamepad2.left_trigger > 0.1;
        if (leftTriggerNow && !prevLeftTrigger) {
            demoFlywheelRunning = !demoFlywheelRunning;
            if (demoFlywheelRunning) {
                ShooterSubsystem.INSTANCE.spinUp(demoTargetRpm);
            } else {
                ShooterSubsystem.INSTANCE.stop();
            }
        }
        prevLeftTrigger = leftTriggerNow;

        // ── Demo: Single-ball fire (Gamepad 2 right trigger — rising edge) ────
        // One ball per press; requires the flywheel to be running.
        boolean gp2RightTriggerNow = gamepad2.right_trigger > 0.1;
        if (gp2RightTriggerNow && !prevRightTrigger) {
            if (demoFlywheelRunning) {
                IntakeWithSensorsSubsystem.INSTANCE.feedSingleBallFullPower();
            }
        }
        prevRightTrigger = gp2RightTriggerNow;

        // ── DumbShoot + boost helper ───────────────────────────────────────────
        // Shared by Gamepad 1 (right bumper / right trigger) and Gamepad 2
        // (right bumper). At-speed check mirrors the main teleop's canFireTriggerShot
        // gate so we don't feed balls into a flywheel that isn't up to speed.
        double shooter1Rpm  = ShooterSubsystem.INSTANCE.getShooter1RPM();
        double shooter2Rpm  = ShooterSubsystem.INSTANCE.getShooter2RPM();
        double avgRpm       = 0.5 * (shooter1Rpm + shooter2Rpm);
        boolean shooterAtSpeed =
                demoTargetRpm >= SHOOT_GATE_MIN_TARGET_RPM &&
                Math.abs(demoTargetRpm - avgRpm) <= SHOOT_GATE_AT_SPEED_TOLERANCE_RPM;

        // ── Gamepad 1: right trigger + right bumper (same as main teleop) ─────
        boolean gp1RightTriggerActive = gamepad1.right_trigger > 0.1;
        boolean gp1RightBumperActive  = gamepad1.right_bumper;
        boolean gp1FireRequested      = gp1RightTriggerActive || gp1RightBumperActive;

        if (gp1FireRequested) {
            // Right trigger locks the robot position while held (same as main teleop)
            hold = gp1RightTriggerActive;

            if (demoFlywheelRunning && shooterAtSpeed &&
                    !dumbShootTimerActive && !dumbShootSettleActive) {
                triggerDumbShoot(nowMs);
            }
        } else if (!gamepad2.right_bumper) {
            // Only clear hold when neither GP1 nor GP2 bumper fire is active
            hold = false;
        }

        // ── Gamepad 2: right bumper — dumbShoot burst (same logic as GP1) ─────
        if (gamepad2.right_bumper) {
            if (demoFlywheelRunning && shooterAtSpeed &&
                    !dumbShootTimerActive && !dumbShootSettleActive) {
                triggerDumbShoot(nowMs);
            }
        }

        // ── Hold-point management (mirrors main teleop) ───────────────────────
        if (hold && !prevHold) {
            PedroComponent.follower().holdPoint(
                    new BezierPoint(PedroComponent.follower().getPose()),
                    PedroComponent.follower().getHeading(),
                    false
            );
        } else if (!hold && prevHold) {
            PedroComponent.follower().startTeleopDrive();
        }
        prevHold = hold;

        // ── LED feedback ───────────────────────────────────────────────────────
        if (demoFlywheelRunning) {
            if (shooterAtSpeed) {
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
            } else {
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.YELLOW);
            }
        } else {
            int balls = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
            if (balls >= 3) {
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
            } else if (balls == 2) {
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.YELLOW);
            } else if (balls == 1) {
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.ORANGE);
            } else {
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.RED);
            }
        }

        // ── Telemetry ──────────────────────────────────────────────────────────
        telemetry.addLine("=== DEMO MODE ===");
        telemetry.addData("Flywheel",       demoFlywheelRunning ? "ON" : "OFF  (GP2 L-trig = toggle)");
        telemetry.addData("Target RPM",     String.format("%.0f  (GP2 L-stick Y)", demoTargetRpm));
        telemetry.addData("Shooter1 RPM",   String.format("%.0f", shooter1Rpm));
        telemetry.addData("Shooter2 RPM",   String.format("%.0f", shooter2Rpm));
        telemetry.addData("At Speed",       shooterAtSpeed);
        telemetry.addLine("---");
        telemetry.addData("Turret Target°", String.format("%.1f  (GP2 L-stick X)", TurretSubsystem.INSTANCE.getTargetAngleDegrees()));
        telemetry.addData("Turret Actual°", String.format("%.1f", TurretSubsystem.INSTANCE.getMeasuredAngleDegrees()));
        telemetry.addLine("---");
        telemetry.addData("Hood Position",  String.format("%.3f  (GP2 R-stick Y)", ShooterSubsystem.INSTANCE.getShooterHoodPosition()));
        telemetry.addLine("---");
        telemetry.addData("Balls",          IntakeWithSensorsSubsystem.INSTANCE.getBallCount());
        telemetry.addData("DumbShoot",      dumbShootTimerActive ? "BURST" : dumbShootSettleActive ? "SETTLE" : "ready");
        telemetry.addData("Alliance",       GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void onStop() {
        super.onStop();
        matchHasStarted      = false;
        demoFlywheelRunning  = false;
        dumbShootTimerActive  = false;
        dumbShootSettleActive = false;
        hold     = false;
        prevHold = false;
        ShooterSubsystem.INSTANCE.stop();
        ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();
    }

    // ── Helpers ────────────────────────────────────────────────────────────────

    /**
     * Trigger a dumbShoot burst with the same boost logic as the main teleop.
     * Must only be called when the at-speed gate and timer guards are already satisfied.
     */
    private void triggerDumbShoot(long nowMs) {
        boolean farNoBoostShot = demoTargetRpm >= FAR_NO_BOOST_RPM_THRESHOLD;

        ShooterSubsystem.INSTANCE.boostOverride = false;
        ShooterSubsystem.INSTANCE.resetHybridShotFeedBoostController();

        if (farNoBoostShot) {
            IntakeWithSensorsSubsystem.INSTANCE.setDumbShootFixedDelayMs(FAR_NO_BOOST_BETWEEN_SHOTS_MS);
        } else {
            IntakeWithSensorsSubsystem.INSTANCE.setDumbShootFixedDelayMs(DEMO_BETWEEN_SHOTS_MS);
        }

        IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();

        dumbShootTimerActive = true;
        dumbShootStartTimeMs = nowMs;

        // Apply boost profile for near/normal shots (skip for high-RPM far shots)
        if (!farNoBoostShot) {
            boolean useFarBoostProfile =
                    DEMO_SHOOT_DISTANCE_IN >= ShooterSubsystem.HYBRID_NEAR_FAR_DISTANCE_THRESHOLD_IN;
            if (ShooterSubsystem.ENABLE_HYBRID_SHOT_FEED_BOOST) {
                ShooterSubsystem.INSTANCE.startHybridShotFeedBoostController(DEMO_SHOOT_DISTANCE_IN);
            } else {
                ShooterSubsystem.INSTANCE.setBoostOn(useFarBoostProfile);
            }
        }
    }
}
