package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.util.CsvLogger;
import java.io.File;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "Pickles 2025 Teleop", group = "Comp")
public class Pickles2025Teleop extends NextFTCOpMode {
    public Pickles2025Teleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        ShooterSubsystem.INSTANCE,
                        IntakeWithSensorsSubsystem.INSTANCE,
                        LEDControlSubsystem.INSTANCE,
                        TurretSubsystem.INSTANCE
                ),
                BulkReadComponent.INSTANCE
                //FateComponent.INSTANCE
        );
    }

    public static Pose startingPoseBlue = new Pose(32.5, 134.375, Math.toRadians(180)); //See ExampleAuto to understand how to use this
    public static Pose startingPoseRed = startingPoseBlue.mirror();
    public static Pose startingPose = startingPoseBlue;

    // logging variables
    private CsvLogger turretLogger;
    private CsvLogger shotInfoLogger;
    private CsvLogger sotmLogger;
    private CsvLogger dumbShootRpmLogger;
    private CsvLogger shotTuningLogger;
    private long lastTurretLogMs = 0L;
    private long lastSotmLogMs = 0L;
    private long lastDumbShootRpmLogMs = 0L;
    public static boolean ENABLE_TURRET_LOGGING = false;
    public static long TURRET_LOG_PERIOD_MS = 25;
    // Turret diagnostics for steady-state tracking/stiction investigation.
    public static double TURRET_DIAG_TARGET_STABLE_DELTA_DEG = 0.15;
    public static double TURRET_DIAG_STICTION_ERROR_MIN_DEG = 0.8;
    public static double TURRET_DIAG_STICTION_VELOCITY_MAX_DEG_PER_SEC = 2.0;
    public static double TURRET_DIAG_SERVO_DEADBAND_POS_GUESS = 0.0010;
    public static long TURRET_DIAG_STICTION_MIN_TIME_MS = 120;
    public static boolean ENABLE_SOTM_LOGGING = false;
    public static long SOTM_LOG_PERIOD_MS = 25;
    public static boolean ENABLE_DUMBSHOOT_RPM_LOGGING = false;
    public static long DUMBSHOOT_RPM_LOG_PERIOD_MS = 10;
    public static boolean DUMBSHOOT_RPM_LOG_EVERY_LOOP = true;
    public static long DUMBSHOOT_RPM_LOG_DURATION_MS = 1100;
    public static int BURST_PROFILE_ID = 0;
    public static long SHOOT_BLOCK_LED_STROBE_MS = 180;
    public static double SHOOT_GATE_AT_SPEED_TOLERANCE_RPM = 75.0;
    public static double SHOOT_GATE_MIN_TARGET_RPM = 500.0;
    public static boolean ENABLE_HYBRID_SHOT_FEED_BOOST = true;
    public static double HYBRID_NEAR_FAR_DISTANCE_THRESHOLD_IN = 105.0;
    public static long HYBRID_CLOSE_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST = 135;
    public static long HYBRID_CLOSE_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST = 115;
    public static long HYBRID_CLOSE_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST = 155;
    public static long HYBRID_FAR_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST = 195;
    public static long HYBRID_FAR_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST = 125;
    public static long HYBRID_FAR_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST = 145;
    public static long HYBRID_PREBOOST1_LEAD_MS = 0;
    public static long HYBRID_PREBOOST2_LEAD_MS = 20;
    public static long HYBRID_PREBOOST3_LEAD_MS = 30;
    public static double HYBRID_PREBOOST1_AMOUNT = 0.0;
    public static double HYBRID_PREBOOST2_AMOUNT = 0.05;
    public static double HYBRID_PREBOOST3_AMOUNT = 0.08;
    public static long HYBRID_BALL1_CONTACT_WINDOW_HALF_WIDTH_MS = 60;
    public static long HYBRID_BALL2_CONTACT_WINDOW_HALF_WIDTH_MS = 45;
    public static long HYBRID_BALL3_CONTACT_WINDOW_HALF_WIDTH_MS = 80;
    public static long HYBRID_MIN_EDGE_GAP_MS = 25;
    public static long HYBRID_TIMER_FALLBACK_EXTRA_MS = 0;
    public static boolean HYBRID_USE_BB1_FALL_FOR_BALL2 = true;
    public static boolean HYBRID_USE_BB1_FALL_FOR_BALL3 = true;
    public static boolean HYBRID_USE_BB2_FALL_FOR_BALL3 = true;
    // Shot tuning mode: same driving/aiming flow, but manual shooter controls and KEEP/IGNORE labels.
    public static boolean SHOT_TUNING_MODE = false;
    public static boolean ENABLE_SHOT_TUNING_LOGGING = false;
    public static double SHOT_TUNING_RPM_STEP = 25.0;
    public static double SHOT_TUNING_HOOD_STEP = 0.005;
    public static double SHOT_TUNING_TARGET_STEP_IN = 1.0;
    public static boolean SHOT_TUNING_REQUIRE_AT_SPEED_TO_FIRE = true;
    public static double SHOT_TUNING_AT_SPEED_TOLERANCE_RPM = 75.0;
    public static double SHOT_TUNING_TARGET_X_IN = 144.0;
    public static double SHOT_TUNING_TARGET_Y_IN = 136.0;
    public static boolean ENABLE_SHOT_INFO_LOGGING = false;
    public static long SHOT_INFO_LOG_TIMEOUT_MS = 2500;
    // Dumbshoot-specific logging window based on observed burst timing:
    // shot1 < 200 ms, shot2 ~150 ms later, shot3 usually ~200 ms later.
    // Keep a small margin for loop/sensor jitter.
    public static long SHOT_INFO_DUMBSHOOT_TIMEOUT_MS = 900;
    // Hold logging at least this long for dumbshoot to avoid early-finalize truncation.
    public static long SHOT_INFO_DUMBSHOOT_MIN_CAPTURE_MS = 750;
    // If shot2 was seen but shot3 is missing, wait this long after shot2 before finalizing.
    public static long SHOT_INFO_DUMBSHOOT_AFTER_SECOND_TIMEOUT_MS = 325;
    // After seeing the final expected shot edge, wait a short grace period before finalize.
    public static long SHOT_INFO_EXPECTED_SHOTS_FINALIZE_GRACE_MS = 90;
    private long logStartMs = 0L;
    private int shotSequenceIdCounter = 0;
    private boolean shotLogSequenceActive = false;
    private long shotSequenceStartMs = 0L;
    private int shotSequenceId = 0;
    private int shotSequenceStartBallCount = 0;
    private int shotSequenceExpectedShots = 0;
    private String shotSequenceReason = "";
    private int shotSequenceBurstProfileId = 0;
    private double shotSequenceStartTargetRpm = 0.0;
    private double shotSequenceStartHoodPos = 0.0;
    private long shotSequenceStartBoostDelayMs = 0L;
    private double shotSequenceStartPreBoostAmount = 0.0;
    private double shotSequenceStartBoostMult1 = 0.0;
    private double shotSequenceStartBoostMult2 = 0.0;
    private int shotSequenceLinkedDumbShootRpmSequenceId = -1;
    private boolean shotSequenceStartBb0 = false;
    private boolean shotSequenceStartBb1 = false;
    private boolean shotSequenceStartBb2 = false;
    private int shotBb0FallCount = 0;
    private int shotBb0RiseCount = 0;
    private int shotBb1FallCount = 0;
    private int shotBb1RiseCount = 0;
    private int shotBb2FallCount = 0;
    private int shotBb2RiseCount = 0;
    private final long[] shotBb0FallMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb0RiseMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb1FallMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb1RiseMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb2FallMs = new long[] {-1L, -1L, -1L};
    private final long[] shotBb2RiseMs = new long[] {-1L, -1L, -1L};
    private final long[] shotIntervalMs = new long[] {-1L, -1L, -1L};
    private int shotBb2ClearLoopCountTotal = 0;
    private int shotBb2ClearStreakCurrent = 0;
    private int shotBb2ClearStreakMax = 0;
    private boolean prevBb0ForShotLog = false;
    private boolean prevBb1ForShotLog = false;
    private boolean prevBb2ForShotLog = false;
    private boolean bbPrevInitializedForShotLog = false;
    private boolean dumbShootRpmLogActive = false;
    private long dumbShootRpmLogStartMs = 0L;
    private int dumbShootRpmLogSequenceId = 0;
    private int dumbShootRpmSequenceCounter = 0;
    private int dumbShootRpmLogExpectedShots = 0;
    private int dumbShootRpmLinkedShotInfoSequenceId = -1;
    private int dumbShootRpmBurstProfileId = 0;
    private int dumbShootRpmBb0RiseCount = 0;
    private int dumbShootRpmBb0FallCount = 0;
    private int dumbShootRpmBb1RiseCount = 0;
    private int dumbShootRpmBb1FallCount = 0;
    private int dumbShootRpmBb2RiseCount = 0;
    private int dumbShootRpmBb2FallCount = 0;
    private boolean dumbShootRpmPrevBbInitialized = false;
    private boolean dumbShootRpmPrevBb0 = false;
    private boolean dumbShootRpmPrevBb1 = false;
    private boolean dumbShootRpmPrevBb2 = false;
    private boolean shotTuningFlywheelEnabled = false;
    private int shotTuningShotIdCounter = 0;
    private boolean shotTuningPendingLabel = false;
    private int shotTuningPendingShotId = 0;
    private long shotTuningPendingFireMs = 0L;
    private int shotTuningPendingStartBallCount = 0;
    private double shotTuningPendingTargetRpm = 0.0;
    private double shotTuningPendingHoodPos = 0.0;
    private double shotTuningPendingRpm1 = 0.0;
    private double shotTuningPendingRpm2 = 0.0;
    private double shotTuningPendingBotX = 0.0;
    private double shotTuningPendingBotY = 0.0;
    private double shotTuningPendingBotHeadingDeg = 0.0;
    private double shotTuningPendingTargetX = 0.0;
    private double shotTuningPendingTargetY = 0.0;
    private double shotTuningPendingOdoDistance = 0.0;
    private double shotTuningPendingTurretTargetDeg = 0.0;
    private double shotTuningPendingTurretMeasuredDeg = 0.0;
    private boolean prevTuningTargetDpadUp = false;
    private boolean prevTuningTargetDpadDown = false;
    private boolean prevTuningTargetDpadLeft = false;
    private boolean prevTuningTargetDpadRight = false;
    private boolean prevTuningRpmDpadUp = false;
    private boolean prevTuningRpmDpadDown = false;
    private String shotGateLedState = "NONE";
    private double prevTurretLogTargetDeg = Double.NaN;
    private double prevTurretLogMeasuredDeg = Double.NaN;
    private double prevTurretLogServoCommandDeg = Double.NaN;
    private double prevTurretLogServoPos = Double.NaN;
    private long prevTurretLogMs = 0L;
    private long turretStictionCandidateStartMs = 0L;

    //public static Pose redShootingTarget = new Pose(127.63, 130.35, Math.toRadians(36));
    public static Pose redShootingTarget = new Pose(144, 136, Math.toRadians(36));
    public static Pose blueShootingTarget = redShootingTarget.mirror();
    Pose MT1PedroPose = new Pose();

    // Adjust these from Panels at runtime
    public static boolean hold = false;
    public static boolean prevHold = false;
    public static boolean SHOW_SMOOTHED = false;
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

    private Pose shootingTargetLocation;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    boolean goToTargetAngle;
    double targetAngleDeg = -135.0;
    double targetAngleRad;
    double propAngleGain = -0.5;
    public static double shooterTargetkP = 0.0165;
    double minAnglePower = 0.075;
    double maxRotate = 0.8;
    double angleAllianceOffset = 0.0;
    double drivePower = 1.0;
    public static double normDrivePower = 1;
    public static double slowedDrivePower = 0.5;
    private Limelight3A limelight;
    private double cameraHeightFromTags = 18.25;

    private double xOffset;
    private double yOffset;
    private double distanceLL;
    private double ODODistance;
    private double yDesired = 17;

    private double areaOffset;
    public static boolean testShooter = false;
    public static double targetRPM = 0;
    public static double targetInitialRPM = 0;
    public static double minDisatanceForShooting = 42.0;

    // SOTM Stage 3 (turret + distance-based time of flight)
    public static boolean SOTM_ENABLED = true;
    public static double SOTM_MIN_SPEED_IN_PER_SEC = 2.0;
    public static double SOTM_LEAD_GAIN = 1.0;
    public static double SOTM_MAX_LEAD_IN = 15.0;
    public static double SOTM_ANGULAR_LEAD_GAIN = 0.25;
    public static double SOTM_OMEGA_FILTER_ALPHA = 0.2;
    // Separate turret-lag feedforward layer (independent from SOTM lead-point math).
    // Equivalent concept to: turretTarget += angularVelocity * kVF.
    public static boolean SOTM_TURRET_LAG_COMP_ENABLED = true;
    public static double SOTM_TURRET_LAG_COMP_SEC = 0.10;
    public static double SOTM_TURRET_LAG_COMP_MAX_DEG = 12.0;
    // When true, keep SOTM solution (turret + ballistic distance) live all the time.
    public static boolean SOTM_ALWAYS_TRACK_TARGETS = true;
    // Legacy toggle (intentionally unused now): flywheel enable is controlled only by
    // shooterFollowEnabled (left-bumper latch or 3-ball auto-enable).
    public static boolean SOTM_ALWAYS_PRIME_SHOOTER = false;
    public static double SOTM_FIRE_AIM_TOLERANCE_DEG = 4.0;
    public static double SOTM_FIRE_MAX_TURRET_SPEED_DEG_PER_SEC = 220.0;
    public static boolean SOTM_REQUIRE_TURRET_READY_FOR_FIRE = false;
    public static double SOTM_REACHABILITY_TOLERANCE_DEG = 1.0;
    public static boolean SOTM_USE_TOF_LOOKUP = true;
    public static double SOTM_MECHANICAL_DELAY_SEC = 0.15;
    public static double SOTM_BALL_TRANSFER_TIME_SEC = 0.0;
    public static double SOTM_ESTIMATED_BALL_SPEED_IN_PER_SEC = 300.0;
    private static final double[] SOTM_TOF_DISTANCE_IN = {60.0, 80.0, 100.0, 120.0};
    private static final double[] SOTM_TOF_FLIGHT_TIME_SEC = {0.15, 0.20, 0.27, 0.35};

    // Auto-stop shooter timeout after starting a dumbShoot burst (ms)
    public static long DUMBSHOOT_SHOOTER_TIMEOUT_MS = 750;

    public static double shooterHoodPos = 0;
    private boolean hasResults = false;
    private boolean selectAllianceSide = false;

    private boolean shoot = false;

    private boolean adjustLimelight = false;
    private boolean adjustOdo = true;
    // Edge-detect left trigger so we only start one single-ball feed per press
    private boolean prevLeftTriggerActive = false;
    // Latches a single-shot request until shooter RPM is in range
    private boolean singleShotPending = false;
    // State for auto-stopping shooter after dumbShoot
    private boolean dumbShootTimerActive = false;
    private long dumbShootStartTimeMs = 0L;
    // Track if we're already showing the "too close" warning strobe
    private boolean tooCloseWarningActive = false;
    // Latches flywheel tracking mode so RPM keeps following location without holding a button.
    private boolean shooterFollowEnabled = false;
    private boolean prevGamepad1LeftBumper = false;
    private double sotmFilteredOmegaRadPerSec = 0.0;
    private boolean sotmOmegaFilterInitialized = false;
    // True only after Start is pressed; prevents flywheel spin-up during Init.
    private boolean matchHasStarted = false;
    // True when this TeleOp immediately follows an auton run in the same match.
    private boolean teleopStartedFromAuton = false;
    private boolean hybridPrevBbInitialized = false;
    private boolean hybridPrevBb1 = false;
    private boolean hybridPrevBb2 = false;
    private boolean hybridShotFeedBoostActive = false;
    private boolean hybridShotFeedUsesFarProfile = false;
    private long hybridShotFeedStartMs = 0L;
    private long hybridBall1ContactStartMsEst = -1L;
    private long hybridBall2ContactStartMsEst = -1L;
    private long hybridBall3ContactStartMsEst = -1L;
    private long hybridLastAcceptedEdgeMs = -1L;
    private long hybridLastAdvanceAtRelMs = -1L;
    private String hybridLastAdvanceReason = "NONE";

    private enum HybridShotFeedBoostPhase {
        IDLE,
        WAIT_BALL1_CONTACT,
        WAIT_BALL2_CONTACT,
        WAIT_BALL3_CONTACT,
        COMPLETE
    }

    private HybridShotFeedBoostPhase hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.IDLE;


    @Override
    public void onInit() {
        matchHasStarted = false;
        targetRPM = 0.0;
        ShooterSubsystem.INSTANCE.stop();
        shooterFollowEnabled = false;
        dumbShootTimerActive = false;
        singleShotPending = false;
        shotTuningFlywheelEnabled = false;
        shotTuningPendingLabel = false;
        shotTuningShotIdCounter = 0;
        shooterHoodPos = 0.0;
        prevTuningTargetDpadUp = false;
        prevTuningTargetDpadDown = false;
        prevTuningTargetDpadLeft = false;
        prevTuningTargetDpadRight = false;
        prevTuningRpmDpadUp = false;
        prevTuningRpmDpadDown = false;
        shotGateLedState = "NONE";
        hold = false;
        prevHold = false;
        resetHybridShotFeedBoostController();
        hybridPrevBbInitialized = false;

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        teleopStartedFromAuton = (GlobalRobotData.endAutonPose != null) && (GlobalRobotData.hasAutonRun);
        if (teleopStartedFromAuton) {
            startingPose = GlobalRobotData.endAutonPose;
            GlobalRobotData.hasAutonRun = false;
        } else {
            selectAllianceSide = true;
        }

        initializeTurretStartupReference(teleopStartedFromAuton);

        //PedroComponent.follower().setStartingPose(startingPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        windowShooter1 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowShooter2 = new double[Math.max(1, SMOOTH_WINDOW)];
        windowOuttake = new double[Math.max(1, SMOOTH_WINDOW)];

        LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
        TurretSubsystem.INSTANCE.setPeriodicAbsoluteEncoderReadEnabled(ENABLE_TURRET_LOGGING);

        // Carry over ball count from auton if available
        if (GlobalRobotData.endAutonBallCount >= 0) {
            IntakeWithSensorsSubsystem.INSTANCE.setBallCount(GlobalRobotData.endAutonBallCount);
        }

        /* pathChain = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(45), 0.8))
                .build(); */


            turretLogger = new CsvLogger("pickles2025_turret");
            turretLogger.start(
                    "t_ms," +
                            "match_t_ms," +
                            "bot_x," +
                            "bot_y," +
                            "bot_heading_deg," +
                            "shoot_target_x," +
                            "shoot_target_y," +
                            "field_angle_deg," +
                            "angle_error_deg," +
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
                            "turret_log_dt_ms," +
                            "turret_target_delta_deg," +
                            "turret_measured_delta_deg," +
                            "turret_servo_command_delta_deg," +
                            "turret_servo_pos_delta," +
                            "turret_trim_as_servo_delta_pos," +
                            "turret_cmd_above_deadband_guess," +
                            "turret_target_stable," +
                            "turret_limit_clipped," +
                            "turret_stiction_suspect," +
                            "turret_ready," +
                            "driving_cmd," +
                            "strafe_cmd," +
                            "rotate_cmd," +
                            "ododistance," +
                            "x_offset," +
                            "y_offset," +
                            "has_results," +
                            "loop_time_ms"
            );

            if (ENABLE_SOTM_LOGGING) {
                sotmLogger = new CsvLogger("pickles2025_sotm");
                sotmLogger.start(
                        "t_ms," +
                                "match_t_ms," +
                                "sotm_fire_request_active," +
                                "sotm_control_active," +
                                "right_bumper_active," +
                                "right_trigger_active," +
                                "bot_x," +
                                "bot_y," +
                                "bot_heading_deg," +
                                "target_x," +
                                "target_y," +
                                "field_angle_deg," +
                                "angle_error_deg," +
                                "real_distance_in," +
                                "sotm_valid," +
                                "sotm_lead_applied," +
                                "sotm_speed_in_s," +
                                "sotm_vx_in_s," +
                                "sotm_vy_in_s," +
                                "sotm_omega_raw_deg_s," +
                                "sotm_omega_deg_s," +
                                "sotm_total_tof_s," +
                                "sotm_lead_x_in," +
                                "sotm_lead_y_in," +
                                "sotm_radial_vel_in_s," +
                                "sotm_effective_dist_in," +
                                "sotm_ballistic_dist_in," +
                                "sotm_turret_aim_deg," +
                                "sotm_turret_lag_comp_deg," +
                                "turret_target_deg," +
                                "turret_measured_deg," +
                                "turret_ready," +
                                "sotm_turret_gate," +
                                "sotm_turret_goal_error_deg," +
                                "sotm_turret_constraint_error_deg," +
                                "sotm_turret_reachable," +
                                "sotm_turret_speed_deg_s," +
                                "sotm_turret_speed_gate," +
                                "sotm_fire_gate," +
                                "sotm_can_shoot_gate," +
                                "shooter_target_rpm," +
                                "shooter_rpm1," +
                                "shooter_rpm2," +
                                "shooter_at_speed_75," +
                                "shooter_battery_v," +
                                "shooter_battery_v_filtered," +
                                "shooter_voltage_comp_gain," +
                                "shooter_cmd_pre_vcomp," +
                                "shooter_cmd_post_vcomp," +
                                "shooter_cmd_saturated," +
                                "dumbshoot_timer_active," +
                                "hold_state," +
                                "drive_cmd," +
                                "strafe_cmd," +
                                "rotate_cmd," +
                                "loop_time_ms"
                );
            } else {
                sotmLogger = null;
            }

            logStartMs = 0L;
            lastTurretLogMs = 0L;
            lastSotmLogMs = 0L;
            lastDumbShootRpmLogMs = 0L;
            prevTurretLogTargetDeg = Double.NaN;
            prevTurretLogMeasuredDeg = Double.NaN;
            prevTurretLogServoCommandDeg = Double.NaN;
            prevTurretLogServoPos = Double.NaN;
            prevTurretLogMs = 0L;
            turretStictionCandidateStartMs = 0L;
            dumbShootRpmLogActive = false;
            dumbShootRpmLogStartMs = 0L;
            dumbShootRpmLogSequenceId = 0;
            dumbShootRpmSequenceCounter = 0;
            dumbShootRpmLogExpectedShots = 0;
            dumbShootRpmLinkedShotInfoSequenceId = -1;
            dumbShootRpmBurstProfileId = 0;
            dumbShootRpmBb0RiseCount = 0;
            dumbShootRpmBb0FallCount = 0;
            dumbShootRpmBb1RiseCount = 0;
            dumbShootRpmBb1FallCount = 0;
            dumbShootRpmBb2RiseCount = 0;
            dumbShootRpmBb2FallCount = 0;
            dumbShootRpmPrevBbInitialized = false;
            shotTuningPendingLabel = false;
            shotTuningShotIdCounter = 0;

            if (ENABLE_SHOT_INFO_LOGGING) {
                shotInfoLogger = new CsvLogger("pickles2025_shot_breakbeam");
                shotInfoLogger.start(
                        "t_ms," +
                                "match_t_ms," +
                                "sequence_id," +
                                "trigger_reason," +
                                "start_ball_count," +
                                "expected_shots," +
                                "start_bb0," +
                                "start_bb1," +
                                "start_bb2," +
                                "bb0_fall_1_ms," +
                                "bb0_rise_1_ms," +
                                "bb0_fall_2_ms," +
                                "bb0_rise_2_ms," +
                                "bb0_fall_3_ms," +
                                "bb0_rise_3_ms," +
                                "bb1_fall_1_ms," +
                                "bb1_rise_1_ms," +
                                "bb1_fall_2_ms," +
                                "bb1_rise_2_ms," +
                                "bb1_fall_3_ms," +
                                "bb1_rise_3_ms," +
                                "bb2_fall_1_ms," +
                                "bb2_rise_1_ms," +
                                "bb2_fall_2_ms," +
                                "bb2_rise_2_ms," +
                                "bb2_fall_3_ms," +
                                "bb2_rise_3_ms," +
                                "shot1_interval_ms," +
                                "shot2_interval_ms," +
                                "shot3_interval_ms," +
                                "bb2_clear_gap_1to2_ms," +
                                "bb2_clear_gap_2to3_ms," +
                                "bb2_clear_loops_total," +
                                "bb2_clear_streak_max_loops," +
                                "bb0_fall_count," +
                                "bb0_rise_count," +
                                "bb1_fall_count," +
                                "bb1_rise_count," +
                                "bb2_fall_count," +
                                "bb2_rise_count," +
                                "end_reason," +
                                "duration_ms," +
                                "burst_profile_id," +
                                "start_target_rpm," +
                                "start_hood_pos," +
                                "start_boost_delay_ms," +
                                "start_pre_boost_amount," +
                                "start_boost_mult1," +
                                "start_boost_mult2," +
                                "linked_dumbshoot_rpm_sequence_id"
                );
            } else {
                shotInfoLogger = null;
            }

            if (ENABLE_DUMBSHOOT_RPM_LOGGING) {
                dumbShootRpmLogger = new CsvLogger("pickles2025_dumbshoot_rpm");
                dumbShootRpmLogger.start(
                        "t_ms," +
                                "match_t_ms," +
                                "sequence_id," +
                                "shot_info_sequence_id," +
                                "burst_profile_id," +
                                "t_since_start_ms," +
                                "expected_shots," +
                                "dumbshoot_timer_active," +
                                "shooter_target_rpm," +
                                "shooter_rpm1," +
                                "shooter_rpm2," +
                                "shooter_avg_rpm," +
                                "shooter_rpm_delta_avg," +
                                "shooter_power1," +
                                "shooter_power2," +
                                "shooter_at_speed_75," +
                                "shooter_battery_v," +
                                "shooter_battery_v_filtered," +
                                "shooter_voltage_comp_gain," +
                                "shooter_cmd_pre_vcomp," +
                                "shooter_cmd_post_vcomp," +
                                "shooter_cmd_saturated," +
                                "boost_active," +
                                "second_boost_active," +
                                "boost_override," +
                                "shooter_boost_mult1," +
                                "shooter_boost_mult2," +
                                "hybrid_feed_boost_active," +
                                "hybrid_phase," +
                                "hybrid_t_since_shot_feed_start_ms," +
                                "hybrid_expected_contact_ms," +
                                "hybrid_preboost_amount_active," +
                                "hybrid_last_advance_reason," +
                                "hybrid_last_advance_rel_ms," +
                                "bb0," +
                                "bb1," +
                                "bb2," +
                                "bb0_rise_edge," +
                                "bb0_fall_edge," +
                                "bb1_rise_edge," +
                                "bb1_fall_edge," +
                                "bb2_rise_edge," +
                                "bb2_fall_edge," +
                                "bb0_rise_count_total," +
                                "bb0_fall_count_total," +
                                "bb1_rise_count_total," +
                                "bb1_fall_count_total," +
                                "bb2_rise_count_total," +
                                "bb2_fall_count_total," +
                                "ball_count," +
                                "loop_time_ms"
                );
            } else {
                dumbShootRpmLogger = null;
            }

            if (ENABLE_SHOT_TUNING_LOGGING) {
                shotTuningLogger = new CsvLogger("pickles2025_shot_tuning");
                shotTuningLogger.start(
                        "t_ms," +
                                "match_t_ms," +
                                "shot_id," +
                                "label," +
                                "label_reason," +
                                "fire_t_ms," +
                                "t_since_fire_ms," +
                                "start_ball_count," +
                                "target_x," +
                                "target_y," +
                                "bot_x_fire," +
                                "bot_y_fire," +
                                "bot_heading_deg_fire," +
                                "ododistance_fire," +
                                "target_rpm_fire," +
                                "hood_pos_fire," +
                                "rpm1_fire," +
                                "rpm2_fire," +
                                "target_rpm_label," +
                                "hood_pos_label," +
                                "rpm1_label," +
                                "rpm2_label," +
                                "turret_target_deg_fire," +
                                "turret_measured_deg_fire," +
                                "turret_target_deg_label," +
                                "turret_measured_deg_label," +
                                "boost_active_label," +
                                "second_boost_active_label," +
                                "boost_override_label"
                );
            } else {
                shotTuningLogger = null;
            }

    }

    private void initializeTurretStartupReference(boolean fromAutonTransition) {
        double expectedTurretAngleDegrees = TurretSubsystem.STARTUP_EXPECTED_TURRET_ANGLE_DEGREES;
        if (fromAutonTransition && Double.isFinite(GlobalRobotData.endAutonTurretAngleDegrees)) {
            expectedTurretAngleDegrees = GlobalRobotData.endAutonTurretAngleDegrees;
        }

        if (!fromAutonTransition) {
            // Practice TeleOp path: allow turret move during init to a known pose.
            TurretSubsystem.INSTANCE.moveServosToStartupZeroPosition();
        }

        TurretSubsystem.INSTANCE.waitForStartupServoSettle();
        double learnedTurretAngleDegrees =
                TurretSubsystem.INSTANCE.learnAbsoluteTurretAngleFromExpected(expectedTurretAngleDegrees);
        TurretSubsystem.INSTANCE.setQuadratureOffsetFromKnownTurretAngle(learnedTurretAngleDegrees);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {

        if (selectAllianceSide) {
            if (gamepad1.xWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.BLUE;
                startingPose = startingPoseBlue;
            } else if (gamepad1.bWasPressed()) {
                GlobalRobotData.allianceSide = GlobalRobotData.COLOR.RED;
                startingPose = startingPoseRed;
            }

            telemetry.addLine("Hello Pickle of the robot");
            telemetry.addLine("This is an Mr. Todone Speaking,");
            telemetry.addLine("----------------------------------------------");
            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                telemetry.addLine("Favorite fruit: Blueberries!!! (Blue)");
            } else {
                telemetry.addLine("Favorite fruit: Raspberries!!! (Red)");
            }

            telemetry.update();

        }
    }

    @Override
    public void onStartButtonPressed() {
        matchHasStarted = true;
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        PedroComponent.follower().setStartingPose(startingPose);

        PedroComponent.follower().startTeleopDrive();

        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED) {
            shootingTargetLocation = redShootingTarget;
        } else {
            shootingTargetLocation = blueShootingTarget;
        }
        SHOT_TUNING_TARGET_X_IN = shootingTargetLocation.getX();
        SHOT_TUNING_TARGET_Y_IN = shootingTargetLocation.getY();

        TurretSubsystem.INSTANCE.center();
        sotmOmegaFilterInitialized = false;
        sotmFilteredOmegaRadPerSec = 0.0;
        prevTurretLogTargetDeg = Double.NaN;
        prevTurretLogMeasuredDeg = Double.NaN;
        prevTurretLogServoCommandDeg = Double.NaN;
        prevTurretLogServoPos = Double.NaN;
        prevTurretLogMs = 0L;
        turretStictionCandidateStartMs = 0L;
        dumbShootRpmLogActive = false;
        dumbShootRpmLogStartMs = 0L;
        dumbShootRpmLogSequenceId = 0;
        dumbShootRpmSequenceCounter = 0;
        dumbShootRpmLogExpectedShots = 0;
        dumbShootRpmLinkedShotInfoSequenceId = -1;
        dumbShootRpmBurstProfileId = 0;
        dumbShootRpmBb0RiseCount = 0;
        dumbShootRpmBb0FallCount = 0;
        dumbShootRpmBb1RiseCount = 0;
        dumbShootRpmBb1FallCount = 0;
        dumbShootRpmBb2RiseCount = 0;
        dumbShootRpmBb2FallCount = 0;
        dumbShootRpmPrevBbInitialized = false;
        lastDumbShootRpmLogMs = 0L;
        shotTuningFlywheelEnabled = false;
        shotTuningPendingLabel = false;
        prevTuningTargetDpadUp = false;
        prevTuningTargetDpadDown = false;
        prevTuningTargetDpadLeft = false;
        prevTuningTargetDpadRight = false;
        prevTuningRpmDpadUp = false;
        prevTuningRpmDpadDown = false;
        shotGateLedState = "NONE";
        shooterHoodPos = ShooterSubsystem.INSTANCE.getShooterHoodPosition();

        logStartMs = System.currentTimeMillis();
    }

    @Override
    public void onUpdate() {
        //Call this once per loop
        timer.start();
        long nowMs = System.currentTimeMillis();
        TurretSubsystem.INSTANCE.setPeriodicAbsoluteEncoderReadEnabled(ENABLE_TURRET_LOGGING);

        // Auto-stop shooter and intake after a dumbShoot burst has been active long enough
        if (dumbShootTimerActive &&
                nowMs - dumbShootStartTimeMs >= DUMBSHOOT_SHOOTER_TIMEOUT_MS) {
            ShooterSubsystem.INSTANCE.stop();
            IntakeWithSensorsSubsystem.INSTANCE.stop();
            IntakeWithSensorsSubsystem.INSTANCE.validateBallCountAfterShoot();
            IntakeWithSensorsSubsystem.INSTANCE.intakeForward();
            dumbShootTimerActive = false;
            shooterFollowEnabled = false;
            resetHybridShotFeedBoostController();
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            xOffset = result.getTx();
            yOffset = result.getTy();
            areaOffset = result.getTa();
            distanceLL = cameraHeightFromTags / (Math.tan(Math.toRadians(yOffset)));
        }
        ODODistance = PedroComponent.follower().getPose().distanceFrom(shootingTargetLocation);

        double driving = (-gamepad1.left_stick_y) * drivePower;
        double strafe = (-gamepad1.left_stick_x) * drivePower;
        double rotate = (-gamepad1.right_stick_x) * 0.5;
        boolean rightTriggerActive = gamepad1.right_trigger > 0.1;
        boolean rightBumperActive = gamepad1.right_bumper;
        boolean dpadUpActive = !SHOT_TUNING_MODE && gamepad1.dpad_up;
        boolean sotmFireRequestActive = !SHOT_TUNING_MODE && (rightTriggerActive || rightBumperActive);
        boolean sotmControlActive = SOTM_ENABLED && (SOTM_ALWAYS_TRACK_TARGETS || sotmFireRequestActive);

        if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
            driving *= -1;
            strafe *= -1;
        }
        Pose currentBotPose = PedroComponent.follower().getPose();
        double botHeadingRad = currentBotPose.getHeading();
        double botxvalue = currentBotPose.getX(); //gettingxvalue :D
        double botyvalue = currentBotPose.getY(); //gettingyvalue :D

        Pose ftcCoordPose   = currentBotPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
        limelight.updateRobotOrientation(Math.toDegrees(ftcCoordPose.getHeading()));

        if (SHOT_TUNING_MODE) {
            boolean tuningTargetUp = gamepad1.dpad_up;
            boolean tuningTargetDown = gamepad1.dpad_down;
            boolean tuningTargetLeft = gamepad1.dpad_left;
            boolean tuningTargetRight = gamepad1.dpad_right;
            double targetStep = Math.max(0.0, SHOT_TUNING_TARGET_STEP_IN);
            if (tuningTargetUp && !prevTuningTargetDpadUp) {
                SHOT_TUNING_TARGET_Y_IN += targetStep;
            }
            if (tuningTargetDown && !prevTuningTargetDpadDown) {
                SHOT_TUNING_TARGET_Y_IN -= targetStep;
            }
            if (tuningTargetRight && !prevTuningTargetDpadRight) {
                SHOT_TUNING_TARGET_X_IN += targetStep;
            }
            if (tuningTargetLeft && !prevTuningTargetDpadLeft) {
                SHOT_TUNING_TARGET_X_IN -= targetStep;
            }
            prevTuningTargetDpadUp = tuningTargetUp;
            prevTuningTargetDpadDown = tuningTargetDown;
            prevTuningTargetDpadLeft = tuningTargetLeft;
            prevTuningTargetDpadRight = tuningTargetRight;
        }

        double shootTargetX = SHOT_TUNING_MODE
                ? SHOT_TUNING_TARGET_X_IN
                : shootingTargetLocation.getX();
        double shootTargetY = SHOT_TUNING_MODE
                ? SHOT_TUNING_TARGET_Y_IN
                : (botyvalue > 106 ? shootingTargetLocation.getY() - 1 : shootingTargetLocation.getY() + 3);

        // Vector from robot -> target
        double dx = shootTargetX - botxvalue;
        double dy = shootTargetY - botyvalue;

        // 1) Field-relative angle to the target (0 rad along +X, CCW positive)
        double fieldAngleRad = Math.atan2(dy, dx);
        // - fieldAngleDeg  -> "absolute field heading I want the robot to face"
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);

        // 2) Robot-relative angle error (how much you need to turn or point the turret)
        double angleErrorRad = normalizeRadians(fieldAngleRad - botHeadingRad);
        // - angleErrorDeg  -> "how many degrees left/right from current heading"
        double angleErrorDeg = Math.toDegrees(angleErrorRad);

        Vector robotVelocity = PedroComponent.follower().getVelocity();
        double sotmVx = robotVelocity.getXComponent();
        double sotmVy = robotVelocity.getYComponent();
        double sotmSpeed = robotVelocity.getMagnitude();
        double sotmOmegaRawRadPerSec = PedroComponent.follower().poseTracker.getAngularVelocity();
        double sotmOmegaRadPerSec = updateSotmFilteredOmega(sotmOmegaRawRadPerSec);

        SOTMResult sotmResult = calculateSotmResult(
                botxvalue,
                botyvalue,
                botHeadingRad,
                shootTargetX,
                shootTargetY,
                sotmVx,
                sotmVy,
                sotmSpeed,
                sotmOmegaRadPerSec,
                sotmControlActive
        );

        // During official-match init after auton, keep turret fixed until Start is pressed.
        if (matchHasStarted || !teleopStartedFromAuton) {
            // Always track with turret. When SOTM is active and moving, this target is lead-compensated.
            TurretSubsystem.INSTANCE.setTargetAngleFromRobotFrontRelativeDegrees(sotmResult.turretRobotRelativeAimDeg);
        }
        double shooterDistanceForBallistics = sotmResult.distanceForBallisticsInches;
        double turretMeasuredDeg = TurretSubsystem.INSTANCE.getMeasuredAngleDegrees();
        double turretMeasuredVelDegPerSec = TurretSubsystem.INSTANCE.getMeasuredVelocityDegPerSec();
        double turretCommandedTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();
        double desiredTurretAimUnwrappedDeg =
                robotFrontRelativeToTurretUnwrappedDegrees(sotmResult.turretRobotRelativeAimDeg);
        double turretGoalErrorDeg =
                smallestWrappedDeltaDegrees(desiredTurretAimUnwrappedDeg, turretMeasuredDeg);
        double turretConstraintErrorDeg =
                smallestWrappedDeltaDegrees(desiredTurretAimUnwrappedDeg, turretCommandedTargetDeg);
        boolean turretTargetReachable =
                Math.abs(turretConstraintErrorDeg) <= Math.max(0.0, SOTM_REACHABILITY_TOLERANCE_DEG);
        boolean turretAimAtGoal =
                Math.abs(turretGoalErrorDeg) <= Math.max(0.0, SOTM_FIRE_AIM_TOLERANCE_DEG);
        boolean turretSpeedGateSatisfied =
                Math.abs(turretMeasuredVelDegPerSec) <= Math.max(0.0, SOTM_FIRE_MAX_TURRET_SPEED_DEG_PER_SEC);
        boolean turretReadyGateSatisfied =
                !SOTM_REQUIRE_TURRET_READY_FOR_FIRE || TurretSubsystem.INSTANCE.isTurretReady();
        boolean turretAimGateSatisfied =
                turretTargetReachable &&
                turretAimAtGoal &&
                turretSpeedGateSatisfied &&
                turretReadyGateSatisfied;
        boolean sotmFireGateSatisfied = !sotmControlActive || sotmResult.valid;
        boolean canShootAtGoal = turretAimGateSatisfied && sotmFireGateSatisfied;


        double angletangent = 0;
        double shootingangle = 0;
        // Add location based shooting angle here eventually
        //double shootingangle = Math.toDegrees(Math.atan2(144-botyvalue,botxvalue)
        if (gamepad1.y) {
            PedroComponent.follower().setPose(new Pose(71, 8, Math.toRadians(270)));
        }

        // Legacy limelight heading-assist block.
        // Keep it only when SOTM is disabled, otherwise it fights moving-shot turret targeting.
        if (dpadUpActive && !SOTM_ENABLED) {
            adjustLimelight = true;
            if (result != null && result.isValid()) {
                if (ODODistance < 100) {
                    Pose3D MT1Pose = result.getBotpose();
                    double MT1xInches = DistanceUnit.METER.toInches(MT1Pose.getPosition().x);
                    double MT1yInches = DistanceUnit.METER.toInches(MT1Pose.getPosition().y);
                    Pose2D MT1Pose2d = new Pose2D(DistanceUnit.INCH, MT1xInches, MT1yInches, AngleUnit.DEGREES, MT1Pose.getOrientation().getYaw());
                    Pose MT1FTCStandardPose = PoseConverter.pose2DToPose(MT1Pose2d, InvertedFTCCoordinates.INSTANCE);
                    MT1PedroPose = new Pose((MT1FTCStandardPose.getY() + 72), (-(MT1FTCStandardPose.getX()) +72), MT1FTCStandardPose.getHeading() - Math.toRadians(90));

                    PedroComponent.follower().setPose(MT1PedroPose);
                }
                targetRPM = calculateShooterRPMOdoDistance(this.ODODistance);

                if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.RED) {
                    List<LLResultTypes.FiducialResult> tag24Results = result.getFiducialResults().stream()
                            .filter(r -> r.getFiducialId() == 24)
                            .collect(Collectors.toList());

                    if (!tag24Results.isEmpty()) {
                        hasResults = true;
                        double targetX = tag24Results.get(0).getTargetXDegrees();
                        if (targetX != 0.5 && tag24Results.get(0).getTargetYDegrees() > 10) {
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                        else if(targetX != 0.5){
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                    } else {
                        hasResults = false;
                    }
                } else {
                    List<LLResultTypes.FiducialResult> tag20Results = result.getFiducialResults().stream()
                            .filter(r -> r.getFiducialId() == 20)
                            .collect(Collectors.toList());

                    if (!tag20Results.isEmpty()) {
                        hasResults = true;
                        double targetX = tag20Results.get(0).getTargetXDegrees();
                        if (targetX != -0.5 && tag20Results.get(0).getTargetYDegrees() > 10) {
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                        else if(targetX !=  -0.5) {
                            rotate = (-targetX) * (shooterTargetkP + 0.01);
                            goToTargetAngle = false;
                        }
                    } else {
                        hasResults = false;
                    }
                }

                if (hasResults) {
                    LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.WHITE);
//                    if (shoot) {
//                        if (hasResults && yOffset > 9.) {
//                            ShooterSubsystem.INSTANCE.setClosePID();
//                            targetRPM = calculateShooterRPM(yOffset);
//                        } else if (hasResults && yOffset <= 9.) {
//                            ShooterSubsystem.INSTANCE.setFarPID();
//                            targetRPM = 4330;
//                        }
//                        ShooterSubsystem.INSTANCE.spinUp(targetRPM);
                    }
//                } else {
//                    LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.GREEN);
//                }

            }
        } else {
            // no valid result or bumper not held
            // LED status based on number of balls
            if (ODODistance < minDisatanceForShooting) {
                // Only call startStrobe once when entering the "too close" state
                if (!tooCloseWarningActive) {
                    LEDControlSubsystem.INSTANCE.startStrobe(LEDControlSubsystem.LedColor.OFF, LEDControlSubsystem.LedColor.RED, 500);
                    tooCloseWarningActive = true;
                }
            } else {
                tooCloseWarningActive = false;
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
        }

        boolean leftBumperPressedNow = gamepad1.left_bumper;
        boolean leftBumperRisingEdge = leftBumperPressedNow && !prevGamepad1LeftBumper;
        if (!SHOT_TUNING_MODE) {
            if (leftBumperRisingEdge) {
                shooterFollowEnabled = true;
            }
            if (IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) {
                shooterFollowEnabled = true;
            }
        } else {
            shooterFollowEnabled = false;
        }

        // Continuously compute desired RPM from current aiming mode, but only spin flywheel
        // when explicitly enabled (left-bumper latch or 3-ball auto-enable).
        if (!SHOT_TUNING_MODE) {
            targetRPM = sotmControlActive
                    ? calculateShooterRPMOdoDistance(shooterDistanceForBallistics)
                    : calculateShooterRPMOdoDistance(this.ODODistance);

            // Allow RB/RT fire request to actively prime flywheel RPM so shoot gating can clear.
            // This still avoids pre-spinning during Init or idle driving.
            boolean shooterPrimeRequested = shooterFollowEnabled || sotmFireRequestActive;
            if (matchHasStarted && shooterPrimeRequested) {
                ShooterSubsystem.INSTANCE.spinUp(targetRPM);
                adjustOdo = true;
            }
        }

        if (leftBumperPressedNow || shooterFollowEnabled || sotmControlActive) {
            goToTargetAngle = false;
//        } else if (gamepad1.right_trigger > 0.1) {
//            targetAngleDeg = 180.0 + angleAllianceOffset;
//            goToTargetAngle = true;
//            adjustOdo = false;
//        } else if (gamepad1.left_trigger > 0.1) {
//            targetAngleDeg = -90.0 + angleAllianceOffset;
//            goToTargetAngle = true;
//            adjustOdo = false;
//        } else if (gamepad1.dpad_left) {
//            targetAngleDeg = 90.0 + angleAllianceOffset;
//            goToTargetAngle = true;
//            adjustOdo = false;
//        } else if (gamepad1.dpad_up) {
//            targetAngleDeg = 0.0 + angleAllianceOffset;
//            goToTargetAngle = true;
            //adjustOdo = false;
        } else {
            goToTargetAngle = false;
            adjustOdo = false;
        }

        targetAngleRad = Math.toRadians(targetAngleDeg);

//        if (targetRPM == 0 && IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3) {
//            targetRPM = 3000;
//            ShooterSubsystem.INSTANCE.spinUp(targetRPM);
//        }


        //mR. TODONE 😎👌👌
        if (goToTargetAngle) {
            double targetAngleDiff = botHeadingRad - targetAngleRad;
            if (targetAngleDiff > Math.PI) {
                targetAngleDiff = (targetAngleDiff - 2 * (Math.PI));
            } else if (targetAngleDiff < -Math.PI) {
                targetAngleDiff = (2 * (Math.PI) + targetAngleDiff);
            }
            rotate = targetAngleDiff * propAngleGain;
            if (rotate > 0.0) {
                rotate = rotate + minAnglePower;
            } else if (rotate < 0.0) {
                rotate = rotate - minAnglePower;
            }
            rotate = Math.max(Math.min(rotate, maxRotate), -maxRotate);
        }
        if (!automatedDrive) {
            //Make the last parameter false for field-centric

            //This is the normal version to use in the TeleOp
            if (!hold) {
                PedroComponent.follower().setTeleOpDrive(
                        driving,
                        strafe,
                        rotate,
                        false // field Centric
                );
            }
        }

        //Automated PathFollowing
       /* if (gamepad1.aWasPressed()) {
            PedroComponent.follower().followPath(pathChain.get());
            automatedDrive = true;
        }*/


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !PedroComponent.follower().isBusy())) {
            PedroComponent.follower().startTeleopDrive();
            automatedDrive = false;
        }

        double rpmShooter1 = ShooterSubsystem.INSTANCE.getShooter1RPM();
        double rpmShooter2 = ShooterSubsystem.INSTANCE.getShooter2RPM();
        double rpmOuttake = IntakeWithSensorsSubsystem.INSTANCE.getMotor3RPM();
        boolean shooterAtSpeed75 = isShooterReadyForFeed(
                SHOOT_GATE_AT_SPEED_TOLERANCE_RPM,
                rpmShooter1,
                rpmShooter2
        );

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
        telemetryM.addData("Calc Shooter1_RPM", ShooterSubsystem.INSTANCE.getShooter1RpmDelta());
        telemetryM.addData("Shooter2_RPM", rpmShooter2);
        telemetryM.addData("Calc Shooter2_RPM", ShooterSubsystem.INSTANCE.getShooter1RpmDelta());
        telemetryM.addData("Calc Avg_RPM", ShooterSubsystem.INSTANCE.getAverageRpmDelta());
        telemetryM.addData("Outtake_RPM", rpmOuttake);

        if (SHOW_SMOOTHED) {
            telemetryM.addData("Shooter1_RPM (smoothed)", rpmShooter1Smoothed);
            telemetryM.addData("Shooter2_RPM (smoothed)", rpmShooter2Smoothed);
            telemetryM.addData("Outtake_RPM (smoothed)", rpmOuttakeSmoothed);
        }

        boolean bb0 = IntakeWithSensorsSubsystem.INSTANCE.isSensor0Broken();
        boolean bb1 = IntakeWithSensorsSubsystem.INSTANCE.isSensor1Broken();
        boolean bb2 = IntakeWithSensorsSubsystem.INSTANCE.isSensor2Broken();
        boolean bb1FallEdgeForHybrid = false;
        boolean bb2FallEdgeForHybrid = false;
        if (hybridPrevBbInitialized) {
            bb1FallEdgeForHybrid = hybridPrevBb1 && !bb1;
            bb2FallEdgeForHybrid = hybridPrevBb2 && !bb2;
        }
        hybridPrevBb1 = bb1;
        hybridPrevBb2 = bb2;
        hybridPrevBbInitialized = true;

        updateShotInfoBreakbeamTracking(nowMs, bb0, bb1, bb2);
        maybeLogDumbShootRpmSample(nowMs, bb0, bb1, bb2, rpmShooter1, rpmShooter2);
        updateHybridShotFeedBoostController(nowMs, bb1FallEdgeForHybrid, bb2FallEdgeForHybrid);

        telemetryM.addData("ballCount", IntakeWithSensorsSubsystem.INSTANCE.getBallCount());
        telemetryM.addData("BB_sensor0", bb0 ? 1 : 0);
        telemetryM.addData("BB_sensor1", bb1 ? 1 : 0);
        telemetryM.addData("BB_sensor2", bb2 ? 1 : 0);

        // =============================================
        // DEBUG: Intake state flags for troubleshooting
        // =============================================
        telemetry.addData("INT_isIntaking", IntakeWithSensorsSubsystem.INSTANCE.isIntakingActive());
        telemetry.addData("INT_shooting", IntakeWithSensorsSubsystem.INSTANCE.isShooting());
        telemetry.addData("INT_singleBallFeedActive", IntakeWithSensorsSubsystem.INSTANCE.isSingleBallFeedActive());
        telemetry.addData("INT_multiSingleShotActive", IntakeWithSensorsSubsystem.INSTANCE.isMultiSingleShotActive());
        telemetry.addData("INT_shootSeqActive", IntakeWithSensorsSubsystem.INSTANCE.isShootSequenceActive());
        telemetry.addData("INT_shotInProgress", IntakeWithSensorsSubsystem.INSTANCE.isShotInProgress());
        telemetry.addData("INT_m1Enabled", IntakeWithSensorsSubsystem.INSTANCE.isMotor1Enabled());
        telemetry.addData("INT_m2Enabled", IntakeWithSensorsSubsystem.INSTANCE.isMotor2Enabled());
        telemetry.addData("INT_m3Enabled", IntakeWithSensorsSubsystem.INSTANCE.isMotor3Enabled());
        telemetry.addData("INT_direction", IntakeWithSensorsSubsystem.INSTANCE.getCurrentDirection());
        telemetry.addData("INT_multiReq", IntakeWithSensorsSubsystem.INSTANCE.getMultiSingleShotRequested());
        telemetry.addData("INT_multiDone", IntakeWithSensorsSubsystem.INSTANCE.getMultiSingleShotCompleted());

        double loopTimeMs = timer.getMs();
        telemetryM.addData("LoopTime_ms", loopTimeMs);
        telemetry.addData("rotate", rotate);
        telemetry.addData("turretTargetDeg", TurretSubsystem.INSTANCE.getTargetAngleDegrees());
        telemetry.addData("turretMeasuredDeg", TurretSubsystem.INSTANCE.getMeasuredAngleDegrees());
        telemetry.addData("fieldAngleDeg", fieldAngleDeg);
        telemetry.addData("angleErrorDeg", angleErrorDeg);
        telemetry.addData("turretReady", TurretSubsystem.INSTANCE.isTurretReady());
        telemetry.addData("boostActive", ShooterSubsystem.INSTANCE.boostActive);
        telemetry.addData("shooterFollowEnabled", shooterFollowEnabled);
        telemetry.addData("SOTM_active", sotmControlActive);
        telemetry.addData("SOTM_valid", sotmResult.valid);
        telemetry.addData("SOTM_leadApplied", sotmResult.leadApplied);
        telemetry.addData("SOTM_speed", sotmResult.speedInPerSec);
        telemetry.addData("SOTM_totalTofSec", sotmResult.totalTimeSeconds);
        telemetry.addData("SOTM_leadX", sotmResult.leadXInches);
        telemetry.addData("SOTM_leadY", sotmResult.leadYInches);
        telemetry.addData("SOTM_radialVel", sotmResult.radialVelocityInPerSec);
        telemetry.addData("SOTM_omegaRawDegS", Math.toDegrees(sotmOmegaRawRadPerSec));
        telemetry.addData("SOTM_omegaFiltDegS", Math.toDegrees(sotmOmegaRadPerSec));
        telemetry.addData("SOTM_effectiveDist", sotmResult.effectiveDistanceInches);
        telemetry.addData("SOTM_turretAimDeg", sotmResult.turretRobotRelativeAimDeg);
        telemetry.addData("SOTM_turretLagCompDeg", sotmResult.turretLagCompensationDeg);
        telemetry.addData("SOTM_turretGate", turretAimGateSatisfied);
        telemetry.addData("SOTM_turretGoalErrDeg", turretGoalErrorDeg);
        telemetry.addData("SOTM_turretConstraintErrDeg", turretConstraintErrorDeg);
        telemetry.addData("SOTM_turretReachable", turretTargetReachable);
        telemetry.addData("SOTM_turretSpeedDegS", turretMeasuredVelDegPerSec);
        telemetry.addData("SOTM_turretSpeedGate", turretSpeedGateSatisfied);
        telemetry.addData("SOTM_fireGate", sotmFireGateSatisfied);
        telemetry.addData("SOTM_canShootGate", canShootAtGoal);
        telemetry.addData("SOTM_shooterAtSpeed75", shooterAtSpeed75);
        telemetry.addData("SHOOT_batteryV", ShooterSubsystem.INSTANCE.getBatteryVoltageRaw());
        telemetry.addData("SHOOT_batteryVFiltered", ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered());
        telemetry.addData("SHOOT_voltageCompGain", ShooterSubsystem.INSTANCE.getVoltageCompGain());
        telemetry.addData("SHOOT_cmdPreVComp", ShooterSubsystem.INSTANCE.getCommandPreVoltageComp());
        telemetry.addData("SHOOT_cmdPostVComp", ShooterSubsystem.INSTANCE.getCommandPostVoltageComp());
        telemetry.addData("SHOOT_cmdSaturated", ShooterSubsystem.INSTANCE.isCommandSaturated());
        telemetry.addData("SHOOT_boostMult1", ShooterSubsystem.INSTANCE.getActiveBoostMultiplier1());
        telemetry.addData("SHOOT_boostMult2", ShooterSubsystem.INSTANCE.getActiveBoostMultiplier2());
        telemetry.addData("SHOOT_burstProfileId", BURST_PROFILE_ID);
        telemetry.addData("HYBRID_feedBoostActive", hybridShotFeedBoostActive);
        telemetry.addData("HYBRID_phase", hybridShotFeedBoostPhase.name());
        telemetry.addData("HYBRID_tSinceShotFeedStartMs",
                hybridShotFeedBoostActive ? (nowMs - hybridShotFeedStartMs) : -1);
        telemetry.addData("HYBRID_expectedContactMs", getHybridExpectedContactMsForCurrentPhase());
        telemetry.addData("HYBRID_preBoostAmountActive", ShooterSubsystem.INSTANCE.getActivePreBoostAmount());
        telemetry.addData("HYBRID_lastAdvanceReason", hybridLastAdvanceReason);
        telemetry.addData("HYBRID_lastAdvanceRelMs", hybridLastAdvanceAtRelMs);
        telemetry.addData("SOTM_tooCloseBlock", tooCloseWarningActive);
        telemetry.addData("SOTM_shotGateLedState", shotGateLedState);
        telemetry.addData("SHOT_TUNE_mode", SHOT_TUNING_MODE);
        telemetry.addData("SHOT_TUNE_flywheelEnabled", shotTuningFlywheelEnabled);
        telemetry.addData("SHOT_TUNE_targetRPM", targetRPM);
        telemetry.addData("SHOT_TUNE_hoodPos", shooterHoodPos);
        telemetry.addData("SHOT_TUNE_targetX", SHOT_TUNING_TARGET_X_IN);
        telemetry.addData("SHOT_TUNE_targetY", SHOT_TUNING_TARGET_Y_IN);
        telemetry.addData("SHOT_TUNE_pendingLabel", shotTuningPendingLabel);
        telemetry.addData("SHOT_TUNE_pendingShotId", shotTuningPendingShotId);
        telemetry.addData("SHOT_TUNE_controls",
                "g2RB toggle flywheel, g2A fire, g2Y/X hood+/- , g2DpadUp/Down rpm+/- , g1Dpad target, g1X keep, g1B ignore");
        telemetryM.addData("targetRPM", ShooterSubsystem.INSTANCE.getTargetRpm());

        if (ENABLE_TURRET_LOGGING && turretLogger != null) {
            long nowLogMs = System.currentTimeMillis();

            if (nowLogMs - lastTurretLogMs >= TURRET_LOG_PERIOD_MS) {
                lastTurretLogMs = nowLogMs;

                double turretTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();
                double turretTargetCorrectedDeg = TurretSubsystem.INSTANCE.getCorrectedTargetAngleDegrees();
                double turretCommandedDeg = TurretSubsystem.INSTANCE.getCommandedAngleDegrees();
                double turretServoCommandDeg = TurretSubsystem.INSTANCE.getServoCommandAngleDegrees();
                double turretErrorDeg = turretTargetDeg - turretMeasuredDeg;
                double turretOuterTrimDeg = TurretSubsystem.INSTANCE.getOuterLoopTrimDegrees();
                double turretCommandDiffDeg = TurretSubsystem.INSTANCE.getCommandDiffDegrees();
                double turretRateStepDeg = TurretSubsystem.INSTANCE.getRateLimitedStepDegrees();
                double turretLeftCmdDeg = TurretSubsystem.INSTANCE.getLeftTurretAngleCommandDegrees();
                double turretRightCmdDeg = TurretSubsystem.INSTANCE.getRightTurretAngleCommandDegrees();
                double turretServoPos = TurretSubsystem.INSTANCE.getCurrentServoPosition();
                double turretLeftServoPos = TurretSubsystem.INSTANCE.getCurrentLeftServoPosition();
                double turretRightServoPos = TurretSubsystem.INSTANCE.getCurrentRightServoPosition();
                double turretQuadRawDeg = TurretSubsystem.INSTANCE.getQuadRawAngleDegrees();
                double turretQuadOffsetDeg = TurretSubsystem.INSTANCE.getQuadratureOffsetDegrees();
                double turretEncoderVelDegPerSec = TurretSubsystem.INSTANCE.getMeasuredVelocityDegPerSec();
                double turretAbsRawDeg = TurretSubsystem.INSTANCE.getAbsoluteEncoderRawDegrees();
                double turretAbsDeg = TurretSubsystem.INSTANCE.getAbsoluteEncoderTurretAngleDegrees();
                double turretAbsDeltaDeg = TurretSubsystem.INSTANCE.getAbsoluteEncoderTurretDeltaDegrees();
                double turretLearnedServoOffsetDeg = TurretSubsystem.INSTANCE.getLearnedServoCommandOffsetDegrees();
                double turretAbsStartupErrorDeg = TurretSubsystem.INSTANCE.getAbsoluteStartupErrorDegrees();
                int turretEncoderTicks = TurretSubsystem.INSTANCE.getCurrentEncoderTicks();

                while (turretErrorDeg > 180) turretErrorDeg -= 360;
                while (turretErrorDeg < -180) turretErrorDeg += 360;

                double turretLogDtMs = 0.0;
                if (prevTurretLogMs > 0L && nowLogMs >= prevTurretLogMs) {
                    turretLogDtMs = nowLogMs - prevTurretLogMs;
                }
                double turretTargetDeltaDeg = 0.0;
                if (Double.isFinite(prevTurretLogTargetDeg)) {
                    turretTargetDeltaDeg = smallestWrappedDeltaDegrees(turretTargetDeg, prevTurretLogTargetDeg);
                }
                double turretMeasuredDeltaDeg = 0.0;
                if (Double.isFinite(prevTurretLogMeasuredDeg)) {
                    turretMeasuredDeltaDeg = smallestWrappedDeltaDegrees(turretMeasuredDeg, prevTurretLogMeasuredDeg);
                }
                double turretServoCommandDeltaDeg = 0.0;
                if (Double.isFinite(prevTurretLogServoCommandDeg)) {
                    turretServoCommandDeltaDeg = smallestWrappedDeltaDegrees(turretServoCommandDeg, prevTurretLogServoCommandDeg);
                }
                double turretServoPosDelta = 0.0;
                if (Double.isFinite(prevTurretLogServoPos)) {
                    turretServoPosDelta = turretServoPos - prevTurretLogServoPos;
                }

                double unclippedServoCommandDeg = turretCommandedDeg + turretOuterTrimDeg;
                boolean turretLimitClipped =
                        Math.abs(unclippedServoCommandDeg - turretServoCommandDeg) > 1e-6;
                double turretTrimAsServoDeltaPos =
                        Math.abs(turretOuterTrimDeg) / Math.max(1e-6, TurretSubsystem.TURRET_TRAVEL_DEGREES);
                boolean turretCmdAboveDeadbandGuess =
                        turretTrimAsServoDeltaPos >= Math.max(0.0, TURRET_DIAG_SERVO_DEADBAND_POS_GUESS);
                boolean turretTargetStable =
                        Math.abs(turretTargetDeltaDeg) <= Math.max(0.0, TURRET_DIAG_TARGET_STABLE_DELTA_DEG);
                boolean stictionCandidateNow =
                        turretTargetStable &&
                        Math.abs(turretErrorDeg) >= Math.max(0.0, TURRET_DIAG_STICTION_ERROR_MIN_DEG) &&
                        Math.abs(turretEncoderVelDegPerSec) <= Math.max(0.0, TURRET_DIAG_STICTION_VELOCITY_MAX_DEG_PER_SEC) &&
                        turretCmdAboveDeadbandGuess &&
                        !turretLimitClipped;
                if (stictionCandidateNow) {
                    if (turretStictionCandidateStartMs == 0L) {
                        turretStictionCandidateStartMs = nowLogMs;
                    }
                } else {
                    turretStictionCandidateStartMs = 0L;
                }
                boolean turretStictionSuspect =
                        stictionCandidateNow &&
                        turretStictionCandidateStartMs > 0L &&
                        (nowLogMs - turretStictionCandidateStartMs) >= Math.max(0L, TURRET_DIAG_STICTION_MIN_TIME_MS);

                long matchT = (logStartMs == 0L) ? 0L : (nowLogMs - logStartMs);

                turretLogger.addRow(
                        nowLogMs,
                        matchT,
                        botxvalue,
                        botyvalue,
                        Math.toDegrees(botHeadingRad),
                        shootTargetX,
                        shootTargetY,
                        fieldAngleDeg,
                        angleErrorDeg,
                        turretTargetDeg,
                        turretTargetCorrectedDeg,
                        turretCommandedDeg,
                        turretServoCommandDeg,
                        turretMeasuredDeg,
                        turretErrorDeg,
                        turretOuterTrimDeg,
                        turretCommandDiffDeg,
                        turretRateStepDeg,
                        turretLeftCmdDeg,
                        turretRightCmdDeg,
                        turretServoPos,
                        turretLeftServoPos,
                        turretRightServoPos,
                        turretQuadRawDeg,
                        turretQuadOffsetDeg,
                        turretEncoderVelDegPerSec,
                        turretAbsRawDeg,
                        turretAbsDeg,
                        turretAbsDeltaDeg,
                        turretLearnedServoOffsetDeg,
                        turretAbsStartupErrorDeg,
                        turretEncoderTicks,
                        turretLogDtMs,
                        turretTargetDeltaDeg,
                        turretMeasuredDeltaDeg,
                        turretServoCommandDeltaDeg,
                        turretServoPosDelta,
                        turretTrimAsServoDeltaPos,
                        turretCmdAboveDeadbandGuess,
                        turretTargetStable,
                        turretLimitClipped,
                        turretStictionSuspect,
                        TurretSubsystem.INSTANCE.isTurretReady(),
                        driving,
                        strafe,
                        rotate,
                        ODODistance,
                        xOffset,
                        yOffset,
                        hasResults,
                        loopTimeMs
                );

                prevTurretLogTargetDeg = turretTargetDeg;
                prevTurretLogMeasuredDeg = turretMeasuredDeg;
                prevTurretLogServoCommandDeg = turretServoCommandDeg;
                prevTurretLogServoPos = turretServoPos;
                prevTurretLogMs = nowLogMs;
            }
        }

        if (ENABLE_SOTM_LOGGING && sotmLogger != null) {
            long nowSotmLogMs = System.currentTimeMillis();
            if (nowSotmLogMs - lastSotmLogMs >= SOTM_LOG_PERIOD_MS) {
                lastSotmLogMs = nowSotmLogMs;
                long matchT = (logStartMs == 0L) ? 0L : (nowSotmLogMs - logStartMs);

                sotmLogger.addRow(
                        nowSotmLogMs,
                        matchT,
                        sotmFireRequestActive,
                        sotmControlActive,
                        rightBumperActive,
                        rightTriggerActive,
                        botxvalue,
                        botyvalue,
                        Math.toDegrees(botHeadingRad),
                        shootTargetX,
                        shootTargetY,
                        fieldAngleDeg,
                        angleErrorDeg,
                        ODODistance,
                        sotmResult.valid,
                        sotmResult.leadApplied,
                        sotmResult.speedInPerSec,
                        sotmVx,
                        sotmVy,
                        Math.toDegrees(sotmOmegaRawRadPerSec),
                        Math.toDegrees(sotmOmegaRadPerSec),
                        sotmResult.totalTimeSeconds,
                        sotmResult.leadXInches,
                        sotmResult.leadYInches,
                        sotmResult.radialVelocityInPerSec,
                        sotmResult.effectiveDistanceInches,
                        shooterDistanceForBallistics,
                        sotmResult.turretRobotRelativeAimDeg,
                        sotmResult.turretLagCompensationDeg,
                        TurretSubsystem.INSTANCE.getTargetAngleDegrees(),
                        TurretSubsystem.INSTANCE.getMeasuredAngleDegrees(),
                        TurretSubsystem.INSTANCE.isTurretReady(),
                        turretAimGateSatisfied,
                        turretGoalErrorDeg,
                        turretConstraintErrorDeg,
                        turretTargetReachable,
                        turretMeasuredVelDegPerSec,
                        turretSpeedGateSatisfied,
                        sotmFireGateSatisfied,
                        canShootAtGoal,
                        ShooterSubsystem.INSTANCE.getTargetRpm(),
                        rpmShooter1,
                        rpmShooter2,
                        shooterAtSpeed75,
                        ShooterSubsystem.INSTANCE.getBatteryVoltageRaw(),
                        ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered(),
                        ShooterSubsystem.INSTANCE.getVoltageCompGain(),
                        ShooterSubsystem.INSTANCE.getCommandPreVoltageComp(),
                        ShooterSubsystem.INSTANCE.getCommandPostVoltageComp(),
                        ShooterSubsystem.INSTANCE.isCommandSaturated(),
                        dumbShootTimerActive,
                        hold,
                        driving,
                        strafe,
                        rotate,
                        loopTimeMs
                );
            }
        }

        if (testShooter) {
            telemetry.addData("kP", ShooterSubsystem.kP);
            telemetry.addData("kI", ShooterSubsystem.kI);
            telemetry.addData("kD", ShooterSubsystem.kD);
            telemetry.addData("kS", ShooterSubsystem.kS);
            telemetry.addData("kV", ShooterSubsystem.kV);
            telemetry.addData("kA", ShooterSubsystem.kA);
            telemetry.addData("HEADROOM", ShooterSubsystem.HEADROOM);
            telemetry.addData("SLEW_PER_SECOND", ShooterSubsystem.SLEW_PER_SECOND);
            telemetry.addData("I_ZONE", ShooterSubsystem.I_ZONE);
        }

        if (result == null) {
            telemetry.addData("Limelight", "No result object");
        } else if (!result.isValid()) {
            telemetry.addData("Limelight", "No valid target");
        } else {
            telemetry.addData("Limelight", "Target seen!");
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("LL distance", distanceLL);
            Pose3D MT2Pose = result.getBotpose_MT2();
            //telemetry.addData("MegaTag 2 Raw Angle", MT2Pose.getOrientation().getYaw());
            //telemetry.addData("MegaTag 2 Raw X", MT2Pose.getPosition().x);
            //telemetry.addData("MegaTag 2 Raw y", MT2Pose.getPosition().y);
//            telemetry.addData("MegaTag 1 Raw Angle", MT2Pose.getOrientation().getYaw());
//            telemetry.addData("MegaTag 1 Raw X", MT1Pose.getPosition().x);
//            telemetry.addData("MegaTag 1 Raw y", MT1Pose.getPosition().y);

            //double MT2xInches = DistanceUnit.METER.toInches(MT2Pose.getPosition().x);
            //double MT2yInches = DistanceUnit.METER.toInches(MT2Pose.getPosition().y);

            //Pose MT2FTCStandardPose = PoseConverter.pose2DToPose(MT2Pose2d, InvertedFTCCoordinates.INSTANCE);

            //Pose MT2PedroPose = MT2FTCStandardPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            //MT1FTCStandardPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            //telemetry.addData("MegaTag 2 Angle", Math.toDegrees(MT2PedroPose.getPose().getHeading()));
            //telemetry.addData("MegaTag 2 X", MT2PedroPose.getPose().getX());
            //telemetry.addData("MegaTag 2 y", MT2PedroPose.getPose().getY());
            //telemetry.addData("STD MegaTag 1 Angle", Math.toDegrees(MT1FTCStandardPose.getPose().getHeading()));
            //telemetry.addData("STD MegaTag 1 X", MT1FTCStandardPose.getPose().getX());
            //telemetry.addData("STD MegaTag 1 y", MT1FTCStandardPose.getPose().getY());

            //telemetry.addData("MegaTag 2 Angle", Math.toDegrees(MT2PedroPose.getPose().getHeading()));
            //telemetry.addData("MegaTag 2 X", MT2PedroPose.getPose().getX());
            //telemetry.addData("MegaTag 2 y", MT2PedroPose.getPose().getY());
            telemetry.addData("Pedro MegaTag 1 Angle", Math.toDegrees(MT1PedroPose.getPose().getHeading()));
            telemetry.addData("Pedro MegaTag 1 X", MT1PedroPose.getPose().getX());
            telemetry.addData("Pedro MegaTag 1 y", MT1PedroPose.getPose().getY());

        }
        telemetry.addData("ODO distance", ODODistance);
        telemetry.addData("ODO X-Location", botxvalue);
        telemetry.addData("ODO Y-Location", botyvalue);
        telemetry.addData("ODO Angle", Math.toDegrees(botHeadingRad));



        //Start Ian's control
        boolean leftTriggerActive = gamepad2.left_trigger > 0.1;
        if (!SHOT_TUNING_MODE && shotTuningPendingLabel) {
            labelShotTuningSample("UNRATED", "mode_exit");
        }

        if (SHOT_TUNING_MODE) {
            hold = false;
            dumbShootTimerActive = false;
            shooterFollowEnabled = false;
            resetHybridShotFeedBoostController();

            boolean tuningRpmUp = gamepad2.dpad_up;
            boolean tuningRpmDown = gamepad2.dpad_down;
            double rpmStep = Math.max(0.0, SHOT_TUNING_RPM_STEP);
            if (tuningRpmUp && !prevTuningRpmDpadUp) {
                targetRPM = Math.max(0.0, targetRPM + rpmStep);
            }
            if (tuningRpmDown && !prevTuningRpmDpadDown) {
                targetRPM = Math.max(0.0, targetRPM - rpmStep);
            }
            prevTuningRpmDpadUp = tuningRpmUp;
            prevTuningRpmDpadDown = tuningRpmDown;

            double hoodStep = Math.max(0.0, SHOT_TUNING_HOOD_STEP);
            if (gamepad2.yWasPressed()) {
                shooterHoodPos = Math.min(1.0, shooterHoodPos + hoodStep);
            }
            if (gamepad2.xWasPressed()) {
                shooterHoodPos = Math.max(0.0, shooterHoodPos - hoodStep);
            }

            if (gamepad2.rightBumperWasPressed()) {
                shotTuningFlywheelEnabled = !shotTuningFlywheelEnabled;
            }
            if (gamepad2.leftBumperWasPressed()) {
                shotTuningFlywheelEnabled = false;
            }

            if (shotTuningFlywheelEnabled && matchHasStarted) {
                ShooterSubsystem.INSTANCE.spinUp(targetRPM);
            } else {
                ShooterSubsystem.INSTANCE.stop();
            }

            boolean shotTuningAtSpeed =
                    !SHOT_TUNING_REQUIRE_AT_SPEED_TO_FIRE ||
                    isShooterReadyForFeed(
                            SHOT_TUNING_AT_SPEED_TOLERANCE_RPM,
                            rpmShooter1,
                            rpmShooter2
                    );
            boolean shotTuningCanFire = canShootAtGoal && shotTuningAtSpeed && !tooCloseWarningActive;
            if (gamepad2.aWasPressed() && shotTuningCanFire) {
                if (shotTuningPendingLabel) {
                    labelShotTuningSample("UNRATED", "next_shot_before_label");
                }
                if (IntakeWithSensorsSubsystem.INSTANCE.feedSingleBallFullPower()) {
                    shotTuningShotIdCounter++;
                    shotTuningPendingLabel = true;
                    shotTuningPendingShotId = shotTuningShotIdCounter;
                    shotTuningPendingFireMs = nowMs;
                    shotTuningPendingStartBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
                    shotTuningPendingTargetRpm = targetRPM;
                    shotTuningPendingHoodPos = shooterHoodPos;
                    shotTuningPendingRpm1 = rpmShooter1;
                    shotTuningPendingRpm2 = rpmShooter2;
                    shotTuningPendingBotX = botxvalue;
                    shotTuningPendingBotY = botyvalue;
                    shotTuningPendingBotHeadingDeg = Math.toDegrees(botHeadingRad);
                    shotTuningPendingTargetX = shootTargetX;
                    shotTuningPendingTargetY = shootTargetY;
                    shotTuningPendingOdoDistance = ODODistance;
                    shotTuningPendingTurretTargetDeg = TurretSubsystem.INSTANCE.getTargetAngleDegrees();
                    shotTuningPendingTurretMeasuredDeg = TurretSubsystem.INSTANCE.getMeasuredAngleDegrees();
                }
            }

            if (gamepad1.xWasPressed() && shotTuningPendingLabel) {
                labelShotTuningSample("KEEP", "manual_keep");
            } else if (gamepad1.bWasPressed() && shotTuningPendingLabel) {
                labelShotTuningSample("IGNORE", "manual_ignore");
            }

            maybeFinalizeShotInfoSequence(nowMs, false, leftTriggerActive);
            singleShotPending = false;
            shotGateLedState = "NONE";
        } else {
            if (gamepad2.rightBumperWasPressed()) {
                this.shoot = true;
                // Starting a new spin-up cancels any previous dumbShoot timeout
                dumbShootTimerActive = false;
                if (testShooter) {

                }else if (!hasResults || this.adjustOdo) {
                    //ShooterSubsystem.INSTANCE.setClosePID();
                    targetRPM = calculateShooterRPMOdoDistance(this.ODODistance);
                    //ShooterSubsystem.INSTANCE.spinUp(targetRPM);
                    //targetRPM = 2950;
                } else if (hasResults && yOffset > 9.) {
                    ShooterSubsystem.INSTANCE.setClosePID();
                    targetRPM = calculateShooterRPM(yOffset);
                } else if (hasResults && yOffset <= 9.) {
                    ShooterSubsystem.INSTANCE.setFarPID();
                    targetRPM = 4330;
                }
                //ShooterSubsystem.INSTANCE.increaseShooterRPMBy10();
                telemetry.addData("Target Shooter Speed", targetRPM);
                ShooterSubsystem.INSTANCE.spinUp(targetRPM);
            } else if (gamepad2.leftBumperWasPressed()) {
                ShooterSubsystem.INSTANCE.stop();
                dumbShootTimerActive = false;
                shooterFollowEnabled = false;
                resetHybridShotFeedBoostController();
                //ShooterSubsystem.INSTANCE.decreaseShooterRPMBy10();
            }

            if ((gamepad2.xWasPressed()) &&
                    shooterAtSpeed75 &&
                    canShootAtGoal) {
                int startBallCount = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
                boolean started = IntakeWithSensorsSubsystem.INSTANCE.shootMultipleSingleShots(startBallCount);
                if (started) {
                    startShotInfoSequence("multi_single_shot", nowMs, startBallCount, bb0, bb1, bb2);
                }
    //            ShooterSubsystem.INSTANCE.decreaseShooterHoodPosInc();
            }
    //        if (gamepad2.yWasPressed()) {
    //            ShooterSubsystem.INSTANCE.increaseShooterHoodPosInc();
    //        }

            boolean canFireTriggerShot =
                    shooterAtSpeed75 &&
                    canShootAtGoal &&
                    !tooCloseWarningActive;
            String desiredShotGateLedState = "NONE";
            if (sotmFireRequestActive && !tooCloseWarningActive) {
                if (!turretAimGateSatisfied) {
                    desiredShotGateLedState = "TURRET_BLOCK";
                } else if (!shooterAtSpeed75) {
                    desiredShotGateLedState = "RPM_BLOCK";
                }
            }
            if (!desiredShotGateLedState.equals(shotGateLedState)) {
                if ("TURRET_BLOCK".equals(desiredShotGateLedState)) {
                    LEDControlSubsystem.INSTANCE.startStrobe(
                            LEDControlSubsystem.LedColor.OFF,
                            LEDControlSubsystem.LedColor.ORANGE,
                            Math.max(10L, SHOOT_BLOCK_LED_STROBE_MS)
                    );
                } else if ("RPM_BLOCK".equals(desiredShotGateLedState)) {
                    LEDControlSubsystem.INSTANCE.startStrobe(
                            LEDControlSubsystem.LedColor.OFF,
                            LEDControlSubsystem.LedColor.BLUE,
                            Math.max(10L, SHOOT_BLOCK_LED_STROBE_MS)
                    );
                }
                shotGateLedState = desiredShotGateLedState;
            }
            maybeFinalizeShotInfoSequence(nowMs, sotmFireRequestActive, leftTriggerActive);
            if (sotmFireRequestActive && !tooCloseWarningActive) {
                // Only right-trigger shooting should lock robot translation/rotation.
                // Right-bumper SOTM stays fully driveable.
                hold = rightTriggerActive;

                boolean useFarBoostProfile = ODODistance >= HYBRID_NEAR_FAR_DISTANCE_THRESHOLD_IN;
                if (canFireTriggerShot && !dumbShootTimerActive) {
                    int startBallCountBeforeDumbShoot = IntakeWithSensorsSubsystem.INSTANCE.getBallCount();
                    ShooterSubsystem.INSTANCE.boostOverride = false;
                    IntakeWithSensorsSubsystem.INSTANCE.setDumbShootDistanceForDelayInches(ODODistance);
                    IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();
                    startShotInfoSequence("dumbshoot", nowMs, startBallCountBeforeDumbShoot, bb0, bb1, bb2);
                    shotSequenceLinkedDumbShootRpmSequenceId = startDumbShootRpmLogSequence(
                            nowMs,
                            startBallCountBeforeDumbShoot,
                            bb0,
                            bb1,
                            bb2,
                            shotSequenceId
                    );
                    dumbShootTimerActive = true;
                    dumbShootStartTimeMs = nowMs;
                    if (ENABLE_HYBRID_SHOT_FEED_BOOST) {
                        startHybridShotFeedBoostController(nowMs, ODODistance);
                    } else {
                        ShooterSubsystem.INSTANCE.setBoostOn(useFarBoostProfile);
                    }
                }
            }  else if (leftTriggerActive && !prevLeftTriggerActive) {
                // Latch a single-shot request; actual firing waits for RPM to be in range
                hold = true;
                singleShotPending = true;
            } else if (gamepad2.aWasPressed()) {
                hold = false;
                this.shoot = false;
                //targetRPM = 1000;
                IntakeWithSensorsSubsystem.INSTANCE.intakeForward();//Hoping Forward is Intake (maybe change the method name)
                ShooterSubsystem.INSTANCE.stop();
                shooterFollowEnabled = false;
                dumbShootTimerActive = false;
                resetHybridShotFeedBoostController();
                this.hasResults = false;
                LEDControlSubsystem.INSTANCE.setBoth(LEDControlSubsystem.LedColor.RED);
            } else if (gamepad2.b) {
                hold = false;
                IntakeWithSensorsSubsystem.INSTANCE.intakeReverse();
            } else {
                hold = false;
            }

            // If a single shot is pending and left trigger is still held (and right trigger is not),
            // wait for shooter RPM to be within tolerance, then fire exactly one ball.
            if (leftTriggerActive && !sotmFireRequestActive && singleShotPending &&
                    canShootAtGoal &&
                    shooterAtSpeed75) { // 75 RPM window for 58-RPM resolution
                if (IntakeWithSensorsSubsystem.INSTANCE.feedSingleBallFullPower()) {
                    startShotInfoSequence("single_ball_feed", nowMs, IntakeWithSensorsSubsystem.INSTANCE.getBallCount(), bb0, bb1, bb2);
                    singleShotPending = false;
                }
            }

            // If the left trigger is released, clear any pending single-shot request so the
            // next press is treated as a new request.
            if (!leftTriggerActive && prevLeftTriggerActive) {
                singleShotPending = false;
                hold = false;
            }
        }

        prevLeftTriggerActive = leftTriggerActive;
        prevGamepad1LeftBumper = leftBumperPressedNow;

        if (hold && !prevHold) {
            PedroComponent.follower().holdPoint(new BezierPoint(PedroComponent.follower().getPose()), PedroComponent.follower().getHeading(), false);
        } else if (!hold && prevHold){
            PedroComponent.follower().startTeleopDrive();
        }

        prevHold = hold;


        if (SHOT_TUNING_MODE) {
            // Keep manual hood value in shot tuning mode.
        } else if (testShooter) {
            // Do nothing
        } else if (sotmControlActive) {
            this.shooterHoodPos = calculateShooterHoodOdoDistance(shooterDistanceForBallistics);
        } else if (hasResults && !adjustOdo) {  //if limelight doesn't have results then use ODO Distance - Thinking that it would be better to always use ODO distance unless pressing a button to use limelight?
            //this.shooterHoodPos = getHoodPositionFromDistance(this.ODODistance);
            //this.shooterHoodPos = 0.05;
//        } else{
            this.shooterHoodPos = getHoodPosition(yOffset);

        }
        else{
            this.shooterHoodPos = calculateShooterHoodOdoDistance(this.ODODistance);
        }
//        }
        ShooterSubsystem.INSTANCE.shooterHoodDrive(this.shooterHoodPos);

        telemetryM.update(telemetry);
        telemetry.update();
        timer.end();
    }

    @Override
    public void onStop() {
        super.onStop();
        matchHasStarted = false;
        ShooterSubsystem.INSTANCE.stop();
        targetRPM = 0.0;
        resetHybridShotFeedBoostController();

        if (ENABLE_TURRET_LOGGING && turretLogger != null) {
            File savedFile = turretLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Turret CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Turret CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_SOTM_LOGGING && sotmLogger != null) {
            File savedFile = sotmLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "SOTM CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "SOTM CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_SHOT_INFO_LOGGING && shotInfoLogger != null) {
            // Ensure an in-progress sequence is not lost.
            if (shotLogSequenceActive) {
                finalizeShotInfoSequence("opmode_stop", System.currentTimeMillis());
            }
            File savedFile = shotInfoLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Shot-info CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Shot-info CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_DUMBSHOOT_RPM_LOGGING && dumbShootRpmLogger != null) {
            File savedFile = dumbShootRpmLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Dumbshoot RPM CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Dumbshoot RPM CSV was not saved (logger not started, empty, or save failed).");
            }
        }

        if (ENABLE_SHOT_TUNING_LOGGING && shotTuningLogger != null) {
            if (shotTuningPendingLabel) {
                labelShotTuningSample("UNRATED", "opmode_stop");
            }
            File savedFile = shotTuningLogger.save();
            if (savedFile != null) {
                RobotLog.ii("Pickles2025Teleop", "Shot tuning CSV saved: " + savedFile.getAbsolutePath());
            } else {
                RobotLog.ww("Pickles2025Teleop", "Shot tuning CSV was not saved (logger not started, empty, or save failed).");
            }
        }
    }

    private void resetHybridShotFeedBoostController() {
        hybridShotFeedBoostActive = false;
        hybridShotFeedUsesFarProfile = false;
        hybridShotFeedStartMs = 0L;
        hybridBall1ContactStartMsEst = -1L;
        hybridBall2ContactStartMsEst = -1L;
        hybridBall3ContactStartMsEst = -1L;
        hybridLastAcceptedEdgeMs = -1L;
        hybridLastAdvanceAtRelMs = -1L;
        hybridLastAdvanceReason = "NONE";
        hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.IDLE;
        ShooterSubsystem.INSTANCE.clearPreBoostAmountOverride();
        ShooterSubsystem.INSTANCE.setPreBoostWindow(false);
        ShooterSubsystem.INSTANCE.setContactWindow(false);
        ShooterSubsystem.INSTANCE.setRecoveryWindow(false);
    }

    private void startHybridShotFeedBoostController(long nowMs, double shooterDistanceInches) {
        resetHybridShotFeedBoostController();
        hybridShotFeedBoostActive = true;
        hybridShotFeedUsesFarProfile =
                shooterDistanceInches >= Math.max(0.0, HYBRID_NEAR_FAR_DISTANCE_THRESHOLD_IN);

        long dtShotFeedStartToBall1ContactMsEst = hybridShotFeedUsesFarProfile
                ? Math.max(0L, HYBRID_FAR_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST)
                : Math.max(0L, HYBRID_CLOSE_DT_SHOT_FEED_START_TO_BALL1_CONTACT_MS_EST);
        long dtBall1ToBall2ContactStartMsEst = hybridShotFeedUsesFarProfile
                ? Math.max(0L, HYBRID_FAR_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST)
                : Math.max(0L, HYBRID_CLOSE_DT_BALL1_TO_BALL2_CONTACT_START_MS_EST);
        long dtBall2ToBall3ContactStartMsEst = hybridShotFeedUsesFarProfile
                ? Math.max(0L, HYBRID_FAR_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST)
                : Math.max(0L, HYBRID_CLOSE_DT_BALL2_TO_BALL3_CONTACT_START_MS_EST);

        hybridShotFeedStartMs = nowMs;
        hybridBall1ContactStartMsEst = dtShotFeedStartToBall1ContactMsEst;
        hybridBall2ContactStartMsEst = hybridBall1ContactStartMsEst + dtBall1ToBall2ContactStartMsEst;
        hybridBall3ContactStartMsEst = hybridBall2ContactStartMsEst + dtBall2ToBall3ContactStartMsEst;
        hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT;
        hybridLastAdvanceReason = "SHOT_FEED_START";
        hybridLastAdvanceAtRelMs = 0L;
    }

    private void advanceHybridShotFeedBoostPhase(long nowMs, String reason, boolean fromEdge) {
        if (!hybridShotFeedBoostActive) return;

        long sinceShotFeedStartMs = nowMs - hybridShotFeedStartMs;
        hybridLastAdvanceReason = reason;
        hybridLastAdvanceAtRelMs = sinceShotFeedStartMs;
        if (fromEdge) {
            hybridLastAcceptedEdgeMs = nowMs;
        }

        // One-shot boost re-arm at each accepted/fallback contact event.
        ShooterSubsystem.INSTANCE.setBoostOn(hybridShotFeedUsesFarProfile);

        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT;
        } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT;
        } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            hybridShotFeedBoostPhase = HybridShotFeedBoostPhase.COMPLETE;
            hybridShotFeedBoostActive = false;
            ShooterSubsystem.INSTANCE.setPreBoostWindow(false);
        }
    }

    private void updateHybridShotFeedBoostController(long nowMs, boolean bb1FallEdge, boolean bb2FallEdge) {
        if (!ENABLE_HYBRID_SHOT_FEED_BOOST) {
            ShooterSubsystem.INSTANCE.clearPreBoostAmountOverride();
            ShooterSubsystem.INSTANCE.setPreBoostWindow(false);
            return;
        }
        if (!hybridShotFeedBoostActive) {
            ShooterSubsystem.INSTANCE.clearPreBoostAmountOverride();
            ShooterSubsystem.INSTANCE.setPreBoostWindow(false);
            return;
        }

        long sinceShotFeedStartMs = nowMs - hybridShotFeedStartMs;
        long expectedContactMs = getHybridExpectedContactMsForCurrentPhase();
        long preBoostLeadMs = getHybridPreBoostLeadMsForCurrentPhase();

        boolean preBoostActive = expectedContactMs >= 0 &&
                preBoostLeadMs > 0 &&
                sinceShotFeedStartMs >= Math.max(0L, expectedContactMs - preBoostLeadMs) &&
                sinceShotFeedStartMs < expectedContactMs;
        if (preBoostActive) {
            ShooterSubsystem.INSTANCE.setPreBoostAmountOverride(getHybridPreBoostAmountForCurrentPhase());
        } else {
            ShooterSubsystem.INSTANCE.clearPreBoostAmountOverride();
        }
        ShooterSubsystem.INSTANCE.setPreBoostWindow(preBoostActive);

        if (expectedContactMs < 0) {
            return;
        }

        boolean edgeAllowed = hybridLastAcceptedEdgeMs < 0L ||
                nowMs - hybridLastAcceptedEdgeMs >= Math.max(0L, HYBRID_MIN_EDGE_GAP_MS);
        boolean inExpectedWindow = isInHybridWindow(
                sinceShotFeedStartMs,
                expectedContactMs,
                getHybridContactWindowHalfWidthMs()
        );

        if (edgeAllowed && inExpectedWindow) {
            if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
                if (bb2FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL1_BB2_FALL_EDGE", true);
                    return;
                }
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
                if (bb2FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL2_BB2_FALL_EDGE", true);
                    return;
                }
                if (HYBRID_USE_BB1_FALL_FOR_BALL2 && bb1FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL2_BB1_FALL_EDGE", true);
                    return;
                }
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
                if (HYBRID_USE_BB1_FALL_FOR_BALL3 && bb1FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL3_BB1_FALL_EDGE", true);
                    return;
                }
                if (HYBRID_USE_BB2_FALL_FOR_BALL3 && bb2FallEdge) {
                    advanceHybridShotFeedBoostPhase(nowMs, "BALL3_BB2_FALL_EDGE", true);
                    return;
                }
            }
        }

        if (sinceShotFeedStartMs >= expectedContactMs + Math.max(0L, HYBRID_TIMER_FALLBACK_EXTRA_MS)) {
            if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL1_TIMER_FALLBACK", false);
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL2_TIMER_FALLBACK", false);
            } else if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
                advanceHybridShotFeedBoostPhase(nowMs, "BALL3_TIMER_FALLBACK", false);
            }
        }
    }

    private long getHybridExpectedContactMsForCurrentPhase() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return hybridBall1ContactStartMsEst;
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return hybridBall2ContactStartMsEst;
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return hybridBall3ContactStartMsEst;
        }
        return -1L;
    }

    private long getHybridPreBoostLeadMsForCurrentPhase() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return Math.max(0L, HYBRID_PREBOOST1_LEAD_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return Math.max(0L, HYBRID_PREBOOST2_LEAD_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return Math.max(0L, HYBRID_PREBOOST3_LEAD_MS);
        }
        return 0L;
    }

    private double getHybridPreBoostAmountForCurrentPhase() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return Math.max(0.0, HYBRID_PREBOOST1_AMOUNT);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return Math.max(0.0, HYBRID_PREBOOST2_AMOUNT);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return Math.max(0.0, HYBRID_PREBOOST3_AMOUNT);
        }
        return 0.0;
    }

    private long getHybridContactWindowHalfWidthMs() {
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL1_CONTACT) {
            return Math.max(0L, HYBRID_BALL1_CONTACT_WINDOW_HALF_WIDTH_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL2_CONTACT) {
            return Math.max(0L, HYBRID_BALL2_CONTACT_WINDOW_HALF_WIDTH_MS);
        }
        if (hybridShotFeedBoostPhase == HybridShotFeedBoostPhase.WAIT_BALL3_CONTACT) {
            return Math.max(0L, HYBRID_BALL3_CONTACT_WINDOW_HALF_WIDTH_MS);
        }
        return 0L;
    }

    private boolean isInHybridWindow(long valueMs, long centerMs, long halfWidthMs) {
        return Math.abs(valueMs - centerMs) <= Math.max(0L, halfWidthMs);
    }

    private boolean isShooterReadyForFeed(double toleranceRpm, double rpmShooter1, double rpmShooter2) {
        double targetRpm = ShooterSubsystem.INSTANCE.getTargetRpm();
        if (targetRpm < SHOOT_GATE_MIN_TARGET_RPM) {
            return false;
        }
        double averageShooterRpm = 0.5 * (rpmShooter1 + rpmShooter2);
        return Math.abs(targetRpm - averageShooterRpm) <= toleranceRpm;
    }

    private void labelShotTuningSample(String label, String labelReason) {
        if (!ENABLE_SHOT_TUNING_LOGGING || shotTuningLogger == null) return;
        if (!shotTuningPendingLabel) return;

        long nowMs = System.currentTimeMillis();
        long matchT = (logStartMs == 0L) ? 0L : (nowMs - logStartMs);
        long sinceFireMs = nowMs - shotTuningPendingFireMs;
        double rpm1Now = ShooterSubsystem.INSTANCE.getShooter1RPM();
        double rpm2Now = ShooterSubsystem.INSTANCE.getShooter2RPM();
        shotTuningLogger.addRow(
                nowMs,
                matchT,
                shotTuningPendingShotId,
                label,
                labelReason,
                shotTuningPendingFireMs,
                sinceFireMs,
                shotTuningPendingStartBallCount,
                shotTuningPendingTargetX,
                shotTuningPendingTargetY,
                shotTuningPendingBotX,
                shotTuningPendingBotY,
                shotTuningPendingBotHeadingDeg,
                shotTuningPendingOdoDistance,
                shotTuningPendingTargetRpm,
                shotTuningPendingHoodPos,
                shotTuningPendingRpm1,
                shotTuningPendingRpm2,
                targetRPM,
                shooterHoodPos,
                rpm1Now,
                rpm2Now,
                shotTuningPendingTurretTargetDeg,
                shotTuningPendingTurretMeasuredDeg,
                TurretSubsystem.INSTANCE.getTargetAngleDegrees(),
                TurretSubsystem.INSTANCE.getMeasuredAngleDegrees(),
                ShooterSubsystem.INSTANCE.boostActive,
                ShooterSubsystem.INSTANCE.secondBoostActive,
                ShooterSubsystem.INSTANCE.boostOverride
        );

        shotTuningPendingLabel = false;
    }

    private int startDumbShootRpmLogSequence(
            long nowMs,
            int expectedShots,
            boolean bb0,
            boolean bb1,
            boolean bb2,
            int linkedShotInfoSequenceId
    ) {
        if (!ENABLE_DUMBSHOOT_RPM_LOGGING || dumbShootRpmLogger == null) return -1;
        dumbShootRpmSequenceCounter++;
        dumbShootRpmLogSequenceId = dumbShootRpmSequenceCounter;
        dumbShootRpmLogStartMs = nowMs;
        dumbShootRpmLogExpectedShots = Math.max(0, expectedShots);
        dumbShootRpmLinkedShotInfoSequenceId = linkedShotInfoSequenceId;
        dumbShootRpmBurstProfileId = BURST_PROFILE_ID;
        dumbShootRpmBb0RiseCount = 0;
        dumbShootRpmBb0FallCount = 0;
        dumbShootRpmBb1RiseCount = 0;
        dumbShootRpmBb1FallCount = 0;
        dumbShootRpmBb2RiseCount = 0;
        dumbShootRpmBb2FallCount = 0;
        dumbShootRpmLogActive = true;
        dumbShootRpmPrevBbInitialized = true;
        dumbShootRpmPrevBb0 = bb0;
        dumbShootRpmPrevBb1 = bb1;
        dumbShootRpmPrevBb2 = bb2;
        lastDumbShootRpmLogMs = 0L;
        return dumbShootRpmLogSequenceId;
    }

    private void maybeLogDumbShootRpmSample(
            long nowMs,
            boolean bb0,
            boolean bb1,
            boolean bb2,
            double rpmShooter1,
            double rpmShooter2
    ) {
        if (!ENABLE_DUMBSHOOT_RPM_LOGGING || dumbShootRpmLogger == null) return;
        if (!dumbShootRpmLogActive) return;

        long sinceStartMs = nowMs - dumbShootRpmLogStartMs;
        if (sinceStartMs > Math.max(0L, DUMBSHOOT_RPM_LOG_DURATION_MS)) {
            dumbShootRpmLogActive = false;
            return;
        }

        if (lastDumbShootRpmLogMs > 0L &&
                !DUMBSHOOT_RPM_LOG_EVERY_LOOP &&
                nowMs - lastDumbShootRpmLogMs < Math.max(1L, DUMBSHOOT_RPM_LOG_PERIOD_MS)) {
            return;
        }
        lastDumbShootRpmLogMs = nowMs;

        boolean bb0RiseEdge = false;
        boolean bb0FallEdge = false;
        boolean bb1RiseEdge = false;
        boolean bb1FallEdge = false;
        boolean bb2RiseEdge = false;
        boolean bb2FallEdge = false;
        if (dumbShootRpmPrevBbInitialized) {
            bb0RiseEdge = !dumbShootRpmPrevBb0 && bb0;
            bb0FallEdge = dumbShootRpmPrevBb0 && !bb0;
            bb1RiseEdge = !dumbShootRpmPrevBb1 && bb1;
            bb1FallEdge = dumbShootRpmPrevBb1 && !bb1;
            bb2RiseEdge = !dumbShootRpmPrevBb2 && bb2;
            bb2FallEdge = dumbShootRpmPrevBb2 && !bb2;
        }
        if (bb0RiseEdge) dumbShootRpmBb0RiseCount++;
        if (bb0FallEdge) dumbShootRpmBb0FallCount++;
        if (bb1RiseEdge) dumbShootRpmBb1RiseCount++;
        if (bb1FallEdge) dumbShootRpmBb1FallCount++;
        if (bb2RiseEdge) dumbShootRpmBb2RiseCount++;
        if (bb2FallEdge) dumbShootRpmBb2FallCount++;
        dumbShootRpmPrevBb0 = bb0;
        dumbShootRpmPrevBb1 = bb1;
        dumbShootRpmPrevBb2 = bb2;
        dumbShootRpmPrevBbInitialized = true;

        long matchT = (logStartMs == 0L) ? 0L : (nowMs - logStartMs);
        double shooterAvgRpm = 0.5 * (rpmShooter1 + rpmShooter2);
        double loopTimeMs = timer.getMs();
        dumbShootRpmLogger.addRow(
                nowMs,
                matchT,
                dumbShootRpmLogSequenceId,
                dumbShootRpmLinkedShotInfoSequenceId,
                dumbShootRpmBurstProfileId,
                sinceStartMs,
                dumbShootRpmLogExpectedShots,
                dumbShootTimerActive,
                ShooterSubsystem.INSTANCE.getTargetRpm(),
                rpmShooter1,
                rpmShooter2,
                shooterAvgRpm,
                ShooterSubsystem.INSTANCE.getAverageRpmDelta(),
                ShooterSubsystem.INSTANCE.getShooter1Power(),
                ShooterSubsystem.INSTANCE.getShooter2Power(),
                isShooterReadyForFeed(
                        SHOOT_GATE_AT_SPEED_TOLERANCE_RPM,
                        rpmShooter1,
                        rpmShooter2
                ),
                ShooterSubsystem.INSTANCE.getBatteryVoltageRaw(),
                ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered(),
                ShooterSubsystem.INSTANCE.getVoltageCompGain(),
                ShooterSubsystem.INSTANCE.getCommandPreVoltageComp(),
                ShooterSubsystem.INSTANCE.getCommandPostVoltageComp(),
                ShooterSubsystem.INSTANCE.isCommandSaturated(),
                ShooterSubsystem.INSTANCE.boostActive,
                ShooterSubsystem.INSTANCE.secondBoostActive,
                ShooterSubsystem.INSTANCE.boostOverride,
                ShooterSubsystem.INSTANCE.getActiveBoostMultiplier1(),
                ShooterSubsystem.INSTANCE.getActiveBoostMultiplier2(),
                hybridShotFeedBoostActive,
                hybridShotFeedBoostPhase.name(),
                hybridShotFeedBoostActive ? (nowMs - hybridShotFeedStartMs) : -1L,
                getHybridExpectedContactMsForCurrentPhase(),
                ShooterSubsystem.INSTANCE.getActivePreBoostAmount(),
                hybridLastAdvanceReason,
                hybridLastAdvanceAtRelMs,
                bb0,
                bb1,
                bb2,
                bb0RiseEdge,
                bb0FallEdge,
                bb1RiseEdge,
                bb1FallEdge,
                bb2RiseEdge,
                bb2FallEdge,
                dumbShootRpmBb0RiseCount,
                dumbShootRpmBb0FallCount,
                dumbShootRpmBb1RiseCount,
                dumbShootRpmBb1FallCount,
                dumbShootRpmBb2RiseCount,
                dumbShootRpmBb2FallCount,
                IntakeWithSensorsSubsystem.INSTANCE.getBallCount(),
                loopTimeMs
        );
    }

    private void startShotInfoSequence(
            String reason,
            long nowMs,
            int startBallCount,
            boolean bb0,
            boolean bb1,
            boolean bb2
    ) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;
        if (shotLogSequenceActive) return;

        shotLogSequenceActive = true;
        shotSequenceStartMs = nowMs;
        shotSequenceId = ++shotSequenceIdCounter;
        shotSequenceStartBallCount = Math.max(0, startBallCount);
        shotSequenceExpectedShots = Math.max(1, Math.min(3, shotSequenceStartBallCount));
        shotSequenceReason = reason;
        shotSequenceBurstProfileId = BURST_PROFILE_ID;
        shotSequenceStartTargetRpm = ShooterSubsystem.INSTANCE.getTargetRpm();
        shotSequenceStartHoodPos = shooterHoodPos;
        shotSequenceStartBoostDelayMs = ShooterSubsystem.BOOST_DELAY_MS;
        shotSequenceStartPreBoostAmount =
                ENABLE_HYBRID_SHOT_FEED_BOOST
                        ? Math.max(0.0, HYBRID_PREBOOST1_AMOUNT)
                        : 0.0;
        shotSequenceStartBoostMult1 = ShooterSubsystem.INSTANCE.getActiveBoostMultiplier1();
        shotSequenceStartBoostMult2 = ShooterSubsystem.INSTANCE.getActiveBoostMultiplier2();
        shotSequenceLinkedDumbShootRpmSequenceId = -1;
        shotSequenceStartBb0 = bb0;
        shotSequenceStartBb1 = bb1;
        shotSequenceStartBb2 = bb2;
        shotBb0FallCount = 0;
        shotBb0RiseCount = 0;
        shotBb1FallCount = 0;
        shotBb1RiseCount = 0;
        shotBb2FallCount = 0;
        shotBb2RiseCount = 0;
        shotBb2ClearLoopCountTotal = 0;
        shotBb2ClearStreakCurrent = 0;
        shotBb2ClearStreakMax = 0;

        for (int i = 0; i < 3; i++) {
            shotBb0FallMs[i] = -1L;
            shotBb0RiseMs[i] = -1L;
            shotBb1FallMs[i] = -1L;
            shotBb1RiseMs[i] = -1L;
            shotBb2FallMs[i] = -1L;
            shotBb2RiseMs[i] = -1L;
            shotIntervalMs[i] = -1L;
        }
    }

    private void updateShotInfoBreakbeamTracking(long nowMs, boolean bb0, boolean bb1, boolean bb2) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;

        if (!bbPrevInitializedForShotLog) {
            prevBb0ForShotLog = bb0;
            prevBb1ForShotLog = bb1;
            prevBb2ForShotLog = bb2;
            bbPrevInitializedForShotLog = true;
            return;
        }

        if (!shotLogSequenceActive) {
            prevBb0ForShotLog = bb0;
            prevBb1ForShotLog = bb1;
            prevBb2ForShotLog = bb2;
            return;
        }

        long relMs = nowMs - shotSequenceStartMs;

        // Loop-level "clear beam" observability for sensor2.
        // bb2 == true means broken; clear/unbroken is !bb2.
        if (!bb2) {
            shotBb2ClearLoopCountTotal++;
            shotBb2ClearStreakCurrent++;
            if (shotBb2ClearStreakCurrent > shotBb2ClearStreakMax) {
                shotBb2ClearStreakMax = shotBb2ClearStreakCurrent;
            }
        } else {
            shotBb2ClearStreakCurrent = 0;
        }

        if (!prevBb0ForShotLog && bb0) {
            if (shotBb0RiseCount < 3) shotBb0RiseMs[shotBb0RiseCount] = relMs;
            shotBb0RiseCount++;
        } else if (prevBb0ForShotLog && !bb0) {
            if (shotBb0FallCount < 3) shotBb0FallMs[shotBb0FallCount] = relMs;
            shotBb0FallCount++;
        }

        if (!prevBb1ForShotLog && bb1) {
            if (shotBb1RiseCount < 3) shotBb1RiseMs[shotBb1RiseCount] = relMs;
            shotBb1RiseCount++;
        } else if (prevBb1ForShotLog && !bb1) {
            if (shotBb1FallCount < 3) shotBb1FallMs[shotBb1FallCount] = relMs;
            shotBb1FallCount++;
        }

        if (!prevBb2ForShotLog && bb2) {
            if (shotBb2RiseCount < 3) shotBb2RiseMs[shotBb2RiseCount] = relMs;
            shotBb2RiseCount++;
        } else if (prevBb2ForShotLog && !bb2) {
            if (shotBb2FallCount < 3) shotBb2FallMs[shotBb2FallCount] = relMs;
            shotBb2FallCount++;
            if (shotBb2FallCount == 1) {
                shotIntervalMs[0] = shotBb2FallMs[0];
            } else if (shotBb2FallCount == 2) {
                shotIntervalMs[1] = shotBb2FallMs[1] - shotBb2FallMs[0];
            } else if (shotBb2FallCount == 3) {
                shotIntervalMs[2] = shotBb2FallMs[2] - shotBb2FallMs[1];
            }
        }

        prevBb0ForShotLog = bb0;
        prevBb1ForShotLog = bb1;
        prevBb2ForShotLog = bb2;
    }

    private void maybeFinalizeShotInfoSequence(long nowMs, boolean rightTriggerActive, boolean leftTriggerActive) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;
        if (!shotLogSequenceActive) return;

        long elapsed = nowMs - shotSequenceStartMs;
        long lastBb2FallRelMs = shotBb2FallCount > 0
                ? shotBb2FallMs[Math.min(shotBb2FallCount, 3) - 1]
                : -1L;
        long sinceLastBb2FallMs = lastBb2FallRelMs >= 0 ? (elapsed - lastBb2FallRelMs) : Long.MAX_VALUE;
        boolean expectedShotsSeen = shotBb2FallCount >= shotSequenceExpectedShots;
        boolean expectedShotsSettled =
                expectedShotsSeen &&
                sinceLastBb2FallMs >= Math.max(0L, SHOT_INFO_EXPECTED_SHOTS_FINALIZE_GRACE_MS);

        // For dumbshoot: avoid early "idle_end" finalization, which can truncate
        // breakbeam capture before all expected transitions occur.
        if ("dumbshoot".equals(shotSequenceReason)) {
            boolean minCaptureSatisfied =
                    elapsed >= Math.max(0L, SHOT_INFO_DUMBSHOOT_MIN_CAPTURE_MS);
            if (expectedShotsSettled && minCaptureSatisfied) {
                finalizeShotInfoSequence("expected_shots_seen", nowMs);
                return;
            }

            // If two shots were detected, prefer a short targeted window for shot3.
            if (shotSequenceExpectedShots >= 3 && shotBb2FallCount >= 2 && shotBb2FallMs[1] >= 0L) {
                long sinceSecondShotMs = elapsed - shotBb2FallMs[1];
                if (minCaptureSatisfied &&
                        sinceSecondShotMs >= Math.max(0L, SHOT_INFO_DUMBSHOOT_AFTER_SECOND_TIMEOUT_MS)) {
                    finalizeShotInfoSequence("third_missing_after_second", nowMs);
                    return;
                }
            }

            if (elapsed >= Math.max(0L, SHOT_INFO_DUMBSHOOT_TIMEOUT_MS)) {
                finalizeShotInfoSequence("dumbshoot_timeout", nowMs);
            }
            return;
        }

        if (expectedShotsSettled) {
            finalizeShotInfoSequence("expected_shots_seen", nowMs);
            return;
        }

        if (elapsed >= SHOT_INFO_LOG_TIMEOUT_MS) {
            finalizeShotInfoSequence("timeout", nowMs);
            return;
        }

        // If triggers are released and intake/shoot modes are idle, finalize early after short grace.
        boolean intakeBusy =
                IntakeWithSensorsSubsystem.INSTANCE.isSingleBallFeedActive() ||
                        IntakeWithSensorsSubsystem.INSTANCE.isMultiSingleShotActive() ||
                        IntakeWithSensorsSubsystem.INSTANCE.isShootSequenceActive() ||
                        IntakeWithSensorsSubsystem.INSTANCE.isShooting();
        if (!rightTriggerActive && !leftTriggerActive && !intakeBusy && elapsed > 300) {
            finalizeShotInfoSequence("idle_end", nowMs);
        }
    }

    private void finalizeShotInfoSequence(String endReason, long nowMs) {
        if (!ENABLE_SHOT_INFO_LOGGING || shotInfoLogger == null) return;
        if (!shotLogSequenceActive) return;

        long matchT = (logStartMs == 0L) ? 0L : (nowMs - logStartMs);
        long durationMs = nowMs - shotSequenceStartMs;
        long bb2ClearGap12Ms = (shotBb2RiseMs[0] >= 0L && shotBb2FallMs[1] >= 0L)
                ? (shotBb2FallMs[1] - shotBb2RiseMs[0])
                : -1L;
        long bb2ClearGap23Ms = (shotBb2RiseMs[1] >= 0L && shotBb2FallMs[2] >= 0L)
                ? (shotBb2FallMs[2] - shotBb2RiseMs[1])
                : -1L;

        shotInfoLogger.addRow(
                nowMs,
                matchT,
                shotSequenceId,
                shotSequenceReason,
                shotSequenceStartBallCount,
                shotSequenceExpectedShots,
                shotSequenceStartBb0,
                shotSequenceStartBb1,
                shotSequenceStartBb2,
                shotBb0FallMs[0],
                shotBb0RiseMs[0],
                shotBb0FallMs[1],
                shotBb0RiseMs[1],
                shotBb0FallMs[2],
                shotBb0RiseMs[2],
                shotBb1FallMs[0],
                shotBb1RiseMs[0],
                shotBb1FallMs[1],
                shotBb1RiseMs[1],
                shotBb1FallMs[2],
                shotBb1RiseMs[2],
                shotBb2FallMs[0],
                shotBb2RiseMs[0],
                shotBb2FallMs[1],
                shotBb2RiseMs[1],
                shotBb2FallMs[2],
                shotBb2RiseMs[2],
                shotIntervalMs[0],
                shotIntervalMs[1],
                shotIntervalMs[2],
                bb2ClearGap12Ms,
                bb2ClearGap23Ms,
                shotBb2ClearLoopCountTotal,
                shotBb2ClearStreakMax,
                shotBb0FallCount,
                shotBb0RiseCount,
                shotBb1FallCount,
                shotBb1RiseCount,
                shotBb2FallCount,
                shotBb2RiseCount,
                endReason,
                durationMs,
                shotSequenceBurstProfileId,
                shotSequenceStartTargetRpm,
                shotSequenceStartHoodPos,
                shotSequenceStartBoostDelayMs,
                shotSequenceStartPreBoostAmount,
                shotSequenceStartBoostMult1,
                shotSequenceStartBoostMult2,
                shotSequenceLinkedDumbShootRpmSequenceId
        );

        shotLogSequenceActive = false;
        shotSequenceStartMs = 0L;
        shotSequenceStartBallCount = 0;
        shotSequenceExpectedShots = 0;
        shotSequenceReason = "";
        shotSequenceBurstProfileId = 0;
        shotSequenceStartTargetRpm = 0.0;
        shotSequenceStartHoodPos = 0.0;
        shotSequenceStartBoostDelayMs = 0L;
        shotSequenceStartPreBoostAmount = 0.0;
        shotSequenceStartBoostMult1 = 0.0;
        shotSequenceStartBoostMult2 = 0.0;
        shotSequenceLinkedDumbShootRpmSequenceId = -1;
    }

    private static class SOTMResult {
        final boolean valid;
        final boolean leadApplied;
        final double speedInPerSec;
        final double totalTimeSeconds;
        final double leadXInches;
        final double leadYInches;
        final double radialVelocityInPerSec;
        final double effectiveDistanceInches;
        final double distanceForBallisticsInches;
        final double turretRobotRelativeAimDeg;
        final double turretLagCompensationDeg;

        SOTMResult(
                boolean valid,
                boolean leadApplied,
                double speedInPerSec,
                double totalTimeSeconds,
                double leadXInches,
                double leadYInches,
                double radialVelocityInPerSec,
                double effectiveDistanceInches,
                double distanceForBallisticsInches,
                double turretRobotRelativeAimDeg,
                double turretLagCompensationDeg
        ) {
            this.valid = valid;
            this.leadApplied = leadApplied;
            this.speedInPerSec = speedInPerSec;
            this.totalTimeSeconds = totalTimeSeconds;
            this.leadXInches = leadXInches;
            this.leadYInches = leadYInches;
            this.radialVelocityInPerSec = radialVelocityInPerSec;
            this.effectiveDistanceInches = effectiveDistanceInches;
            this.distanceForBallisticsInches = distanceForBallisticsInches;
            this.turretRobotRelativeAimDeg = turretRobotRelativeAimDeg;
            this.turretLagCompensationDeg = turretLagCompensationDeg;
        }
    }

    private SOTMResult calculateSotmResult(
            double botX,
            double botY,
            double botHeadingRad,
            double targetX,
            double targetY,
            double vxInPerSec,
            double vyInPerSec,
            double speedInPerSec,
            double omegaRadPerSec,
            boolean sotmControlActive
    ) {
        double dx = targetX - botX;
        double dy = targetY - botY;
        double realDistance = Math.hypot(dx, dy);
        if (!Double.isFinite(realDistance) || realDistance < 1e-6) {
            return new SOTMResult(
                    false,
                    false,
                    speedInPerSec,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    minDisatanceForShooting,
                    0.0,
                    0.0
            );
        }

        double totalTimeSec = getSotmTotalTimeSeconds(realDistance);
        double leadX = 0.0;
        double leadY = 0.0;
        boolean leadApplied = sotmControlActive && speedInPerSec > SOTM_MIN_SPEED_IN_PER_SEC;
        if (leadApplied) {
            leadX = vxInPerSec * totalTimeSec * SOTM_LEAD_GAIN;
            leadY = vyInPerSec * totalTimeSec * SOTM_LEAD_GAIN;
            double leadMagnitude = Math.hypot(leadX, leadY);
            if (leadMagnitude > SOTM_MAX_LEAD_IN && leadMagnitude > 1e-6) {
                double scale = SOTM_MAX_LEAD_IN / leadMagnitude;
                leadX *= scale;
                leadY *= scale;
            }
        }

        double virtualTargetX = targetX - leadX;
        double virtualTargetY = targetY - leadY;
        double virtualFieldAngleRad = Math.atan2(virtualTargetY - botY, virtualTargetX - botX);
        double turretRobotRelativeAimDeg = Math.toDegrees(normalizeRadians(virtualFieldAngleRad - botHeadingRad));
        if (leadApplied) {
            turretRobotRelativeAimDeg += Math.toDegrees(
                    omegaRadPerSec * totalTimeSec * SOTM_ANGULAR_LEAD_GAIN
            );
        }

        double turretLagCompensationDeg = 0.0;
        if (sotmControlActive && SOTM_TURRET_LAG_COMP_ENABLED) {
            turretLagCompensationDeg = Math.toDegrees(omegaRadPerSec) * SOTM_TURRET_LAG_COMP_SEC;
            double maxLagCompDeg = Math.max(0.0, SOTM_TURRET_LAG_COMP_MAX_DEG);
            turretLagCompensationDeg = Math.max(
                    -maxLagCompDeg,
                    Math.min(maxLagCompDeg, turretLagCompensationDeg)
            );
            turretRobotRelativeAimDeg += turretLagCompensationDeg;
        }

        // Keep effective distance tied to real target vector (Stage 3 guidance, avoids double-counting).
        double radialVelocityInPerSec = (vxInPerSec * dx + vyInPerSec * dy) / realDistance;
        double effectiveDistance = Math.max(0.0, realDistance - (radialVelocityInPerSec * totalTimeSec));
        double distanceForBallistics = leadApplied
                ? Math.max(minDisatanceForShooting, effectiveDistance)
                : realDistance;

        boolean valid =
                Double.isFinite(totalTimeSec) &&
                Double.isFinite(leadX) &&
                Double.isFinite(leadY) &&
                Double.isFinite(radialVelocityInPerSec) &&
                Double.isFinite(effectiveDistance) &&
                Double.isFinite(distanceForBallistics) &&
                Double.isFinite(turretRobotRelativeAimDeg);

        return new SOTMResult(
                valid,
                leadApplied,
                speedInPerSec,
                totalTimeSec,
                leadX,
                leadY,
                radialVelocityInPerSec,
                effectiveDistance,
                distanceForBallistics,
                turretRobotRelativeAimDeg,
                turretLagCompensationDeg
        );
    }

    private double getSotmTotalTimeSeconds(double distanceInches) {
        return SOTM_MECHANICAL_DELAY_SEC +
                SOTM_BALL_TRANSFER_TIME_SEC +
                getSotmFlightTimeSeconds(distanceInches);
    }

    private double getSotmFlightTimeSeconds(double distanceInches) {
        if (SOTM_USE_TOF_LOOKUP) {
            return interpolateSotmLookup(
                    distanceInches,
                    SOTM_TOF_DISTANCE_IN,
                    SOTM_TOF_FLIGHT_TIME_SEC
            );
        }
        return distanceInches / Math.max(1e-6, SOTM_ESTIMATED_BALL_SPEED_IN_PER_SEC);
    }

    private static double interpolateSotmLookup(double x, double[] xs, double[] ys) {
        if (xs.length == 0 || ys.length == 0 || xs.length != ys.length) return 0.0;
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];

        for (int i = 0; i < last; i++) {
            double x0 = xs[i];
            double x1 = xs[i + 1];
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / Math.max(1e-6, x1 - x0);
                return ys[i] + (t * (ys[i + 1] - ys[i]));
            }
        }
        return ys[last];
    }

    private double updateSotmFilteredOmega(double rawOmegaRadPerSec) {
        double alpha = Math.max(0.0, Math.min(1.0, SOTM_OMEGA_FILTER_ALPHA));
        if (!sotmOmegaFilterInitialized) {
            sotmFilteredOmegaRadPerSec = rawOmegaRadPerSec;
            sotmOmegaFilterInitialized = true;
            return sotmFilteredOmegaRadPerSec;
        }
        sotmFilteredOmegaRadPerSec =
                (alpha * rawOmegaRadPerSec) + ((1.0 - alpha) * sotmFilteredOmegaRadPerSec);
        return sotmFilteredOmegaRadPerSec;
    }

    private static double robotFrontRelativeToTurretUnwrappedDegrees(double robotFrontRelativeDegrees) {
        return (TurretSubsystem.ROBOT_FRONT_RELATIVE_SIGN * robotFrontRelativeDegrees) +
                TurretSubsystem.ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES;
    }

    private static double smallestWrappedDeltaDegrees(double targetDegrees, double currentDegrees) {
        double delta = targetDegrees - currentDegrees;
        while (delta > 180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;
        return delta;
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public static double calculateShooterRPMOdoDistance(double odoDistance) {
        //odoDistance > 110 ? 16.9425 * odoDistance + 1990.45 :
//        return  odoDistance > 110 ? 16.9425 * odoDistance + 1990.45 : 16.9425 * odoDistance + 1984.45;
        return  odoDistance > 110 ? 16.9425 * odoDistance + 1950.45 : 16.6925 * odoDistance + 2010.45;
    }

    public static double calculateShooterHoodOdoDistance(double odoDistance) {
        double hoodPos = 0.00585 * odoDistance - 0.14;

        if (hoodPos < 0) return 0;
        if (hoodPos > 0.5) return 0.5;
        return hoodPos;
    }
    //OLD STUFF
    public static double calculateShooterRPM(double yOffset) {
        if (yOffset > 17)
            return 3350;

        double rpm = 0;
        if (yOffset >= 17 && yOffset < 18) {
            rpm = -117.65 * yOffset + 5385.3;
        }
        else if (yOffset >= 16 && yOffset < 17) {
            rpm = 23.685 * yOffset + 3022.5;
        }
        else if (yOffset >= 15 && yOffset < 16) {
            rpm = 26.838 * Math.pow(yOffset, 2) -
                    871.7 * yOffset + 10466;
        }
        else if (yOffset >= 14 && yOffset < 15) {
            rpm = -127.72 * yOffset + 5433.5;

        }
        else if (yOffset >= 13 && yOffset < 14) {
            rpm = -152.89 * yOffset + 5697.3;
        }
        else if (yOffset >= 12 && yOffset < 13) {
            rpm = -167.96 * yOffset + 5830.5;
        }
        else if (yOffset >= 11 && yOffset < 12) {
            rpm = -14.425 * yOffset + 3956.9;
        }
        else if (yOffset >= 10 && yOffset < 11) {
            rpm = -89.495 * yOffset + 4836.9;
        }
        else if (yOffset >= 9 && yOffset < 10) {
            rpm = 4000;
        }
        else{
            rpm = 4250;
        }

        return rpm;
    }

    public static double getHoodPosition(double yOffset) {

        if (yOffset >= 17.08) {
            return 0.3;
        }

        // Small 0.3 dip around 16.14
        if (yOffset >= 16.3 && yOffset <= 16.) {
            return 0.3;
        }

        // Small 0.3 dip around 14.18
        if (yOffset >= 14.3 && yOffset <= 14.) {
            return 0.3;
        }

        // --- 0.5 region (12.3 → 12.0 generalized) ---
        if (yOffset > 12.0 && yOffset < 12.9) {
            return 0.5;
        }

        // --- Main 0.4 region (everything else inside your measured range) ---
        if (yOffset >= 9.6) {
            return 0.4;
        }

        if (yOffset >= 10 && yOffset < 11) {
            return 0.5;
        }

        if (yOffset >= 11 && yOffset < 12) {
            return 0.5;
        }

        if (yOffset <= 10) {
            return 0.5;
        }

        // --- Below recorded range, assume last known ---
        return 0.4;
    }

    public static double getHoodPositionFromDistance(double distanceIn) {

        double distanceMm = distanceIn * 25.4;
        // Default hood position if distance is outside known range
        double hoodPos = 0.35;

        if (distanceMm <= 1379) {          // 1282–1379 mm
            hoodPos = 0.2;
        } else if (distanceMm <= 1637) {   // 1626–1637 mm
            hoodPos = 0.3;
        } else if (distanceMm <= 1910) {   // 1850–1910 mm
            hoodPos = 0.4;
        } else if (distanceMm <= 2299) {   // 2105–2299 mm
            hoodPos = 0.3;
        } else if (distanceMm <= 2552) {   // 2552 mm
            hoodPos = 0.4;
        } else if (distanceMm > 2552) {    // 2945 mm
            hoodPos = 0.5;
        }

        return hoodPos;
    }
}



