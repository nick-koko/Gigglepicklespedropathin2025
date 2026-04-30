package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.GlobalRobotData;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.CsvLogger;

import java.io.File;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
//@Autonomous(name = "closeBlueSide", group = "Comp")
public class farAutonPaths_Worlds extends NextFTCOpMode{
    public static double autonShooterRPM = 4100;
    public static double autonShooterHoodServoPos = 0.525;
    public static double pickupBrakingStrength = 1.0;
	public static double pickupCornerBrakingStrength = 0.1;
    public static boolean ENABLE_AUTON_LOGGING = true;
    public static long AUTON_LOG_PERIOD_MS = 25L;
    public static double AUTON_SHOOTER_AT_SPEED_TOLERANCE_RPM = 75.0;
    public static double redTargetPoseXOffset = -5.0;
    public static double redTargetPoseYOffset = 0.0;
    public static long AUTON_FAR_NO_BOOST_DUMBSHOOT_DELAY_MS = 100L;
    public static double AUTON_FAR_NO_BOOST_SHOOT_DURATION_SEC = 0.2;
    public long LIMELIGHT_MISSING_LED_STROBE_MS = 250L;

    private CsvLogger autonLogger;
    private long autonLogStartMs = 0L;
    private long lastAutonLogMs = 0L;
    private long lastAutonLoopMs = 0L;


    public final Pose startPoseBlue = new Pose(64, 8.5, Math.toRadians(180)); // Start Pose of our robot
    private final Pose scorePoseCloseBlue = new Pose(56.5, 20.5, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose targetPoseCloseBlue = new Pose(0.0, 144.0, Math.toRadians(180));

    private final Pose pickup1PoseBlue = new Pose(40, 33.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose pickup1CP1Blue = new Pose(53, 32, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CP2Blue = new Pose( 46, 32, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup1EndPoseBlue = new Pose( 22, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup1CPPathBlue = new Pose(60, 40, Math.toRadians(180));

    private final Pose pickup2PoseBlue = new Pose(40, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP1Blue = new Pose(54, 41.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2CP2Blue = new Pose( 52.0, 59, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2EndPoseBlue = new Pose(23.5, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right

    private final Pose pickup2GateLeverEndPoseBlue = new Pose(19, 60.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2GateLeverPushPoseBlue = new Pose(23, 73.0, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2GateLeverPushCP1Blue = new Pose(22.5, 57, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2GateLeverPushCP2Blue = new Pose(28.5, 69.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2GateLeverPushHoldPoseBlue = new Pose(17, 70.0, Math.toRadians(180));
    private final Pose getPickup2CPPathBlue = new Pose(61, 58, Math.toRadians(180));

    private final Pose pickup3PoseBlue = new Pose(40, 39.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP1Blue = new Pose(37,90  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup3CP2Blue = new Pose( 57, 39, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3EndPoseBlue = new Pose(20, 35.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickup3CPPathBlue = new Pose(40, 45, Math.toRadians(180));

    private final Pose pickupBZonePoseBlue = new Pose(29.5, 11, Math.toRadians(190)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupBZoneCP1Blue = new Pose(53,16, Math.toRadians(190)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupBZoneCP2Blue = new Pose( 46, 12, Math.toRadians(190)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickupBZoneEndPoseBlue = new Pose(12, 10, Math.toRadians(190)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickupBZoneCPPathBlue = new Pose(44, 59, Math.toRadians(235));

    private final Pose pickupTZonePoseBlue = new Pose(9, 46, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupTZoneCP1Blue = new Pose(42,55.5, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupTZoneCP2Blue = new Pose( 8, 64.5, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickupTZonePreEndPoseBlue = new Pose(9, 37, Math.toRadians(235)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right

    private final Pose pickupTZoneEndPoseBlue = new Pose(8, 13, Math.toRadians(255)); // Highest (First Set) of Artifacts from the Spike Mark. //move +3 Y to the right
    private final Pose getPickupTZoneCPPathBlue = new Pose(44, 59, Math.toRadians(255));


    private final Pose offLineLeverBlue = new Pose(30, 20, Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose offLineCloseBlue = new Pose(47, 27, Math.toRadians(180));

    public final Pose startPoseRed = startPoseBlue.mirror(144);
    private final Pose scorePoseCloseRed = scorePoseCloseBlue.mirror(144);
    private final Pose targetPoseCloseRed = targetPoseCloseBlue.mirror(144);

    private final Pose pickup1PoseRed = pickup1PoseBlue.mirror(144);
    private final Pose pickup1CP1Red = pickup1CP1Blue.mirror(144);
    private final Pose pickup1CP2Red = pickup1CP2Blue.mirror(144);
    private final Pose pickup1EndPoseRed = pickup1EndPoseBlue.mirror(144);
    private final Pose pickup2PoseRed = pickup2PoseBlue.mirror(144);
    private final Pose pickup2CP1Red = pickup2CP1Blue.mirror(144);
    private final Pose pickup2CP2Red = pickup2CP2Blue.mirror(144);
    private final Pose pickup2EndPoseRed = pickup2EndPoseBlue.mirror(144);
    private final Pose pickup2GateLeverEndPoseRed = pickup2GateLeverEndPoseBlue.mirror(144);
    private final Pose pickup2GateLeverPushPoseRed = pickup2GateLeverPushPoseBlue.mirror(144);
    private final Pose pickup2GateLeverPushHoldPoseRed = pickup2GateLeverPushHoldPoseBlue.mirror(144);
    private final Pose pickup2GateLeverPushCP1Red = pickup2GateLeverPushCP1Blue.mirror(144);
    private final Pose pickup2GateLeverPushCP2Red = pickup2GateLeverPushCP2Blue.mirror(144);
    private final Pose pickup3PoseRed = pickup3PoseBlue.mirror(144);
    private final Pose pickup3CP1Red = pickup3CP1Blue.mirror(144);
    private final Pose pickup3CP2Red = pickup3CP2Blue.mirror(144);
    private final Pose pickup3EndPoseRed = pickup3EndPoseBlue.mirror(144);
    private final Pose getPickup3CPPathRed = getPickup3CPPathBlue.mirror(144);
    private final Pose pickupBZonePoseRed = pickupBZonePoseBlue.mirror(144);
    private final Pose pickupBZoneCP1Red = pickupBZoneCP1Blue.mirror(144);
    private final Pose pickupBZoneCP2Red = pickupBZoneCP2Blue.mirror(144);
    private final Pose pickupBZoneEndPoseRed = pickupBZoneEndPoseBlue.mirror(144);
    private final Pose getPickup1CPPathRed = getPickup1CPPathBlue.mirror(144);
    private final Pose getPickup2CPPathRed = getPickup2CPPathBlue.mirror(144);
    private final Pose getPickupBZoneCPPathRed = getPickupBZoneCPPathBlue.mirror(144);

    private final Pose pickupTZonePoseRed = pickupTZonePoseBlue.mirror(144);
    private final Pose pickupTZoneCP1Red = pickupTZoneCP1Blue.mirror(144);
    private final Pose pickupTZoneCP2Red = pickupTZoneCP2Blue.mirror(144);
    private final Pose pickupTZoneEndPoseRed = pickupTZoneEndPoseBlue.mirror(144);
    private final Pose pickupTZonePreEndPoseRed = pickupTZonePreEndPoseBlue.mirror(144);
    private final Pose getPickupTZoneCPPathRed = getPickupTZoneCPPathBlue.mirror(144);

    private final Pose offLineLeverRed = offLineLeverBlue.mirror(144);
    private final Pose offLineCloseRed = offLineCloseBlue.mirror(144);

    public Pose currentGoodPose = new Pose(0,0,0);

    public Pose startPose;
    public Pose scorePoseClose;
    public Pose targetPoseClose;

    public Pose pickup1Pose;
    public Pose pickup1CP1;
    public Pose pickup1CP2;
    public Pose getPickup1CPPath;
    public Pose pickup1EndPose;

    public Pose pickup2Pose;
    public Pose pickup2CP1;
    public Pose pickup2CP2;

    public Pose pickup2EndPose;
    public Pose getPickup2CPPath;

    public Pose pickup2GateLeverEndPose;

    public Pose pickup2GateLeverPushPose;
    public Pose pickup2GateLeverPushHoldPose;
    public Pose pickup2GateLeverPushCP1;
    public Pose pickup2GateLeverPushCP2;

    public Pose pickup3Pose;
    public Pose pickup3CP1;
    public Pose pickup3CP2;

    public Pose pickup3EndPose;
    public Pose getPickup3CPPath;

    public Pose pickupBZonePose;
    public Pose pickupBZoneCP1;
    public Pose pickupBZoneCP2;

    public Pose pickupBZoneEndPose;
    public Pose getPickupBZoneCPPath;

    public Pose pickupTZonePose;
    public Pose pickupTZoneCP1;
    public Pose pickupTZoneCP2;

    public Pose pickupTZonePreEndPose;

    public Pose pickupTZoneEndPose;
    public Pose getPickupTZoneCPPath;

    public Pose offLineLever;
    public Pose offLineClose;

    private PathChain firstshootpath, firstPickup,firstPickupEnd, secondPickup, secondPickupEnd, thirdPickup, thirdPickupEnd, TZonePickup, TZonePickupEnd, secondPickupGateLeverEnd, secondPickupGateLeverShootEnd, BZonePickup, BZonePickupEnd, moveOffLineLever, moveOffLineClose;

    protected void startAutonLogger() {
        autonLogStartMs = System.currentTimeMillis();
        lastAutonLogMs = 0L;
        lastAutonLoopMs = 0L;

        if (!ENABLE_AUTON_LOGGING) {
            autonLogger = null;
            return;
        }

        autonLogger = new CsvLogger(getClass().getSimpleName() + "_auton");
        autonLogger.start(
                "t_ms," +
                        "match_t_ms," +
                        "bot_x," +
                        "bot_y," +
                        "bot_heading_deg," +
                        "turret_target_deg," +
                        "turret_measured_deg," +
                        "shooter_target_rpm," +
                        "shooter_rpm1," +
                        "shooter_rpm2," +
                        "shooter_at_speed_75," +
                        "shooter_battery_v," +
                        "shooter_battery_v_filtered," +
                        "intake_active," +
                        "intake_direction," +
                        "intake_m1_rpm," +
                        "intake_m3_rpm," +
                        "ball_count," +
                        "loop_time_ms"
        );
    }

    protected void logAutonLoop() {
        long nowMs = System.currentTimeMillis();
        long loopTimeMs = (lastAutonLoopMs == 0L) ? 0L : (nowMs - lastAutonLoopMs);
        lastAutonLoopMs = nowMs;

        Pose pose = PedroComponent.follower().getPose();
        if ((pose.getX() == 0) && (pose.getY() == 0) && (pose.getHeading() == 0)) {
        } else {
            currentGoodPose = pose;
        }

        if (!ENABLE_AUTON_LOGGING || autonLogger == null) {
            return;
        }
        if (lastAutonLogMs != 0L && nowMs - lastAutonLogMs < AUTON_LOG_PERIOD_MS) {
            return;
        }
        lastAutonLogMs = nowMs;

        autonLogger.addRow(
                nowMs,
                (autonLogStartMs == 0L) ? 0L : (nowMs - autonLogStartMs),
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()),
                TurretSubsystem.INSTANCE.getTargetAngleDegrees(),
                TurretSubsystem.INSTANCE.getMeasuredAngleDegrees(),
                ShooterSubsystem.INSTANCE.getTargetRpm(),
                ShooterSubsystem.INSTANCE.getShooter1RPM(),
                ShooterSubsystem.INSTANCE.getShooter2RPM(),
                ShooterSubsystem.INSTANCE.isAtSpeed(AUTON_SHOOTER_AT_SPEED_TOLERANCE_RPM),
                ShooterSubsystem.INSTANCE.getBatteryVoltageRaw(),
                ShooterSubsystem.INSTANCE.getBatteryVoltageFiltered(),
                IntakeWithSensorsSubsystem.INSTANCE.isIntakingActive(),
                IntakeWithSensorsSubsystem.INSTANCE.getCurrentDirection(),
                IntakeWithSensorsSubsystem.INSTANCE.getMotor1RPM(),
                IntakeWithSensorsSubsystem.INSTANCE.getMotor3RPM(),
                IntakeWithSensorsSubsystem.INSTANCE.getBallCount(),
                loopTimeMs
        );
    }

    protected void saveAutonLogger() {
        if (!ENABLE_AUTON_LOGGING || autonLogger == null) {
            return;
        }

        File savedFile = autonLogger.save();
        if (savedFile != null) {
            RobotLog.ii(getClass().getSimpleName(), "Auton CSV saved: " + savedFile.getAbsolutePath());
        } else {
            RobotLog.ww(getClass().getSimpleName(), "Auton CSV was not saved.");
        }
    }

    private Pose withRedTargetPoseOffset(Pose targetPose) {
        return new Pose(
                targetPose.getX() + redTargetPoseXOffset,
                targetPose.getY() + redTargetPoseYOffset,
                targetPose.getHeading()
        );
    }

        public void buildPaths() {

            if (GlobalRobotData.allianceSide == GlobalRobotData.COLOR.BLUE) {
                startPose = startPoseBlue;
                scorePoseClose = scorePoseCloseBlue;
                targetPoseClose = targetPoseCloseBlue;
                pickup1Pose = pickup1PoseBlue;
                pickup1CP1 = pickup1CP1Blue;
                pickup1CP2 = pickup1CP2Blue;
                getPickup1CPPath = getPickup1CPPathBlue;
                pickup1EndPose = pickup1EndPoseBlue;
                pickup2Pose = pickup2PoseBlue;
                pickup2CP1 = pickup2CP1Blue;
                pickup2CP2 = pickup2CP2Blue;
                pickup2EndPose = pickup2EndPoseBlue;
                pickup2GateLeverEndPose = pickup2GateLeverEndPoseBlue;
                pickup2GateLeverPushPose = pickup2GateLeverPushPoseBlue;
                pickup2GateLeverPushHoldPose = pickup2GateLeverPushHoldPoseBlue;
                pickup2GateLeverPushCP1 = pickup2GateLeverPushCP1Blue;
                pickup2GateLeverPushCP2 = pickup2GateLeverPushCP2Blue;
                getPickup2CPPath = getPickup2CPPathBlue;
                pickup3Pose = pickup3PoseBlue;
                pickup3CP1 = pickup3CP1Blue;
                pickup3CP2 = pickup3CP2Blue;
                pickup3EndPose = pickup3EndPoseBlue;
                getPickup3CPPath = getPickup3CPPathBlue;
                pickupBZonePose = pickupBZonePoseBlue;
                pickupBZoneCP1 = pickupBZoneCP1Blue;
                pickupBZoneCP2 = pickupBZoneCP2Blue;
                pickupBZoneEndPose = pickupBZoneEndPoseBlue;
                getPickupBZoneCPPath = getPickupBZoneCPPathBlue;
                pickupTZonePose = pickupTZonePoseBlue;
                pickupTZoneCP1 = pickupTZoneCP1Blue;
                pickupTZoneCP2 = pickupTZoneCP2Blue;
                pickupTZonePreEndPose = pickupTZonePreEndPoseBlue;
                pickupTZoneEndPose = pickupTZoneEndPoseBlue;
                getPickupTZoneCPPath = getPickupTZoneCPPathBlue;
                offLineLever = offLineLeverBlue;
                offLineClose = offLineCloseBlue;
            } else {
                startPose = startPoseRed;
                scorePoseClose = scorePoseCloseRed;
                targetPoseClose = withRedTargetPoseOffset(targetPoseCloseRed);
                pickup1Pose = pickup1PoseRed;
                pickup1CP1 = pickup1CP1Red;
                pickup1CP2 = pickup1CP2Red;
                pickup1EndPose = pickup1EndPoseRed;
                getPickup1CPPath = getPickup1CPPathRed;
                pickup2Pose = pickup2PoseRed;
                pickup2CP1 = pickup2CP1Red;
                pickup2CP2 = pickup2CP2Red;
                pickup2EndPose = pickup2EndPoseRed;
                pickup2GateLeverEndPose = pickup2GateLeverEndPoseRed;
                pickup2GateLeverPushPose = pickup2GateLeverPushPoseRed;
                pickup2GateLeverPushHoldPose = pickup2GateLeverPushHoldPoseRed;
                pickup2GateLeverPushCP1 = pickup2GateLeverPushCP1Red;
                pickup2GateLeverPushCP2 = pickup2GateLeverPushCP2Red;
                getPickup2CPPath = getPickup2CPPathRed;
                pickup3Pose = pickup3PoseRed;
                pickup3CP1 = pickup3CP1Red;
                pickup3CP2 = pickup3CP2Red;
                pickup3EndPose = pickup3EndPoseRed;
                getPickup3CPPath = getPickup3CPPathRed;
                pickupBZonePose = pickupBZonePoseRed;
                pickupBZoneCP1 = pickupBZoneCP1Red;
                pickupBZoneCP2 = pickupBZoneCP2Red;
                getPickupBZoneCPPath = getPickupBZoneCPPathRed;
                pickupTZonePreEndPose = pickupTZonePreEndPoseRed;
                pickupBZoneEndPose = pickupBZoneEndPoseRed;
                pickupTZonePose = pickupTZonePoseRed;
                pickupTZoneCP1 = pickupTZoneCP1Red;
                pickupTZoneCP2 = pickupTZoneCP2Red;
                pickupTZoneEndPose = pickupTZoneEndPoseRed;
                getPickupTZoneCPPath = getPickupTZoneCPPathRed;
                offLineLever = offLineLeverRed;
                offLineClose = offLineCloseRed;

            }

            // if we want to change braking for a pathchain, then use .setNoDeceleration() or .setBrakingStrength(double set) after a path is added
            firstshootpath=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(startPose,scorePoseClose)

                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePoseClose.getHeading())
                    .build();

            firstPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup1CP1,
                                    pickup1CP2,
                                    pickup1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup1Pose.getHeading(),.25)
                    .setNoDeceleration()
                    .build();

            firstPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())

                    .addPath(new BezierLine(pickup1EndPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup2CP1,
                                    pickup2CP2,
                                    pickup2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup2Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            secondPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())

                    .addPath(new BezierLine(pickup2EndPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupGateLeverEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, pickup2GateLeverEndPose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2GateLeverEndPose.getHeading())

                    .addPath(new BezierCurve(
                            pickup2GateLeverEndPose,
                            pickup2GateLeverPushCP1,
                            pickup2GateLeverPushCP2,
                            pickup2GateLeverPushPose))
                    .setLinearHeadingInterpolation(pickup2GateLeverEndPose.getHeading(), pickup2GateLeverPushPose.getHeading())

                    .addPath(new BezierLine(pickup2GateLeverPushPose, pickup2GateLeverPushHoldPose))
                    .setLinearHeadingInterpolation(pickup2GateLeverPushPose.getHeading(), pickup2GateLeverPushHoldPose.getHeading())

                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            secondPickupGateLeverShootEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup2GateLeverPushPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup2GateLeverPushPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            thirdPickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickup3CP1,
                                    pickup3CP2,
                                    pickup3Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickup3Pose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            thirdPickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3EndPose.getHeading())

                    .addPath(new BezierLine(pickup3EndPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            BZonePickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickupBZoneCP1,
                                    pickupBZoneCP2,
                                    pickupBZonePose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickupBZonePose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            BZonePickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickupBZonePose, pickupBZoneEndPose))
                    .setLinearHeadingInterpolation(pickupBZonePose.getHeading(), pickupBZoneEndPose.getHeading())

                    .addPath(new BezierLine(pickupBZoneEndPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickupBZoneEndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupCornerBrakingStrength)
                    .build();

            TZonePickup= PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePoseClose,
                                    pickupTZoneCP1,
                                    pickupTZoneCP2,
                                    pickupTZonePose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), pickupBZonePose.getHeading(),.75)
                    .setNoDeceleration()
                    .build();

            TZonePickupEnd = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(pickupTZonePose, pickupTZonePreEndPose))
                    .setLinearHeadingInterpolation(pickupTZonePose.getHeading(), pickupTZonePreEndPose.getHeading())

                    .addPath(new BezierLine(pickupTZonePreEndPose, pickupTZoneEndPose))
                    .setLinearHeadingInterpolation(pickupTZonePreEndPose.getHeading(), pickupTZoneEndPose.getHeading())

                    .addPath(new BezierLine(pickupTZoneEndPose, scorePoseClose))
                    .setLinearHeadingInterpolation(pickupTZoneEndPose.getHeading(), scorePoseClose.getHeading())
                    .setBrakingStrength(pickupBrakingStrength)
                    .build();

            moveOffLineLever=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseClose,offLineLever)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineLever.getHeading())
                    .build();

            moveOffLineClose=PedroComponent.follower().pathBuilder()
                    .addPath(
                            new BezierLine(scorePoseClose,offLineClose)

                    )
                    .setLinearHeadingInterpolation(scorePoseClose.getHeading(), offLineClose.getHeading())
                    .build();


        }
    // if you want to reduce power for a path, use FollowPath(pathchain, holdend, maxpower)

    /**
     * Command that monitors ball count and finishes when intake has 3 balls.
     * Use this with ParallelRaceGroup to end a path early when full.
     *
     * Example usage:
     *   new ParallelRaceGroup(
     *       new FollowPath(somePath),
     *       IntakeUntilFull()
     *   )
     */
    public Command IntakeUntilFull() {
        return new LambdaCommand()
                .setIsDone(() -> IntakeWithSensorsSubsystem.INSTANCE.getBallCount() >= 3)
                .setInterruptible(true)
                .named("IntakeUntilFull");
    }

    /**
     * Shoots all balls using the same fixed-delay dumbShoot profile used for far teleop shots.
     * This spaces the balls out instead of feeding multiple single shots back-to-back.
     *
     * Note: Make sure the shooter is spun up before calling this command!
     */
    public Command ShootAllBalls() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    IntakeWithSensorsSubsystem.INSTANCE.setDumbShootFixedDelayMs(AUTON_FAR_NO_BOOST_DUMBSHOOT_DELAY_MS);
                    IntakeWithSensorsSubsystem.INSTANCE.dumbShoot();
                }),
                new Delay(AUTON_FAR_NO_BOOST_SHOOT_DURATION_SEC),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    public Command CloseShootPreload() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstshootpath),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                        new InstantCommand(() -> ShooterSubsystem.INSTANCE.shooterHoodDrive(autonShooterHoodServoPos)),
                        new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose))
                ),
                new Delay(1.20),  //Could replace this with shooting a ball
                //new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                ShootAllBalls(),
                //new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoToFirstPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.2),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }
    public Command ClosePickupAndShootFirstRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(firstPickupEnd),
                        new SequentialGroup(
                                new Delay(0.1),
                                new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),

                new Delay(0.0),
                ShootAllBalls(),
                //new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                //new Delay(0.50),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoTo2ndPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(secondPickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
            );
    }

    public Command ClosePickupAndGateLever2ndRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(secondPickupGateLeverEnd),
                        new SequentialGroup(
                                new Delay(0.1),
                                new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                )
        );
    }

    public Command ClosePickupShootAfterGateLever2ndRow() {
        return new SequentialGroup(
                new FollowPath(secondPickupGateLeverShootEnd),
                new Delay(0.8),
                //new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                //new Delay(0.50),
                ShootAllBalls(),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }


    public Command ClosePickupGateLeverShoot2ndRow() {
        return new SequentialGroup(
                    new ParallelGroup(
                        new FollowPath(secondPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new Delay(0.8),
                //new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                //new Delay(0.50),
                ShootAllBalls(),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
        );
    }

    public Command CloseGoTo3rdPickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }


    public Command ClosePickupAndShoot3rdRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(thirdPickupEnd),
                        new SequentialGroup(
                                new Delay(0.5),
                                new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new Delay(0.5),
                //new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                //new Delay(0.5),
                ShootAllBalls(),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    public Command CloseGoToBZonePickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(BZonePickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }


    public Command ClosePickupAndShootBZoneRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(BZonePickupEnd),
                        new SequentialGroup(
                                new Delay(0.1),
                                new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new Delay(0.0),
                //new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                //new Delay(0.5),
                ShootAllBalls(),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    public Command CloseGoToTZonePickupLine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(BZonePickup),
                        new SequentialGroup(
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0)),
                                new Delay(0.5),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.intakeForward())
                        )
                )
        );
    }


    public Command ClosePickupAndShootTZoneRow() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(BZonePickupEnd),
                        new SequentialGroup(
                                new Delay(0.1),
                                new InstantCommand(() -> TurretSubsystem.INSTANCE.aimAtFieldPoint(targetPoseClose)),
                                new InstantCommand(() -> ShooterSubsystem.INSTANCE.spinUp(autonShooterRPM)),
                                new Delay(1.0),
                                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop())
                        )
                ),
                new Delay(0.0),
                //new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.dumbShoot()),
                //new Delay(0.5),
                ShootAllBalls(),
                new InstantCommand(() -> ShooterSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.stop()),
                new InstantCommand(() -> IntakeWithSensorsSubsystem.INSTANCE.setBallCount(0))
        );
    }

    public Command CloseMoveOffLine() {
        return new SequentialGroup(
                new FollowPath(moveOffLineClose)
        );
    }

    public Command CloseMoveOffLineToLever() {
        return new SequentialGroup(
                new FollowPath(moveOffLineLever)
        );
    }

}

