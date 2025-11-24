package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceDetector;
import org.firstinspires.ftc.teamcode.util.AutoSelector;
import org.firstinspires.ftc.teamcode.util.AutoSelector.AutoMode;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import org.firstinspires.ftc.teamcode.vision.AprilTagVisionManager;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.vision.AprilTagVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.EnumMap;

/**
 * Base RR 1.0 autonomous for 2026DECODE.
 *
 * Responsibilities:
 *  - Read alliance + auto mode from digital inputs (AllianceDetector + AutoSelector).
 *  - Resolve a starting pose for the combo.
 *  - Construct MecanumDrive (PinpointLocalizer is wired inside).
 *  - Build and run an Action provided by subclasses.
 *
 * All units: inches and radians.
 */
public abstract class BaseAutoRR extends LinearOpMode {
    // ---- Tunable field coordinates (inches, radians) ----
    // Shooting pose per AutoMode. Replace with real DECODE coordinates.
    public static double RED_SHOOT_X   = -48.0;
    public static double RED_SHOOT_Y   = 48.0;
    public static double RED_SHOOT_HE  =  Math.toRadians(-45.0);

    public static double BLUE_SHOOT_X  =  48.0;
    public static double BLUE_SHOOT_Y  =  48.0;
    public static double BLUE_SHOOT_HE = Math.toRadians(45.0);   // facing "back" toward blue goal

    // ---- Spike stack geometry (DECODE) ----
    // translation: +x is toward blue, +y is toward obelisk
    // heading: obelisk at 180, red at +145, blue at -145

    // RED:
    public static double RED_SPIKE_X = -48.0;          // 24" in from red goal wall
    public static double RED_SPIKE1_Y = 12.0;         // PPG spike (closest to goal)
    public static double RED_SPIKE2_Y = -12.0;         // PGP
    public static double RED_SPIKE3_Y = -36.0;         // GPP
    public static double RED_HUMAN_X = -62.0;
    public static double RED_HUMAN_Y = 63.0;

    // BLUE
    public static double BLUE_SPIKE_X =  48.0;         // 24" in from blue goal wall
    public static double BLUE_SPIKE1_Y =  60.0;        // PPG
    public static double BLUE_SPIKE2_Y =  84.0;        // PGP
    public static double BLUE_SPIKE3_Y = 108.0;        // GPP
    public static double BLUE_HUMAN_X = 62.0;
    public static double BLUE_HUMAN_Y = 63.0;

    // --- Motif detection shared by all RR autos ---
    private AprilTagVision motifVision = null;
    private String motifCode = "NONE";
    private boolean motifLocked = false;

    protected TelemetryUtil T;
    protected AllianceDetector allianceDetector;
    protected AutoSelector autoSelector;
    protected AprilTagVisionManager vision;

    private final EnumMap<AutoMode, Pose2d> startPoses = new EnumMap<>(AutoMode.class);

    public BaseAutoRR() {
        // translation: +x is toward blue, +y is toward obelisk
        // heading: obelisk at 180, red at +145, blue at -145
        startPoses.put(AutoMode.RED1,  new Pose2d(new Vector2d(-39.0, 64.0), Math.toRadians(-90.0)));
//        startPoses.put(AutoMode.RED1,  new Pose2d(new Vector2d(-48, 48.0), Math.toRadians(90.0)));
        startPoses.put(AutoMode.RED2,  new Pose2d(new Vector2d(-55.5, 39.5), Math.toRadians(-165.0)));
        startPoses.put(AutoMode.BLUE1, new Pose2d(new Vector2d(39.0, 64.0), Math.toRadians(90.0)));
        startPoses.put(AutoMode.BLUE2, new Pose2d(new Vector2d(55.5, 39.5), Math.toRadians(165.0)));
        startPoses.put(AutoMode.NONE,  new Pose2d(new Vector2d(0.0, 0.0), 0.0));
    }

    /**
     * Subclasses implement this to build the RR Action sequence.
     */
    protected abstract Action buildRoutine(MecanumDrive drive,
                                           Alliance alliance,
                                           AutoMode autoMode,
                                           Pose2d startPose);

    /**
     * Override if you want a different start-pose lookup.
     */
    protected Pose2d getStartPose(Alliance alliance, AutoMode mode) {
        Pose2d pose = startPoses.get(mode);
        if (pose == null) {
            return new Pose2d(new Vector2d(0.0, 0.0), 0.0);
        }
        return pose;
    }

    @Override
    public void runOpMode() {
        T = new TelemetryUtil(this);
        allianceDetector = new AllianceDetector(hardwareMap);
        autoSelector = new AutoSelector(hardwareMap);

        Alliance alliance = Alliance.NONE;
        AutoMode autoMode = AutoMode.NONE;

        vision = new AprilTagVisionManager(hardwareMap, T);

        // Reset motif state for this run.
        motifCode = "NONE";
        motifLocked = false;

        // Pre-start loop: update switch state, display selection, and scan motif.
        while (!isStarted() && !isStopRequested()) {
            alliance = allianceDetector.determineAlliance();
            autoMode = autoSelector.select(alliance);

            // Keep motifcam running and update motifCode opportunistically.
            ensureMotifVision();
            if (motifVision != null && !motifLocked) {
                String detected = detectMotifOnce(motifVision);
                if (!"NONE".equals(detected)) {
                    motifCode = detected;
                }
            }

            T.banner(1, "RR Auto Init");
            T.t(1, "Alliance", alliance);
            T.t(1, "AutoMode", autoMode);
            T.t(1, "Motif code", motifCode);
//            T.t(2, "FlagA asserted", allianceDetector.isFlagAAsserted());
//            T.t(2, "FlagB asserted", allianceDetector.isFlagBAsserted());
//            T.t(2, "AutoA asserted", autoSelector.isAutoAAsserted());
//            T.t(2, "AutoB asserted", autoSelector.isAutoBAsserted());

            // allow subclasses to do their own init-loop behavior
            initLoopExtended();

            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        // Once start is pressed, give the camera up to MOTIF_SCAN_TIMEOUT_MS more to lock.
        lockMotifAtStart();

        Pose2d startPose = getStartPose(alliance, autoMode);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action routine = buildRoutine(drive, alliance, autoMode, startPose);

        if (routine != null) {
            Actions.runBlocking(routine);
        }
    }

    /**
     * Optional hook for subclasses to run extra logic during the INIT-loop
     * (before start is pressed). This runs once per init-loop iteration.
     *
     * Typical use: indexer preloading, mechanism homing, etc.
     */
    protected void initLoopExtended() {
        // default: no-op
    }

    protected String getMotifCode() {
        return motifCode;
    }

    /** Lazily construct the motif camera pipeline (motifcam). Safe to call repeatedly. */
    private void ensureMotifVision() {
        if (motifVision != null) return;
        try {
            motifVision = new AprilTagVision(hardwareMap);
        } catch (Exception e) {
            // Camera failed to init; leave motifVision == null and report once.
            T.t(1, "Motif error", "motifcam init failed");
        }
    }

    /** Classify the current frame from motifcam as GPP / PGP / PPG / NONE. */
    protected String detectMotifOnce(AprilTagVision mv) {
        if (mv == null || mv.atag == null) return "NONE";

        List<AprilTagDetection> dets = mv.atag.getDetections();
        if (dets == null || dets.isEmpty()) return "NONE";

        for (AprilTagDetection d : dets) {
            if (d == null) continue;
            int id = d.id;
            if (id == Constants.Vision.TAG_MOTIF_GPP) return "GPP";
            if (id == Constants.Vision.TAG_MOTIF_PGP) return "PGP";
            if (id == Constants.Vision.TAG_MOTIF_PPG) return "PPG";
        }
        return "NONE";
    }

    /**
     * Block for up to MOTIF_SCAN_TIMEOUT_MS after start to freeze motifCode.
     * If we never see a motif, we lock "NONE".
     */
    private void lockMotifAtStart() {
        if (motifLocked) {
            // Already locked from init loop; just make sure camera is off.
            if (motifVision != null && motifVision.portal != null) {
                motifVision.portal.stopStreaming();
            }
            return;
        }

        ensureMotifVision();
        long timeoutMs = Constants.Vision.MOTIF_SCAN_TIMEOUT_MS;
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()
                && "NONE".equals(motifCode)
                && (timeoutMs <= 0 || System.currentTimeMillis() - startMs < timeoutMs)) {

            if (motifVision != null) {
                String detected = detectMotifOnce(motifVision);
                if (!"NONE".equals(detected)) {
                    motifCode = detected;
                }
            }

            T.t(1, "Motif code (locking)", motifCode);
            telemetry.update();
            idle();
        }

        motifLocked = true;

        if (motifVision != null && motifVision.portal != null) {
            motifVision.portal.stopStreaming();
        }

        T.banner(1, "Motif locked: " + motifCode);
    }

    public void playAudio(String text, int delayMs) {
        try {
            telemetry.speak(text);
            wait(delayMs);
        } catch (Exception e) {
            //do nothing
        }
    }

    /**
     * Shooter poses per alliance/mode.
     */
    Pose2d getShootPoseFor(Alliance alliance, AutoMode mode) {
        switch (alliance) {
            case RED:
                return new Pose2d(new Vector2d(RED_SHOOT_X, RED_SHOOT_Y), RED_SHOOT_HE);
            case BLUE:
                return new Pose2d(new Vector2d(BLUE_SHOOT_X, BLUE_SHOOT_Y), BLUE_SHOOT_HE);
            case NONE:
            default:
                // Fallback: shouldn't ever be used in a real match.
                return new Pose2d(new Vector2d(0.0, 0.0), 0.0);
        }
    }

    /**
     * Human poses per alliance/mode.
     */
    private Pose2d getHumanPoseFor(Alliance alliance, AutoMode mode) {
        switch (alliance) {
            case RED:
                return new Pose2d(new Vector2d(RED_HUMAN_X, RED_HUMAN_Y), Math.toRadians(90.0));
            case BLUE:
                return new Pose2d(new Vector2d(BLUE_HUMAN_X, BLUE_HUMAN_Y), Math.toRadians(-90.0));
            case NONE:
            default:
                return new Pose2d(new Vector2d(0.0, 0.0), 0.0);
        }
    }

    /**
     * Stack poses per alliance/mode.
     */
    Pose2d[] getStackPosesFor(Alliance alliance, AutoMode mode) {
        switch (alliance) {
            case RED:
                return new Pose2d[]{
                        new Pose2d(new Vector2d(RED_SPIKE_X, RED_SPIKE1_Y), 0.0),
                        new Pose2d(new Vector2d(RED_SPIKE_X, RED_SPIKE2_Y), 0.0),
                        new Pose2d(new Vector2d(RED_SPIKE_X, RED_SPIKE3_Y), 0.0),
                };
            case BLUE:
                return new Pose2d[]{
                        new Pose2d(new Vector2d(BLUE_SPIKE_X, BLUE_SPIKE1_Y), Math.PI),
                        new Pose2d(new Vector2d(BLUE_SPIKE_X, BLUE_SPIKE2_Y), Math.PI),
                        new Pose2d(new Vector2d(BLUE_SPIKE_X, BLUE_SPIKE3_Y), Math.PI),
                };
            case NONE:
            default:
                return new Pose2d[0];
        }
    }
}
