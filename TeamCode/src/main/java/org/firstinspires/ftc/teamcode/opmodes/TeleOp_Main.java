package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.Item;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.vision.AprilTagVisionManager;

import org.json.JSONObject;

import java.io.File;
import java.io.FileReader;

/**
 * Main TeleOp for 2026 DECODE robot.
 *
 * Driver (g1):
 *   LS (x/y)         = translation
 *   RS.x             = rotation
 *   LB (hold)        = precision scale (Constants.Drive.PRECISION_SCALE)
 *   START (tap)      = toggle field/robot centric
 *   LB+START (tap)   = toggle heading provider (IMU <-> Pinpoint)
 *   BACK (tap)       = zero heading for current provider
 *
 * Operator (g2):
 *   X                = intake
 *   LB+X             = outtake
 *   RT (hold)        = shot sequence (ShooterSubsystem.handleRightTrigger)
 *   LT (tap)         = advance indexer 1 slot (manual step)
 *   D.LEFT (hold)    = reverse indexer for unjam; on release, re-acquire a slot
 *   B (tap)          = clear queue and rescan all 3 slots, then motif-rotate
 *
 * Other:
 *   - Shooter idles on start().
 *   - MotifCam is only used during autonomous; TeleOp runs odocam-only fusion.
 *   - Alliance + motif may be injected from autonomous via setMatchConfig().
 */
@TeleOp(name = "TeleOp_Main", group = "Main")
public class TeleOp_Main extends OpMode {

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;
    private HeadingProvider imuHeading, pinpointHeading, heading;
    private AprilTagVisionManager vision;
    private AllianceDetector allianceDetector;
    private TelemetryUtil T;

    // Drive state
    private boolean useField = Constants.Drive.FIELD_CENTRIC_DEFAULT;
    private SlewRateLimiter slewX, slewY, slewR;

    // Driver edge tracking
    private boolean lastG1Start = false;
    private boolean lastG1Back  = false;

    // Operator edge tracking
    private boolean lastG2LT        = false;
    private boolean lastG2Y         = false;
    private boolean lastG2B         = false;
    private boolean lastG2DpadLeft  = false;

    // Indexer / queue state
    private boolean reindexMode   = false;
    private boolean queueWasFull  = false;
    private long intakeReverseUntilMs = 0;
    private boolean unjamActive   = false;

    // Match configuration injected from autonomous (optional)
    public static Alliance matchAlliance = Alliance.NONE;
    public static String  matchMotifCode = "NONE"; // "GPP","PGP","PPG" or "NONE"

    @Override
    public void init() {
        loadPinpointCalibrationIfPresent();

        T = new TelemetryUtil(this);
        drive = new DriveSubsystem(hardwareMap);

        imuHeading      = new IMUHeading(hardwareMap);
        pinpointHeading = new PinpointHeading(hardwareMap);
        // Default heading provider is Pinpoint for TeleOp
        heading         = pinpointHeading;

        intake   = new IntakeSubsystem(hardwareMap);
        indexer  = new IndexerSubsystem(hardwareMap);
        shooter  = new ShooterSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);

        vision           = new AprilTagVisionManager(hardwareMap, T);
        allianceDetector = new AllianceDetector(hardwareMap);

        slewX = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewY = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewR = new SlewRateLimiter(Constants.Drive.SLEW_ROTATION);

        // Begin homing the indexer; it will stop itself on the first slot edge.
//        indexer.startStep();
        //TODO: This isnt working, stopping too early
    }

    /** Load Pinpoint calibration offsets from JSON if present. */
    private void loadPinpointCalibrationIfPresent() {
        try {
            File f = new File(Constants.Pinpoint.CALIBRATION_JSON);
            if (!f.exists()) return;

            StringBuilder sb = new StringBuilder();
            try (FileReader fr = new FileReader(f)) {
                int c;
                while ((c = fr.read()) != -1) sb.append((char) c);
            }
            JSONObject o = new JSONObject(sb.toString());

            // Load scalar tuning values from JSON (track width & forward offset in mm)
            double trackWidth = o.optDouble(
                    "trackWidthMm",
                    Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM - Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM)
            );
            double fwdOffset = o.optDouble(
                    "forwardOffsetMm",
                    0.5 * (Constants.Pinpoint.X_POD_FORWARD_OFFSET_MM + Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM)
            );

            // 1) Forward offsets: keep both pods at the same forward distance from the CoR.
            Constants.Pinpoint.X_POD_FORWARD_OFFSET_MM = fwdOffset;
            Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM = fwdOffset;

            // 2) Side offsets: scale the existing asymmetric geometry to match trackWidth.
            double left0  = Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM);  // left pod (+Y)
            double right0 = Math.abs(Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM);  // right pod (-Y)
            double baseTrack = left0 + right0;
            if (baseTrack > 1e-6) {
                double scale = trackWidth / baseTrack;
                double newLeft  = left0  * scale;
                double newRight = right0 * scale;

                // Left pod is +Y, right pod is -Y
                Constants.Pinpoint.X_POD_SIDE_OFFSET_MM =  newLeft;
                Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM = -newRight;

                // Keep convenience TRACK_WIDTH_MM in sync
                Constants.Pinpoint.TRACK_WIDTH_MM =
                        Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM - Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM);
            }

            telemetry.addLine("Pinpoint calib loaded");
        } catch (Exception e) {
            telemetry.addLine("Pinpoint calib load failed");
        }
    }

    @Override
    public void init_loop() {
        // Keep indexer state machine running while homing.
        indexer.loop();
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.idle();
        shooter.resetFireControl();

        reindexMode   = false;
        queueWasFull  = isQueueFull();
        unjamActive   = false;

        lastG1Start = false;
        lastG1Back  = false;

        lastG2LT        = false;
        lastG2Y         = false;
        lastG2B         = false;
        lastG2DpadLeft  = false;
    }

    @Override
    public void loop() {
        Gamepad g1 = gamepad1;
        Gamepad g2 = gamepad2;

        // Alliance: prefer injected matchAlliance if Autonomous set it; otherwise fall back to switches.
        Alliance alliance = (matchAlliance != Alliance.NONE)
                ? matchAlliance
                : allianceDetector.determineAlliance();

        // ===== Elevator (Driver g1, gated by A) =====
        boolean armElevator = g1.a;
        if (armElevator) {
            elevator.commandUp(g1.dpad_up);
            elevator.commandDown(g1.dpad_down);
        } else {
            elevator.commandUp(false);
            elevator.commandDown(false);
        }
        elevator.loop();
        boolean lockout = elevator.isLockout();

        // ===== Vision (OdoCam only during TeleOp) =====
        double stickMag = Math.hypot(g1.left_stick_x, g1.left_stick_y);
        boolean allowOdoFuse = Constants.Vision.ODO_FUSE_DURING_TELEOP &&
                !lockout &&
                (stickMag < Constants.Vision.ODO_STICK_THRESH);
        boolean preferRedGoal = (alliance == Alliance.RED);
        // pauseVision = lockout; allowMotifScan = false (MotifCam only used in Autonomous)
        vision.loop(lockout, false, allowOdoFuse, preferRedGoal);

        // ===== Driver controls (g1) =====

        // START: edge detect
        boolean startNow   = g1.start;
        boolean startEdge  = startNow && !lastG1Start;
        lastG1Start        = startNow;

        // BACK: zero current heading on edge
        boolean backNow  = g1.back;
        boolean backEdge = backNow && !lastG1Back;
        lastG1Back       = backNow;
        if (backEdge) {
            heading.zeroHeading();
        }

        // LB+START => toggle heading provider; START alone => toggle field/robot
        if (startEdge) {
            if (g1.left_bumper) {
                // Toggle heading provider between IMU and Pinpoint
                heading = (heading == imuHeading) ? pinpointHeading : imuHeading;
            } else {
                // Toggle drive frame
                useField = !useField;
            }
        }

        // ===== Operator mechanisms (g2) =====

        // Base intake intent: X / LB+X
        boolean intakeFwdCmd = g2.x && !g2.left_bumper;
        boolean outtakeCmd   = g2.x &&  g2.left_bumper;
        if (outtakeCmd) {
            intake.outtake();
//        } else if (intakeFwdCmd) {
//            intake.intake();
        } else {
            intake.stop();
        }

        // Indexer core state machine
        indexer.loop();

        // D-pad LEFT: unjam mode (reverse while held, then re-acquire a slot)
        boolean dpadLeftNow       = g2.dpad_left;
        boolean dpadLeftPressed   = dpadLeftNow && !lastG2DpadLeft;
        boolean dpadLeftReleased  = !dpadLeftNow && lastG2DpadLeft;
        lastG2DpadLeft            = dpadLeftNow;

        if (dpadLeftPressed) {
            unjamActive = true;
            indexer.startUnjamReverse();
        }
        if (dpadLeftReleased) {
            indexer.stopUnjam();
            unjamActive = false;
            if (!indexer.isStepping()) {
                indexer.startStep();
            }
        }

        // LT: single manual step (disabled while reindexing or unjamming)
        boolean ltNow  = g2.left_trigger > 0.5;
        boolean ltEdge = ltNow && !lastG2LT;
        lastG2LT       = ltNow;
        if (!reindexMode && !unjamActive && ltEdge && !indexer.isStepping()) {
            indexer.startStep();
        }

        // B: clear queue + rescan, or cycle motif when LB held
        boolean bNow  = g2.b;
        boolean bEdge = bNow && !lastG2B;
        lastG2B       = bNow;

        if (bEdge && g2.left_bumper) {
            // LB + B: cycle static motif code
            cycleMotifCode();
        } else if (bEdge && !indexer.isStepping() && !unjamActive) {
            // Bare B: clear queue and start rescan
            indexer.clearAll();
            reindexMode = true;
        }

        // Re-index / rescan state machine: fill S0/S1L/S2, then rotate for motif
        if (reindexMode && !unjamActive) {
            boolean queueFull = isQueueFull();
            if (!queueFull && !indexer.isStepping()) {
                boolean colorPresent = indexer.detectAtS1L();
                if (colorPresent) {
                    indexer.startStep();
                } else {
                    // No clean color detected at S1L; give up on rescan for now.
                    reindexMode = false;
                }
            } else if (queueFull && !indexer.isStepping()) {
                applyMotifRotationIfAvailable();
                reindexMode = false;
            }
        }

        // Shooter stop on g2.Y (edge)
        boolean yNow  = g2.y;
        boolean yEdge = yNow && !lastG2Y;
        lastG2Y       = yNow;
        if (yEdge) {
            shooter.stop();
            shooter.resetFireControl();
        }

        // Shooter shot sequence on g2 RT
        boolean shootTrigger       = g2.right_trigger > 0.5;
        boolean autoAdvanceEnabled = !shootTrigger && !reindexMode && !unjamActive;

        // While not reindexing/unjamming, allow queue to auto-fill as in Test_Intake_Indexer
        handleIndexerAutoAdvance(autoAdvanceEnabled);

        shooter.handleRightTrigger(shootTrigger, indexer);
        shooter.loop();

        // ==== Queue / intake gating and motif rotation ====

        int nowMs = (int) System.currentTimeMillis();
        boolean queueNowFull = isQueueFull();

        // If we just initialized, we just set the reverse timer to now
        intakeReverseUntilMs = (intakeReverseUntilMs == 0) ? nowMs + Constants.Intake.FULL_QUEUE_SPITBACK_MS : intakeReverseUntilMs;

        // Edge: queue just became full in normal mode -> schedule intake backdrive
        if (!queueWasFull && queueNowFull && !reindexMode && !unjamActive) {
            intakeReverseUntilMs = nowMs + Constants.Intake.FULL_QUEUE_SPITBACK_MS;
        }

        boolean indexerStepping   = indexer.isStepping();
        boolean autoReverseActive = nowMs < intakeReverseUntilMs;

        // Override intake based on indexer / queue state
        if (autoReverseActive) {
            // Spit-back window: always run intake in reverse to reject any 4th artifact
            intake.outtake();
        } else if (outtakeCmd) {
            // Manual outtake still works even when queue is full
            intake.outtake();
        } else if (indexerStepping || isQueueFull()) {
            // <<< THIS is the "lock out intake when full" rule >>>
            // Don't pull new artifacts while the indexer is moving or already full
            intake.stop();
        } else if (intakeFwdCmd) {
            intake.intake();
        }

    // Else: keep whatever the base X/idle logic chose

        // Queue-full motif rotation (outside explicit rescan)
        if (!queueWasFull && queueNowFull && !indexerStepping) {
            applyMotifRotationIfAvailable();
        }
        queueWasFull = queueNowFull;

        // ===== Drive (g1 LS translation, RS.x rotation) =====
        double lx = g1.left_stick_x;
        double ly = -g1.left_stick_y;
        double rx = g1.right_stick_x;

        // shaping
        lx = Shaping.translate(lx);
        ly = Shaping.translate(ly);
        rx = Shaping.rotate(rx);

        // precision scaling on LB
        double scale = g1.left_bumper ? Constants.Drive.PRECISION_SCALE : 1.0;
        lx *= scale;
        ly *= scale;
        if (Constants.Drive.PRECISION_SCALES_ROTATION) {
            rx *= scale;
        }

        // field-centric transform
        double x = lx;
        double y = ly;
        if (useField) {
            double h = heading.getHeadingRad();
            double cos = Math.cos(-h);
            double sin = Math.sin(-h);
            double tx = x * cos - y * sin;
            double ty = x * sin + y * cos;
            x = tx;
            y = ty;
        }

        // rotation sign convention: drive expects +CCW
        double rot = rx * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);

        // optional slew limiting
        if (Constants.Drive.SLEW_ENABLED) {
            x   = slewX.calculate(x);
            y   = slewY.calculate(y);
            rot = slewR.calculate(rot);
        }

        drive.driveRobotCentric(x, y, rot);

        // ===== Telemetry =====
        double headingDeg = Math.toDegrees(heading.getHeadingRad());

        telemetry.addLine("=== Match ===");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Motif", matchMotifCode);
        telemetry.addLine("=== Robot ===");
        telemetry.addData("DriveMode", useField ? "FIELD" : "ROBOT");
        telemetry.addData("HeadingProvider", (heading == imuHeading) ? "IMU" : "PINPOINT");
        telemetry.addData("HeadingDeg", headingDeg);
        telemetry.addData("Queue", "S0:%s S1:%s S2:%s", indexer.getS0(), indexer.getS1L(), indexer.getS2());

        telemetry.update();
    }

    /** Queue is full when all three slots are non-NONE. */
    private boolean isQueueFull() {
        return indexer.getS0()  != Item.NONE &&
                indexer.getS1L() != Item.NONE &&
                indexer.getS2()  != Item.NONE;
    }

    /** Rotate queue into motif order if a valid motif code is known. */
    private void applyMotifRotationIfAvailable() {
        if ("GPP".equals(matchMotifCode) ||
                "PGP".equals(matchMotifCode) ||
                "PPG".equals(matchMotifCode)) {
            indexer.rotateForMotif(matchMotifCode);
        }
    }

    /** Cycle matchMotifCode: (other/NONE)->PGP->GPP->PPG->PGP->... */
    private void cycleMotifCode() {
        if ("PGP".equals(matchMotifCode)) {
            matchMotifCode = "GPP";
        } else if ("GPP".equals(matchMotifCode)) {
            matchMotifCode = "PPG";
        } else if ("PPG".equals(matchMotifCode)) {
            matchMotifCode = "PGP";
        } else {
            // Any invalid/none value goes to the first in the cycle
            matchMotifCode = "PGP";
        }
    }

    /**
     * Auto-advance rule (mirrors Test_Intake_Indexer behavior):
     *  - If not stepping and queue not full:
     *      * detectAtS1L(); on valid color, startStep().
     *  - If queue full:
     *      * keep S1L's color refreshed, but do not auto-advance.
     */
    private void handleIndexerAutoAdvance(boolean autoAdvanceEnabled) {
        boolean isStepping = indexer.isStepping();
        boolean queueFull  = isQueueFull();

        if (autoAdvanceEnabled) {
            if (!isStepping && !queueFull) {
                boolean colorPresent = indexer.detectAtS1L();
                if (colorPresent) {
                    indexer.startStep();
                }
            } else if (!isStepping && queueFull) {
                indexer.detectAtS1L();
            }
        } else {
            if (!isStepping) {
                indexer.detectAtS1L();
            }
        }
    }

    /**
     * Optional helper for Autonomous to inject match configuration for TeleOp.
     */
    public static void setMatchConfig(Alliance alliance, String motifCode) {
        matchAlliance = (alliance != null) ? alliance : Alliance.NONE;
        matchMotifCode = (motifCode != null) ? motifCode : "NONE";
    }
}
