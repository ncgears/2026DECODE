package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.Item;
import org.firstinspires.ftc.teamcode.subsystems.IMUHeading;
import org.firstinspires.ftc.teamcode.subsystems.PinpointHeading;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceDetector;
import org.firstinspires.ftc.teamcode.util.AutoSelector;
import org.firstinspires.ftc.teamcode.vision.AprilTagVision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Test_Subsystems
 *
 * Diagnostics TeleOp that reports subsystem-level detections:
 *  - Alliance from AllianceDetector (+ AutoSelector auto mode)
 *  - Indexer *raw* detected color at S1L (PURPLE / GREEN / NONE, non-sticky)
 *  - Motif from motifcam (TAG_MOTIF_* IDs -> "GPP"/"PGP"/"PPG")
 *  - IMU heading + Pinpoint heading
 *
 * Behavior:
 *  - During init_loop(): snapshot mode. Press g2.A to refresh all readings.
 *  - During loop(): live mode, continuously updates all readings.
 */
@TeleOp(name = "Test_Subsystems", group = "Diagnostics")
public class Test_Subsystems extends OpMode {

    private AllianceDetector allianceDetector;
    private IndexerSubsystem indexer;
    private IMUHeading imuHeading;
    private PinpointHeading pinpointHeading;
    private AprilTagVision motifVision;
    private AutoSelector autoSelector;

    // Snapshot state (used during init_loop)
    private Alliance snapshotAlliance = Alliance.NONE;
    private Item snapshotColor        = Item.NONE;
    private String snapshotMotif      = "NONE";
    private double snapshotImuHeadingRad      = 0.0;
    private double snapshotPinpointHeadingRad = 0.0;
    private AutoSelector.AutoMode snapshotAuto = AutoSelector.AutoMode.NONE;

    private boolean lastAPressed = false;

    @Override
    public void init() {
        allianceDetector = new AllianceDetector(hardwareMap);
        indexer          = new IndexerSubsystem(hardwareMap);
        imuHeading       = new IMUHeading(hardwareMap);
        pinpointHeading  = new PinpointHeading(hardwareMap);
        motifVision      = new AprilTagVision(hardwareMap); // uses MOTIF_CAM_NAME in Constants
        autoSelector     = new AutoSelector(hardwareMap);

        snapshotAlliance            = Alliance.NONE;
        snapshotColor               = Item.NONE;
        snapshotMotif               = "NONE";
        snapshotImuHeadingRad       = 0.0;
        snapshotPinpointHeadingRad  = 0.0;
        snapshotAuto                = AutoSelector.AutoMode.NONE;
        lastAPressed                = false;

        telemetry.addLine("Test_Subsystems: INIT.");
        telemetry.addLine("Press g2.A in INIT to snapshot subsystem detections.");
    }

    @Override
    public void init_loop() {
        // Keep any internal state machines updated
        indexer.loop();

        // Snapshot-on-press behavior
        boolean aNow  = gamepad2.a;
        boolean aEdge = aNow && !lastAPressed;
        lastAPressed  = aNow;

        if (aEdge) {
            // Alliance snapshot
            snapshotAlliance = allianceDetector.determineAlliance();

            // Auto snapshot (combine alliance + auto switch)
            snapshotAuto = autoSelector.select(snapshotAlliance);

            // Color snapshot: RAW sensor view, non-sticky
            snapshotColor = indexer.peekColorAtS1LRaw();

            // Motif snapshot from motifcam
            snapshotMotif = detectMotifOnce();

            // Heading snapshots
            snapshotImuHeadingRad      = imuHeading.getHeadingRad();
            snapshotPinpointHeadingRad = pinpointHeading.getHeadingRad();
        }

        telemetry.addLine("=== Snapshot mode (INIT) ===");
        telemetry.addLine("Press g2.A to refresh snapshot.");

        telemetry.addLine("== Alliance ==");
        telemetry.addData("Alliance (snapshot)", snapshotAlliance);
        telemetry.addData("Auto (snapshot)", snapshotAuto);

        telemetry.addLine("== Indexer detected color at S1L (RAW, snapshot) ==");
        telemetry.addData("Color", fmtItem(snapshotColor));

        telemetry.addLine("== Motif (snapshot) ==");
        telemetry.addData("Motif code", snapshotMotif);
        telemetry.addData(
                "Tags",
                "%d=GPP, %d=PGP, %d=PPG",
                Constants.Vision.TAG_MOTIF_GPP,
                Constants.Vision.TAG_MOTIF_PGP,
                Constants.Vision.TAG_MOTIF_PPG
        );

        telemetry.addLine("== Heading / Pinpoint (snapshot) ==");
        telemetry.addData("IMU heading",
                "%.3f rad (%.1f deg)",
                snapshotImuHeadingRad,
                Math.toDegrees(snapshotImuHeadingRad));
        telemetry.addData("Pinpoint heading",
                "%.3f rad (%.1f deg)",
                snapshotPinpointHeadingRad,
                Math.toDegrees(snapshotPinpointHeadingRad));

        telemetry.update();
    }

    @Override
    public void start() {
        // Nothing special; we just switch from snapshot mode to live mode.
    }

    @Override
    public void loop() {
        // Keep any state machines up to date
        indexer.loop();

        // Live values (continuously updating)
        Alliance alliance = allianceDetector.determineAlliance();
        AutoSelector.AutoMode autoMode = autoSelector.select(alliance);

        // RAW color at S1L, non-sticky
        Item color = indexer.peekColorAtS1LRaw();

        String motif = detectMotifOnce();

        double imuHeadingRad      = imuHeading.getHeadingRad();
        double pinpointHeadingRad = pinpointHeading.getHeadingRad();

        telemetry.addLine("=== Live subsystem detections (RUN) ===");

        telemetry.addLine("== Alliance ==");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Auto", autoMode);

        telemetry.addLine("== Indexer detected color at S1L (RAW) ==");
        telemetry.addData("Color", fmtItem(color));

        telemetry.addLine("== Motif (motifcam) ==");
        telemetry.addData("Motif code", motif);
        telemetry.addData(
                "Tags",
                "%d=GPP, %d=PGP, %d=PPG",
                Constants.Vision.TAG_MOTIF_GPP,
                Constants.Vision.TAG_MOTIF_PGP,
                Constants.Vision.TAG_MOTIF_PPG
        );

        telemetry.addLine("== Heading / Pinpoint ==");
        telemetry.addData("IMU heading",
                "%.3f rad (%.1f deg)",
                imuHeadingRad,
                Math.toDegrees(imuHeadingRad));
        telemetry.addData("Pinpoint heading",
                "%.3f rad (%.1f deg)",
                pinpointHeadingRad,
                Math.toDegrees(pinpointHeadingRad));

        telemetry.update();
    }

    // --------- Helpers ---------

    /** Map IndexerSubsystem.Item to readable string. */
    private String fmtItem(Item item) {
        if (item == null) return "null";
        switch (item) {
            case PURPLE: return "PURPLE";
            case GREEN:  return "GREEN";
            case NONE:
            default:     return "NONE";
        }
    }

    /**
     * Inspect current AprilTag detections from motifcam and map to motif code.
     * Returns "GPP","PGP","PPG", or "NONE" if no motif tag is visible.
     */
    private String detectMotifOnce() {
        if (motifVision == null || motifVision.atag == null) return "NONE";

        List<AprilTagDetection> dets = motifVision.atag.getDetections();
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
}
