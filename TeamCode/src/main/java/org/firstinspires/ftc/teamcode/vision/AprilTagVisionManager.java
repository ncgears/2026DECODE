package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/** Manages two VisionPortals: motifcam (motif tags) and odocam (goal tags). */
public class AprilTagVisionManager {
    private final HardwareMap hw;
    private final TelemetryUtil T;

    private VisionPortal portalMotif, portalOdo;
    public AprilTagProcessor procMotif, procOdo;
    public AprilTagLibrary lib;

    private boolean motifKnown = false;
    private String motifCode = ""; // "GPP","PGP","PPG"
    private long lastFuseMs = 0;
    private boolean previewsOn = false;

    private int lastSeenId = -1, stableCount = 0;
    private long scanStartMs = 0;

    public AprilTagVisionManager(HardwareMap hw, TelemetryUtil T) {
        this.hw = hw; this.T = T;
        lib = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        buildOdoPortal(); // keep odocam available by default
    }

    private void buildMotifPortal() {
        if (portalMotif != null) return;
        procMotif = new AprilTagProcessor.Builder().setTagLibrary(lib).build();
        WebcamName cam = hw.get(WebcamName.class, Constants.Vision.MOTIF_CAM_NAME);
        portalMotif = new VisionPortal.Builder()
                .setCamera(cam)
                .addProcessor(procMotif)
                .setCameraResolution(new Size(Constants.Vision.STREAM_WIDTH, Constants.Vision.STREAM_HEIGHT))
                .enableLiveView(previewsOn)
                .build();
    }
    private void buildOdoPortal() {
        if (portalOdo != null) return;
        procOdo = new AprilTagProcessor.Builder().setTagLibrary(lib).build();
        WebcamName cam = hw.get(WebcamName.class, Constants.Vision.ODO_CAM_NAME);
        portalOdo = new VisionPortal.Builder()
                .setCamera(cam)
                .addProcessor(procOdo)
                .setCameraResolution(new Size(Constants.Vision.STREAM_WIDTH, Constants.Vision.STREAM_HEIGHT))
                .enableLiveView(previewsOn)
                .build();
    }

    /** Toggle camera previews (if supported). */
    public void setPreviewsOn(boolean on) {
        previewsOn = on;
        if (portalMotif != null) { if (on) portalMotif.resumeStreaming(); else portalMotif.stopStreaming(); }
        if (portalOdo   != null) { if (on) portalOdo.resumeStreaming();   else portalOdo.stopStreaming(); }
    }

    public boolean isMotifKnown() { return motifKnown; }
    public String  getMotifCode() { return motifCode; }

    /** Begin a motif scan window; will close automatically when motif is acquired or timed out. */
    public void startMotifScanWindow(long durationMs) {
        if (motifKnown) return;
        buildMotifPortal();
        scanStartMs = System.currentTimeMillis();
    }
    /** Stop motif portal to save CPU. */
    public void stopMotifScan() {
        if (portalMotif != null) { portalMotif.close(); portalMotif = null; procMotif = null; }
    }

    /** One-shot motif capture policy: require 2 consecutive frames of a valid motif tag. */
    private void motifLoop() {
        if (portalMotif == null || procMotif == null) return;
        List<AprilTagDetection> dets = procMotif.getDetections();
        int seenId = -1;
        for (AprilTagDetection d : dets) {
            if (d.id == Constants.Vision.TAG_MOTIF_GPP ||
                d.id == Constants.Vision.TAG_MOTIF_PGP ||
                d.id == Constants.Vision.TAG_MOTIF_PPG) { seenId = d.id; break; }
        }
        if (seenId != -1) {
            if (lastSeenId == seenId) stableCount++; else { lastSeenId = seenId; stableCount = 1; }
            if (stableCount >= 2) {
                motifKnown = true;
                motifCode = (seenId == Constants.Vision.TAG_MOTIF_GPP) ? "GPP"
                          : (seenId == Constants.Vision.TAG_MOTIF_PGP) ? "PGP" : "PPG";
                T.banner(2, "MOTIF ACQUIRED: " + motifCode);
                stopMotifScan(); // disable motifcam immediately
            }
        }
        // Auto-timeout window (up to 6s total; caller supplies duration)
        if (scanStartMs != 0 && System.currentTimeMillis() - scanStartMs > 6000) {
            stopMotifScan();
        }
    }

    /** Odo fusion placeholder: logs limited corrections when an alliance goal tag is visible. */
    private void odoLoop(boolean preferRedGoal) {
        if (portalOdo == null || procOdo == null) return;
        List<AprilTagDetection> dets = procOdo.getDetections();
        AprilTagDetection best = null;
        for (AprilTagDetection d : dets) {
            if ((preferRedGoal && d.id == Constants.Vision.TAG_GOAL_RED) ||
                (!preferRedGoal && d.id == Constants.Vision.TAG_GOAL_BLUE)) { best = d; break; }
        }
        if (best != null && best.metadata != null && best.ftcPose != null) {
            double range = best.ftcPose.range;
            double yaw   = best.ftcPose.yaw;
            if (range <= Constants.Vision.ODO_MAX_RANGE_M && Math.abs(yaw) <= Constants.Vision.ODO_MAX_ABS_YAW_DEG) {
                double dx = clamp(best.ftcPose.x, -Constants.Vision.ODO_MAX_DELTA_M,  Constants.Vision.ODO_MAX_DELTA_M);
                double dy = clamp(best.ftcPose.y, -Constants.Vision.ODO_MAX_DELTA_M,  Constants.Vision.ODO_MAX_DELTA_M);
                double dh = clamp(best.ftcPose.yaw, -Constants.Vision.ODO_MAX_DELTA_DEG, Constants.Vision.ODO_MAX_DELTA_DEG);
                T.banner(3, String.format("ODO FUSE: dXY=(%.02f,%.02f)m dH=%.1f°", dx, dy, dh));
                // TODO: Blend these corrections into Pinpoint pose when driver is integrated.
            }
        }
    }

    /**
     * Call from TeleOp loop.
     * @param pauseVision            if true, pause/stop streaming (elevator lockout)
     * @param allowMotifScan         true if motif is unknown and we are currently requesting to scan
     * @param allowOdoFuse           true if driver sticks are calm enough to apply corrections
     * @param preferRedGoal          alliance preference for goal tags
     */
    public void loop(boolean pauseVision, boolean allowMotifScan, boolean allowOdoFuse, boolean preferRedGoal) {
        long now = System.currentTimeMillis();
        if (pauseVision) {
            if (portalMotif != null) portalMotif.stopStreaming();
            if (portalOdo   != null) portalOdo.stopStreaming();
            return;
        } else if (previewsOn) {
            if (portalMotif != null) portalMotif.resumeStreaming();
            if (portalOdo   != null) portalOdo.resumeStreaming();
        }

        if (allowMotifScan && !motifKnown) motifLoop();

        if (allowOdoFuse && (now - lastFuseMs) >= (int)(1000.0/Constants.Vision.ODO_FUSE_RATE_HZ)) {
            lastFuseMs = now;
            odoLoop(preferRedGoal);
        }
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
