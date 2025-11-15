package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Vision_MotifCam_Test
 *
 * Single-camera AprilTag test for the MOTIF camera (Constants.Vision.MOTIF_CAM_NAME).
 */
@TeleOp(name = "Vision_MotifCam_Test", group = "Vision")
//@Disabled
public class Vision_MotifCam_Test extends OpMode {

    private AprilTagProcessor proc;
    private VisionPortal portal;

    @Override
    public void init() {
        AprilTagLibrary tagLib = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        proc = buildProcessor(tagLib);

        WebcamName cam = hardwareMap.get(
                WebcamName.class, Constants.Vision.MOTIF_CAM_NAME);

        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(
                        new Size(Constants.Vision.STREAM_WIDTH,
                                Constants.Vision.STREAM_HEIGHT))
                .addProcessor(proc)
                .build();

        telemetry.addLine("Vision_MotifCam_Test init complete.");
        telemetry.addData("Camera", Constants.Vision.MOTIF_CAM_NAME);
        telemetry.addData("Resolution", "%dx%d",
                Constants.Vision.STREAM_WIDTH,
                Constants.Vision.STREAM_HEIGHT);
    }

    @Override
    public void loop() {
        if (portal != null) {
            telemetry.addData("MOTIF state", portal.getCameraState());
            telemetry.addData("MOTIF fps",   "%.1f", portal.getFps());
        }

        telemetry.addLine("=== MOTIFCAM detections ===");
        logDetections("M", proc);

        telemetry.update();
    }

    private void logDetections(String prefix, AprilTagProcessor p) {
        if (p == null) {
            telemetry.addLine("  (processor is null)");
            return;
        }

        List<AprilTagDetection> detections = p.getDetections();
        if (detections == null || detections.isEmpty()) {
            telemetry.addLine("  (none)");
            return;
        }

        int i = 0;
        for (AprilTagDetection d : detections) {
            if (d == null) {
                telemetry.addData("  [%d] null detection, skipped", i++);
                continue;
            }

            int id = d.id;

            if (d.ftcPose == null) {
                telemetry.addData(prefix + " tag", id);
                telemetry.addLine("    (pose not available yet)");
                i++;
                continue;
            }

            double rangeM     = d.ftcPose.range;
            double bearingDeg = d.ftcPose.bearing;
            double yawDeg     = d.ftcPose.yaw;
            double x          = d.ftcPose.x;
            double y          = d.ftcPose.y;
            double z          = d.ftcPose.z;

            telemetry.addData(prefix + " tag", id);
            telemetry.addData("    id",      id);
            telemetry.addData("    range",   "%.2f m", rangeM);
            telemetry.addData("    bearing", "%.1f deg", bearingDeg);
            telemetry.addData("    yaw",     "%.1f deg", yawDeg);
            telemetry.addData("    x,y,z",   "%.2f, %.2f, %.2f", x, y, z);

            i++;
        }
    }

    private AprilTagProcessor buildProcessor(AprilTagLibrary lib) {
        AprilTagProcessor.Builder b = new AprilTagProcessor.Builder()
                .setTagLibrary(lib)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true);

        // TODO: fill these once you calibrate motifcam at this resolution:
        double fx = 905.168;
        double fy = 905.168;
        double cx = 667.265;
        double cy = 356.696;
        b.setLensIntrinsics(fx, fy, cx, cy);

        return b.build();
    }

    @Override
    public void stop() {
        if (portal != null) portal.close();
    }
}
