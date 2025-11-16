package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

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
 * Vision_OdoCam_Test
 *
 * Single-camera AprilTag test for the ODO camera (Constants.Vision.ODO_CAM_NAME).
 * - Uses one VisionPortal (no multi-portal issues).
 * - Uses MJPEG at the resolution in Constants.Vision.
 * - Null-safe detection handling (no random NPEs).
 *
 * NOTE: Pose/range will only be accurate AFTER you set lens intrinsics via
 *       setLensIntrinsics(fx, fy, cx, cy) with calibrated values.
 */
@TeleOp(name = "Test_Vision_OdoCam", group = "Vision")
//@Disabled
public class Test_Vision_OdoCam extends OpMode {

    private AprilTagProcessor proc;
    private VisionPortal portal;

    @Override
    public void init() {
        // Tag library for this season
        AprilTagLibrary tagLib = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        proc = buildProcessor(tagLib);

        WebcamName cam = hardwareMap.get(
                WebcamName.class, Constants.Vision.ODO_CAM_NAME);

        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(
                        new Size(Constants.Vision.STREAM_WIDTH,
                                Constants.Vision.STREAM_HEIGHT))
                // Single portal, so default live view behavior is fine; no container override needed.
                .addProcessor(proc)
                .build();

        telemetry.addLine("Vision_OdoCam_Test init complete.");
        telemetry.addData("Camera", Constants.Vision.ODO_CAM_NAME);
        telemetry.addData("Resolution", "%dx%d",
                Constants.Vision.STREAM_WIDTH,
                Constants.Vision.STREAM_HEIGHT);
    }

    @Override
    public void loop() {
        if (portal != null) {
            telemetry.addData("ODO state", portal.getCameraState());
            telemetry.addData("ODO fps",   "%.1f", portal.getFps());
        }

        telemetry.addLine("=== ODOCAM detections ===");
        logDetections("O", proc);

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

        // Lens intrinsics from Constants.Vision (odo cam calibration)
        b.setLensIntrinsics(
                Constants.Vision.ODO_FX,
                Constants.Vision.ODO_FY,
                Constants.Vision.ODO_CX,
                Constants.Vision.ODO_CY
        );

        return b.build();
    }

    @Override
    public void stop() {
        if (portal != null) portal.close();
    }
}
