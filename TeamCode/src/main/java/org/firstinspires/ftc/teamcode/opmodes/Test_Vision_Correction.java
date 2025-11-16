package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUHeading;
import org.firstinspires.ftc.teamcode.subsystems.PinpointHeading;
import org.firstinspires.ftc.teamcode.util.Shaping;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Test_Vision_Correction
 *
 * Drive+diagnostics OpMode to sanity-check odocam goal-tag localization
 * against IMU/Pinpoint and verify the camera feed is sane.
 *
 * Features:
 *  - g1 sticks drive the robot (field-centric using IMU heading).
 *  - g1.BACK: zero IMU and Pinpoint headings and clear fusion offset.
 *  - Uses odocam and goal tags (TAG_GOAL_RED / TAG_GOAL_BLUE) to form a "vision heading".
 *  - Fuses vision heading into a corrected Pinpoint heading with a small alpha.
 *  - Telemetry shows:
 *      - IMU heading (rad/deg)
 *      - Pinpoint heading (raw & fused)
 *      - Vision heading (rad/deg) when a goal tag is seen
 *      - Camera->tag ftcPose and simple per-tag info
 *      - OdoCam FPS and tag count so you know vision is alive
 */
@TeleOp(name = "Test_Vision_Correction", group = "Diagnostics")
public class Test_Vision_Correction extends OpMode {

    // Subsystems
    private DriveSubsystem drive;
    private IMUHeading imuHeading;
    private PinpointHeading pinpointHeading;

    // Vision (odocam for goal tags)
    private VisionPortal odoPortal;
    private AprilTagProcessor odoProcessor;

    // Heading fusion state
    private double pinCorrectionOffsetRad = 0.0;
    private boolean lastBack = false;

    // Drive shaping
    private SlewRateLimiter slewX, slewY, slewR;

    @Override
    public void init() {
        // Subsystems
        drive = new DriveSubsystem(hardwareMap);
        imuHeading = new IMUHeading(hardwareMap);
        pinpointHeading = new PinpointHeading(hardwareMap);

        // Slew limiters match TeleOp behavior
        slewX = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewY = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewR = new SlewRateLimiter(Constants.Drive.SLEW_ROTATION);

        // Vision: odocam for goal tags
        initOdoVision();

        // Start with both headings zeroed
        imuHeading.zeroHeading();
        pinpointHeading.zeroHeading();
        pinCorrectionOffsetRad = 0.0;

        telemetry.addLine("Test_Vision_Correction: INIT");
        telemetry.addLine("g1 sticks = drive (field-centric via IMU)");
        telemetry.addLine("g1.BACK = zero IMU + Pinpoint + fusion offset");
        telemetry.addLine("Using odocam goal tags to nudge Pinpoint heading.");
    }

    private void initOdoVision() {
        // Tag library: current game defaults (includes goal tags).
        AprilTagLibrary lib = AprilTagGameDatabase.getCurrentGameTagLibrary();

        AprilTagProcessor.Builder pb = new AprilTagProcessor.Builder()
                .setTagLibrary(lib)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawTagID(true);

        // Lens intrinsics from Constants.Vision (odo cam calibration)
        pb.setLensIntrinsics(
                Constants.Vision.ODO_FX,
                Constants.Vision.ODO_FY,
                Constants.Vision.ODO_CX,
                Constants.Vision.ODO_CY
        );

        odoProcessor = pb.build();

        WebcamName cam = hardwareMap.get(WebcamName.class, Constants.Vision.ODO_CAM_NAME);
        odoPortal = new VisionPortal.Builder()
                .setCamera(cam)
                .addProcessor(odoProcessor)
                .setCameraResolution(new Size(Constants.Vision.STREAM_WIDTH, Constants.Vision.STREAM_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    @Override
    public void loop() {
        // 1) Handle heading reset on g1.back
        boolean backNow = gamepad1.back;
        boolean backEdge = backNow && !lastBack;
        lastBack = backNow;

        if (backEdge) {
            imuHeading.zeroHeading();
            pinpointHeading.zeroHeading();
            pinCorrectionOffsetRad = 0.0;
        }

        // 2) Drive control (field-centric using IMU heading)
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        lx = Shaping.translate(lx);
        ly = Shaping.translate(ly);
        rx = Shaping.rotate(rx);

        double scale = gamepad1.left_bumper ? Constants.Drive.PRECISION_SCALE : 1.0;
        double xCmd = lx * scale;
        double yCmd = ly * scale;
        double rCmd = rx * scale;

        // Field-centric using IMU heading
        double h = imuHeading.getHeadingRad();
        double cos = Math.cos(-h), sin = Math.sin(-h);
        double fxCmd = xCmd * cos - yCmd * sin;
        double fyCmd = xCmd * sin + yCmd * cos;
        double rotCmd = rCmd * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);

        if (Constants.Drive.SLEW_ENABLED) {
            fxCmd = slewX.calculate(fxCmd);
            fyCmd = slewY.calculate(fyCmd);
            rotCmd = slewR.calculate(rotCmd);
        }

        drive.driveRobotCentric(fxCmd, fyCmd, rotCmd);

        // 3) Read headings (IMU + Pinpoint)
        double imuRad = imuHeading.getHeadingRad();
        double pinRawRad = pinpointHeading.getHeadingRad();
        double pinFusedRad = wrap(pinRawRad + pinCorrectionOffsetRad);

        // 4) Pull vision data from odocam and update fusion
        List<AprilTagDetection> allDetections = odoProcessor.getDetections();
        AprilTagDetection bestGoal = chooseBestGoalTag(allDetections);
        Double visionHeadingRad = null;  // null => no valid goal tag

        double visX = 0, visY = 0, visYawDeg = 0, visRange = 0;

        if (bestGoal != null && bestGoal.ftcPose != null) {
            // Camera->tag pose in FTC frame (meters & degrees)
            visX = bestGoal.ftcPose.x;    // forward from camera to tag
            visY = bestGoal.ftcPose.y;    // left from camera to tag
            visYawDeg = bestGoal.ftcPose.yaw;
            visRange = Math.hypot(visX, visY);

            // Crude "vision heading" estimate:
            // If the tag is on the goal wall and facing the field,
            // the robot heading is roughly -tagYaw (camera frame).
            double visYawRad = Math.toRadians(visYawDeg);
            visionHeadingRad = wrap(-visYawRad);

            // Fuse into Pinpoint heading with simple offset-based correction
            double err = wrap(visionHeadingRad - pinFusedRad);
            double errDeg = Math.toDegrees(Math.abs(err));
            boolean withinRange = visRange <= Constants.Vision.ODO_MAX_RANGE_M;
            boolean withinDelta = errDeg <= Constants.Vision.ODO_MAX_DELTA_DEG;

            if (withinRange && withinDelta) {
                pinCorrectionOffsetRad += Constants.Vision.ODO_FUSE_ALPHA * err;
                pinFusedRad = wrap(pinRawRad + pinCorrectionOffsetRad);
            }
        }

        // 5) Telemetry

        telemetry.addLine("=== Test_Vision_Correction ===");
        telemetry.addLine("g1.BACK: zero IMU + Pinpoint + fusion offset");

        // Camera health / tag list
        double fps = (odoPortal != null) ? odoPortal.getFps() : 0.0;
        int tagCount = (allDetections != null) ? allDetections.size() : 0;
        telemetry.addData("OdoCam FPS", "%.1f", fps);

        if (tagCount == 0) {
            telemetry.addData("OdoCam tags", "none");
        } else {
            telemetry.addData("OdoCam tags", "#%d", tagCount);

            int idx = 0;
            for (AprilTagDetection d : allDetections) {
                if (d == null || d.ftcPose == null) continue;

                boolean isGoal =
                        (d.id == Constants.Vision.TAG_GOAL_RED) ||
                            (d.id == Constants.Vision.TAG_GOAL_BLUE);

                telemetry.addData(
                        "Tag[" + idx + "]",
                        "id=%d%s, range=%.2f m, bearing=%.1f deg",
                        d.id,
                        isGoal ? " (GOAL)" : "",
                        d.ftcPose.range,
                        d.ftcPose.bearing
                );
                idx++;
                if (idx >= 4) break; // cap to keep DS readable
            }
        }

        telemetry.addLine("== Headings ==");
        telemetry.addData("IMU",
                "%.3f rad (%.1f deg)",
                imuRad, Math.toDegrees(imuRad));
        telemetry.addData("Pinpoint (raw)",
                "%.3f rad (%.1f deg)",
                pinRawRad, Math.toDegrees(pinRawRad));
        telemetry.addData("Pinpoint (fused)",
                "%.3f rad (%.1f deg)",
                pinFusedRad, Math.toDegrees(pinFusedRad));
        telemetry.addData("Fusion offset",
                "%.3f rad (%.1f deg)",
                pinCorrectionOffsetRad, Math.toDegrees(pinCorrectionOffsetRad));

        if (visionHeadingRad != null) {
            double errVsImu = wrap(visionHeadingRad - imuRad);
            double errVsPin = wrap(visionHeadingRad - pinFusedRad);
            telemetry.addLine("== Vision (odocam goal tag) ==");
            telemetry.addData("Goal tag id", bestGoal.id);
            telemetry.addData("Cam->Tag ftcPose",
                    "x=%.3f m, y=%.3f m, yaw=%.1f deg, range=%.3f m",
                    visX, visY, visYawDeg, visRange);
            telemetry.addData("Vision heading",
                    "%.3f rad (%.1f deg)",
                    visionHeadingRad, Math.toDegrees(visionHeadingRad));
            telemetry.addData("Err vs IMU",
                    "%.3f rad (%.1f deg)",
                    errVsImu, Math.toDegrees(errVsImu));
            telemetry.addData("Err vs Pinpoint(fused)",
                    "%.3f rad (%.1f deg)",
                    errVsPin, Math.toDegrees(errVsPin));
        } else {
            telemetry.addLine("== Vision (odocam) ==");
            telemetry.addData(
                    "No goal tag in view",
                    "%d=RED, %d=BLUE",
                    Constants.Vision.TAG_GOAL_RED,
                    Constants.Vision.TAG_GOAL_BLUE
            );
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        if (odoPortal != null) {
            odoPortal.close();
        }
    }

    // ---------- Helpers ----------

    /** Choose the closest goal tag (blue or red) from the detection list, or null if none. */
    private AprilTagDetection chooseBestGoalTag(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) return null;

        AprilTagDetection best = null;
        double bestDist2 = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : detections) {
            if (d == null || d.ftcPose == null) continue;
            int id = d.id;
            if (id != Constants.Vision.TAG_GOAL_BLUE && id != Constants.Vision.TAG_GOAL_RED) continue;

            double x = d.ftcPose.x;
            double y = d.ftcPose.y;
            double dist2 = x * x + y * y;
            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                best = d;
            }
        }
        return best;
    }

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
