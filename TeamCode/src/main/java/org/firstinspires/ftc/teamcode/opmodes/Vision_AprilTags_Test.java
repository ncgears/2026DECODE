package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.AprilTagVision;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


import java.util.List;


@TeleOp(name = "Vision_AprilTags_Test", group = "Vision")
//@Disabled
public class Vision_AprilTags_Test extends OpMode {
    private AprilTagVision vision;
    private TelemetryUtil T;


    @Override public void init() {
        T = new TelemetryUtil(this);
        vision = new AprilTagVision(hardwareMap);
    }


    @Override public void loop() {
        List<AprilTagDetection> d = vision.atag.getDetections();
        T.t(3, "Detections", d.size());
        if (!d.isEmpty()) {
            AprilTagDetection first = d.get(0);
            T.t(4, "id", first.id);
            T.t(4, "center", String.format("(%.1f, %.1f)", first.center.x, first.center.y));
            if (first.metadata != null) {
                T.t(4, "range(m)", first.ftcPose.range);
                T.t(4, "bearing(deg)", first.ftcPose.bearing);
                T.t(4, "yaw(deg)", first.ftcPose.yaw);
            }
        }
        telemetry.update();
    }
}